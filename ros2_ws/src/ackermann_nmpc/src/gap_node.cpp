#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Drift parking into a gap between two boxes on the left, from rest. At rest the lidar finds the gap
// (the side of the boxes towards the car, the two rows of points and the free stretch between them). The
// car goes slowly on a line, then kicks: full lock, full throttle, the rear steps out and the car drifts
// round. It brakes (speed 0: the motor driver locks the rear wheels) when the model, run to the stop,
// says it will end at target; in the slide the steering is chosen every step the same way (bisection),
// and the car slides sideways into the gap. The car holds the line of the start; it kicks where it will
// stop with as much room at its nose as beyond its swept tail, from where the car stopped against the
// kick (kick_ahead, kick_left). The boxes must be where the car will stop sideways (LATERAL_TOL).
// The model is the refit of 2026-09-30, carried from the kick with what was sent and the wheel speed. The
// last gyro reading is gyro_lag older than its message: its yaw rate corrects the model's of that time,
// and the heading now is the gyro's plus what the model turned since.

constexpr double DT_MODEL = 0.005;   // s, model steps in the prediction
constexpr double HORIZON = 1.6;      // s, longest prediction
constexpr double T_RAMP = 1.0;       // s from rest to speed
constexpr double T_ARM = 0.5;        // s armed at rest before starting
constexpr double T_KICK = 2.0;       // s kicking without braking: stop
constexpr double T_SLIDE = 1.5;      // s from the brake to the end
constexpr double IMU_TIMEOUT = 0.1;  // s without the gyro: stop
constexpr double SLIDE_STEER_MIN = -0.15;   // countersteered further the car swings back (hbn runs)
constexpr double LOOKAHEAD = 0.4;    // m, pure pursuit on the line of the approach
constexpr double WHEEL_SLIP = 0.97;  // car over wheels at the launch (0.94-1.00, identification.ipynb)
constexpr double HALF_WIDTH = 0.10;  // m, the car to the outside of the wheels
constexpr double ROW_BREAK = 0.10;   // m between points of one box's row
constexpr double KICK_MIN = 0.35;    // m from the start: the car at speed
constexpr double LATERAL_TOL = 0.06; // m, the boxes from where the car will stop sideways

double interp(double x, const std::vector<double> & xs, const std::vector<double> & ys)
{
  if (x <= xs.front()) {
    return ys.front();
  }
  if (x >= xs.back()) {
    return ys.back();
  }
  const size_t i = std::upper_bound(xs.begin(), xs.end(), x) - xs.begin();
  return ys[i - 1] + (ys[i] - ys[i - 1]) * (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
}

struct State
{
  double vx, vy, r;       // at the CG
  double servo, d;        // steering angle at the servo (rate limited) and at the wheels (lag)
  double ax, ay;          // acceleration of the last step, for the load transfer
};

struct Command
{
  rclcpp::Time t;
  double steer, speed;
};

struct Past
{
  rclcpp::Time t;
  double r, psi;       // the model's
};

class GapNode : public rclcpp::Node
{
public:
  GapNode()
  : Node("gap")
  {
    // vehicle_params.yaml
    wheel_radius_ = declare_parameter<double>("wheel_radius");
    m_ = declare_parameter<double>("mass");
    iz_ = declare_parameter<double>("iz");
    lf_ = declare_parameter<double>("lf");
    lr_ = declare_parameter<double>("lr");
    h_ = declare_parameter<double>("h_cg");
    track_ = declare_parameter<double>("track");
    steer_cmd_ = declare_parameter<std::vector<double>>("steer_cmd");
    steer_angle_ = declare_parameter<std::vector<double>>("steer_angle");
    steer_rate_ = declare_parameter<double>("steer_rate_max");
    steer_dead_ = declare_parameter<double>("steer_dead");
    steer_lag_ = declare_parameter<double>("fit_steer_lag");
    c_ = declare_parameter<double>("tire_c");
    mu_f_ = declare_parameter<double>("fit_mu_f");
    b_f_ = declare_parameter<double>("fit_b_f");
    mu_r_ = declare_parameter<double>("fit_mu_r");
    b_r_ = declare_parameter<double>("fit_b_r");
    lambda_ = declare_parameter<double>("fit_lambda");
    locked_ = declare_parameter<double>("fit_locked");
    locked_y_ = declare_parameter<double>("fit_locked_y");
    lt_x_ = declare_parameter<double>("fit_lt_x");
    lt_y_ = declare_parameter<double>("fit_lt_y");
    wheel_brake_ = declare_parameter<double>("wheel_brake");
    gyro_lag_ = declare_parameter<double>("gyro_lag");
    gyro_scale_ = declare_parameter<double>("gyro_scale");
    // the run
    speed_ = declare_parameter("speed", 0.4);             // m/s on the line
    kick_speed_ = declare_parameter("kick_speed", 1.5);   // m/s command, above what the motor reaches
    target_ = declare_parameter("target", 180.0) * M_PI / 180;
    // m, middle of the stopped car ahead of / left of where the kick started: 0.23-0.29 / 0.38-0.48 in
    // the open-loop runs (hbk_1-8, sideways.ipynb)
    kick_ahead_ = declare_parameter("kick_ahead", 0.25);
    kick_left_ = declare_parameter("kick_left", 0.43);
    side_in_ = declare_parameter("side_in", 0.02);        // m, side of the stopped car inside the boxes
    // m, the tail swings out past where it stops as the car comes round (0.12-0.18 in hbk_1-8): the car
    // stops with as much room ahead of its nose as beyond the swept tail
    sweep_ = declare_parameter("sweep", 0.17);
    slide_ref_ = declare_parameter("slide_ref", 0.2);     // slide steering the brake is decided with
    gap_min_ = declare_parameter("gap_min", 0.40);        // m, a gap outside gap_min .. gap_max: refuse
    gap_max_ = declare_parameter("gap_max", 1.0);
    const double g = 9.81, l = lf_ + lr_;
    nf_ = m_ * g * lr_ / l;
    nr_ = m_ * g * lf_ / l;

    pub_drive_ = create_publisher<ackermann_msgs::msg::AckermannDrive>("/drive", 10);
    pub_arm_ = create_publisher<std_msgs::msg::Bool>("/arm", 10);
    pub_state_ = create_publisher<std_msgs::msg::Float64MultiArray>("gap/state", 10);
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data_raw", 50, [this](const sensor_msgs::msg::Imu & m) { on_imu(m); });
    sub_joints_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, [this](const sensor_msgs::msg::JointState & m) { on_joints(m); });
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::LaserScan & m) { on_scan(m); });
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(20), [this] { tick(); });
  }

private:
  // at rest: points left of the car from a few scans
  void on_scan(const sensor_msgs::msg::LaserScan & m)
  {
    if (phase_ != Phase::WAIT) {
      return;
    }
    if (std::isnan(lidar_x_)) {
      try {
        lidar_x_ = tf_buffer_->lookupTransform("base_footprint", m.header.frame_id, tf2::TimePointZero).transform.translation.x;
      } catch (const tf2::TransformException &) {
        return;    // the transform comes a moment after the start
      }
    }
    for (size_t i = 0; i < m.ranges.size(); i++) {
      if (std::isfinite(m.ranges[i])) {
        const double a = m.angle_min + i * m.angle_increment;
        const double x = m.ranges[i] * std::cos(a) + lidar_x_, y = m.ranges[i] * std::sin(a);
        if (x > -0.3 && x < 2.0 && y > 0.15 && y < 1.0) {
          left_x_.push_back(x);
          left_y_.push_back(y);
        }
      }
    }
    scans_++;
  }

  // the gap: the side of the boxes is the nearest line of points; along it two rows, the gap between
  bool find_gap()
  {
    if (left_y_.size() < 50) {
      return false;
    }
    std::vector<double> ys = left_y_;
    std::sort(ys.begin(), ys.end());
    const double near = ys[ys.size() / 10];
    std::vector<double> side_y, side_x;
    for (size_t i = 0; i < left_x_.size(); i++) {
      if (std::abs(left_y_[i] - near) < 0.03) {
        side_y.push_back(left_y_[i]);
        side_x.push_back(left_x_[i]);
      }
    }
    std::sort(side_y.begin(), side_y.end());
    face_y_ = side_y[side_y.size() / 2];
    std::sort(side_x.begin(), side_x.end());
    // rows of points and the free stretches between them: the widest one is the gap
    double widest = 0.0;
    for (size_t i = 1; i < side_x.size(); i++) {
      if (side_x[i] - side_x[i - 1] > std::max(widest, ROW_BREAK)) {
        widest = side_x[i] - side_x[i - 1];
        gap_from_ = side_x[i - 1];
        gap_to_ = side_x[i];
      }
    }
    return widest > 0.0;
  }

  void on_imu(const sensor_msgs::msg::Imu & m)
  {
    const rclcpp::Time now = get_clock()->now();
    const double gz = m.angular_velocity.z;
    if (phase_ == Phase::WAIT) {    // at rest: the bias
      bias_samples_.push_back(gz);
      last_imu_ = now;
      return;
    }
    const double r = (gz - bias_) * gyro_scale_;
    if (last_imu_.nanoseconds() > 0) {
      heading_ += (now - last_imu_).seconds() * (r + r_) / 2;
    }
    r_ = r;
    last_imu_ = now;
  }

  // wheel speed from the encoder counts (the speed in the message has the firmware's 45 ms EMA)
  void on_joints(const sensor_msgs::msg::JointState & m)
  {
    if (m.position.size() < 2) {
      return;
    }
    const rclcpp::Time t(m.header.stamp);
    const double p = wheel_radius_ * (m.position[0] + m.position[1]) / 2;
    if (joints_t_.nanoseconds() > 0 && t > joints_t_) {
      u_ = (p - joints_p_) / (t - joints_t_).seconds();
    }
    joints_p_ = p;
    joints_t_ = t;
  }

  // one rear wheel at surface speed u under load n: force against its sliding direction; braked, the
  // friction x locked along the wheel and x locked_y across it
  void rear_wheel(double vx, double vy, double u, double n, bool braked, double & fx, double & fy) const
  {
    const double k = braked ? locked_ : 1.0, ky = braked ? locked_y_ / locked_ : 1.0;
    const double sx = (vx - u) / std::max(u, 0.1) / lambda_, sy = vy / std::max(u, 0.1);
    const double s = std::hypot(sx, sy) + 1e-9;
    const double f = k * mu_r_ * std::max(n, 0.0) * std::sin(c_ * std::atan(b_r_ / k * s));
    fx = -f * sx / s;
    fy = -ky * f * sy / s;
  }

  // the refit: load transfer from the last step's acceleration (lt_x, lt_y of the rigid car's)
  void step(State & x, double u, bool braked, double d_cmd, double h) const
  {
    const double vx = std::max(x.vx, 0.05), l = lf_ + lr_;
    const double dn_x = lt_x_ * m_ * x.ax * h_ / l, dn = lt_y_ * m_ * x.ay * h_ / track_;
    const double nf = nf_ - dn_x, nr = nr_ + dn_x;
    const double fyf = mu_f_ * nf * std::sin(c_ * std::atan(b_f_ * (x.d - std::atan((x.vy + lf_ * x.r) / vx))));
    double fxl, fyl, fxr, fyr;
    rear_wheel(vx - track_ / 2 * x.r, x.vy - lr_ * x.r, u, nr / 2 - dn, braked, fxl, fyl);
    rear_wheel(vx + track_ / 2 * x.r, x.vy - lr_ * x.r, u, nr / 2 + dn, braked, fxr, fyr);
    const double fx = fxl + fxr - fyf * std::sin(x.d);
    const double fy = fyf * std::cos(x.d) + fyl + fyr;
    const double mz = lf_ * fyf * std::cos(x.d) - lr_ * (fyl + fyr) + track_ / 2 * (fxr - fxl);
    const double vx_next = std::max(x.vx + h * (fx / m_ + x.vy * x.r), 0.0);
    x.vy += h * (fy / m_ - x.vx * x.r);
    x.r += h * mz / iz_;
    x.vx = vx_next;
    x.servo += std::clamp(d_cmd - x.servo, -steer_rate_ * h, steer_rate_ * h);
    x.d += h * (x.servo - x.d) / steer_lag_;
    x.ax = fx / m_;
    x.ay = fy / m_;
  }

  // the command acting at time t: sent steer_dead before
  const Command & acting(const rclcpp::Time & t) const
  {
    for (auto it = sent_.rbegin(); it != sent_.rend(); ++it) {
      if ((t - it->t).seconds() >= steer_dead_) {
        return *it;
      }
    }
    return sent_.front();
  }

  // heading where the car stops if it brakes with the steering held, from now: the commands already
  // sent act until their dead time is over, then braked (the wheels as the shorted motor)
  double final_heading(State x, double psi, double u, double steer, const rclcpp::Time & now) const
  {
    const double d_hold = interp(steer, steer_cmd_, steer_angle_);
    for (int i = 0; i < HORIZON / DT_MODEL; i++) {
      const rclcpp::Time t = now + rclcpp::Duration::from_seconds(i * DT_MODEL);
      const bool pending = i * DT_MODEL < steer_dead_;
      const Command & c = acting(t);
      const bool braked = pending ? c.speed < 0.02 : true;
      if (braked) {
        u *= std::exp(-DT_MODEL / wheel_brake_);
      }
      step(x, u, braked, pending ? interp(c.steer, steer_cmd_, steer_angle_) : d_hold, DT_MODEL);
      psi += DT_MODEL * x.r;
      if (!pending && std::hypot(x.vx, x.vy + lf_ * x.r) < 0.04 && std::hypot(x.vx, x.vy - lr_ * x.r) < 0.04) {
        break;
      }
    }
    return psi;
  }

  double slide_steering(const rclcpp::Time & now) const
  {
    const double full = steer_cmd_.back(), counter = SLIDE_STEER_MIN;
    if (final_heading(model_, heading_now_, u_, counter, now) >= target_) {
      return counter;
    }
    if (final_heading(model_, heading_now_, u_, full, now) <= target_) {
      return full;
    }
    double lo = counter, hi = full;
    for (int i = 0; i < 8; i++) {
      const double mid = (lo + hi) / 2;
      (final_heading(model_, heading_now_, u_, mid, now) < target_ ? lo : hi) = mid;
    }
    return (lo + hi) / 2;
  }

  // the model over the last step with what was acting and the wheel speed measured; its yaw rate corrected by
  // the gyro's error of when the last reading was taken, and the heading now from the gyro's
  void carry_model(const rclcpp::Time & now)
  {
    for (int i = 0; i < 20; i++) {
      const Command & c = acting(now - rclcpp::Duration::from_seconds(0.02 - 0.001 * i));
      step(model_, u_, c.speed < 0.02, interp(c.steer, steer_cmd_, steer_angle_), 0.001);
      psi_model_ += 0.001 * model_.r;
    }
    history_.push_back({now, model_.r, psi_model_});
    while ((now - history_.front().t).seconds() > gyro_lag_ + 0.1) {
      history_.pop_front();
    }
    // the model when the last gyro reading was taken, between the two steps around it
    const rclcpp::Time then = last_imu_ - rclcpp::Duration::from_seconds(gyro_lag_);
    size_t j = 0;
    while (j + 1 < history_.size() && history_[j + 1].t <= then) {
      j++;
    }
    double r_then = history_[j].r, psi_then = history_[j].psi;
    if (j + 1 < history_.size() && history_[j].t < then) {
      const double a = (then - history_[j].t).seconds() / (history_[j + 1].t - history_[j].t).seconds();
      r_then += a * (history_[j + 1].r - history_[j].r);
      psi_then += a * (history_[j + 1].psi - history_[j].psi);
    }
    // the model from then on, kept history included: else the next reading corrects the same error again
    const double dr = r_ - r_then;
    for (Past & p : history_) {
      if (p.t > then) {
        p.psi += dr * (p.t - then).seconds();
        p.r += dr;
      }
    }
    model_.r += dr;
    psi_model_ += dr * (now - then).seconds();
    heading_now_ = heading_ + psi_model_ - psi_then;
  }

  // arming zeroes the setpoint (firmware, sim_car): only at rest, not with every command
  void arm(bool on)
  {
    std_msgs::msg::Bool a;
    a.data = on;
    pub_arm_->publish(a);
  }

  void send(double speed, double steer, const rclcpp::Time & now)
  {
    ackermann_msgs::msg::AckermannDrive d;
    d.speed = speed;
    d.steering_angle = steer;
    pub_drive_->publish(d);
    sent_.push_back({now, steer, speed});
    while (sent_.size() > 2 && (now - sent_[1].t).seconds() > steer_dead_ + 0.1) {
      sent_.pop_front();
    }
  }

  void tick()
  {
    const auto start = std::chrono::steady_clock::now();
    const rclcpp::Time now = get_clock()->now();
    const double t = (now - t_phase_).seconds();
    if (phase_ != Phase::WAIT && phase_ != Phase::DONE && (now - last_imu_).seconds() > IMU_TIMEOUT) {
      RCLCPP_ERROR(get_logger(), "no gyro for %.2f s: stop", (now - last_imu_).seconds());
      next(Phase::DONE, now);
    }
    switch (phase_) {
      case Phase::WAIT:
        if (bias_samples_.size() >= 50 && scans_ >= 5 && pub_drive_->get_subscription_count() > 0 &&
          pub_arm_->get_subscription_count() > 0)
        {
          if (!find_gap()) {
            RCLCPP_ERROR(get_logger(), "no gap on the left: stop");
            next(Phase::DONE, now);
            break;
          }
          // where the middle of the car stops: the same room at the nose and beyond the swept tail; its side
          // side_in inside the boxes
          mid_x_ = (gap_from_ + gap_to_ - sweep_) / 2;
          const double mid_y = face_y_ + HALF_WIDTH + side_in_;
          RCLCPP_INFO(get_logger(), "gap %.3f .. %.3f m ahead, boxes %.3f m left: kick at %.3f m, the car %.3f m too far in",
            gap_from_, gap_to_, face_y_, mid_x_ - kick_ahead_, kick_left_ - mid_y);
          if (gap_to_ - gap_from_ < gap_min_ || gap_to_ - gap_from_ > gap_max_ || mid_x_ - kick_ahead_ < KICK_MIN ||
            std::abs(kick_left_ - mid_y) > LATERAL_TOL)
          {
            RCLCPP_ERROR(get_logger(), "gap %.3f m (%.2f .. %.2f), kick at %.3f m (min %.2f), sideways %.3f m (max %.2f): stop",
              gap_to_ - gap_from_, gap_min_, gap_max_, mid_x_ - kick_ahead_, KICK_MIN, kick_left_ - mid_y, LATERAL_TOL);
            next(Phase::DONE, now);
            break;
          }
          std::sort(bias_samples_.begin(), bias_samples_.end());
          bias_ = bias_samples_[bias_samples_.size() / 2];
          next(Phase::ARM, now);
        }
        return;
      case Phase::ARM:
        arm(true);
        send(0.0, 0.0, now);
        if (t > T_ARM) {
          heading_ = 0.0;
          next(Phase::STRAIGHT, now);
        }
        break;
      case Phase::STRAIGHT: {
        // on the line of the start: position from the wheels and the gyro heading
        x_ += WHEEL_SLIP * u_ * std::cos(heading_) * 0.02;
        y_ += WHEEL_SLIP * u_ * std::sin(heading_) * 0.02;
        heading_now_ = heading_;
        const double ty = -std::sin(heading_) * LOOKAHEAD - std::cos(heading_) * y_;
        const double d = std::atan(2 * (lf_ + lr_) * ty / (LOOKAHEAD * LOOKAHEAD + y_ * y_));
        const double steer = interp(d, steer_angle_, steer_cmd_);
        // where the car would stop kicking now, along the line
        const double stop_x = x_ + std::cos(heading_) * kick_ahead_ - std::sin(heading_) * kick_left_;
        if (stop_x >= mid_x_) {
          if (t < T_RAMP) {
            RCLCPP_ERROR(get_logger(), "kick at %.3f m, before full speed: stop", x_);
            next(Phase::DONE, now);
            break;
          }
          // the model from the line: along the car at the wheel speed
          const double d_now = interp(steer, steer_cmd_, steer_angle_);
          model_ = {WHEEL_SLIP * u_, 0.0, r_, d_now, d_now, 0.0, 0.0};
          psi_model_ = 0.0;
          history_.clear();
          RCLCPP_INFO(get_logger(), "kick at %.3f m, %.3f m left, heading %.1f deg", x_, y_, heading_ * 180 / M_PI);
          next(Phase::KICK, now);
          send(kick_speed_, steer_cmd_.back(), now);
          break;
        }
        send(speed_ * std::min(1.0, t / T_RAMP), steer, now);
        break;
      }
      case Phase::KICK:
        carry_model(now);
        if (t > T_KICK) {
          RCLCPP_ERROR(get_logger(), "heading %.0f deg after %.1f s of kick: stop", heading_now_ * 180 / M_PI, t);
          next(Phase::DONE, now);
        } else if (final_heading(model_, heading_now_, u_, slide_ref_, now) >= target_) {
          heading_brake_ = heading_now_;
          next(Phase::SLIDE, now);
          steer_ = slide_steering(now);
          send(0.0, steer_, now);
        } else {
          send(kick_speed_, steer_cmd_.back(), now);
        }
        break;
      case Phase::SLIDE:
        carry_model(now);
        steer_ = slide_steering(now);
        send(0.0, steer_, now);
        if (t > T_SLIDE) {
          RCLCPP_INFO(get_logger(), "braked at %.1f deg, stopped at %.1f deg (target %.1f)",
            heading_brake_ * 180 / M_PI, heading_now_ * 180 / M_PI, target_ * 180 / M_PI);
          next(Phase::DONE, now);
        }
        break;
      case Phase::DONE:
        send(0.0, 0.0, now);
        arm(false);
        if (t > 0.5) {
          rclcpp::shutdown();
        }
        break;
    }
    std_msgs::msg::Float64MultiArray s;
    const double tick_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    s.data = {static_cast<double>(phase_), heading_, heading_now_, r_, u_, steer_, model_.vx, model_.vy, model_.r,
              x_, y_, tick_ms};
    pub_state_->publish(s);
  }

  enum class Phase { WAIT, ARM, STRAIGHT, KICK, SLIDE, DONE };

  void next(Phase p, const rclcpp::Time & now)
  {
    phase_ = p;
    t_phase_ = now;
  }

  double wheel_radius_, m_, iz_, lf_, lr_, h_, track_, steer_rate_, steer_dead_, steer_lag_;
  double c_, mu_f_, b_f_, mu_r_, b_r_, lambda_, locked_, locked_y_, lt_x_, lt_y_, wheel_brake_, gyro_lag_, gyro_scale_, nf_, nr_;
  std::vector<double> steer_cmd_, steer_angle_;
  double speed_, kick_speed_, target_, kick_ahead_, kick_left_, side_in_, sweep_, slide_ref_, gap_min_, gap_max_;

  Phase phase_ = Phase::WAIT;
  rclcpp::Time t_phase_{0, 0, RCL_ROS_TIME}, last_imu_{0, 0, RCL_ROS_TIME}, joints_t_{0, 0, RCL_ROS_TIME};
  std::vector<double> bias_samples_, left_x_, left_y_;
  int scans_ = 0;
  double lidar_x_ = NAN, face_y_ = NAN, gap_from_ = NAN, gap_to_ = NAN, mid_x_ = NAN;
  double bias_ = 0.0, heading_ = 0.0, heading_now_ = 0.0, heading_brake_ = 0.0, r_ = 0.0, u_ = 0.0, joints_p_ = 0.0;
  double x_ = 0.0, y_ = 0.0, steer_ = 0.0, psi_model_ = 0.0;
  State model_{0, 0, 0, 0, 0, 0, 0};
  std::deque<Command> sent_;
  std::deque<Past> history_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr pub_drive_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_arm_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_state_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joints_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GapNode>());
  rclcpp::shutdown();
  return 0;
}

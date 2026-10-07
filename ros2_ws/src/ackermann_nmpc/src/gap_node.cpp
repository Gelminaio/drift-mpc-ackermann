#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Drift parking into a gap between two boxes on the left, from rest. At rest the lidar finds the gap
// (the side of the boxes towards the car, the two rows of points and the free stretch between them). The
// car goes slowly on a line, then kicks: full lock, full throttle, the rear steps out and the car drifts
// round. It brakes (speed 0: the motor driver locks the rear wheels) so that the heading where the brake
// acts plus the turn of the slide at full lock measured on the car (slide_turn) is target, the command
// sent when due between the steps; the car slides sideways into the gap. With slide_turn 0 the model, run
// to the stop, decides the brake, and in the slide the steering is chosen by bisection (slide_bisect).
// With donut_turns, first a donut where the car is: full lock, full throttle from rest, braked so that it ends
// at donut_exit after the turns; the gap is found where it stops. With donut_only the run ends there instead: the
// wheels spin in the donut, so the localization is set back where the donut started, turned by the gyro, and asked
// to update at rest. The line and the heading come from the boxes:
// their side is the nearest line of points, at the angle where
// most points line up on it, so the car may start a little off and askew. The car holds the line where it
// will stop with its side side_in inside the boxes, and kicks where it will stop with as much room at its
// nose as beyond its swept tail, from where the car stopped against the kick (kick_ahead, kick_left).
// The model is the refit of 2026-09-30, carried from the kick with what was sent and the wheel speed. The
// last gyro reading is gyro_lag older than its message: its yaw rate corrects the model's of that time,
// and the heading now is the gyro's plus what the model turned since.

constexpr double TICK = 0.02;        // s, the node's step
constexpr double DT_MODEL = 0.005;   // s, model steps in the prediction
constexpr double HORIZON = 1.6;      // s, longest prediction
constexpr double T_RAMP = 1.0;       // s from rest to speed
constexpr double T_ARM = 0.5;        // s armed at rest before starting
constexpr double T_POSE = 8.0;       // s more at rest waiting for the localization before the donut: a new node on
                                     // the Pi gets the second of its transforms 3-6 s late (DDS discovery)
constexpr double T_KICK = 2.0;       // s kicking without braking: stop
constexpr double T_SLIDE = 1.5;      // s from the brake to the end
constexpr double T_DONUT = 1.8;      // s per turn of the donut: stop
constexpr double T_REST = 0.3;       // s at rest after the donut before the scans
constexpr double T_SETTLE = 3.0;     // s from the donut's brake to the gap: stop
constexpr double T_UPDATES = 0.3;    // s between the localization's updates at rest after the donut
constexpr int UPDATES = 6;
constexpr double IMU_TIMEOUT = 0.1;  // s without the gyro: stop
constexpr double SLIDE_STEER_MIN = -0.15;   // countersteered further the car swings back (hbn runs)
constexpr double LOOKAHEAD = 0.3;    // m, pure pursuit on the line of the approach
constexpr double KICK_ALIGN = 0.087; // rad, the car on the line at the kick (~0.3 cm of stop per deg off): else stop
constexpr double KICK_OFF = 0.03;    // m
constexpr double WHEEL_SLIP = 0.97;  // car over wheels at the launch (0.94-1.00, identification.ipynb)
constexpr double HALF_WIDTH = 0.10;  // m, the car to the outside of the wheels
constexpr double KICK_MIN = 0.35;    // m from the start: the car at speed
constexpr double LINE_MAX = 0.30;    // m, the line from the start sideways
constexpr double FACE_MAX = 0.35;    // rad, the side of the boxes against the car at rest
constexpr double FACE_STEP = 0.0087; // rad, 0.5 deg

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
    // m, middle of the stopped car ahead of / left of where the kick started: 0.304 +- 0.022 / 0.421 +- 0.020 in
    // 20 closed-loop runs at full lock (sideways.ipynb)
    kick_ahead_ = declare_parameter("kick_ahead", 0.30);
    kick_left_ = declare_parameter("kick_left", 0.42);
    side_in_ = declare_parameter("side_in", 0.02);        // m, side of the stopped car inside the boxes
    // m, the body sweeps past where it stops as the car comes round (0.122 +- 0.017 in the same runs): the car
    // stops with as much room ahead of its nose as beyond the swept tail
    sweep_ = declare_parameter("sweep", 0.12);
    // slide steering the brake is decided with: full lock, where the model is right; moving the steering off it
    // in the slide the car turned 5 deg less than the model (sideways.ipynb)
    slide_ref_ = declare_parameter("slide_ref", 0.52);
    // in the slide the steering held at slide_ref, or chosen every step by bisection (slide_bisect): moved in
    // the slide, the car turned 5 deg less than the model (gap_1-8, sideways.ipynb)
    slide_bisect_ = declare_parameter("slide_bisect", false);
    // deg the car turns from when the brake acts to the stop at full lock: 36.4 +- 1.3 in the 11 closed-loop runs
    // (braking at 137-149 deg), whatever its yaw rate, where the model's prediction moved the other way
    // (sideways.ipynb). 0: the model decides
    slide_turn_ = declare_parameter("slide_turn", 36.4) * M_PI / 180;
    // donut before the parking: turns (0: none), braked to end at donut_exit (deg, against the start heading)
    donut_turns_ = declare_parameter("donut_turns", 0);
    donut_exit_ = declare_parameter("donut_exit", 180.0) * M_PI / 180;
    // deg the car turns from when the donut's brake acts to the stop: 33.3 and 28.8 on the car (gymk_5, gymk_7)
    donut_slide_ = declare_parameter("donut_slide", 31.0) * M_PI / 180;
    donut_only_ = declare_parameter("donut_only", false);
    collecting_ = donut_turns_ == 0;
    gap_min_ = declare_parameter("gap_min", 0.40);        // m, a gap outside gap_min .. gap_max: refuse
    gap_max_ = declare_parameter("gap_max", 1.0);
    const double g = 9.81, l = lf_ + lr_;
    nf_ = m_ * g * lr_ / l;
    nr_ = m_ * g * lf_ / l;

    pub_drive_ = create_publisher<ackermann_msgs::msg::AckermannDrive>("/drive", 10);
    pub_arm_ = create_publisher<std_msgs::msg::Bool>("/arm", 10);
    pub_state_ = create_publisher<std_msgs::msg::Float64MultiArray>("gap/state", 10);
    pub_initial_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);
    nomotion_ = create_client<std_srvs::srv::Empty>("/request_nomotion_update");
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data_raw", 50, [this](const sensor_msgs::msg::Imu & m) { on_imu(m); });
    sub_joints_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, [this](const sensor_msgs::msg::JointState & m) { on_joints(m); });
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::LaserScan & m) { on_scan(m); });
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(TICK), [this] { tick(); });
  }

  bool finished() const
  {
    return finished_;
  }

private:
  // at rest: points left of the car from a few scans
  void on_scan(const sensor_msgs::msg::LaserScan & m)
  {
    if (!collecting_ || scans_ >= 5) {
      return;    // five scans at rest: more only slow the search down
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
        if (x > -0.3 && x < 2.5 && y > 0.1 && y < 1.2) {
          left_x_.push_back(x);
          left_y_.push_back(y);
        }
      }
    }
    scans_++;
  }

  // the gap: the side of the boxes is the nearest line of points, at the angle where most points line up on it;
  // along it two rows, the gap between. In the frame of that line: x along it, y to its left
  bool find_gap()
  {
    if (left_y_.size() < 50) {
      return false;
    }
    std::vector<double> ys(left_y_.size());
    size_t best = 0;
    double best_near = 0.0;
    for (double a = -FACE_MAX; a <= FACE_MAX; a += FACE_STEP) {
      for (size_t i = 0; i < ys.size(); i++) {
        ys[i] = -std::sin(a) * left_x_[i] + std::cos(a) * left_y_[i];
      }
      std::vector<double> sorted = ys;
      std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 10, sorted.end());
      const double near = sorted[sorted.size() / 10];
      const size_t on = std::count_if(ys.begin(), ys.end(), [near](double y) { return std::abs(y - near) < 0.015; });
      if (on > best) {
        best = on;
        best_near = near;
        face_angle_ = a;
      }
    }
    // finer than the step: a straight line through the points on it
    double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (size_t i = 0; i < ys.size(); i++) {
      if (std::abs(-std::sin(face_angle_) * left_x_[i] + std::cos(face_angle_) * left_y_[i] - best_near) < 0.015) {
        n += 1;
        sx += left_x_[i];
        sy += left_y_[i];
        sxx += left_x_[i] * left_x_[i];
        sxy += left_x_[i] * left_y_[i];
      }
    }
    face_angle_ = std::atan((n * sxy - sx * sy) / (n * sxx - sx * sx));
    std::vector<double> fx(ys.size()), fy(ys.size());
    for (size_t i = 0; i < ys.size(); i++) {
      fx[i] = std::cos(face_angle_) * left_x_[i] + std::sin(face_angle_) * left_y_[i];
      fy[i] = -std::sin(face_angle_) * left_x_[i] + std::cos(face_angle_) * left_y_[i];
    }
    std::vector<double> sorted = fy;
    std::sort(sorted.begin(), sorted.end());
    const double near = sorted[sorted.size() / 10];
    std::vector<double> side_y, side_x;
    for (size_t i = 0; i < fx.size(); i++) {
      if (std::abs(fy[i] - near) < 0.03) {
        side_y.push_back(fy[i]);
        side_x.push_back(fx[i]);
      }
    }
    std::sort(side_y.begin(), side_y.end());
    face_y_ = side_y[side_y.size() / 2];
    std::sort(side_x.begin(), side_x.end());
    // rows of points and the free stretches between them: the gap is the nearest one ahead at least gap_min long
    // (the widest was a wall further along the faces when the boxes stand anywhere: sim_valet_p1)
    for (size_t i = 1; i < side_x.size(); i++) {
      if (side_x[i - 1] > 0.0 && side_x[i] - side_x[i - 1] > gap_min_) {
        gap_from_ = side_x[i - 1];
        gap_to_ = side_x[i];
        return true;
      }
    }
    return false;
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
  // the gyro's error of when the last reading was taken, once per reading, and the heading now from the gyro's
  void carry_model(const rclcpp::Time & now)
  {
    for (int i = 0; i < std::lround(TICK / 0.001); i++) {
      const Command & c = acting(now - rclcpp::Duration::from_seconds(TICK - 0.001 * i));
      step(model_, u_, c.speed < 0.02, interp(c.steer, steer_cmd_, steer_angle_), 0.001);
      psi_model_ += 0.001 * model_.r;
    }
    history_.push_back({now, model_.r, psi_model_});
    while ((now - history_.front().t).seconds() > gyro_lag_ + 0.1) {
      history_.pop_front();
    }
    if (last_imu_ != corrected_imu_) {
      corrected_imu_ = last_imu_;
      // the model when the reading was taken, between the two steps around it
      const rclcpp::Time then = last_imu_ - rclcpp::Duration::from_seconds(gyro_lag_);
      size_t j = 0;
      while (j + 1 < history_.size() && history_[j + 1].t <= then) {
        j++;
      }
      double r_then = history_[j].r;
      psi_then_ = history_[j].psi;
      if (j + 1 < history_.size() && history_[j].t < then) {
        const double a = (then - history_[j].t).seconds() / (history_[j + 1].t - history_[j].t).seconds();
        r_then += a * (history_[j + 1].r - history_[j].r);
        psi_then_ += a * (history_[j + 1].psi - history_[j].psi);
      }
      // the model from then on, kept history included
      const double dr = r_ - r_then;
      for (Past & p : history_) {
        if (p.t > then) {
          p.psi += dr * (p.t - then).seconds();
          p.r += dr;
        }
      }
      model_.r += dr;
      psi_model_ += dr * (now - then).seconds();
    }
    heading_now_ = heading_ + psi_model_ - psi_then_;
  }

  void brake()
  {
    if (brake_timer_) {
      brake_timer_->cancel();
    }
    if (phase_ != Phase::KICK && phase_ != Phase::DONUT) {
      return;    // stopped meanwhile
    }
    const rclcpp::Time now = get_clock()->now();
    heading_brake_ = heading_now_;
    const bool donut = phase_ == Phase::DONUT;
    next(donut ? Phase::SETTLE : Phase::SLIDE, now);
    steer_ = slide_bisect_ && !donut ? slide_steering(now) : slide_ref_;
    send(0.0, steer_, now);
  }

  // after the donut, at rest: the localization set back where the donut started, turned by the gyro, then
  // UPDATES updates without motion
  void relocalize(const rclcpp::Time & now)
  {
    if (updates_ == 0) {
      RCLCPP_INFO(get_logger(), "donut braked at %.1f deg, stopped at %.1f deg (exit %.1f); localization set back to "
        "(%.3f, %.3f, %.1f deg)", heading_brake_ * 180 / M_PI, heading_ * 180 / M_PI,
        (donut_turns_ * 2 * M_PI + donut_exit_) * 180 / M_PI, start_x_, start_y_, (start_yaw_ + heading_) * 180 / M_PI);
    }
    if (updates_ < 3) {
      // three times: the localization takes it best effort
      geometry_msgs::msg::PoseWithCovarianceStamped p;
      p.header.stamp = now;
      p.header.frame_id = "map";
      p.pose.pose.position.x = start_x_;
      p.pose.pose.position.y = start_y_;
      p.pose.pose.orientation.z = std::sin((start_yaw_ + heading_) / 2);
      p.pose.pose.orientation.w = std::cos((start_yaw_ + heading_) / 2);
      p.pose.covariance[0] = p.pose.covariance[7] = 0.25 * 0.25;    // the donut moves the car up to ~0.25 m
      p.pose.covariance[35] = 0.05 * 0.05;
      pub_initial_->publish(p);
      last_update_ = now;
      updates_++;
    } else if ((now - last_update_).seconds() > T_UPDATES) {
      if (updates_ < 3 + UPDATES) {
        nomotion_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
      } else {
        finished_ = true;
        next(Phase::DONE, now);
      }
      last_update_ = now;
      updates_++;
    }
  }

  // where the middle of the car stops: the same room at the nose and beyond the swept tail; its side side_in
  // inside the boxes. The line the car holds to get there
  bool plan_gap()
  {
    if (!find_gap()) {
      RCLCPP_ERROR(get_logger(), "no gap on the left: stop");
      return false;
    }
    mid_x_ = (gap_from_ + gap_to_ - sweep_) / 2;
    y_line_ = face_y_ + HALF_WIDTH + side_in_ - kick_left_;
    RCLCPP_INFO(get_logger(), "gap %.3f .. %.3f m ahead, boxes %.3f m left at %.1f deg: kick at %.3f m, the line %.3f m left",
      gap_from_, gap_to_, face_y_, face_angle_ * 180 / M_PI, mid_x_ - kick_ahead_, y_line_);
    if (gap_to_ - gap_from_ < gap_min_ || gap_to_ - gap_from_ > gap_max_ || mid_x_ - kick_ahead_ < KICK_MIN ||
      std::abs(y_line_) > LINE_MAX)
    {
      RCLCPP_ERROR(get_logger(), "gap %.3f m (%.2f .. %.2f), kick at %.3f m (min %.2f), the line %.3f m left (max %.2f): stop",
        gap_to_ - gap_from_, gap_min_, gap_max_, mid_x_ - kick_ahead_, KICK_MIN, y_line_, LINE_MAX);
      return false;
    }
    return true;
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
        if (bias_samples_.size() >= 50 && (donut_turns_ > 0 || scans_ >= 5) && pub_drive_->get_subscription_count() > 0 &&
          pub_arm_->get_subscription_count() > 0)
        {
          if (donut_turns_ == 0 && !plan_gap()) {
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
          if (donut_turns_ > 0) {
            // the newest localization correction on the newest odometry, as the NMPC: map -> base_footprint at
            // one time fails while the corrections are dated ahead
            try {
              const auto c = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero).transform;
              const auto o = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero).transform;
              const double yc = 2 * std::atan2(c.rotation.z, c.rotation.w);
              start_x_ = c.translation.x + std::cos(yc) * o.translation.x - std::sin(yc) * o.translation.y;
              start_y_ = c.translation.y + std::sin(yc) * o.translation.x + std::cos(yc) * o.translation.y;
              start_yaw_ = yc + 2 * std::atan2(o.rotation.z, o.rotation.w);
              if (t > T_ARM + 0.1) {
                RCLCPP_INFO(get_logger(), "localization after %.1f s", t);
              }
            } catch (const tf2::TransformException & e) {
              if (donut_only_ && t < T_ARM + T_POSE) {
                break;
              }
              if (donut_only_) {
                RCLCPP_ERROR(get_logger(), "no localization before the donut (%s): stop", e.what());
                next(Phase::DONE, now);
                break;
              }
            }
            heading_ = 0.0;
            model_ = {0.0, 0.0, r_, 0.0, 0.0, 0.0, 0.0};
            psi_model_ = 0.0;
            history_.clear();
            next(Phase::DONUT, now);
            send(kick_speed_, steer_cmd_.back(), now);
          } else {
            heading_ = -face_angle_;    // from here on the frame of the boxes
            next(Phase::STRAIGHT, now);
          }
        }
        break;
      case Phase::DONUT: {
        carry_model(now);
        if (brake_timer_ && !brake_timer_->is_canceled()) {
          break;    // the brake goes before the next step
        }
        const double due = (donut_turns_ * 2 * M_PI + donut_exit_ - donut_slide_ - heading_now_) / std::max(model_.r, 0.1) - steer_dead_;
        if (t > T_DONUT * donut_turns_ + T_KICK) {
          RCLCPP_ERROR(get_logger(), "heading %.0f deg after %.1f s of donut: stop", heading_now_ * 180 / M_PI, t);
          next(Phase::DONE, now);
        } else if (due <= 0) {
          brake();
        } else if (due < TICK) {
          brake_timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(due), [this] { brake(); });
        } else {
          send(kick_speed_, steer_cmd_.back(), now);
        }
        break;
      }
      case Phase::SETTLE:
        // braked to a stop, then the scans at rest
        send(0.0, slide_ref_, now);
        if (std::abs(u_) > 0.02 || std::abs(r_) > 0.05) {
          rest_ = now;
        } else if (donut_only_ && (now - rest_).seconds() > T_REST) {
          relocalize(now);
          break;
        } else if (!collecting_ && (now - rest_).seconds() > T_REST) {
          RCLCPP_INFO(get_logger(), "donut braked at %.1f deg, stopped at %.1f deg (exit %.1f)", heading_brake_ * 180 / M_PI,
            heading_ * 180 / M_PI, (donut_turns_ * 2 * M_PI + donut_exit_) * 180 / M_PI);
          left_x_.clear();
          left_y_.clear();
          scans_ = 0;
          collecting_ = true;
        }
        if (collecting_ && scans_ >= 5) {
          collecting_ = false;
          if (!plan_gap()) {
            next(Phase::DONE, now);
            break;
          }
          heading_ = -face_angle_;
          x_ = y_ = 0.0;
          next(Phase::STRAIGHT, now);
        } else if (t > T_SETTLE) {
          RCLCPP_ERROR(get_logger(), "%.1f s after the donut's brake, no gap yet: stop", t);
          next(Phase::DONE, now);
        }
        break;
      case Phase::STRAIGHT: {
        // pure pursuit on the line: position from the wheels and the gyro heading
        x_ += WHEEL_SLIP * u_ * std::cos(heading_) * TICK;
        y_ += WHEEL_SLIP * u_ * std::sin(heading_) * TICK;
        heading_now_ = heading_;
        const double off = y_ - y_line_;
        const double ty = -std::sin(heading_) * LOOKAHEAD - std::cos(heading_) * off;
        const double d = std::atan(2 * (lf_ + lr_) * ty / (LOOKAHEAD * LOOKAHEAD + off * off));
        const double steer = interp(d, steer_angle_, steer_cmd_);
        // where the car would stop kicking now, along the line: kicked on the line, the stop follows from the kick
        // point (turned with the car's heading it moved the wrong way: gymk_9, sim_gym3)
        if (x_ + kick_ahead_ >= mid_x_) {
          if (t < T_RAMP) {
            RCLCPP_ERROR(get_logger(), "kick at %.3f m, before full speed: stop", x_);
            next(Phase::DONE, now);
            break;
          }
          if (std::abs(heading_) > KICK_ALIGN || std::abs(off) > KICK_OFF) {
            RCLCPP_ERROR(get_logger(), "not on the line at the kick (%.3f m off, %.1f deg): stop", off, heading_ * 180 / M_PI);
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
      case Phase::KICK: {
        carry_model(now);
        if (brake_timer_ && !brake_timer_->is_canceled()) {
          break;    // the brake goes before the next step
        }
        // from now until the brake is sent: it acts steer_dead later, the car turning at the rate of now
        const double due = (target_ - slide_turn_ - heading_now_) / std::max(model_.r, 0.1) - steer_dead_;
        if (t > T_KICK) {
          RCLCPP_ERROR(get_logger(), "heading %.0f deg after %.1f s of kick: stop", heading_now_ * 180 / M_PI, t);
          next(Phase::DONE, now);
        } else if (slide_turn_ > 0 && due <= 0) {
          brake();
        } else if (slide_turn_ > 0 && due < TICK) {
          brake_timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(due), [this] { brake(); });
        } else if (slide_turn_ == 0 && final_heading(model_, heading_now_, u_, slide_ref_, now) >= target_) {
          brake();
        } else {
          send(kick_speed_, steer_cmd_.back(), now);
        }
        break;
      }
      case Phase::SLIDE:
        carry_model(now);
        steer_ = slide_bisect_ ? slide_steering(now) : slide_ref_;
        send(0.0, steer_, now);
        if (t > T_SLIDE) {
          RCLCPP_INFO(get_logger(), "braked at %.1f deg, stopped at %.1f deg (target %.1f)",
            heading_brake_ * 180 / M_PI, heading_now_ * 180 / M_PI, target_ * 180 / M_PI);
          finished_ = true;
          next(Phase::DONE, now);
        }
        break;
      case Phase::DONE:
        send(0.0, 0.0, now);
        if (!(donut_only_ && finished_)) {
          arm(false);    // after the donut alone the next controller drives on
        }
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

  enum class Phase { WAIT, ARM, STRAIGHT, KICK, SLIDE, DONE, DONUT, SETTLE };

  void next(Phase p, const rclcpp::Time & now)
  {
    phase_ = p;
    t_phase_ = now;
  }

  double wheel_radius_, m_, iz_, lf_, lr_, h_, track_, steer_rate_, steer_dead_, steer_lag_;
  double c_, mu_f_, b_f_, mu_r_, b_r_, lambda_, locked_, locked_y_, lt_x_, lt_y_, wheel_brake_, gyro_lag_, gyro_scale_, nf_, nr_;
  std::vector<double> steer_cmd_, steer_angle_;
  double speed_, kick_speed_, target_, kick_ahead_, kick_left_, side_in_, sweep_, slide_ref_, gap_min_, gap_max_;
  bool slide_bisect_, collecting_, donut_only_, finished_ = false;
  double slide_turn_, donut_exit_, donut_slide_;
  int64_t donut_turns_;

  Phase phase_ = Phase::WAIT;
  rclcpp::Time t_phase_{0, 0, RCL_ROS_TIME}, last_imu_{0, 0, RCL_ROS_TIME}, joints_t_{0, 0, RCL_ROS_TIME};
  std::vector<double> bias_samples_, left_x_, left_y_;
  int scans_ = 0;
  double lidar_x_ = NAN, face_y_ = NAN, face_angle_ = 0.0, gap_from_ = NAN, gap_to_ = NAN, mid_x_ = NAN, y_line_ = 0.0;
  double bias_ = 0.0, heading_ = 0.0, heading_now_ = 0.0, heading_brake_ = 0.0, r_ = 0.0, u_ = 0.0, joints_p_ = 0.0;
  double x_ = 0.0, y_ = 0.0, steer_ = 0.0, psi_model_ = 0.0, psi_then_ = 0.0;
  rclcpp::Time corrected_imu_{0, 0, RCL_ROS_TIME}, rest_{0, 0, RCL_ROS_TIME}, last_update_{0, 0, RCL_ROS_TIME};
  double start_x_ = 0.0, start_y_ = 0.0, start_yaw_ = NAN;
  int updates_ = 0;
  State model_{0, 0, 0, 0, 0, 0, 0};
  std::deque<Command> sent_;
  std::deque<Past> history_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr pub_drive_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_arm_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_state_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initial_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr nomotion_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joints_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::TimerBase::SharedPtr timer_, brake_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return node->finished() ? 0 : 1;    // 1: stopped before the end
}

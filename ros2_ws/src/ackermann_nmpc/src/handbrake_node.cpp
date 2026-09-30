#include <algorithm>
#include <cmath>
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

// Handbrake turn to a heading, from rest. Straight up to speed, full lock at speed, then at brake_at
// (heading from the gyro) speed 0: the motor driver brakes the rear wheels and the car slides. In the
// slide the steering decides how far it still turns (21-29 deg with the wheels straight, 41-57 at full
// lock, from 136 deg on the tiles): every step the Phase 5 model, rear friction x locked_friction, is run
// to the stop for a steering held constant, and bisection finds the one that stops at target.
// The model state in the slide: yaw rate from the gyro, steering from the commands through the lag,
// vx and vy carried by the model from brake_vx, brake_vy (the wheels are locked).
// With box, the straight ends at a box on the left instead of after straight s: at rest its side is the
// nearest line of points left of the path, and the turn starts when the far corner of that side is
// turn_at m from the rear axle, from the scans (the wheels over-read, 4-6% by the turn). Not
// before T_STEADY: turning while the car still speeds up, the turn is shorter and less repeatable.

constexpr double DT_MODEL = 0.005;   // s, model steps in the prediction
constexpr double HORIZON = 1.5;      // s, longest prediction
constexpr double T_RAMP = 1.0;       // s from rest to speed
constexpr double T_ARM = 0.5;        // s armed at rest before starting
constexpr double T_SLIDE = 1.5;      // s from the brake to the end
constexpr double T_TURN = 3.0;       // s at full lock without reaching brake_at: stop
constexpr double IMU_TIMEOUT = 0.1;  // s without the gyro: stop
// steering command in the slide, from full lock down to this: countersteered further the car swings
// back 9-12 deg at the end of the slide (hb_cs, hbn runs), which the model does not do
constexpr double SLIDE_STEER_MIN = -0.15;
constexpr double FACE_BAND = 0.04;   // m around the side of the box
constexpr double GAP = 0.05;         // m between points of the side at rest
constexpr double BOX_TIMEOUT = 0.5;  // s without the box in the straight: stop
// s from the start: full speed. Turning at 1.44-1.52 s the car went 0.22-0.35 m on from the turn to
// the stop, at 1.62-1.86 s 0.55-0.62 (parking.ipynb)
constexpr double T_STEADY = 1.6;
constexpr double LATE = 0.05;        // m past turn_at at T_STEADY: the box is too close, stop

// as np.interp: linear, held at the ends
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
  double vx, vy, r, d;    // at the CG, steering angle
};

class HandbrakeNode : public rclcpp::Node
{
public:
  HandbrakeNode()
  : Node("handbrake")
  {
    // vehicle_params.yaml
    wheel_radius_ = declare_parameter<double>("wheel_radius");
    m_ = declare_parameter<double>("mass");
    iz_ = declare_parameter<double>("iz");
    lf_ = declare_parameter<double>("lf");
    lr_ = declare_parameter<double>("lr");
    track_ = declare_parameter<double>("track");
    steer_cmd_ = declare_parameter<std::vector<double>>("steer_cmd");
    steer_angle_ = declare_parameter<std::vector<double>>("steer_angle");
    steer_lag_ = declare_parameter<double>("steer_lag");
    brake_lag_ = declare_parameter<double>("brake_lag");
    mu_f_ = declare_parameter<double>("tire_mu_f");
    b_f_ = declare_parameter<double>("tire_b_f");
    mu_r_ = declare_parameter<double>("tire_mu_r");
    b_r_ = declare_parameter<double>("tire_b_r");
    c_ = declare_parameter<double>("tire_c");
    const double g = 9.81, l = lf_ + lr_;
    nf_ = m_ * g * lr_ / l;
    nr_ = m_ * g * lf_ / l;
    // the run
    speed_ = declare_parameter("speed", 1.5);        // m/s command, above what the motor reaches
    straight_ = declare_parameter("straight", 0.5);  // s at speed before the turn
    brake_at_ = declare_parameter("brake_at", 145.0) * M_PI / 180;
    target_ = declare_parameter("target", 180.0) * M_PI / 180;
    // a locked rear slides with more friction than a spinning one (yaw rate of 16 slides, 2026-09-29)
    locked_friction_ = declare_parameter("locked_friction", 1.5);
    // rear axle velocity at the brake, video of the hb_park and hb_sw runs
    brake_vx_ = declare_parameter("brake_vx", 0.82);
    brake_vy_ = declare_parameter("brake_vy", -0.10);
    box_ = declare_parameter("box", false);
    // m, far corner of the box ahead of the rear axle at the turn: ~0.58 m from the turn to the stop,
    // the nose ~0.36 - turn_at m past the corner
    turn_at_ = declare_parameter("turn_at", 0.26);

    pub_drive_ = create_publisher<ackermann_msgs::msg::AckermannDrive>("/drive", 10);
    pub_arm_ = create_publisher<std_msgs::msg::Bool>("/arm", 10);
    pub_state_ = create_publisher<std_msgs::msg::Float64MultiArray>("handbrake/state", 10);
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data_raw", 50, [this](const sensor_msgs::msg::Imu & m) { on_imu(m); });
    sub_joints_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, [this](const sensor_msgs::msg::JointState & m) {
        if (m.velocity.size() >= 2) {
          u_ = wheel_radius_ * (m.velocity[0] + m.velocity[1]) / 2;
        }
      });
    if (box_) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
      sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::LaserScan & m) { on_scan(m); });
    }
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(20), [this] { tick(); });
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan & m)
  {
    if (std::isnan(lidar_x_)) {
      try {
        lidar_x_ = tf_buffer_->lookupTransform("base_footprint", m.header.frame_id, tf2::TimePointZero).transform.translation.x;
      } catch (const tf2::TransformException &) {
        return;    // the transform comes a moment after the start
      }
    }
    // points from the rear axle along the start heading (the launch veers 0-5 deg: seen from the car
    // the side of the box would tilt out of FACE_BAND), and their lidar angles
    const double c = std::cos(heading_), s = std::sin(heading_);
    std::vector<double> xs, ys, as;
    for (size_t i = 0; i < m.ranges.size(); i++) {
      if (std::isfinite(m.ranges[i])) {
        const double a = m.angle_min + i * m.angle_increment;
        const double x = m.ranges[i] * std::cos(a) + lidar_x_, y = m.ranges[i] * std::sin(a);
        xs.push_back(x * c - y * s);
        ys.push_back(x * s + y * c);
        as.push_back(a);
      }
    }
    if (phase_ == Phase::WAIT) {
      // at rest: the side of the box, the nearest points left of the path ahead
      std::vector<double> left;
      for (size_t i = 0; i < xs.size(); i++) {
        if (xs[i] > 0.2 && xs[i] < 2.0 && ys[i] > 0.3 && ys[i] < 1.6) {
          left.push_back(ys[i]);
        }
      }
      if (left.size() < 10) {
        return;
      }
      std::sort(left.begin(), left.end());
      side_y_ = left[left.size() / 10];
    }
    if (std::isnan(side_y_)) {
      return;
    }
    // the far corner: the last point of the side. At rest the side is the unbroken row of points from
    // the nearest one (a wall further on can cross the same line); moving, the points near where the
    // corner should be by now
    std::vector<size_t> side;
    for (size_t i = 0; i < xs.size(); i++) {
      if (std::abs(ys[i] - side_y_) < FACE_BAND && xs[i] > -1.0) {
        side.push_back(i);
      }
    }
    std::sort(side.begin(), side.end(), [&](size_t a, size_t b) { return xs[a] < xs[b]; });
    int far = -1;
    if (phase_ == Phase::WAIT) {
      for (const size_t i : side) {
        if (xs[i] < 0.2) {
          continue;
        }
        if (far >= 0 && xs[i] - xs[far] > GAP) {
          break;
        }
        far = i;
      }
    } else {
      const double expected = far_x_ - u_ * (rclcpp::Time(m.header.stamp) - far_t_).seconds();
      std::vector<double> on_side;
      for (const size_t i : side) {
        if (xs[i] > expected - 0.5 && xs[i] < expected + 0.15) {
          far = i;
          on_side.push_back(ys[i]);
        }
      }
      if (far < 0) {
        return;
      }
      // the car drifts sideways a few cm: the side where it is now
      std::sort(on_side.begin(), on_side.end());
      side_y_ = on_side[on_side.size() / 2];
    }
    if (far < 0) {
      return;
    }
    // when the lidar saw it: rplidar_ros stamps the start of the turn, which begins behind and goes
    // clockwise (a = pi - lidar angle); the sim has scan_time 0
    far_x_ = xs[far];
    far_t_ = rclcpp::Time(m.header.stamp) + rclcpp::Duration::from_seconds(m.scan_time * (M_PI - as[far]) / (2 * M_PI));
    box_scans_++;
  }

  void on_imu(const sensor_msgs::msg::Imu & m)
  {
    const rclcpp::Time now = get_clock()->now();
    const double gz = m.angular_velocity.z;
    if (phase_ == Phase::WAIT) {    // at rest: the bias
      bias_samples_.push_back(gz);
    } else if (last_imu_.nanoseconds() > 0) {
      heading_ += (now - last_imu_).seconds() * (gz - bias_);
    }
    r_ = gz - bias_;
    last_imu_ = now;
  }

  // rear wheel at surface speed u: force against its sliding direction, friction x k
  void rear_wheel(double vx, double vy, double u, double k, double & fx, double & fy) const
  {
    const double sx = (vx - u) / std::max(u, 0.1), sy = vy / std::max(u, 0.1);
    const double s = std::hypot(sx, sy) + 1e-9;
    const double f = k * mu_r_ * nr_ / 2 * std::sin(c_ * std::atan(b_r_ / k * s));
    fx = -f * sx / s;
    fy = -f * sy / s;
  }

  // the model of vehicle_model.py with the rear friction x locked_friction
  State deriv(const State & x, double u, double d_cmd) const
  {
    const double vx = std::max(x.vx, 0.05);
    const double fyf = mu_f_ * nf_ * std::sin(c_ * std::atan(b_f_ * (x.d - std::atan((x.vy + lf_ * x.r) / vx))));
    double fxl, fyl, fxr, fyr;
    rear_wheel(vx - track_ / 2 * x.r, x.vy - lr_ * x.r, u, locked_friction_, fxl, fyl);
    rear_wheel(vx + track_ / 2 * x.r, x.vy - lr_ * x.r, u, locked_friction_, fxr, fyr);
    const double fx = fxl + fxr - fyf * std::sin(x.d);
    const double fy = fyf * std::cos(x.d) + fyl + fyr;
    const double mz = lf_ * fyf * std::cos(x.d) - lr_ * (fyl + fyr) + track_ / 2 * (fxr - fxl);
    return {fx / m_ + x.vy * x.r, fy / m_ - x.vx * x.r, mz / iz_, (d_cmd - x.d) / steer_lag_};
  }

  static void step(State & x, const State & dx, double h)
  {
    x.vx += h * dx.vx;
    x.vy += h * dx.vy;
    x.r += h * dx.r;
    x.d += h * dx.d;
  }

  static bool stopped(const State & x, double u)
  {
    return x.vx < 0.06 && std::abs(x.vy) < 0.03 && u < 0.02;
  }

  // heading where the car stops with the steering held, the wheels braking as the motor driver does
  double final_heading(State x, double psi, double u, double steer) const
  {
    const double d_cmd = interp(steer, steer_cmd_, steer_angle_);
    for (int i = 0; i < HORIZON / DT_MODEL && !stopped(x, u); i++) {
      step(x, deriv(x, u, d_cmd), DT_MODEL);
      psi += DT_MODEL * x.r;
      u *= std::exp(-DT_MODEL / brake_lag_);
    }
    return psi;
  }

  double slide_steering() const
  {
    const double full = steer_cmd_.back(), counter = SLIDE_STEER_MIN;
    if (final_heading(model_, heading_, u_, counter) >= target_) {
      return counter;
    }
    if (final_heading(model_, heading_, u_, full) <= target_) {
      return full;
    }
    double lo = counter, hi = full;
    for (int i = 0; i < 8; i++) {
      const double mid = (lo + hi) / 2;
      (final_heading(model_, heading_, u_, mid) < target_ ? lo : hi) = mid;
    }
    return (lo + hi) / 2;
  }

  // arming zeroes the setpoint (firmware, sim_car): only at rest, not with every command
  void arm(bool on)
  {
    std_msgs::msg::Bool a;
    a.data = on;
    pub_arm_->publish(a);
  }

  void send(double speed, double steer)
  {
    ackermann_msgs::msg::AckermannDrive d;
    d.speed = speed;
    d.steering_angle = steer;
    pub_drive_->publish(d);
  }

  void tick()
  {
    const rclcpp::Time now = get_clock()->now();
    const double t = (now - t_phase_).seconds();
    if (phase_ != Phase::WAIT && phase_ != Phase::DONE && (now - last_imu_).seconds() > IMU_TIMEOUT) {
      RCLCPP_ERROR(get_logger(), "no gyro for %.2f s: stop", (now - last_imu_).seconds());
      next(Phase::DONE, now);
    }
    switch (phase_) {
      case Phase::WAIT:
        if (bias_samples_.size() >= 50 && pub_drive_->get_subscription_count() > 0 &&
          pub_arm_->get_subscription_count() > 0 && (!box_ || box_scans_ >= 3))
        {
          if (box_) {
            RCLCPP_INFO(get_logger(), "box side %.3f m left, far corner %.3f m ahead", side_y_, far_x_);
          }
          std::sort(bias_samples_.begin(), bias_samples_.end());
          bias_ = bias_samples_[bias_samples_.size() / 2];
          next(Phase::ARM, now);
        }
        return;
      case Phase::ARM:
        arm(true);
        send(0.0, 0.0);
        if (t > T_ARM) {
          heading_ = 0.0;
          next(Phase::STRAIGHT, now);
        }
        break;
      case Phase::STRAIGHT:
        send(speed_ * std::min(1.0, t / T_RAMP), 0.0);
        if (box_) {
          // the corner carried from the last scan with the wheels
          const double since = (now - far_t_).seconds();
          far_now_ = far_x_ - u_ * since;
          if (since > BOX_TIMEOUT) {
            RCLCPP_ERROR(get_logger(), "box not seen for %.2f s: stop", since);
            next(Phase::DONE, now);
          } else if (t > T_STEADY && far_now_ < turn_at_ - LATE) {
            RCLCPP_ERROR(get_logger(), "box too close: the corner %.3f m ahead at full speed, stop", far_now_);
            next(Phase::DONE, now);
          } else if (t > T_STEADY && far_now_ <= turn_at_) {
            RCLCPP_INFO(get_logger(), "turn with the far corner %.3f m ahead, %.2f s after the start", far_now_, t);
            next(Phase::TURN, now);
          }
        } else if (t > T_RAMP + straight_) {
          next(Phase::TURN, now);
        }
        break;
      case Phase::TURN:
        send(speed_, steer_cmd_.back());
        if (t > T_TURN) {
          RCLCPP_ERROR(get_logger(), "heading %.0f deg after %.1f s at full lock: stop", heading_ * 180 / M_PI, t);
          next(Phase::DONE, now);
        } else if (heading_ >= brake_at_) {
          const double vy = brake_vy_ + lr_ * r_;
          model_ = {brake_vx_, vy, r_, interp(steer_cmd_.back(), steer_cmd_, steer_angle_)};
          heading_brake_ = heading_;
          next(Phase::SLIDE, now);
        }
        break;
      case Phase::SLIDE: {
        // the model carried over the last step with what was sent, the yaw rate from the gyro
        const double d_cmd = interp(steer_, steer_cmd_, steer_angle_);
        for (int i = 0; i < 20; i++) {
          step(model_, deriv(model_, u_, d_cmd), 0.001);
        }
        model_.vx = std::max(model_.vx, 0.0);
        model_.r = r_;
        steer_ = slide_steering();
        send(0.0, steer_);
        if (t > T_SLIDE) {
          RCLCPP_INFO(get_logger(), "braked at %.1f deg, stopped at %.1f deg (target %.1f)",
            heading_brake_ * 180 / M_PI, heading_ * 180 / M_PI, target_ * 180 / M_PI);
          next(Phase::DONE, now);
        }
        break;
      }
      case Phase::DONE:
        send(0.0, 0.0);
        arm(false);
        if (t > 0.5) {
          rclcpp::shutdown();
        }
        break;
    }
    std_msgs::msg::Float64MultiArray s;
    s.data = {static_cast<double>(phase_), heading_, r_, u_, steer_, model_.vx, model_.vy, far_now_};
    pub_state_->publish(s);
  }

  enum class Phase { WAIT, ARM, STRAIGHT, TURN, SLIDE, DONE };

  void next(Phase p, const rclcpp::Time & now)
  {
    phase_ = p;
    t_phase_ = now;
  }

  double wheel_radius_, m_, iz_, lf_, lr_, track_, steer_lag_, brake_lag_;
  double mu_f_, b_f_, mu_r_, b_r_, c_, nf_, nr_;
  std::vector<double> steer_cmd_, steer_angle_;
  double speed_, straight_, brake_at_, target_, locked_friction_, brake_vx_, brake_vy_, turn_at_;
  bool box_;

  Phase phase_ = Phase::WAIT;
  rclcpp::Time t_phase_{0, 0, RCL_ROS_TIME}, last_imu_{0, 0, RCL_ROS_TIME};
  std::vector<double> bias_samples_;
  double bias_ = 0.0, heading_ = 0.0, heading_brake_ = 0.0, r_ = 0.0, u_ = 0.0, steer_ = 0.52;
  State model_{0, 0, 0, 0};
  double lidar_x_ = NAN, side_y_ = NAN, far_x_ = NAN, far_now_ = NAN;
  rclcpp::Time far_t_{0, 0, RCL_ROS_TIME};
  int box_scans_ = 0;
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
  rclcpp::spin(std::make_shared<HandbrakeNode>());
  rclcpp::shutdown();
  return 0;
}

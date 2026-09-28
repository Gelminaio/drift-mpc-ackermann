#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <string>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_drift.h"

// The drift of ocp.drift_ocp around a cone, from rest: straight ahead up to launch_speed, the
// drift NMPC from the closest point to the cone for hold seconds, then zero commands. The run-up
// uses the localization. In the drift the cone is the reference: seen from the rear axle at
// distance rho and angle atan2(x, y), it gives the offset n = radius - rho and the heading error
// directly. Between scans the model carries them. vx, vy and the steering angle come from the
// previous solve (the wheels no longer tell the ground speed), the yaw rate from the gyro.

constexpr double CONE_WINDOW = 0.15;   // m around where the cone should be
constexpr double CONE_RADIUS = 0.02;   // m at the height of the lidar
constexpr double CONE_TIMEOUT = 0.5;   // s without the cone: stop

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

class DriftNode : public rclcpp::Node
{
public:
  DriftNode()
  : Node("drift")
  {
    // vehicle_params.yaml
    wheel_radius_ = declare_parameter<double>("wheel_radius");
    steer_cmd_ = declare_parameter<std::vector<double>>("steer_cmd");
    steer_angle_ = declare_parameter<std::vector<double>>("steer_angle");
    // the cone, in the map, and the run
    cone_x_ = declare_parameter<double>("cone_x");
    cone_y_ = declare_parameter<double>("cone_y");
    launch_speed_ = declare_parameter("launch_speed", 0.8);  // m/s at the wheels before the drift
    hold_ = declare_parameter("hold", 10.0);                 // s of drift
    mu_scale_ = declare_parameter("mu_scale", 1.0);          // floor friction the NMPC assumes
    max_error_ = declare_parameter("max_error", 0.5);        // m off the circle: stop
    max_pose_age_ = declare_parameter("max_pose_age", 0.2);  // s since the last pose: stop
    // the drift equilibrium and its circle, from drift.py (drift.launch.py)
    radius_ = declare_parameter<double>("drift_radius");
    const double heading = declare_parameter<double>("drift_heading");
    const double vx = declare_parameter<double>("drift_vx");
    const double vy = declare_parameter<double>("drift_vy");
    const double r = declare_parameter<double>("drift_r");
    const double d = declare_parameter<double>("drift_steering");
    const double u = declare_parameter<double>("drift_wheel_speed");

    capsule_ = drift_acados_create_capsule();
    if (drift_acados_create(capsule_) != 0) {
      throw std::runtime_error("acados solver creation failed");
    }
    config_ = drift_acados_get_nlp_config(capsule_);
    dims_ = drift_acados_get_nlp_dims(capsule_);
    in_ = drift_acados_get_nlp_in(capsule_);
    out_ = drift_acados_get_nlp_out(capsule_);
    solver_ = drift_acados_get_nlp_solver(capsule_);
    double yref[9] = {0, heading, vx, vy, r, d, u, 0, 0};
    double p[DRIFT_NP] = {mu_scale_, 1 / radius_};
    for (int j = 0; j <= DRIFT_N; j++) {
      ocp_nlp_cost_model_set(config_, dims_, in_, j, "yref", yref);
      drift_acados_update_params(capsule_, j, p, DRIFT_NP);
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    pub_drive_ = create_publisher<ackermann_msgs::msg::AckermannDrive>("/drive", 10);
    pub_solve_time_ = create_publisher<std_msgs::msg::Float32>("nmpc/solve_time", 10);
    pub_state_ = create_publisher<std_msgs::msg::Float64MultiArray>("drift/state", 10);
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10, [this](const nav_msgs::msg::Odometry & m) {
        vx_ekf_ = m.twist.twist.linear.x;
        r_ = m.twist.twist.angular.z;
      });
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::LaserScan & m) { on_scan(m); });
    sub_joints_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, [this](const sensor_msgs::msg::JointState & m) {
        if (m.velocity.size() >= 2) {
          u_ = wheel_radius_ * (m.velocity[0] + m.velocity[1]) / 2;
        }
      });
    // one control step per first stage of the horizon
    const auto period = std::chrono::duration<double>(in_->Ts[0]);
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                  [this] { tick(); });
  }

  ~DriftNode() override
  {
    drift_acados_free(capsule_);
    drift_acados_free_capsule(capsule_);
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan & m)
  {
    if (history_.empty()) {    // not drifting yet
      return;
    }
    // the lidar faces forward on the center line (URDF)
    if (std::isnan(lidar_x_)) {
      lidar_x_ = tf_buffer_->lookupTransform("base_footprint", m.header.frame_id, tf2::TimePointZero).transform.translation.x;
    }
    // where the cone should be in the lidar frame, from the estimate now
    const double rho = radius_ - n_;
    const double px = rho * std::sin(e_) - lidar_x_, py = rho * std::cos(e_);
    double sx = 0, sy = 0;
    int count = 0;
    for (size_t i = 0; i < m.ranges.size(); i++) {
      const double a = m.angle_min + i * m.angle_increment;
      const double x = m.ranges[i] * std::cos(a), y = m.ranges[i] * std::sin(a);
      if (std::isfinite(m.ranges[i]) && std::hypot(x - px, y - py) < CONE_WINDOW) {
        sx += x;
        sy += y;
        count++;
      }
    }
    if (count < 2) {
      return;
    }
    // the rays see the near side of the cone, evenly spaced across it: its centre is pi/4 radius
    // further along the ray
    const double a = std::atan2(sy, sx);
    const double cx = sx / count + M_PI / 4 * CONE_RADIUS * std::cos(a) + lidar_x_;
    const double cy = sy / count + M_PI / 4 * CONE_RADIUS * std::sin(a);
    // when the cone was seen: rplidar_ros stamps the start of the turn, which begins behind and
    // goes clockwise (a = pi - lidar angle); the sim has scan_time 0
    const rclcpp::Time seen = rclcpp::Time(m.header.stamp) + rclcpp::Duration::from_seconds(m.scan_time * (M_PI - a) / (2 * M_PI));
    if (seen < history_.front().t) {
      return;
    }
    // the estimate at that time: what it missed is still missing now
    const auto & h = *std::min_element(history_.begin(), history_.end(), [&](const auto & p, const auto & q) {
      return std::abs((p.t - seen).seconds()) < std::abs((q.t - seen).seconds());
    });
    n_ += (radius_ - std::hypot(cx, cy)) - h.n;
    e_ += std::remainder(std::atan2(cx, cy) - h.e, 2 * M_PI);
    last_cone_ = seen;
  }

  void publish(double d_cmd, double u_cmd)
  {
    ackermann_msgs::msg::AckermannDrive msg;
    msg.speed = u_cmd;
    // wheel angle -> servo command, inverse of the measured steering map
    msg.steering_angle = interp(d_cmd, steer_angle_, steer_cmd_);
    pub_drive_->publish(msg);
  }

  void stop(const std::string & reason)
  {
    // zero commands for half a second, then the node exits
    const rclcpp::Time now = get_clock()->now();
    if (stopped_.empty()) {
      stopped_ = reason;
      stop_time_ = now;
      RCLCPP_INFO(get_logger(), "stopped: %s", reason.c_str());
    }
    pub_drive_->publish(ackermann_msgs::msg::AckermannDrive());
    if ((now - stop_time_).seconds() > 0.5) {
      rclcpp::shutdown();
    }
  }

  void tick()
  {
    if (!stopped_.empty()) {
      stop(stopped_);
      return;
    }
    const rclcpp::Time now = get_clock()->now();
    if (!drifting_) {
      // the run-up on the localization: straight ahead, the wheel speed ramping up, until the
      // cone is abeam
      geometry_msgs::msg::TransformStamped tf;
      try {
        tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
      } catch (const tf2::TransformException &) {
        if (launched_) {
          stop("no pose");
        } else {
          pub_drive_->publish(ackermann_msgs::msg::AckermannDrive());    // wait for localization
        }
        return;
      }
      const double age = (now - rclcpp::Time(tf.header.stamp)).seconds();
      if (age > max_pose_age_) {
        if (launched_) {
          stop("pose " + std::to_string(age) + " s old");
        } else {
          pub_drive_->publish(ackermann_msgs::msg::AckermannDrive());
        }
        return;
      }
      const double dx = tf.transform.translation.x - cone_x_, dy = tf.transform.translation.y - cone_y_;
      const double yaw = 2 * std::atan2(tf.transform.rotation.z, tf.transform.rotation.w);
      launched_ = true;
      u_cmd_ = std::min(launch_speed_, u_cmd_ + 5.0 * in_->Ts[0]);
      d_cmd_ = 0;
      publish(d_cmd_, u_cmd_);
      if (dx * std::cos(yaw) + dy * std::sin(yaw) < 0) {
        return;
      }
      // the rear axle around the cone, counterclockwise: offset inside the circle, heading
      // error to its tangent
      drifting_ = true;
      drift_start_ = last_cone_ = now;
      vx_ = vx_ekf_;
      n_ = radius_ - std::hypot(dx, dy);
      e_ = std::remainder(yaw - std::atan2(dy, dx) - M_PI / 2, 2 * M_PI);
    }
    if ((now - drift_start_).seconds() > hold_) {
      stop("drift held " + std::to_string(hold_) + " s");
      return;
    }
    if ((now - last_cone_).seconds() > CONE_TIMEOUT) {
      stop("cone lost");
      return;
    }
    if (std::abs(n_) > max_error_) {
      stop(std::to_string(n_) + " m off the circle");
      return;
    }

    // s = 0: the circle is the same everywhere
    double x0[DRIFT_NX] = {0, n_, e_, vx_, vy_, r_, d_, u_, d_cmd_, u_cmd_};
    double x[DRIFT_NX];
    const bool first = !solved_;
    if (first) {
      for (int j = 0; j <= DRIFT_N; j++) {
        ocp_nlp_out_set(config_, dims_, out_, in_, j, "x", x0);
      }
    }
    std_msgs::msg::Float64MultiArray state;
    state.data.assign(x0, x0 + DRIFT_NX);
    pub_state_->publish(state);
    ocp_nlp_constraints_model_set(config_, dims_, in_, out_, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(config_, dims_, in_, out_, 0, "ubx", x0);
    int status = 0;
    double solve_time = 0, t;
    for (int k = 0; k < (first ? 10 : 1); k++) {
      status = drift_acados_solve(capsule_);
      ocp_nlp_get(solver_, "time_tot", &t);
      solve_time += t;
    }
    solved_ = true;
    if (status != 0 && status != 2) {    // 2: iteration limit, the normal case with one iteration
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "acados status %d", status);
    }

    ocp_nlp_out_get(config_, dims_, out_, 1, "x", x);
    history_.push_back({now, n_, e_});
    if (history_.size() > 25) {
      history_.pop_front();
    }
    n_ = x[1];
    e_ = x[2];
    vx_ = x[3];
    vy_ = x[4];
    d_ = x[6];
    d_cmd_ = x[8];
    u_cmd_ = x[9];
    publish(d_cmd_, u_cmd_);
    std_msgs::msg::Float32 st;
    st.data = solve_time * 1e3;
    pub_solve_time_->publish(st);
  }

  struct Estimate
  {
    rclcpp::Time t;
    double n, e;
  };

  double wheel_radius_, cone_x_, cone_y_, launch_speed_, hold_, mu_scale_, max_error_, max_pose_age_, radius_;
  std::vector<double> steer_cmd_, steer_angle_;

  drift_solver_capsule * capsule_;
  ocp_nlp_config * config_;
  ocp_nlp_dims * dims_;
  ocp_nlp_in * in_;
  ocp_nlp_out * out_;
  ocp_nlp_solver * solver_;

  // measured, and estimated one step ahead by the NMPC
  double vx_ekf_ = 0, r_ = 0, u_ = 0, vx_ = 0, vy_ = 0, d_ = 0;
  double d_cmd_ = 0, u_cmd_ = 0;
  double n_ = 0, e_ = 0, lidar_x_ = NAN;    // offset and heading error to the circle, from the cone
  std::deque<Estimate> history_;            // the last 0.5 s of them
  rclcpp::Time last_cone_;
  bool launched_ = false, drifting_ = false, solved_ = false;
  rclcpp::Time drift_start_;
  std::string stopped_;
  rclcpp::Time stop_time_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr pub_drive_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_solve_time_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_state_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joints_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriftNode>());
  rclcpp::shutdown();
  return 0;
}

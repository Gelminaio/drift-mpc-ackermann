#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_drift.h"

// The drift of ocp.drift_ocp around a cone, from rest: straight ahead up to launch_speed, the
// drift NMPC from the closest point to the cone for hold seconds, then zero commands. Measured:
// rear axle pose (localization), yaw rate (EKF), wheel speed (encoders). In the drift vx, vy and
// the steering angle come from the previous solve: the wheels no longer tell the ground speed.

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
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10, [this](const nav_msgs::msg::Odometry & m) {
        vx_ekf_ = m.twist.twist.linear.x;
        r_ = m.twist.twist.angular.z;
      });
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
    const rclcpp::Time now = get_clock()->now();
    const double age = (now - rclcpp::Time(tf.header.stamp)).seconds();
    if (age > max_pose_age_) {
      if (launched_) {
        stop("pose " + std::to_string(age) + " s old");
      } else {
        pub_drive_->publish(ackermann_msgs::msg::AckermannDrive());
      }
      return;
    }

    // the rear axle around the cone, counterclockwise: angle, offset inside the circle, heading
    // error to its tangent
    const double dx = tf.transform.translation.x - cone_x_, dy = tf.transform.translation.y - cone_y_;
    const double yaw = 2 * std::atan2(tf.transform.rotation.z, tf.transform.rotation.w);
    theta_ += std::remainder(std::atan2(dy, dx) - theta_, 2 * M_PI);
    const double n = radius_ - std::hypot(dx, dy);
    const double e_psi = std::remainder(yaw - theta_ - M_PI / 2, 2 * M_PI);

    if (!drifting_) {
      // straight ahead, the wheel speed ramping up, until the cone is abeam
      launched_ = true;
      u_cmd_ = std::min(launch_speed_, u_cmd_ + 5.0 * in_->Ts[0]);
      d_cmd_ = 0;
      publish(d_cmd_, u_cmd_);
      if (dx * std::cos(yaw) + dy * std::sin(yaw) < 0) {
        return;
      }
      drifting_ = true;
      drift_start_ = now;
      vx_ = vx_ekf_;
    }
    if ((now - drift_start_).seconds() > hold_) {
      stop("drift held " + std::to_string(hold_) + " s");
      return;
    }
    if (std::abs(n) > max_error_) {
      stop(std::to_string(n) + " m off the circle");
      return;
    }

    double x0[DRIFT_NX] = {radius_ * theta_, n, e_psi, vx_, vy_, r_, d_, u_, d_cmd_, u_cmd_};
    double x[DRIFT_NX];
    const bool first = !solved_;
    if (first) {
      for (int j = 0; j <= DRIFT_N; j++) {
        ocp_nlp_out_set(config_, dims_, out_, in_, j, "x", x0);
      }
    }
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
  double theta_ = 0;
  bool launched_ = false, drifting_ = false, solved_ = false;
  rclcpp::Time drift_start_;
  std::string stopped_;
  rclcpp::Time stop_time_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr pub_drive_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_solve_time_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joints_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriftNode>());
  rclcpp::shutdown();
  return 0;
}

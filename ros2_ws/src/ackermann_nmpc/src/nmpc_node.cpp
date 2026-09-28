#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
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
#include "acados_solver_nmpc.h"

// The NMPC of ocp.py on the car, one solve per first stage of the horizon (20 ms). Measured: rear
// axle pose in the map (localization), vx and yaw rate (EKF), wheel speed (encoders). Lateral
// velocity and steering angle are not measured: they come from the previous solve, one step
// ahead. Stops as the Pure Pursuit node.

struct Track
{
  std::vector<double> s, x, y, yaw, kappa, v;
  double length;
};

// s,x,y,yaw,kappa,v with a header line, as make_track.py writes it
Track load_track(const std::string & file)
{
  Track t;
  std::ifstream in(file);
  std::string line;
  std::getline(in, line);
  while (std::getline(in, line)) {
    std::stringstream ss(line);
    std::string f;
    std::vector<double> row;
    while (std::getline(ss, f, ',')) {
      row.push_back(std::stod(f));
    }
    t.s.push_back(row[0]);
    t.x.push_back(row[1]);
    t.y.push_back(row[2]);
    t.yaw.push_back(row[3]);
    t.kappa.push_back(row[4]);
    t.v.push_back(row[5]);
  }
  t.length = t.s.back() + std::hypot(t.x.front() - t.x.back(), t.y.front() - t.y.back());
  return t;
}

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

class NmpcNode : public rclcpp::Node
{
public:
  NmpcNode()
  : Node("nmpc")
  {
    // vehicle_params.yaml
    wheel_radius_ = declare_parameter<double>("wheel_radius");
    steer_cmd_ = declare_parameter<std::vector<double>>("steer_cmd");
    steer_angle_ = declare_parameter<std::vector<double>>("steer_angle");
    // controller
    track_ = load_track(declare_parameter<std::string>("track_file"));
    speed_scale_ = declare_parameter("speed_scale", 1.0);    // fraction of the track speed profile
    laps_ = declare_parameter("laps", 3);
    mu_scale_ = declare_parameter("mu_scale", 1.0);          // floor friction the NMPC assumes
    max_error_ = declare_parameter("max_error", 0.5);        // m off the line: stop
    max_pose_age_ = declare_parameter("max_pose_age", 0.2);  // s since the last odometry: stop
    max_correction_age_ = declare_parameter("max_correction_age", 0.5);  // s, localization

    capsule_ = nmpc_acados_create_capsule();
    if (nmpc_acados_create(capsule_) != 0) {
      throw std::runtime_error("acados solver creation failed");
    }
    config_ = nmpc_acados_get_nlp_config(capsule_);
    dims_ = nmpc_acados_get_nlp_dims(capsule_);
    in_ = nmpc_acados_get_nlp_in(capsule_);
    out_ = nmpc_acados_get_nlp_out(capsule_);
    solver_ = nmpc_acados_get_nlp_solver(capsule_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    pub_drive_ = create_publisher<ackermann_msgs::msg::AckermannDrive>("/drive", 10);
    pub_solve_time_ = create_publisher<std_msgs::msg::Float32>("nmpc/solve_time", 10);
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10, [this](const nav_msgs::msg::Odometry & m) {
        vx_ = m.twist.twist.linear.x;
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

  ~NmpcNode() override
  {
    nmpc_acados_free(capsule_);
    nmpc_acados_free_capsule(capsule_);
  }

private:
  // the rear axle onto the line: nearest point near the previous s (the whole line the first
  // time), then along and across its heading; s keeps counting past a lap
  void to_path(double x, double y, double yaw, double & s, double & n, double & e_psi)
  {
    const size_t len = track_.s.size();
    size_t k = 0;
    double best = 1e9;
    if (std::isnan(s_prev_)) {
      for (size_t i = 0; i < len; i++) {
        const double d = std::hypot(track_.x[i] - x, track_.y[i] - y);
        if (d < best) {
          best = d;
          k = i;
        }
      }
    } else {
      const size_t i0 = index(s_prev_);
      for (int j = -10; j < 30; j++) {
        const size_t i = (i0 + len + j) % len;
        const double d = std::hypot(track_.x[i] - x, track_.y[i] - y);
        if (d < best) {
          best = d;
          k = i;
        }
      }
    }
    const double dx = x - track_.x[k], dy = y - track_.y[k], yaw_k = track_.yaw[k];
    s = track_.s[k] + dx * std::cos(yaw_k) + dy * std::sin(yaw_k);
    if (!std::isnan(s_prev_)) {
      s += track_.length * std::round((s_prev_ - s) / track_.length);
    }
    n = -dx * std::sin(yaw_k) + dy * std::cos(yaw_k);
    e_psi = std::remainder(yaw - yaw_k, 2 * M_PI);
  }

  size_t index(double s) const
  {
    const double sm = s - track_.length * std::floor(s / track_.length);
    return std::upper_bound(track_.s.begin(), track_.s.end(), sm) - track_.s.begin() - 1;
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
    // the newest localization correction (map -> odom) on the newest odometry: AMCL answers
    // 0.15-0.2 s after the start of the scan, up to 0.4, the EKF 10 ms after the wheels
    geometry_msgs::msg::TransformStamped correction, odom;
    try {
      correction = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
      odom = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      if (started_) {
        stop("no pose");
      } else {
        pub_drive_->publish(ackermann_msgs::msg::AckermannDrive());    // wait for localization
      }
      return;
    }
    const rclcpp::Time now = get_clock()->now();
    const double age = (now - rclcpp::Time(odom.header.stamp)).seconds();
    const double correction_age = (now - rclcpp::Time(correction.header.stamp)).seconds();
    if (age > max_pose_age_ || correction_age > max_correction_age_) {
      if (started_) {
        stop("odometry " + std::to_string(age) + " s old, localization " + std::to_string(correction_age));
      } else {
        pub_drive_->publish(ackermann_msgs::msg::AckermannDrive());
      }
      return;
    }

    const auto & c = correction.transform, & o = odom.transform;
    const double yaw_c = 2 * std::atan2(c.rotation.z, c.rotation.w);
    const double px = c.translation.x + std::cos(yaw_c) * o.translation.x - std::sin(yaw_c) * o.translation.y;
    const double py = c.translation.y + std::sin(yaw_c) * o.translation.x + std::cos(yaw_c) * o.translation.y;
    double s, n, e_psi;
    to_path(px, py, yaw_c + 2 * std::atan2(o.rotation.z, o.rotation.w), s, n, e_psi);
    s_prev_ = s;
    if (!started_) {
      s_start_ = s;
    }
    if (s - s_start_ >= laps_ * track_.length) {
      stop(std::to_string(laps_) + " laps");
      return;
    }
    if (std::abs(n) > max_error_) {
      stop(std::to_string(n) + " m off the track");
      return;
    }

    double x0[NMPC_NX] = {s, n, e_psi, vx_, vy_, r_, d_, u_, d_cmd_, u_cmd_};
    double x[NMPC_NX];
    if (!started_) {
      for (int j = 0; j <= NMPC_N; j++) {
        ocp_nlp_out_set(config_, dims_, out_, in_, j, "x", x0);
      }
    }
    // curvature and speed of the line where the previous solution puts each stage
    for (int j = 0; j <= NMPC_N; j++) {
      ocp_nlp_out_get(config_, dims_, out_, j, "x", x);
      const size_t i = index(x[0]);
      double p[NMPC_NP] = {mu_scale_, track_.kappa[i]};
      nmpc_acados_update_params(capsule_, j, p, NMPC_NP);
      double yref[5] = {0, 0, speed_scale_ * track_.v[i], 0, 0};
      ocp_nlp_cost_model_set(config_, dims_, in_, j, "yref", yref);
    }
    ocp_nlp_constraints_model_set(config_, dims_, in_, out_, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(config_, dims_, in_, out_, 0, "ubx", x0);

    int status = 0;
    double solve_time = 0, t;
    for (int k = 0; k < (started_ ? 1 : 10); k++) {
      status = nmpc_acados_solve(capsule_);
      ocp_nlp_get(solver_, "time_tot", &t);
      solve_time += t;
    }
    started_ = true;
    if (status != 0 && status != 2) {    // 2: iteration limit, the normal case with one iteration
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "acados status %d", status);
    }

    ocp_nlp_out_get(config_, dims_, out_, 1, "x", x);
    vy_ = x[4];
    d_ = x[6];
    d_cmd_ = x[8];
    u_cmd_ = x[9];
    publish(d_cmd_, u_cmd_);
    std_msgs::msg::Float32 st;
    st.data = solve_time * 1e3;
    pub_solve_time_->publish(st);
  }

  double wheel_radius_, speed_scale_, mu_scale_, max_error_, max_pose_age_, max_correction_age_;
  int64_t laps_;
  std::vector<double> steer_cmd_, steer_angle_;
  Track track_;

  nmpc_solver_capsule * capsule_;
  ocp_nlp_config * config_;
  ocp_nlp_dims * dims_;
  ocp_nlp_in * in_;
  ocp_nlp_out * out_;
  ocp_nlp_solver * solver_;

  // measured, and estimated one step ahead by the NMPC
  double vx_ = 0, r_ = 0, u_ = 0, vy_ = 0, d_ = 0;
  double d_cmd_ = 0, u_cmd_ = 0;
  double s_prev_ = NAN, s_start_ = 0;
  bool started_ = false;
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
  rclcpp::spin(std::make_shared<NmpcNode>());
  rclcpp::shutdown();
  return 0;
}

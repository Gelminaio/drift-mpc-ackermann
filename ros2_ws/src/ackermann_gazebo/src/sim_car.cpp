#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <yaml-cpp/yaml.h>

// The ESP32 and the car in Gazebo. /drive and /arm in, /joint_states, /imu/data_raw and
// /steering_angle out at 50 Hz as the firmware does; the car moves by the Phase 5 model
// (scripts/vehicle_model.py), RK4 at the world step. /ground_truth: the true state.
// Parameter mu_scale: floor friction against the tiles, can change while driving.

namespace ackermann_gazebo
{

// firmware/include/config.h
constexpr double CMD_TIMEOUT = 0.5;                   // s without /drive: soft stop
constexpr double SOFTSTOP_RAMP = 2.0;                 // m/s2
constexpr double PID_DEADBAND = 0.02;                 // m/s, below it duty 0
constexpr double SERVO_MAX = 30.0 * M_PI / 180.0;     // rad
constexpr double WHEEL_EMA_TAU = 0.045;               // s, alpha 0.2 every 10 ms
constexpr double GYRO_VAR = 0.07 * 0.07;              // (rad/s)^2, reported
constexpr double ACCEL_VAR = 2.5 * 2.5;               // (m/s2)^2, reported

// IMU noise on tiles, sd at rest and growing with speed from vibration (pp_run1/3, 2026-09-27)
constexpr double GYRO_SD_REST = 0.0014, GYRO_SD_PER_V = 0.063;    // rad/s, rad/s per m/s
constexpr double ACCEL_SD_REST = 0.07, ACCEL_SD_PER_V = 3.0;      // m/s2, m/s2 per m/s

constexpr double V_KIN = 0.1;    // m/s, below it the tire model is singular: roll without slip

// map position of the CG, heading, the Phase 5 states, rear wheel surface speed
enum { X, Y, PSI, VX, VY, R, D, U, N };
using State = std::array<double, N>;

struct Params
{
  double mass, iz, lf, lr, track, wheel_radius, nf, nr;
  double mu_f, b_f, mu_r, b_r, c, lambda, load_transfer, h_cg, steer_lag, brake_lag, v_max;
  std::vector<double> steer_cmd, steer_angle, speed_lag_v, speed_lag;
};

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

// Floor friction mu_scale times the tiles: the peak force scales with it, the stiffness at
// small slip does not (brush model), so mu -> mu_scale mu and B -> B / mu_scale.

// one rear wheel at surface speed u under load n: force against its sliding direction, the
// longitudinal slip weighed down by lambda (1 in the model)
void rear_wheel(const Params & p, double mu_scale, double n, double vx, double vy, double u, double & fx, double & fy)
{
  const double sx = (vx - u) / std::max(u, 0.1) / p.lambda, sy = vy / std::max(u, 0.1);
  const double s = std::hypot(sx, sy) + 1e-9;
  const double f = mu_scale * p.mu_r * n * std::sin(p.c * std::atan(p.b_r / mu_scale * s));
  fx = -f * sx / s;
  fy = -f * sy / s;
}

State deriv(const Params & p, double mu_scale, const State & x, double d_cmd, double u_cmd, double tau_u)
{
  const double vx = std::max(x[VX], 0.05), vy = x[VY], r = x[R], d = x[D];
  const double alpha_f = d - std::atan((vy + p.lf * r) / vx);
  const double fyf = mu_scale * p.mu_f * p.nf * std::sin(p.c * std::atan(p.b_f / mu_scale * alpha_f));
  // load from the inner (left) to the outer rear wheel: load_transfer of m ay h / T (0 in the model)
  const double dn = p.load_transfer * p.mass * vx * r * p.h_cg / p.track;
  double fxl, fyl, fxr, fyr;
  rear_wheel(p, mu_scale, std::max(p.nr / 2 - dn, 0.0), vx - p.track / 2 * r, vy - p.lr * r, x[U], fxl, fyl);
  rear_wheel(p, mu_scale, std::max(p.nr / 2 + dn, 0.0), vx + p.track / 2 * r, vy - p.lr * r, x[U], fxr, fyr);
  const double fx = fxl + fxr - fyf * std::sin(d);
  const double fy = fyf * std::cos(d) + fyl + fyr;
  const double mz = p.lf * fyf * std::cos(d) - p.lr * (fyl + fyr) + p.track / 2 * (fxr - fxl);

  State dx;
  dx[X] = x[VX] * std::cos(x[PSI]) - x[VY] * std::sin(x[PSI]);
  dx[Y] = x[VX] * std::sin(x[PSI]) + x[VY] * std::cos(x[PSI]);
  dx[PSI] = x[R];
  dx[VX] = fx / p.mass + x[VY] * x[R];
  dx[VY] = fy / p.mass - x[VX] * x[R];
  dx[R] = mz / p.iz;
  dx[D] = (d_cmd - x[D]) / p.steer_lag;
  dx[U] = (u_cmd - x[U]) / tau_u;
  return dx;
}

State add(const State & x, const State & dx, double h)
{
  State y;
  for (int i = 0; i < N; i++) {
    y[i] = x[i] + h * dx[i];
  }
  return y;
}

class SimCar : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
public:
  ~SimCar() override
  {
    if (executor_) {
      executor_->cancel();
      spin_thread_.join();
    }
  }

  void Configure(const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(entity);
    const YAML::Node y = YAML::LoadFile(ament_index_cpp::get_package_share_directory("ackermann_description") +
                                        "/config/vehicle_params.yaml")["/**"]["ros__parameters"];
    p_.mass = y["mass"].as<double>();
    p_.iz = y["iz"].as<double>();
    p_.lf = y["lf"].as<double>();
    p_.lr = y["lr"].as<double>();
    p_.track = y["track"].as<double>();
    p_.wheel_radius = y["wheel_radius"].as<double>();
    p_.nf = p_.mass * 9.81 * p_.lr / (p_.lf + p_.lr);
    p_.nr = p_.mass * 9.81 * p_.lf / (p_.lf + p_.lr);
    p_.mu_f = y["tire_mu_f"].as<double>();
    p_.b_f = y["tire_b_f"].as<double>();
    p_.mu_r = y["tire_mu_r"].as<double>();
    p_.b_r = y["tire_b_r"].as<double>();
    p_.c = y["tire_c"].as<double>();
    p_.h_cg = y["h_cg"].as<double>();
    p_.steer_lag = y["steer_lag"].as<double>();
    p_.brake_lag = y["brake_lag"].as<double>();
    p_.v_max = y["v_max"].as<double>();
    p_.steer_cmd = y["steer_cmd"].as<std::vector<double>>();
    p_.steer_angle = y["steer_angle"].as<std::vector<double>>();
    p_.speed_lag_v = y["speed_lag_v"].as<std::vector<double>>();
    p_.speed_lag = y["speed_lag"].as<std::vector<double>>();
    // IMU from the CG, body frame; the URDF gives it from the rear axle
    rho_x_ = sdf->Get<double>("imu_x") - p_.lr;
    rho_y_ = sdf->Get<double>("imu_y");

    if (!rclcpp::ok()) {
      // Gazebo keeps its own Ctrl-C handling
      rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    }
    node_ = rclcpp::Node::make_shared("sim_car");
    rcl_interfaces::msg::ParameterDescriptor range;
    range.floating_point_range.resize(1);
    range.floating_point_range[0].from_value = 0.1;
    range.floating_point_range[0].to_value = 2.0;
    node_->declare_parameter("mu_scale", 1.0, range);
    // the tire of this car, the model's unless set: to test a controller against a car unlike its
    // model. mu_f, b_f, mu_r, b_r, c, lambda, load_transfer
    node_->declare_parameter("tire", std::vector<double>{p_.mu_f, p_.b_f, p_.mu_r, p_.b_r, p_.c, 1.0, 0.0});
    pub_joints_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    pub_imu_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    pub_steering_ = node_->create_publisher<std_msgs::msg::Float32>("steering_angle", 10);
    pub_truth_ = node_->create_publisher<nav_msgs::msg::Odometry>("ground_truth", 10);
    sub_drive_ = node_->create_subscription<ackermann_msgs::msg::AckermannDrive>(
      "drive", 10, [this](const ackermann_msgs::msg::AckermannDrive & m) {
        std::lock_guard<std::mutex> lock(mutex_);
        servo_ = std::clamp(static_cast<double>(m.steering_angle), -SERVO_MAX, SERVO_MAX);
        setpoint_ = m.speed;
        last_cmd_ = now_;
      });
    sub_arm_ = node_->create_subscription<std_msgs::msg::Bool>(
      "arm", 10, [this](const std_msgs::msg::Bool & m) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (m.data) {
          setpoint_ = 0.0;    // start from rest
        }
        armed_ = m.data;
      });
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this] { executor_->spin(); });
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused) {
      return;
    }
    if (!spawned_) {
      // at rest where `create` put it, which is after Configure. The model origin is
      // base_footprint, on the rear axle
      const gz::math::Pose3d pose = gz::sim::worldPose(model_.Entity(), ecm);
      x_.fill(0.0);
      x_[PSI] = pose.Rot().Yaw();
      x_[X] = pose.Pos().X() + p_.lr * std::cos(x_[PSI]);
      x_[Y] = pose.Pos().Y() + p_.lr * std::sin(x_[PSI]);
      spawned_ = true;
    }
    const double now = std::chrono::duration<double>(info.simTime).count();
    const double h = std::chrono::duration<double>(info.dt).count();

    double servo, setpoint;
    bool armed;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      now_ = now;
      if (armed_ && last_cmd_ >= 0.0 && now - last_cmd_ > CMD_TIMEOUT) {
        setpoint_ -= std::clamp(setpoint_, -SOFTSTOP_RAMP * h, SOFTSTOP_RAMP * h);
      }
      servo = servo_;
      setpoint = setpoint_;
      armed = armed_;
    }

    const double d_cmd = interp(servo, p_.steer_cmd, p_.steer_angle);
    double u_cmd = 0.0, tau_u = p_.brake_lag;
    if (armed && std::abs(setpoint) >= PID_DEADBAND) {
      u_cmd = std::clamp(setpoint, -p_.v_max, p_.v_max);
      tau_u = interp(std::abs(setpoint), p_.speed_lag_v, p_.speed_lag);
    }
    const auto tire = node_->get_parameter("tire").as_double_array();
    p_.mu_f = tire[0];
    p_.b_f = tire[1];
    p_.mu_r = tire[2];
    p_.b_r = tire[3];
    p_.c = tire[4];
    p_.lambda = tire[5];
    p_.load_transfer = tire[6];
    step(node_->get_parameter("mu_scale").as_double(), d_cmd, u_cmd, tau_u, h);
    wheel_ema_ += h / WHEEL_EMA_TAU * (x_[U] - wheel_ema_);
    wheel_angle_ += x_[U] / p_.wheel_radius * h;

    const double c = std::cos(x_[PSI]), s = std::sin(x_[PSI]);
    model_.SetWorldPoseCmd(ecm, gz::math::Pose3d(x_[X] - p_.lr * c, x_[Y] - p_.lr * s, 0, 0, 0, x_[PSI]));

    if (info.simTime >= next_publish_) {
      next_publish_ += std::chrono::milliseconds(20);
      publish(rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count(), RCL_ROS_TIME),
              servo);
    }
  }

private:
  void step(double mu_scale, double d_cmd, double u_cmd, double tau_u, double h)
  {
    State & x = x_;
    if (std::max(x[VX], x[U]) < V_KIN) {
      // the kinematic bicycle, rear axle without slip
      x[D] += h * (d_cmd - x[D]) / p_.steer_lag;
      x[U] += h * (u_cmd - x[U]) / tau_u;
      x[VX] = x[U];
      x[R] = x[U] * std::tan(x[D]) / (p_.lf + p_.lr);
      x[VY] = p_.lr * x[R];
      x[PSI] += h * x[R];
      x[X] += h * (x[VX] * std::cos(x[PSI]) - x[VY] * std::sin(x[PSI]));
      x[Y] += h * (x[VX] * std::sin(x[PSI]) + x[VY] * std::cos(x[PSI]));
      return;
    }
    const State k1 = deriv(p_, mu_scale, x, d_cmd, u_cmd, tau_u);
    const State k2 = deriv(p_, mu_scale, add(x, k1, h / 2), d_cmd, u_cmd, tau_u);
    const State k3 = deriv(p_, mu_scale, add(x, k2, h / 2), d_cmd, u_cmd, tau_u);
    const State k4 = deriv(p_, mu_scale, add(x, k3, h), d_cmd, u_cmd, tau_u);
    for (int i = 0; i < N; i++) {
      x[i] += h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
  }

  void publish(const rclcpp::Time & stamp, double servo)
  {
    // acceleration over the last 20 ms at the IMU, body frame
    const double c = std::cos(x_[PSI]), s = std::sin(x_[PSI]), r = x_[R];
    const double vxw = x_[VX] * c - x_[VY] * s, vyw = x_[VX] * s + x_[VY] * c;
    const double axw = (vxw - vxw_prev_) / 0.02, ayw = (vyw - vyw_prev_) / 0.02;
    const double r_dot = (r - r_prev_) / 0.02;
    const double ax = axw * c + ayw * s - r_dot * rho_y_ - r * r * rho_x_;
    const double ay = -axw * s + ayw * c + r_dot * rho_x_ - r * r * rho_y_;
    vxw_prev_ = vxw;
    vyw_prev_ = vyw;
    r_prev_ = r;

    sensor_msgs::msg::JointState joints;
    joints.header.stamp = stamp;
    joints.position = {wheel_angle_, wheel_angle_};
    joints.velocity = {wheel_ema_ / p_.wheel_radius, wheel_ema_ / p_.wheel_radius};
    pub_joints_->publish(joints);

    // imu_link is rotated -90 deg about z
    const double gyro_sd = std::max(GYRO_SD_REST, GYRO_SD_PER_V * std::abs(x_[U]));
    const double accel_sd = std::max(ACCEL_SD_REST, ACCEL_SD_PER_V * std::abs(x_[U]));
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "imu_link";
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity.x = gyro_sd * noise_(rng_);
    imu.angular_velocity.y = gyro_sd * noise_(rng_);
    imu.angular_velocity.z = r + gyro_sd * noise_(rng_);
    imu.linear_acceleration.x = ay + accel_sd * noise_(rng_);
    imu.linear_acceleration.y = -ax + accel_sd * noise_(rng_);
    imu.linear_acceleration.z = accel_sd * noise_(rng_);
    for (int i = 0; i < 3; i++) {
      imu.angular_velocity_covariance[4 * i] = GYRO_VAR;
      imu.linear_acceleration_covariance[4 * i] = ACCEL_VAR;
    }
    pub_imu_->publish(imu);

    std_msgs::msg::Float32 steering;
    steering.data = servo;
    pub_steering_->publish(steering);

    // base_footprint in the map, the world frame of room.sdf; velocities in base_footprint
    nav_msgs::msg::Odometry truth;
    truth.header.stamp = stamp;
    truth.header.frame_id = "map";
    truth.child_frame_id = "base_footprint";
    truth.pose.pose.position.x = x_[X] - p_.lr * c;
    truth.pose.pose.position.y = x_[Y] - p_.lr * s;
    truth.pose.pose.orientation.z = std::sin(x_[PSI] / 2);
    truth.pose.pose.orientation.w = std::cos(x_[PSI] / 2);
    truth.twist.twist.linear.x = x_[VX];
    truth.twist.twist.linear.y = x_[VY] - p_.lr * r;
    truth.twist.twist.angular.z = r;
    pub_truth_->publish(truth);
  }

  Params p_;
  gz::sim::Model model_;
  State x_;
  bool spawned_ = false;
  double rho_x_, rho_y_;
  double wheel_ema_ = 0.0, wheel_angle_ = 0.0;     // m/s as /joint_states reports it, rad
  double vxw_prev_ = 0.0, vyw_prev_ = 0.0, r_prev_ = 0.0;
  std::chrono::steady_clock::duration next_publish_{0};
  std::mt19937 rng_;
  std::normal_distribution<double> noise_;

  // the ESP32 state, written by the ROS callbacks
  std::mutex mutex_;
  double servo_ = 0.0;        // rad, servo command
  double setpoint_ = 0.0;     // m/s, both rear wheels
  bool armed_ = false;
  double last_cmd_ = -1.0;    // s, sim time of the last /drive, -1 before the first
  double now_ = 0.0;

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joints_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_steering_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_truth_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr sub_drive_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_arm_;
};

}  // namespace ackermann_gazebo

GZ_ADD_PLUGIN(ackermann_gazebo::SimCar, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)

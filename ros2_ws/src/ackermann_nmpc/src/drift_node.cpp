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

// The drift of ocp.drift_ocp around a cone, from rest. The cone is the reference: the node keeps
// where it is from the rear axle, from the scans, carried between them by the motion of the car;
// the map only tells where to look for it at the start. Straight ahead up to launch_speed, pure
// pursuit on the line the car points along at the start with the cone a radius to its left. Just
// before the cone is abeam, full lock and full throttle until the car spins (the donut of Phase 5),
// then the drift NMPC for hold seconds, then zero commands. Seen from the rear axle at distance rho
// and angle atan2(x, y), the cone gives the offset to the circle n = radius - rho and the heading
// error. vx and vy come from the previous solve (the wheels no longer tell the ground speed),
// corrected at each scan by how the cone moved since the last one; the steering angle from the
// previous solve, the yaw rate from the gyro (EKF).

constexpr double CONE_WINDOW = 0.15;   // m around where the cone should be
constexpr double CONE_TIMEOUT = 0.5;   // s without the cone: stop
constexpr double LOOKAHEAD = 0.4;      // m, pure pursuit in the run-up
constexpr double HANDOVER = 0.10;      // m before the cone is abeam: the steering lag, 0.17 s
constexpr double VEL_GAIN = 0.5;       // share of the difference to the velocity from the cone
// the entry: on the car only a step to full torque at full lock breaks the rear loose (the model
// lets it go with less), a speed above what the motor reaches saturates the speed loop
constexpr double KICK_SPEED = 1.5;     // m/s
constexpr double KICK_YAW_RATE = 3.0;  // rad/s: the donut, the car turns at most 2.8 on its tires
constexpr double KICK_TIMEOUT = 1.5;   // s

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

struct Estimate
{
  rclcpp::Time t;
  double cx, cy, psi, vx, vy;
};

class DriftNode : public rclcpp::Node
{
public:
  DriftNode()
  : Node("drift")
  {
    // vehicle_params.yaml
    wheel_radius_ = declare_parameter<double>("wheel_radius");
    wheelbase_ = declare_parameter<double>("wheelbase");
    lr_ = declare_parameter<double>("lr");
    v_max_ = declare_parameter<double>("v_max");
    steer_cmd_ = declare_parameter<std::vector<double>>("steer_cmd");
    steer_angle_ = declare_parameter<std::vector<double>>("steer_angle");
    // the cone, in the map, and the run
    cone_x_ = declare_parameter<double>("cone_x");
    cone_y_ = declare_parameter<double>("cone_y");
    cone_radius_ = declare_parameter<double>("cone_radius");    // m, at the height of the lidar
    launch_speed_ = declare_parameter("launch_speed", 0.8);  // m/s at the wheels before the drift
    hold_ = declare_parameter("hold", 10.0);                 // s of drift
    mu_scale_ = declare_parameter("mu_scale", 1.0);          // floor friction the NMPC assumes
    max_error_ = declare_parameter("max_error", 0.5);        // m off the circle: stop
    // rad: the car regrips when the steering opens below ~14 deg in a drift (open-loop sweep),
    // the models do not know it
    const double steer_floor = declare_parameter("steer_floor", 0.24);
    // the drift equilibrium and its circle, from drift.py (drift.launch.py)
    radius_ = declare_parameter<double>("drift_radius");
    const double heading = declare_parameter<double>("drift_heading");
    const double vx = declare_parameter<double>("drift_vx");
    const double vy = declare_parameter<double>("drift_vy");
    const double r = declare_parameter<double>("drift_r");
    const double d = declare_parameter<double>("drift_steering");
    const double u = declare_parameter<double>("drift_wheel_speed");
    // the donut at full lock and that wheel speed: where the NMPC starts after the entry
    donut_vx_ = declare_parameter<double>("donut_vx");
    donut_vy_ = declare_parameter<double>("donut_vy");
    full_lock_ = steer_angle_.back();

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
    for (int j = 1; j < DRIFT_N; j++) {
      double lbx[4];    // on n, vx, d_cmd, u_cmd (ocp.py)
      ocp_nlp_constraints_model_get(config_, dims_, in_, j, "lbx", lbx);
      lbx[2] = steer_floor;
      ocp_nlp_constraints_model_set(config_, dims_, in_, out_, j, "lbx", lbx);
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
    if (history_.empty()) {    // no cone to look for yet
      return;
    }
    // the lidar faces forward on the center line (URDF)
    if (std::isnan(lidar_x_)) {
      lidar_x_ = tf_buffer_->lookupTransform("base_footprint", m.header.frame_id, tf2::TimePointZero).transform.translation.x;
    }
    // when the cone was seen: rplidar_ros stamps the start of the turn, which begins behind and
    // goes clockwise (a = pi - lidar angle); the sim has scan_time 0
    const rclcpp::Time stamp(m.header.stamp);
    auto when = [&](double a) { return stamp + rclcpp::Duration::from_seconds(m.scan_time * (M_PI - a) / (2 * M_PI)); };
    // where the cone should be when the lidar looked at it, in the lidar frame
    const auto s0 = at(stamp);
    const auto p = at(when(std::atan2(s0->cy, s0->cx - lidar_x_)));
    // the points there, then again around where they are: a window off the cone would cut it and
    // pull the centre towards the prediction
    double px = p->cx - lidar_x_, py = p->cy, sx = 0, sy = 0;
    int count = 0;
    for (const double window : {CONE_WINDOW, 2 * cone_radius_ + 0.02}) {
      sx = sy = 0;
      count = 0;
      for (size_t i = 0; i < m.ranges.size(); i++) {
        const double a = m.angle_min + i * m.angle_increment;
        const double x = m.ranges[i] * std::cos(a), y = m.ranges[i] * std::sin(a);
        if (std::isfinite(m.ranges[i]) && std::hypot(x - px, y - py) < window) {
          sx += x;
          sy += y;
          count++;
        }
      }
      if (count < 2) {
        return;
      }
      px = sx / count;
      py = sy / count;
    }
    // the rays see the near side of the cone, evenly spaced across it: its centre is pi/4 radius
    // further along the ray
    const double a = std::atan2(sy, sx);
    const double cx = sx / count + M_PI / 4 * cone_radius_ * std::cos(a) + lidar_x_;
    const double cy = sy / count + M_PI / 4 * cone_radius_ * std::sin(a);
    const rclcpp::Time seen = when(a);
    if (seen < history_.front().t) {
      return;
    }
    // the estimate at that time: what it missed is still missing now, turned by the yaw since,
    // and in the estimates kept since, which the next scan compares against
    const auto h = at(seen);
    const double dx = cx - h->cx, dy = cy - h->cy, psi = h->psi;
    for (auto q = h; q != history_.end(); q++) {
      const double turn = q->psi - psi;
      q->cx += std::cos(turn) * dx + std::sin(turn) * dy;
      q->cy += -std::sin(turn) * dx + std::cos(turn) * dy;
    }
    cx_ = history_.back().cx;
    cy_ = history_.back().cy;

    // the velocity of the rear axle: the cone stands still, so where it moved in the car frame
    // since the last scan, turned back by the yaw between, is where the car went
    const double dt = (seen - last_cone_).seconds(), turn = psi - last_psi_;
    if (scans_ > 0 && dt > 0.05 && dt < 0.4) {
      const double mx = last_cx_ - (std::cos(turn) * cx - std::sin(turn) * cy);
      const double my = last_cy_ - (std::sin(turn) * cx + std::cos(turn) * cy);
      // per second, in the frame half-way between the scans; vy at the CG
      cone_vx_ = (std::cos(turn / 2) * mx + std::sin(turn / 2) * my) / dt;
      cone_vy_ = (-std::sin(turn / 2) * mx + std::cos(turn / 2) * my) / dt + lr_ * r_;
      cone_v_time_ = last_cone_ + rclcpp::Duration::from_seconds(dt / 2);
      if (drifting_ && cone_v_time_ > drift_start_) {
        // against the estimate at that time, as the position
        const auto m = at(cone_v_time_);
        vx_ += VEL_GAIN * (cone_vx_ - m->vx);
        vy_ += VEL_GAIN * (cone_vy_ - m->vy);
      }
    }
    last_cx_ = cx;
    last_cy_ = cy;
    last_psi_ = psi;
    last_cone_ = seen;
    scans_++;
  }

  // the kept estimate closest to t
  std::deque<Estimate>::iterator at(const rclcpp::Time & t)
  {
    return std::min_element(history_.begin(), history_.end(), [&](const Estimate & p, const Estimate & q) {
      return std::abs((p.t - t).seconds()) < std::abs((q.t - t).seconds());
    });
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
    if (history_.empty()) {
      // where to look for the cone: the map, once, the car at rest on the start mark
      geometry_msgs::msg::TransformStamped tf;
      try {
        tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
      } catch (const tf2::TransformException &) {
        pub_drive_->publish(ackermann_msgs::msg::AckermannDrive());    // wait for localization
        return;
      }
      const double yaw = 2 * std::atan2(tf.transform.rotation.z, tf.transform.rotation.w);
      const double dx = cone_x_ - tf.transform.translation.x, dy = cone_y_ - tf.transform.translation.y;
      cx_ = std::cos(yaw) * dx + std::sin(yaw) * dy;
      cy_ = -std::sin(yaw) * dx + std::cos(yaw) * dy;
      history_.push_back({now, cx_, cy_, psi_, vx_, vy_});
      pub_drive_->publish(ackermann_msgs::msg::AckermannDrive());
      return;
    }
    // the car moved since the last step: the cone goes back by the rear axle velocity and turns
    // against the yaw rate. In the run-up the tires grip, in the entry the wheels spin: the
    // velocity from the cone
    const double dt = (now - history_.back().t).seconds();
    double vx = vx_ekf_, vy = 0.0;
    if (drifting_) {
      vx = vx_;
      vy = vy_ - lr_ * r_;
    } else if (kicking_) {
      vx = cone_vx_;
      vy = cone_vy_ - lr_ * r_;
    }
    const double x = cx_ - vx * dt, y = cy_ - vy * dt, turn = r_ * dt;
    cx_ = std::cos(turn) * x + std::sin(turn) * y;
    cy_ = -std::sin(turn) * x + std::cos(turn) * y;
    psi_ += turn;
    history_.push_back({now, cx_, cy_, psi_, vx_, vy_});
    if (history_.size() > 25) {
      history_.pop_front();
    }

    // before moving: the cone in three scans, and the friction node up (3 s to start on the Pi)
    if (!launched_ && (scans_ < 3 || count_publishers("drift/mu") == 0)) {
      pub_drive_->publish(ackermann_msgs::msg::AckermannDrive());
      return;
    }
    launched_ = true;
    if ((now - last_cone_).seconds() > CONE_TIMEOUT) {
      stop("cone lost");
      return;
    }
    if (!drifting_) {
      // the run-up, the wheel speed ramping up: pure pursuit on the line of the start, from how
      // far the cone is from where it should be (a radius to the left) and the yaw since
      const double offset = radius_ - (std::sin(psi_) * cx_ + std::cos(psi_) * cy_);
      const double ty = -std::sin(psi_) * LOOKAHEAD - std::cos(psi_) * offset;
      if (!kicking_) {
        d_cmd_ = std::atan(2 * wheelbase_ * ty / (LOOKAHEAD * LOOKAHEAD + offset * offset));
        u_cmd_ = std::min(launch_speed_, u_cmd_ + 5.0 * in_->Ts[0]);
        publish(d_cmd_, u_cmd_);
        if (cx_ > HANDOVER) {
          return;
        }
        kicking_ = true;
        kick_start_ = now;
      }
      publish(full_lock_, KICK_SPEED);
      if (r_ < KICK_YAW_RATE) {
        if ((now - kick_start_).seconds() > KICK_TIMEOUT) {
          stop("no drift after the entry");
        }
        return;
      }
      drifting_ = true;
      drift_start_ = now;
      // the car as the cone saw it last, else the donut of the model
      const bool fresh = scans_ > 1 && (now - cone_v_time_).seconds() < 0.3;
      vx_ = fresh ? cone_vx_ : donut_vx_;
      vy_ = fresh ? cone_vy_ : donut_vy_;
      d_ = d_cmd_ = full_lock_;
      u_cmd_ = v_max_;
    }
    const double n = radius_ - std::hypot(cx_, cy_), e = std::atan2(cx_, cy_);
    if ((now - drift_start_).seconds() > hold_) {
      stop("drift held " + std::to_string(hold_) + " s");
      return;
    }
    if (std::abs(n) > max_error_) {
      stop(std::to_string(n) + " m off the circle");
      return;
    }

    // s = 0: the circle is the same everywhere
    double x0[DRIFT_NX] = {0, n, e, vx_, vy_, r_, d_, u_, d_cmd_, u_cmd_};
    double x1[DRIFT_NX];
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

    ocp_nlp_out_get(config_, dims_, out_, 1, "x", x1);
    vx_ = x1[3];
    vy_ = x1[4];
    d_ = x1[6];
    d_cmd_ = x1[8];
    u_cmd_ = x1[9];
    publish(d_cmd_, u_cmd_);
    std_msgs::msg::Float32 st;
    st.data = solve_time * 1e3;
    pub_solve_time_->publish(st);
  }

  double wheel_radius_, wheelbase_, lr_, v_max_, cone_x_, cone_y_, cone_radius_, launch_speed_, hold_, mu_scale_, max_error_,
    radius_, donut_vx_, donut_vy_, full_lock_;
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
  double cx_ = 0, cy_ = 0, psi_ = 0;    // the cone from the rear axle, the yaw since the start
  double last_cx_ = 0, last_cy_ = 0, last_psi_ = 0;    // where the last scan saw the cone
  double cone_vx_ = 0, cone_vy_ = 0;   // the velocity from the last two scans
  rclcpp::Time cone_v_time_{0, 0, RCL_ROS_TIME};
  std::deque<Estimate> history_;       // the last 0.5 s of them
  double lidar_x_ = NAN;
  rclcpp::Time last_cone_{0, 0, RCL_ROS_TIME};
  int scans_ = 0;
  bool launched_ = false, kicking_ = false, drifting_ = false, solved_ = false;
  rclcpp::Time kick_start_, drift_start_;
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

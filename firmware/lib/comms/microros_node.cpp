#include "microros_node.h"

#include "config.h"
#if USE_MICROROS

#include "globals.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <ackermann_msgs/msg/ackermann_drive.h>

#include <math.h>

namespace comms
{
    static rclc_support_t support;
    static rcl_allocator_t allocator;
    static rcl_node_t node;
    static rclc_executor_t executor;

    static rcl_publisher_t pub_joint;
    static rcl_publisher_t pub_imu;
    static rcl_publisher_t pub_steering;
    static rcl_subscription_t sub_cmdvel;

    static rcl_subscription_t sub_arm;
    static rcl_subscription_t sub_drive;
    static ackermann_msgs__msg__AckermannDrive msg_drive;
    static std_msgs__msg__Bool msg_arm;

    static sensor_msgs__msg__JointState msg_joint;
    static sensor_msgs__msg__Imu msg_imu;
    static std_msgs__msg__Float32 msg_steering;
    static geometry_msgs__msg__Twist msg_cmdvel;

    static double joint_positions[2];
    static double joint_velocities[2];

    static bool rcl_ok(rcl_ret_t rc) { return rc == RCL_RET_OK; }

    // agent (Pi) clock once rmw_uros_sync_session has run on this session
    static void set_stamp(builtin_interfaces__msg__Time &stamp)
    {
        const int64_t ns = rmw_uros_epoch_nanos();
        stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
        stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    }

    static void cmdvel_callback(const void *msgin)
    {
        const geometry_msgs__msg__Twist *m = static_cast<const geometry_msgs__msg__Twist *>(msgin);

        const float v = static_cast<float>(m->linear.x);
        const float omega = static_cast<float>(m->angular.z);

        float steering_deg = 0.0f;
        if (fabsf(v) > 0.05f)
        {
            const float delta_rad = atanf(omega * WHEELBASE_M / v);
            steering_deg = delta_rad * 180.0f / static_cast<float>(M_PI);
        }
        g_servo.setAngle(steering_deg);

        g_vehicle_state.wheel_left.velocity_setpoint_mps = v;
        g_vehicle_state.wheel_right.velocity_setpoint_mps = v;

        g_safety.notifyCommand(millis());
    }

    static void drive_callback(const void *msgin)
    {
        const ackermann_msgs__msg__AckermannDrive *m =
            static_cast<const ackermann_msgs__msg__AckermannDrive *>(msgin);

        float steering_deg = m->steering_angle * 180.0f / static_cast<float>(M_PI);
        if (steering_deg > SERVO_ANGLE_MAX_DEG)
            steering_deg = SERVO_ANGLE_MAX_DEG;
        if (steering_deg < SERVO_ANGLE_MIN_DEG)
            steering_deg = SERVO_ANGLE_MIN_DEG;
        g_servo.setAngle(steering_deg);

        const float v = m->speed;
        g_vehicle_state.wheel_left.velocity_setpoint_mps = v;
        g_vehicle_state.wheel_right.velocity_setpoint_mps = v;

        g_safety.notifyCommand(millis());
    }

    static void arm_callback(const void *msgin)
    {
        const std_msgs__msg__Bool *m = static_cast<const std_msgs__msg__Bool *>(msgin);
        if (m->data)
        {
            g_safety.clearEmergency();
            g_safety.arm();
        }
        else
        {
            g_safety.disarm();
        }
    }

    void MicroRosNode::begin()
    {
        set_microros_serial_transports(Serial);
        delay(2000);
    }

    bool MicroRosNode::createEntities()
    {
        allocator = rcl_get_default_allocator();

        if (!rcl_ok(rclc_support_init(&support, 0, NULL, &allocator)))
            return false;

        if (!rcl_ok(rclc_node_init_default(&node, "ackermann_firmware", "", &support)))
            return false;

        if (!rcl_ok(rclc_publisher_init_default(
                &pub_joint, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                "joint_states")))
            return false;

        if (!rcl_ok(rclc_publisher_init_default(
                &pub_imu, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                "imu/data_raw")))
            return false;

        if (!rcl_ok(rclc_publisher_init_default(
                &pub_steering, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                "steering_angle")))
            return false;

        if (!rcl_ok(rclc_subscription_init_default(
                &sub_cmdvel, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                "cmd_vel")))
            return false;

        if (!rcl_ok(rclc_subscription_init_default(
                &sub_arm, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                "arm")))
            return false;

        if (!rcl_ok(rclc_subscription_init_default(
                &sub_drive, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(ackermann_msgs, msg, AckermannDrive),
                "drive")))
            return false;

        if (!rcl_ok(rclc_executor_init(&executor, &support.context, 3, &allocator)))
            return false;
        if (!rcl_ok(rclc_executor_add_subscription(
                &executor, &sub_cmdvel, &msg_cmdvel,
                &cmdvel_callback, ON_NEW_DATA)))
            return false;
        if (!rcl_ok(rclc_executor_add_subscription(
                &executor, &sub_arm, &msg_arm,
                &arm_callback, ON_NEW_DATA)))
            return false;
        if (!rcl_ok(rclc_executor_add_subscription(
                &executor, &sub_drive, &msg_drive,
                &drive_callback, ON_NEW_DATA)))
            return false;

        msg_joint.position.data = joint_positions;
        msg_joint.position.size = 2;
        msg_joint.position.capacity = 2;
        msg_joint.velocity.data = joint_velocities;
        msg_joint.velocity.size = 2;
        msg_joint.velocity.capacity = 2;

        rosidl_runtime_c__String__assign(&msg_imu.header.frame_id, "imu_link");
        msg_imu.orientation_covariance[0] = -1.0; // no orientation (REP 145)
        for (int i = 0; i < 3; i++)
        {
            msg_imu.angular_velocity_covariance[4 * i] = IMU_GYRO_VAR;
            msg_imu.linear_acceleration_covariance[4 * i] = IMU_ACCEL_VAR;
        }

        return true;
    }

    void MicroRosNode::destroyEntities()
    {
        // the agent may be gone: don't wait for it to confirm each deletion
        rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
        (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

        rcl_publisher_fini(&pub_joint, &node);
        rcl_publisher_fini(&pub_imu, &node);
        rcl_publisher_fini(&pub_steering, &node);
        rcl_subscription_fini(&sub_cmdvel, &node);
        rcl_subscription_fini(&sub_arm, &node);
        rcl_subscription_fini(&sub_drive, &node);
        rclc_executor_fini(&executor);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
    }

    void MicroRosNode::publishJointStates()
    {
        set_stamp(msg_joint.header.stamp);

        const float r = WHEEL_RADIUS_M;
        joint_positions[0] =
            (double)g_encoder_left.getCount() / ENCODER_TICKS_PER_REV * 2.0 * M_PI;
        joint_positions[1] =
            (double)g_encoder_right.getCount() / ENCODER_TICKS_PER_REV * 2.0 * M_PI;
        joint_velocities[0] = g_vehicle_state.wheel_left.velocity_mps / r;
        joint_velocities[1] = g_vehicle_state.wheel_right.velocity_mps / r;

        rcl_publish(&pub_joint, &msg_joint, NULL);
    }

    void MicroRosNode::publishImu()
    {
        const ImuData &imu = g_vehicle_state.imu;

        set_stamp(msg_imu.header.stamp);

        msg_imu.orientation.w = imu.qw;
        msg_imu.orientation.x = imu.qx;
        msg_imu.orientation.y = imu.qy;
        msg_imu.orientation.z = imu.qz;

        msg_imu.angular_velocity.x = imu.gyro_x;
        msg_imu.angular_velocity.y = imu.gyro_y;
        msg_imu.angular_velocity.z = imu.gyro_z;

        msg_imu.linear_acceleration.x = imu.lin_acc_x;
        msg_imu.linear_acceleration.y = imu.lin_acc_y;
        msg_imu.linear_acceleration.z = imu.lin_acc_z;

        rcl_publish(&pub_imu, &msg_imu, NULL);
    }

    void MicroRosNode::publishSteering()
    {
        msg_steering.data = g_servo.getAngle() * static_cast<float>(M_PI) / 180.0f;
        rcl_publish(&pub_steering, &msg_steering, NULL);
    }

    void MicroRosNode::spinOnce()
    {
        const uint32_t now = millis();

        switch (state_)
        {
        case AgentState::WAITING:
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
                state_ = AgentState::AVAILABLE;
            break;

        case AgentState::AVAILABLE:
            if (createEntities())
            {
                rmw_uros_sync_session(100);
                last_sync_ms_ = now;
                state_ = AgentState::CONNECTED;
            }
            else
            {
                destroyEntities();
                state_ = AgentState::WAITING;
            }
            break;

        case AgentState::CONNECTED:
            if (now - last_ping_ms_ >= 500)
            {
                last_ping_ms_ = now;
                if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK)
                {
                    state_ = AgentState::DISCONNECTED;
                    break;
                }
            }

            // the ESP32 crystal drifts against the Pi clock
            if (now - last_sync_ms_ >= 10000)
            {
                last_sync_ms_ = now;
                rmw_uros_sync_session(100);
            }

            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

            if (now - last_publish_ms_ >= 20)
            {
                last_publish_ms_ = now;
                publishJointStates();
                publishImu();
                publishSteering();
            }
            break;

        case AgentState::DISCONNECTED:
            destroyEntities();
            state_ = AgentState::WAITING;
            break;
        }
    }
}
#endif
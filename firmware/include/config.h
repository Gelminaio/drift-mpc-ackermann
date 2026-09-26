#pragma once

// Set to 1 to use micro-ROS, 0 to use the legacy serial command parser (just for debug)
#ifndef USE_MICROROS
#define USE_MICROROS 1
#endif

#if USE_MICROROS
#define LOG(...) ((void)0)
#define LOGLN(...) ((void)0)
#else
#define LOG(...) Serial.printf(__VA_ARGS__)
#define LOGLN(...) Serial.printf(__VA_ARGS__)
#endif

// config.h — pinout, hardware constants, timings.

// I2C (BNO085 IMU)
constexpr int PIN_I2C_SDA = 21;
constexpr int PIN_I2C_SCL = 22;
constexpr uint32_t I2C_FREQUENCY_HZ = 400000;
constexpr uint8_t BNO085_I2C_ADDRESS = 0x4A;

// Right BTS7960
constexpr int PIN_MOTOR_R_PWM_FWD = 25;
constexpr int PIN_MOTOR_R_PWM_REV = 26;

// Left BTS7960
constexpr int PIN_MOTOR_L_PWM_FWD = 27;
constexpr int PIN_MOTOR_L_PWM_REV = 14;

// Encoders
constexpr int PIN_ENCODER_R_A = 34;
constexpr int PIN_ENCODER_R_B = 35;
constexpr int PIN_ENCODER_L_A = 36;
constexpr int PIN_ENCODER_L_B = 39;

// Encoder driver constants
constexpr uint16_t ENCODER_GLITCH_FILTER_NS = 1000; // ignore pulses <1us (motor noise rejection)
constexpr uint32_t ENCODER_VELOCITY_WINDOW_MS = 40; // velocity computed over this window

// PCNT unit assignments (ESP32 has 8 PCNT units: 0-7)
constexpr int ENCODER_R_PCNT_UNIT = 0;
constexpr int ENCODER_L_PCNT_UNIT = 1;
constexpr float SERVO_DIRECTION = -1.0f; // mirror, positive = left

// Steering servo
constexpr int PIN_SERVO_STEERING = 33;
constexpr float SERVO_ANGLE_MAX_DEG = 30.0f;
constexpr float SERVO_ANGLE_MIN_DEG = -30.0f;

// PWM pulse-width calibration in microseconds (established through some tests)
constexpr uint32_t SERVO_PULSE_CENTER_US = 1460; // straight wheels (0° degrees)
constexpr uint32_t SERVO_PULSE_LEFT_US = 960;    // full left  (-30 degrees)
constexpr uint32_t SERVO_PULSE_RIGHT_US = 1960;  // full right (+30 degrees)

// LEDC channel assignment
constexpr int SERVO_LEDC_CHANNEL = 0;
constexpr int MOTOR_R_FWD_LEDC_CHANNEL = 2;
constexpr int MOTOR_R_REV_LEDC_CHANNEL = 3;
constexpr int MOTOR_L_FWD_LEDC_CHANNEL = 4;
constexpr int MOTOR_L_REV_LEDC_CHANNEL = 5;

// Motor control constants
constexpr int16_t MOTOR_DUTY_MAX = 1023;
constexpr int16_t MOTOR_DUTY_MIN = -1023;
constexpr uint32_t MOTOR_DEAD_TIME_MS = 2;

// LED
constexpr int PIN_LED = 2;

// PWM (LEDC) settings
constexpr uint32_t MOTOR_PWM_FREQ_HZ = 20000;
constexpr uint8_t MOTOR_PWM_RESOLUTION = 10;
constexpr uint32_t SERVO_PWM_FREQ_HZ = 50;
constexpr uint8_t SERVO_PWM_RESOLUTION = 16;

// Vehicle physical parameters
// wheelbase, track and radius: same as ros2_ws/src/ackermann_description/config/vehicle_params.yaml
constexpr float WHEEL_RADIUS_M = 0.0299f;  // slick tires, driven straight runs (step 5.5)
constexpr float WHEELBASE_M = 0.173f;      // measured
constexpr float TRACK_WIDTH_M = 0.1745f;   // (specific for my motor)
constexpr int ENCODER_TICKS_PER_REV = 660; // measured (specific for my motor)
constexpr float GEAR_RATIO = 1.0f;

// IMU driver constants
constexpr uint16_t IMU_REPORT_INTERVAL_MS = 5; // 200hz

// IMU noise while driving on tiles, steady steps at 0.6-0.8 m/s: gyro std 0.055-0.086 rad/s,
// accel std 1.9-3.0 m/s2. At rest 0.002 and 0.05: it is vibration, not the sensor (step 6.2)
constexpr double IMU_GYRO_VAR = 0.07 * 0.07;  // (rad/s)^2
constexpr double IMU_ACCEL_VAR = 2.5 * 2.5;   // (m/s^2)^2

// PID velocity controller — initial gains (refined via tuning)
constexpr float PID_KP_INITIAL = 1500.0f; // tuned manually, validated 0.2/0.5/1.0 m/s (rise time of about 260ms, almost 0 overshoot, steady state of +- 0,4%)
constexpr float PID_KI_INITIAL = 6000.0f;
constexpr float PID_KD_INITIAL = 30.0f; // small D on EMA-filtered measurement

constexpr float PID_INTEGRAL_CLAMP = MOTOR_DUTY_MAX / PID_KI_INITIAL; // Ki * clamp = full duty

constexpr int16_t PID_OUTPUT_MIN = -1023;
constexpr int16_t PID_OUTPUT_MAX = +1023;

constexpr float PID_DEADBAND_MPS = 0.02f;

// Control & safety limits
constexpr float VEHICLE_MAX_LINEAR_VEL_MPS = 1.3f; // measured 1,37 m/s at full duty (raised vehicle)
constexpr float VEHICLE_MAX_ANGULAR_VEL_RPS = 4.0f;

// Safety layer constants
constexpr uint32_t SAFETY_CMD_TIMEOUT_MS = 500;        // if no command for this long => soft-stop
constexpr float SAFETY_SOFTSTOP_RAMP_MPS_PER_S = 2.0f; // decel rate during a soft-stop

// Stall detection
constexpr float SAFETY_STALL_SETPOINT_MPS = 0.3f; // setpoint above which we expect motion
constexpr float SAFETY_STALL_ACTUAL_MPS = 0.05f;  // below this = "not moving"
constexpr uint32_t SAFETY_STALL_TIME_MS = 500;    // sustained stall before e-stop

// Hardware Task Watchdog
constexpr uint32_t SAFETY_TWDT_TIMEOUT_S = 1; // chip resets if a task starves > 1s

// Task timing (periods in ms)
constexpr uint32_t PERIOD_MOTOR_CONTROL_MS = 10;
constexpr uint32_t PERIOD_IMU_MS = 5; // polling frequency (diffent from IMU_REPORT_INTERVAL_MS)
constexpr uint32_t PERIOD_COMMS_MS = 20;
constexpr uint32_t PERIOD_SAFETY_MS = 20;
constexpr uint32_t PERIOD_TELEMETRY_MS = 20; // 50hz (temp, will be changed)

// Task stack sizes (bytes) and priorities (0 = lowest, 24 = highest)
constexpr uint32_t STACK_MOTOR_CONTROL = 4096;
constexpr uint32_t STACK_IMU = 4096;
constexpr uint32_t STACK_COMMS = 16384; // micro-ROS needs a large stack (previous: 8192)
constexpr uint32_t STACK_SAFETY = 2048;
constexpr uint32_t STACK_TELEMETRY = 4096;

constexpr UBaseType_t PRIO_SAFETY = 5;
constexpr UBaseType_t PRIO_MOTOR_CONTROL = 4;
constexpr UBaseType_t PRIO_IMU = 3;
constexpr UBaseType_t PRIO_COMMS = 2;
constexpr UBaseType_t PRIO_TELEMETRY = 1;

// Core assignment, ESP32 has 2 cores: 0 and 1
constexpr BaseType_t CORE_CONTROL = 1; // real-time (critical)
constexpr BaseType_t CORE_COMMS = 0;   // communications and telemetry

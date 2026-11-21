// Unit tests of control::PIDController, the wheel speed loop (100 Hz on the ESP32).
// Run on the PC: pio test -e native
#include <unity.h>

#include "pid_controller.h" // first: it brings Arduino.h, which config.h relies on
#include "config.h"

using control::PIDController;

static constexpr float DT = PERIOD_MOTOR_CONTROL_MS / 1000.0f;

void setUp(void) {}
void tearDown(void) {}

// P term alone: output = Kp * error, nothing carried over between steps
void test_proportional_term(void)
{
    PIDController pid(2.0f, 0.0f, 0.0f, DT, 10.0f, -100.0f, 100.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, pid.compute(1.0f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f, pid.compute(0.0f, 0.5f));
}

// The output never leaves [output_min, output_max]: the PWM duty range of the BTS7960
void test_output_is_clamped_to_the_duty_range(void)
{
    PIDController pid(1000.0f, 0.0f, 0.0f, DT, 10.0f, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    TEST_ASSERT_EQUAL_FLOAT(PID_OUTPUT_MAX, pid.compute(5.0f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(PID_OUTPUT_MIN, pid.compute(-5.0f, 0.0f));
}

// I term: a constant error adds error * dt to the integral at every step
void test_integral_accumulates_error_times_dt(void)
{
    PIDController pid(0.0f, 1.0f, 0.0f, DT, 10.0f, -100.0f, 100.0f);
    for (int i = 0; i < 10; ++i)
        pid.compute(1.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 10 * 1.0f * DT, pid.getIntegral());
}

// Anti-windup: after a long saturation the integral sits at the clamp, so when the error
// changes sign the output follows within clamp / (|error| * dt) steps, not after seconds
void test_anti_windup_bounds_the_recovery(void)
{
    const float clamp = 0.5f;
    PIDController pid(0.0f, 100.0f, 0.0f, DT, clamp, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    for (int i = 0; i < 1000; ++i) // 10 s of error, e.g. a wheel held by hand
        pid.compute(1.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(clamp, pid.getIntegral());

    int steps = 0;
    while (pid.compute(-1.0f, 0.0f) > 0.0f && steps < 1000)
        ++steps;
    TEST_ASSERT_LESS_OR_EQUAL_INT(51, steps); // 0.5 / (1 * 0.01) = 50 steps, 0.5 s

    for (int i = 0; i < 1000; ++i)
        pid.compute(-1.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(-clamp, pid.getIntegral());
}

// D term on the measurement: a setpoint step gives no derivative kick, a moving
// measurement does (d = -Kd * dmeasurement / dt)
void test_derivative_acts_on_the_measurement(void)
{
    PIDController pid(0.0f, 0.0f, 0.5f, DT, 10.0f, -100.0f, 100.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.compute(0.0f, 1.0f)); // first step: no previous measurement
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.compute(5.0f, 1.0f)); // setpoint jumps: no kick
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -5.0f, pid.compute(5.0f, 1.1f));
}

// reset() clears the integral and the previous measurement: the first step after it has
// no derivative term even if the measurement jumped meanwhile
void test_reset_clears_the_state(void)
{
    PIDController pid(0.0f, 1.0f, 0.5f, DT, 10.0f, -100.0f, 100.0f);
    for (int i = 0; i < 5; ++i)
        pid.compute(1.0f, 0.0f);
    pid.reset();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.getIntegral());
    // only the I term of this step: 1.0 * (1 - 3) * dt
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -2.0f * DT, pid.compute(1.0f, 3.0f));
}

// setGains() retunes at run time, as the live tuning command does
void test_gains_can_be_changed_at_run_time(void)
{
    PIDController pid(1.0f, 0.0f, 0.0f, DT, 10.0f, -100.0f, 100.0f);
    pid.setGains(3.0f, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, pid.getKp());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.0f, pid.compute(1.0f, 0.0f));
}

// config.h sizes the integral clamp so that the I term alone reaches full duty, no more
void test_integral_clamp_matches_full_duty(void)
{
    TEST_ASSERT_FLOAT_WITHIN(0.5f, (float)MOTOR_DUTY_MAX, PID_KI_INITIAL * PID_INTEGRAL_CLAMP);
}

// The tuned gains of config.h on a first-order wheel with dry friction (rough numbers:
// 1.37 m/s at full duty as measured, 150 ms time constant, 0.5 m/s^2 of friction)
static float run_wheel(PIDController &pid, float setpoint, float seconds)
{
    const float km = 1.37f / MOTOR_DUTY_MAX, tau = 0.15f, friction = 0.5f;
    float v = 0.0f;
    const int steps = (int)(seconds / DT + 0.5f);
    for (int k = 0; k < steps; ++k)
    {
        const float duty = pid.compute(setpoint, v);
        v += DT * ((km * duty - v) / tau - (v > 0.0f ? friction : 0.0f));
    }
    return v;
}

// The integral removes the steady-state error that friction leaves to a P controller
void test_tuned_gains_reach_the_setpoint_despite_friction(void)
{
    PIDController pid(PID_KP_INITIAL, PID_KI_INITIAL, PID_KD_INITIAL, DT, PID_INTEGRAL_CLAMP,
                      PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.5f, run_wheel(pid, 0.5f, 2.0f)); // within 1 % after 2 s

    PIDController p_only(PID_KP_INITIAL, 0.0f, PID_KD_INITIAL, DT, PID_INTEGRAL_CLAMP,
                         PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.1f, 0.5f - run_wheel(p_only, 0.5f, 2.0f)); // 20 % short or more
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_proportional_term);
    RUN_TEST(test_output_is_clamped_to_the_duty_range);
    RUN_TEST(test_integral_accumulates_error_times_dt);
    RUN_TEST(test_anti_windup_bounds_the_recovery);
    RUN_TEST(test_derivative_acts_on_the_measurement);
    RUN_TEST(test_reset_clears_the_state);
    RUN_TEST(test_gains_can_be_changed_at_run_time);
    RUN_TEST(test_integral_clamp_matches_full_duty);
    RUN_TEST(test_tuned_gains_reach_the_setpoint_despite_friction);
    return UNITY_END();
}

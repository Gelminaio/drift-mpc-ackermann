// Unit tests of control::SafetySupervisor, the state machine that decides whether the
// motors may move (safety task, 50 Hz on the ESP32). Run on the PC: pio test -e native
#include <unity.h>

#include "safety_supervisor.h" // first: it brings Arduino.h, which config.h relies on
#include "config.h"

using control::SafetySupervisor;

void setUp(void) {}
void tearDown(void) {}

static void step(SafetySupervisor &s, uint32_t t, float left, float left_sp, float right, float right_sp)
{
    s.notifyCommand(t); // commands keep arriving: only the stall logic is under test
    s.update(t, left, left_sp, right, right_sp);
}

// After boot the motors are off until someone arms them
void test_boots_disarmed(void)
{
    SafetySupervisor s;
    TEST_ASSERT_TRUE(s.state() == SafetyState::DISARMED);
    TEST_ASSERT_FALSE(s.motionAllowed());
}

void test_arm_and_disarm(void)
{
    SafetySupervisor s;
    s.arm();
    TEST_ASSERT_TRUE(s.state() == SafetyState::ARMED);
    TEST_ASSERT_TRUE(s.motionAllowed());
    s.disarm();
    TEST_ASSERT_TRUE(s.state() == SafetyState::DISARMED);
    TEST_ASSERT_FALSE(s.motionAllowed());
}

// No command for more than SAFETY_CMD_TIMEOUT_MS (the Pi crashed, the cable came off):
// SOFT_STOP, motion blocked; commands arriving again re-arm it
void test_command_timeout_soft_stops_and_resumes(void)
{
    SafetySupervisor s;
    s.arm();
    s.notifyCommand(1000);
    s.update(1000 + SAFETY_CMD_TIMEOUT_MS, 0.0f, 0.0f, 0.0f, 0.0f); // at the limit: still armed
    TEST_ASSERT_TRUE(s.state() == SafetyState::ARMED);
    s.update(1000 + SAFETY_CMD_TIMEOUT_MS + 1, 0.0f, 0.0f, 0.0f, 0.0f);
    TEST_ASSERT_TRUE(s.state() == SafetyState::SOFT_STOP);
    TEST_ASSERT_FALSE(s.motionAllowed());
    TEST_ASSERT_EQUAL_STRING("command timeout", s.reason());

    s.notifyCommand(2000);
    s.update(2000, 0.0f, 0.0f, 0.0f, 0.0f);
    TEST_ASSERT_TRUE(s.state() == SafetyState::ARMED);
}

// A wheel commanded to move that does not move for more than SAFETY_STALL_TIME_MS
// (blocked, motor unplugged): EMERGENCY, and the reason says which wheel
void test_stall_trips_the_emergency(void)
{
    SafetySupervisor s;
    s.arm();
    uint32_t t = 1000;
    for (; t <= 1000 + SAFETY_STALL_TIME_MS; t += PERIOD_SAFETY_MS)
    {
        step(s, t, 0.0f, 0.5f, 0.5f, 0.5f); // left wheel stuck at 0 with 0.5 m/s asked
        TEST_ASSERT_TRUE(s.state() == SafetyState::ARMED);
    }
    step(s, t, 0.0f, 0.5f, 0.5f, 0.5f);
    TEST_ASSERT_TRUE(s.state() == SafetyState::EMERGENCY);
    TEST_ASSERT_FALSE(s.motionAllowed());
    TEST_ASSERT_EQUAL_STRING("stall: left wheel", s.reason());
}

void test_stall_reports_the_right_wheel(void)
{
    SafetySupervisor s;
    s.arm();
    for (uint32_t t = 1000; t <= 1000 + SAFETY_STALL_TIME_MS + PERIOD_SAFETY_MS; t += PERIOD_SAFETY_MS)
        step(s, t, 0.5f, 0.5f, 0.0f, -0.5f); // right wheel stuck, reverse asked
    TEST_ASSERT_TRUE(s.state() == SafetyState::EMERGENCY);
    TEST_ASSERT_EQUAL_STRING("stall: right wheel", s.reason());
}

// EMERGENCY latches: arm() is refused until the emergency is cleared, and clearing it
// leaves the car disarmed, not moving
void test_emergency_latches_until_cleared(void)
{
    SafetySupervisor s;
    s.arm();
    for (uint32_t t = 1000; t <= 1000 + SAFETY_STALL_TIME_MS + PERIOD_SAFETY_MS; t += PERIOD_SAFETY_MS)
        step(s, t, 0.0f, 0.5f, 0.5f, 0.5f);
    TEST_ASSERT_TRUE(s.state() == SafetyState::EMERGENCY);

    s.arm();
    TEST_ASSERT_TRUE(s.state() == SafetyState::EMERGENCY);
    s.clearEmergency();
    TEST_ASSERT_TRUE(s.state() == SafetyState::DISARMED);
    TEST_ASSERT_FALSE(s.motionAllowed());
    s.arm();
    TEST_ASSERT_TRUE(s.state() == SafetyState::ARMED);
}

// A wheel slow to start (stalled for less than the limit) is forgiven, and the timer
// starts again from zero at the next stall
void test_short_stalls_are_forgiven(void)
{
    SafetySupervisor s;
    s.arm();
    step(s, 1000, 0.0f, 0.5f, 0.5f, 0.5f); // stall starts
    step(s, 1300, 0.4f, 0.5f, 0.5f, 0.5f); // moving: timer reset
    step(s, 1600, 0.0f, 0.5f, 0.5f, 0.5f); // stalled again, new start
    step(s, 2000, 0.0f, 0.5f, 0.5f, 0.5f); // 400 ms of this stall: under the limit
    TEST_ASSERT_TRUE(s.state() == SafetyState::ARMED);
}

// Below SAFETY_STALL_SETPOINT_MPS a wheel may stand still (deadband, static friction):
// that is never a stall
void test_slow_setpoints_are_not_stalls(void)
{
    SafetySupervisor s;
    s.arm();
    for (uint32_t t = 1000; t < 3000; t += PERIOD_SAFETY_MS)
        step(s, t, 0.0f, 0.2f, 0.0f, -0.2f);
    TEST_ASSERT_TRUE(s.state() == SafetyState::ARMED);
}

// Disarmed, nothing can move, so stalls and timeouts change nothing
void test_disarmed_ignores_stalls_and_timeouts(void)
{
    SafetySupervisor s;
    s.notifyCommand(1000);
    for (uint32_t t = 1000; t < 5000; t += PERIOD_SAFETY_MS)
        s.update(t, 0.0f, 1.0f, 0.0f, 1.0f);
    TEST_ASSERT_TRUE(s.state() == SafetyState::DISARMED);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_boots_disarmed);
    RUN_TEST(test_arm_and_disarm);
    RUN_TEST(test_command_timeout_soft_stops_and_resumes);
    RUN_TEST(test_stall_trips_the_emergency);
    RUN_TEST(test_stall_reports_the_right_wheel);
    RUN_TEST(test_emergency_latches_until_cleared);
    RUN_TEST(test_short_stalls_are_forgiven);
    RUN_TEST(test_slow_setpoints_are_not_stalls);
    RUN_TEST(test_disarmed_ignores_stalls_and_timeouts);
    return UNITY_END();
}

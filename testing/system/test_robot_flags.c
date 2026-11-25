/* Host test for robot_flags.h / robot_faults.h (packed flag word with a lock
 * around every write).
 * gcc -std=c11 -Wall -Wextra -Iapp_program testing/system/test_robot_flags.c -o
 * /tmp/rf && /tmp/rf */
#include "../../app_program/barrc_types.h"
#include <stdio.h>

static int32_t g_lock_depth;
static int32_t g_max_depth;
static int32_t g_locks;
static int32_t g_unlocks;
#define ROBOT_FLAGS_IRQ_T int
#define ROBOT_FLAGS_LOCK(s) do { (s) = g_lock_depth++; g_locks++; if \
    (g_lock_depth > g_max_depth) g_max_depth = g_lock_depth; } while (0)
#define ROBOT_FLAGS_UNLOCK(s) do { (void)(s); g_lock_depth--; g_unlocks++; } \
    while (0)
#include "robot_flags.h"
#include "robot_faults.h"

static int32_t s_failures;
#define CHECK(cond, text) do { if (cond) { printf("  PASS  %s\n", text); } \
    else { printf("  FAIL  %s\n", text); s_failures++; } } while (0)

int main (void)
{
    robot_flags_t flags = 0;
    robot_flags_t faults = 0;

    robot_flags_set(&flags, 1u << 0);
    robot_flags_set(&flags, 1u << 3);
    CHECK((robot_flags_test(&flags, 1u << 0))
          && (robot_flags_test(&flags, 1u << 3)), "set two bits");
    CHECK(!robot_flags_test(&flags, 1u << 1), "other bits stay clear");

    robot_flags_clear(&flags, 1u << 0);
    CHECK((!robot_flags_test(&flags, 1u << 0))
          && (robot_flags_test(&flags, 1u << 3)),
          "clear one bit, the other stays");

    robot_flags_assign(&flags, 1u << 5, true);
    robot_flags_assign(&flags, 1u << 3, false);
    CHECK((robot_flags_test(&flags, 1u << 5))
          && (!robot_flags_test(&flags, 1u << 3)), "assign on and off");

    CHECK(true == robot_flags_test_and_clear(&flags, 1u << 5),
          "test_and_clear returns true when set");
    CHECK(!robot_flags_test(&flags, 1u << 5), "test_and_clear cleared the bit");
    CHECK(false == robot_flags_test_and_clear(&flags, 1u << 5),
          "test_and_clear returns false when clear");

    CHECK(false == robot_flags_test(&flags, (1u << 5) | (1u << 6)),
          "test with a mask of several bits, none set");
    robot_flags_set(&flags, 1u << 6);
    CHECK(true == robot_flags_test(&flags, (1u << 5) | (1u << 6)),
          "test with a mask of several bits, one set");

    robot_flags_update(&flags, 0x0Fu, 0x05u);
    CHECK(0x05u == (flags & 0x0Fu), "update replaces only the masked bits");
    robot_flags_set(&flags, 1u << 6);
    robot_flags_set_nl(&flags, 1u << 7);
    robot_flags_clear_nl(&flags, 1u << 6);
    CHECK((robot_flags_test(&flags, 1u << 7))
          && (!robot_flags_test(&flags, 1u << 6)), "no-lock variants work");

    /* Every locked operation must take and release the lock exactly once, never
     * leaving it held. */
    CHECK(0 == g_lock_depth, "lock is released after every call");
    CHECK(g_locks == g_unlocks, "locks and unlocks are balanced");
    CHECK(1 == g_max_depth, "no nested locking inside one call");

    /* Fault bitmask: independent bits, all fit in 16 bits, each distinct. */
    uint32_t all =
            ROBOT_FAULT_MOTOR_STALL | ROBOT_FAULT_ESTOP | ROBOT_FAULT_ENCODER
            | ROBOT_FAULT_IMU |
            ROBOT_FAULT_SONAR | ROBOT_FAULT_SERVO | ROBOT_FAULT_B5_OTHER
            | ROBOT_FAULT_NAV |
            ROBOT_FAULT_LIFTED | ROBOT_FAULT_LINE_LOST
            | ROBOT_FAULT_WIFI_LOST | ROBOT_FAULT_MQTT_LOST;
    CHECK(all == (1u << ROBOT_FAULT_MASK_BITS) - 1u,
          "12 distinct fault bits, no overlap, no gaps");
    CHECK(all <= 0xFFFFu, "fault mask fits in 16 bits");

    robot_flags_set(&faults, ROBOT_FAULT_MOTOR_STALL);
    robot_flags_set(&faults, ROBOT_FAULT_LIFTED);
    robot_flags_clear(&faults, ROBOT_FAULT_MOTOR_STALL);
    CHECK(ROBOT_FAULT_LIFTED == faults, "faults update independently");

    printf("%s\n", 0 == s_failures ? "ALL PASSED" : "SOME FAILED");
    return 0 == s_failures ? 0 : 1;
}

/*** end of file ***/

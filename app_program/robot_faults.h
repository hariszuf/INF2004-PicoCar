/*
 * robot_faults - one shared fault bitmask for the whole car.
 *
 * Each fault condition is one bit of a single 16-bit mask instead of a separate
 * integer variable. Bits are only ever
 * changed through robot_flags_assign()/robot_flags_set()/robot_flags_clear()
 * (see robot_flags.h), so concurrent
 * updates from different tasks cannot lose each other's bits.
 */
#ifndef ROBOT_FAULTS_H
#define ROBOT_FAULTS_H

#include <stdint.h>

/* motor driven but wheel not turning (Buddy 2), latched */
#define ROBOT_FAULT_MOTOR_STALL (1u << 0)
/* emergency stop requested in motion control (Buddy 2) */
#define ROBOT_FAULT_ESTOP       (1u << 1)
/* encoder fault (reserved for Buddy 2) */
#define ROBOT_FAULT_ENCODER     (1u << 2)
/* IMU not usable (reserved for Buddy 4) */
#define ROBOT_FAULT_IMU         (1u << 3)
/* ultrasonic sensor fault / no echo from the sensor (Buddy 5) */
#define ROBOT_FAULT_SONAR       (1u << 4)
#define ROBOT_FAULT_SERVO       (1u << 5)   /* sonar servo fault (Buddy 5) */
/* other Buddy 5 fault: timeout, no route, feedback, pose, profile, controller
 */
#define ROBOT_FAULT_B5_OTHER    (1u << 6)
/* navigation failed / pose mismatch */
#define ROBOT_FAULT_NAV         (1u << 7)
/* robot lifted or overturned (Buddy 4) */
#define ROBOT_FAULT_LIFTED      (1u << 8)
/* line lost outside recovery (Buddy 3) */
#define ROBOT_FAULT_LINE_LOST   (1u << 9)
#define ROBOT_FAULT_WIFI_LOST   (1u << 10)  /* Wi-Fi link down */
/* MQTT broker / dashboard link down */
#define ROBOT_FAULT_MQTT_LOST   (1u << 11)

#define ROBOT_FAULT_MASK_BITS 12u

#endif /* ROBOT_FAULTS_H */

/*** end of file ***/

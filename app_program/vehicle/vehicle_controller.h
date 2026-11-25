#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

int32_t vehicle_controller_start (bool imu_ready);
void vehicle_controller_disable_imu (void);
void vehicle_controller_execution_status (int32_t * state, int32_t * reason);

/* Shared fault bitmask, one bit per condition (robot_faults.h). */
uint32_t vehicle_fault_mask (void);

bool line_controller_active (void);
void b5_line_command_received (uint32_t command);
void b5_remote_command_intent (uint32_t command);

#endif

/*** end of file ***/

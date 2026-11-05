#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

int32_t vehicle_controller_start(bool imu_ready);
void vehicle_controller_disable_imu(void);

bool line_controller_active(void);
void b5_line_command_received(unsigned command);

#endif

#ifndef BUDDY4_DEVICE_H
#define BUDDY4_DEVICE_H
#include "buddy4_imu.h"
/* Bridge to the professor port's registered iica device. The IMU task owns
 * this context exclusively; each register read is a single T_I2C_EXEC request. */
typedef struct {
    int descriptor;
    uint8_t register_address, slave_address;
    bool pending;
    int last_error;
} buddy4_device_t;
buddy4_bus_t buddy4_device_bus(buddy4_device_t *device);
#endif

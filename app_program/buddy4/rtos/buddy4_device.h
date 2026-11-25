#ifndef BUDDY4_DEVICE_H
#define BUDDY4_DEVICE_H
#include "../../barrc_types.h"
#include "buddy4_imu.h"
/* Bridge to the professor port's registered iica device. The IMU task owns
 * this context exclusively; each register read is a single T_I2C_EXEC request.
 */
typedef struct
{
    int32_t descriptor;
    uint8_t register_address;
    uint8_t slave_address;
    bool pending;
    uint32_t timeout_trace[32];
    int32_t last_error;
    bool last_operation_read;
} buddy4_device_t;
buddy4_bus_t buddy4_device_bus (buddy4_device_t * device);
#endif

/*** end of file ***/

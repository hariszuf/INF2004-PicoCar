#ifndef PICOCAR_BUDDY4_H
#define PICOCAR_BUDDY4_H
#include <tk/tkernel.h>
#include "imu_detect.h"

typedef struct {
    imu_raw_t raw;
    imu_result_t result;
    uint32_t sequence;
    uint8_t valid;
} buddy4_status_t;

ER buddy4_init(void);
ER buddy4_start(void);
/* Latest snapshot for motion-control/telemetry; E_OBJ until calibrated/valid. */
ER buddy4_get_latest(buddy4_status_t *out);
#endif

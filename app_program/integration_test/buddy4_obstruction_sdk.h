#ifndef BUDDY4_OBSTRUCTION_SDK_H
#define BUDDY4_OBSTRUCTION_SDK_H
#include "buddy4_obstruction.h"
#include "buddy4_imu.h"
#include "drive_control.h"

/* SDK-only software freshness qualification. begin/end bound the actual call
 * to motion_update(), NOT an invented encoder acquisition timestamp. Buddy 2
 * exposes neither a sensor timestamp nor encoder-health validity. A dead encoder
 * is indistinguishable from a stationary wheel: assessment remains POSSIBLE.
 * No SDK APIs or motor setters are called here; core can be reused by MTK with
 * a proper native producer. Do not use this adapter on periodically copied status.
 * Tilt evidence uses valid filtered pitch/roll <=3/2 degrees plus flat/level
 * hysteresis state and no motor terrain compensation. An already established
 * observer may pause through <=40 ms of pitch-only excursion; it cannot count
 * motion/slowdown while paused. Roll, terrain and all safety faults cancel.
 * Raw angles must be finite
 * but their instantaneous amplitude is not an orientation veto. Filter lag and
 * shallow ramps remain ambiguous; this is only a possible-obstruction observer. */
typedef struct {
    buddy4_obstruction_t observer;
    motion_status_t status;
    drive_control_t demand;
    bool have_update, pending, timing_ok, has_imu;
    uint32_t skipped;
    uint64_t begin_us, end_us, last_imu_us;
} buddy4_obstruction_sdk_t;

void buddy4_obstruction_sdk_init(buddy4_obstruction_sdk_t *a);
/* Call exactly once after each actual motor update with its before/after times
 * and the command context governing that update. First interval is unqualified. */
void buddy4_obstruction_sdk_note_update(buddy4_obstruction_sdk_t *a,
    const motion_status_t *status,const drive_control_t *demand,
    uint64_t begin_us,uint64_t end_us,uint32_t skipped);
/* Call AFTER serial/safety/mode processing on every loop, including loops with
 * no new motor/IMU poll. A processed S therefore cancels before confirmation.
 * A pending motor observation can wait for the next NEW IMU sample while all
 * existing age/gap/skew limits hold. Waiting polls never advance evidence. */
buddy4_obstruction_output_t buddy4_obstruction_sdk_poll(buddy4_obstruction_sdk_t *a,
    const drive_control_t *control,const buddy4_imu_t *imu,bool usb,uint64_t now_us);
#endif

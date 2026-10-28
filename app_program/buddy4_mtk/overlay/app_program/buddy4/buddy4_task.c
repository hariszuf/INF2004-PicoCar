#include "buddy4_imu.h"
#include "buddy4_device.h"
#include "buddy4_impact.h"
#include "buddy4_motion.h"
#include "buddy4_encoder_yaw.h"
#include <string.h>
#ifdef BUDDY4_MTK_HOST_TEST
#include "tests/mtk_stubs/device_api.h"
#else
#define PROHIBIT_DEF_SIZE_T
#include <tk/tkernel.h>
#undef PROHIBIT_DEF_SIZE_T
#include <tm/tmonitor.h>
#endif
#include "buddy4_task.h"

#define RETRY_US 1000000u
#define IMU_MAX_AGE_US 40000u
#define ENCODER_MAX_AGE_US 100000u
#define TASK_PRIORITY 10
#define TASK_STACK 8192
static ID data_mutex, task_id;
static bool initialised;
static buddy4_device_t device;
static buddy4_imu_t imu;
static buddy4_impact_t impact;
static buddy4_motion_t motion;
static buddy4_encoder_yaw_t yaw;
static buddy4_task_snapshot_t published, next;
static buddy4_task_speed_t mailbox;
static uint64_t retry_us, last_speed_us;
static bool has_speed;

ER buddy4_task_time_us(uint64_t *out)
{
    if (!out) return E_PAR;
    SYSTIM t;
    ER e = tk_get_otm(&t); /* Monotonic operating time, not settable wall time. */
    if (e >= E_OK)
        *out = (((uint64_t)(uint32_t)t.hi << 32) | t.lo) * 1000u;
    return e;
}

ER buddy4_task_submit_speed(const buddy4_task_speed_t *speed)
{
    if (!speed) return E_PAR;
    if (!initialised) return E_OBJ;
    ER e = tk_loc_mtx(data_mutex, TMO_FEVR);
    if (e < E_OK) return e;
    mailbox = *speed;
    return tk_unl_mtx(data_mutex);
}

static bool expired(uint64_t now, uint64_t sample, uint32_t limit)
{
    return now < sample || now - sample > limit;
}

ER buddy4_task_get(buddy4_task_snapshot_t *out)
{
    if (!out) return E_PAR;
    if (!initialised) return E_OBJ;
    ER e = tk_loc_mtx(data_mutex, TMO_FEVR);
    if (e < E_OK) return e;
    *out = published;
    e = tk_unl_mtx(data_mutex);
    if (e < E_OK) return e;
    uint64_t now;
    e = buddy4_task_time_us(&now);
    if (e < E_OK || expired(now, out->sample_us, IMU_MAX_AGE_US)) {
        out->valid = false;
        out->bump_event = out->side_event_ended = false;
    }
    if (e < E_OK || expired(now, out->poll_us, IMU_MAX_AGE_US)) {
        out->impact_valid = out->possible_impact = false;
    }
    if (e < E_OK || expired(now, out->encoder_sample_us, ENCODER_MAX_AGE_US - 1u)) {
        out->motion_valid = out->yaw_valid = false;
        out->motion_state = BUDDY4_MOTION_UNKNOWN;
    }
    return e;
}

static void publish(void)
{
    if (tk_loc_mtx(data_mutex, TMO_FEVR) >= E_OK) {
        published = next;
        (void)tk_unl_mtx(data_mutex);
    }
}

/* Single task owns all mutable processing. This function is also exercised
 * by host tests with mocked device/kernel calls, never by another task. */
static void process_once(uint64_t now)
{
    buddy4_task_speed_t speed = {0};
    if (tk_loc_mtx(data_mutex, TMO_FEVR) >= E_OK) {
        speed = mailbox;
        (void)tk_unl_mtx(data_mutex);
    }
    if (!imu.ready && now >= retry_us) {
        (void)buddy4_configure(&imu);
        retry_us = now + RETRY_US;
    }
    uint64_t acquired = now;
    ER clock_error = buddy4_task_time_us(&acquired);
    if (clock_error < E_OK) {
        /* Do not present a retained time as a fresh acquisition. */
        imu.ready = false;
        next.sequence++;
        next.poll_us = now;
        next.valid = next.impact_valid = next.motion_valid = next.yaw_valid = false;
        next.bump_event = next.side_event_ended = next.possible_impact = false;
        next.poll_result = BUDDY4_NOT_READY;
        next.device_error = (int)clock_error;
        buddy4_impact_update(&impact, 0, 0, 0, false, now, now);
        publish();
        return;
    }
    buddy4_result_t result = buddy4_poll(&imu, acquired);
    buddy4_output_t *o = &imu.output;
    buddy4_impact_update(&impact, o->accel_g[0], o->accel_g[1],
        o->accel_g[2], result == BUDDY4_FRESH && o->valid,
        o->timestamp_us, acquired);
    if (!speed.valid) {
        buddy4_motion_update(&motion, 0, 0, false, acquired, acquired);
        buddy4_encoder_yaw_update(&yaw, 0, 0, false, acquired, acquired);
    } else if (!has_speed || speed.sample_us != last_speed_us) {
        buddy4_motion_update(&motion, speed.left_cm_s, speed.right_cm_s,
            true, speed.sample_us, acquired);
        buddy4_encoder_yaw_update(&yaw, speed.left_cm_s, speed.right_cm_s,
            true, speed.sample_us, acquired);
        has_speed = true;
        last_speed_us = speed.sample_us;
    }
    buddy4_motion_output_t mo = buddy4_motion_get(&motion, acquired);
    buddy4_encoder_yaw_output_t yo = buddy4_encoder_yaw_get(&yaw, acquired);
    buddy4_impact_output_t io = buddy4_impact_get(&impact, acquired);
    next.sequence++;
    next.poll_us = acquired; next.sample_us = o->timestamp_us;
    next.valid = o->valid; next.poll_result = result;
    next.io_errors = o->error_count; next.device_error = device.last_error;
    memcpy(next.raw, o->raw, sizeof(next.raw));
    memcpy(next.acceleration_g, o->accel_g, sizeof(next.acceleration_g));
    memcpy(next.uncorrected_g, o->uncorrected_accel_g, sizeof(next.uncorrected_g));
    next.magnitude_g = o->magnitude_g;
    next.pitch_deg = o->filtered_pitch_deg; next.roll_deg = o->filtered_roll_deg;
    next.height_cm = o->wheel_height_cm;
    next.terrain = o->terrain; next.side = o->side;
    next.bump_event = o->transient_event && o->valid;
    next.side_event_ended = o->event_ended && o->valid;
    if (next.side_event_ended) {
        const side_event_t *ev = &o->ended_event;
        next.completed_events++;
        next.last_event = (buddy4_task_event_t){ev->side, ev->start_us,
            o->timestamp_us, ev->peak_confirmed, ev->median_candidate_valid,
            ev->peak_height_cm, ev->median_candidate_height_cm};
    }
    next.impact_valid = io.valid; next.possible_impact = io.possible_impact;
    next.impact_count = io.event_count; next.impact_time_us = io.event_us;
    next.impact_deviation_g = io.deviation_g;
    next.motion_state = mo.state; next.motion_valid = mo.valid;
    next.yaw_valid = yo.valid; next.encoder_sample_us = yo.sample_us;
    next.encoder_yaw_deg_s = yo.encoder_yaw_rate_estimate_deg_s;
    publish();
}

static uint64_t advance_release(uint64_t release, uint64_t now)
{
    uint64_t target = release + BUDDY4_PERIOD_US;
    if (target <= now) {
        uint64_t missed = (now - target) / BUDDY4_PERIOD_US + 1;
        target += missed * BUDDY4_PERIOD_US;
        if (missed > UINT32_MAX - next.skipped_slots) next.skipped_slots = UINT32_MAX;
        else next.skipped_slots += (uint32_t)missed;
    }
    return target;
}

static void sensor_task(INT code, void *info)
{
    (void)code; (void)info;
    uint64_t release = 0;
    (void)buddy4_task_time_us(&release);
    for (;;) {
        uint64_t now;
        ER e = buddy4_task_time_us(&now);
        if (e < E_OK) {
            next.valid = next.impact_valid = next.motion_valid = next.yaw_valid = false;
            next.bump_event = next.side_event_ended = next.possible_impact = false;
            next.device_error = (int)e;
            publish();
            (void)tk_dly_tsk(20);
            continue;
        }
        process_once(now);
        (void)buddy4_task_time_us(&now);
        release = advance_release(release, now);
        while (buddy4_task_time_us(&now) >= E_OK && now < release)
            (void)tk_dly_tsk((RELTIM)((release - now + 999u) / 1000u));
    }
}

ER buddy4_task_init(void)
{
    if (initialised) return E_OBJ;
    T_CMTX m = {0}; m.mtxatr = TA_INHERIT;
    data_mutex = tk_cre_mtx(&m);
    if (data_mutex < E_OK) return data_mutex;
    ID dd = tk_opn_dev((UB *)"iica", TD_UPDATE | TD_EXCL);
    if (dd < E_OK) { (void)tk_del_mtx(data_mutex); return (ER)dd; }
    device = (buddy4_device_t){.descriptor = (int)dd};
    buddy4_init(&imu, buddy4_device_bus(&device), buddy4_default_config());
    (void)buddy4_impact_init(&impact, buddy4_impact_default_config());
    (void)buddy4_motion_init(&motion, buddy4_motion_default_config());
    (void)buddy4_encoder_yaw_init(&yaw, buddy4_encoder_yaw_default_config());
    memset(&next, 0, sizeof(next));
    memset(&mailbox, 0, sizeof(mailbox));
    next.poll_result = BUDDY4_NOT_READY;
    published = next;
    retry_us = last_speed_us = 0; has_speed = false;
    initialised = true;
    return E_OK;
}

ER buddy4_task_start(void)
{
    if (!initialised || task_id > 0) return E_OBJ;
    T_CTSK t = {0};
    t.tskatr = TA_HLNG | TA_RNG3; t.task = sensor_task;
    t.itskpri = TASK_PRIORITY; t.stksz = TASK_STACK;
    ID id = tk_cre_tsk(&t);
    if (id < E_OK) return (ER)id;
    ER e = tk_sta_tsk(id, 0);
    if (e < E_OK) { (void)tk_del_tsk(id); return e; }
    task_id = id;
    return E_OK;
}

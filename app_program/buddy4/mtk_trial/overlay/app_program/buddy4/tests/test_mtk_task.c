#define main original_imu_test_main
#include "test_imu.c"
#undef main
#include "../buddy4_task.c"
#include <setjmp.h>

static fake_t bus_sensor;
static uint64_t fake_clock;
static int lock_held;
static int lock_failure;
static int open_failure;
static int start_failure;
static int deleted_mutex;
static int deleted_task;
static int device_calls;
static int fail_device;
static int short_device;
static int flag_failure;
static int cyclic_failure;
static int cyclic_start_failure;
static int deleted_flag;
static int deleted_cyclic;
static UINT release_bits;
static jmp_buf task_exit;
static int release_fault;
static int release_fault_seen;
static int clock_failure;
ID tk_cre_flg (const T_CFLG *f)
{
    (void)f;
    return flag_failure ? flag_failure : 4;
}

ER tk_del_flg (ID id)
{
    assert(4 == id);
    deleted_flag++;
    release_bits = 0;
    return 0;
}

ER tk_set_flg (ID id, UINT bits)
{
    assert(4 == id);
    release_bits |= bits;
    return 0;
}

ER tk_wai_flg (ID id, UINT bits, UINT mode, UINT *out, int timeout)
{
    assert((4 == id) && (1 == bits) && (mode == (TWF_ORW | TWF_BITCLR))
           && (TMO_FEVR == timeout));

    if (release_fault)
    {
        if (release_fault_seen++)
        {
            longjmp(task_exit, 1);
        }

        if (1 == release_fault)
        {
            return -49; /* E_RLWAI: forced release of forever wait. */
        }

        clock_failure = -5;
        *out = 1;
        return E_OK;
    }

    if (!(release_bits & bits))
    {
        return E_TMOUT;
    }

    *out = release_bits;
    release_bits &= ~bits;
    return 0;
}

ID tk_cre_cyc (const T_CCYC *c)
{
    assert((20 == c->cyctim) && (20 == c->cycphs)
           && (c->cychdr == release_handler));
    return cyclic_failure ? cyclic_failure : 5;
}

ER tk_sta_cyc (ID id)
{
    assert(5 == id);
    return cyclic_start_failure;
}

ER tk_stp_cyc (ID id)
{
    assert(5 == id);
    return 0;
}

ER tk_del_cyc (ID id)
{
    assert(5 == id);
    deleted_cyclic++;
    return 0;
}

void buddy4_i2c_timeout_trace (uint32_t unit, uint32_t out[32])
{
    assert(0 == unit);

    for (uint32_t idx = 0; idx < 32; idx++)
    {
        out[idx] = 100 + idx;
    }
}

static uint64_t fake_transfer_us;
ID tk_cre_mtx (const T_CMTX *m)
{
    assert(TA_INHERIT == m->mtxatr);
    return 1;
}

ER tk_del_mtx (ID id)
{
    assert(1 == id);
    deleted_mutex++;
    return 0;
}

ER tk_loc_mtx (ID id, int timeout)
{
    (void)timeout;
    assert((1 == id) && (!lock_held));

    if (lock_failure)
    {
        return lock_failure;
    }

    lock_held = 1;
    return 0;
}

ER tk_unl_mtx (ID id)
{
    assert((1 == id) && lock_held);
    lock_held = 0;
    return 0;
}

ID tk_opn_dev (UB *name, UINT mode)
{
    assert((strcmp((char*)name, "iica") == 0)
           && (mode == (TD_UPDATE | TD_EXCL)));
    return open_failure ? open_failure : 3;
}

ER tk_cls_dev (ID id, UINT option)
{
    (void)id;
    (void)option;
    return 0;
}

ER tk_get_otm (SYSTIM *t)
{
    if (clock_failure)
    {
        return clock_failure;
    }

    t->hi = (int32_t)((fake_clock / 1000) >> 32);
    t->lo = (uint32_t)(fake_clock / 1000);
    return 0;
}

ID tk_cre_tsk (const T_CTSK *t)
{
    assert((8192 == t->stksz) && (t->task));
    return 2;
}

ER tk_sta_tsk (ID id, INT code)
{
    assert((2 == id) && (0 == code));
    return start_failure;
}

ER tk_del_tsk (ID id)
{
    assert(2 == id);
    deleted_task++;
    return 0;
}

ER tk_dly_tsk (RELTIM delay)
{
    assert((!lock_held) && (delay > 0));
    fake_clock += (uint64_t)delay * 1000;
    return 0;
}

ER tk_swri_dev (ID dd, int start, const void * buf, SZ size, SZ *actual)
{
    assert((3 == dd) && (!lock_held));
    device_calls++;
    fake_clock += fake_transfer_us;

    if (fail_device)
    {
        return fail_device < 0 ? fail_device : E_IO;
    }

    *actual = short_device ? size - 1 : size;
    if (TDN_I2C_EXEC == start)
    {
        const T_I2C_EXEC *r = buf;
        assert((size == (SZ)sizeof(*r)) && (0x1d == r->sadr)
               && (1 == r->snd_size));
        assert((1 == r->rcv_size) || (6 == r->rcv_size));

        if (6 == r->rcv_size)
        {
            assert(0xa8 == r->snd_data[0]);
        }

        for (SZ idx = 0; idx < r->rcv_size; idx++)
        {
            r->rcv_data[idx] = bus_sensor.regs[(r->snd_data[0] & 0x7f) + idx];
        }
    }
    else
    {
        const uint8_t * p = buf;
        assert((0x1d == start) && (2 == size));
        bus_sensor.regs[p[0]] = p[1];
    }

    return 0;
}

static void tick (uint64_t now)
{
    fake_clock = now;
    process_once(now);
}

static void calibrated_roll (float deg)
{
    float x = sinf(0.97f*0.01745329252f);
    float y = sinf((deg + 0.61f) * 0.01745329252f);
    float z = -sqrtf(1 - x * x - y * y);
    float a[] = {x, y, z};
    const float bias[] = {0.03799f, 0.03790f, 0.08411f};
    const float scale[] = {1.01301f, 1.03419f, 1.01223f};
    vector(&bus_sensor, a[0] / scale[0] + bias[0], a[1] / scale[1] + bias[1],
           a[2] / scale[2] + bias[2]);
}

static void test_impact_release_faults (void)
{
    /* Exercise the actual task loop, including its failure backoff, then stop
     * at the next wait. A failed release/time read must break confirmation. */
    for (int kind = 1; kind <= 2; kind++)
    {
        assert(buddy4_impact_init(&impact, buddy4_impact_default_config()));
        buddy4_impact_update(&impact, 0, 0, 1, true, 5000000, 5000000);
        buddy4_impact_update(&impact, 0, 0, 1.6f, true, 5020000, 5020000);
        assert(1 == impact.pending_samples);
        next.poll_us = fake_clock = 5020000;
        release_fault = kind;
        release_fault_seen = 0;

        if (!setjmp(task_exit))
        {
            sensor_task(0, NULL);
        }

        release_fault = clock_failure = 0;
        assert((!next.impact_valid) && (!next.possible_impact));
        /* Without invalidation, this 40 ms gap falsely completes old evidence.
         */
        buddy4_impact_update(&impact, 0, 0, 1.6f, true, 5060000, 5060000);
        assert((impact.output.valid) && (!impact.output.possible_impact));
        assert(0 == impact.output.event_count);
        /* Repeat with retained history: failure clears a pulse but cannot
         * erase a count or bypass cooldown/rearming. */
        assert(buddy4_impact_init(&impact, buddy4_impact_default_config()));
        buddy4_impact_update(&impact, 0, 0, 1, true, 6000000, 6000000);
        buddy4_impact_update(&impact, 0, 0, 1.6f, true, 6020000, 6020000);
        buddy4_impact_update(&impact, 0, 0, 1.6f, true, 6040000, 6040000);
        assert((impact.output.possible_impact)
               && (1 == impact.output.event_count));
        next.poll_us = fake_clock = 6040000;
        release_fault = kind;
        release_fault_seen = 0;

        if (!setjmp(task_exit))
        {
            sensor_task(0, NULL);
        }

        release_fault = clock_failure = 0;
        assert((!impact.output.valid) && (!impact.output.possible_impact));
        assert((1 == impact.output.event_count)
               && (6040000 == impact.output.event_us));
        published = next;
        published.impact_valid = impact.output.valid;
        published.possible_impact = impact.output.possible_impact;
        published.impact_count = impact.output.event_count;
        published.impact_time_us = impact.output.event_us;
        fake_clock = 6060000;
        buddy4_task_snapshot_t retained;
        assert(E_OK == buddy4_task_get(&retained));
        assert((!retained.impact_valid) && (!retained.possible_impact));
        assert((1 == retained.impact_count)
               && (6040000 == retained.impact_time_us));

        for (uint64_t t = 6080000; t <= 6120000; t += 20000)
        {
            buddy4_impact_update(&impact, 0, 0, 1, true, t, t);
        }

        for (uint64_t t = 6140000; t <= 6160000; t += 20000)
        {
            buddy4_impact_update(&impact, 0, 0, 1.6f, true, t, t);
        }

        assert((!impact.output.possible_impact)
               && (1 == impact.output.event_count));
    }
}

static void task_horizontal (float x)
{
    vector(&bus_sensor,
           x / imu.config.accel_scale[0] + imu.config.accel_bias_g[0],
           imu.config.accel_bias_g[1],
           -1 / imu.config.accel_scale[2] + imu.config.accel_bias_g[2]);
}

static void test_horizontal_impact_task (void)
{
    /* Actual owner -> native bridge -> production-calibrated XYZ -> snapshot.
     * The original magnitude-only observer cannot confirm this disturbance. */
    assert(buddy4_impact_init(&impact, buddy4_impact_default_config()));
    task_horizontal(0);
    tick(10000000);
    task_horizontal(0.45f);
    tick(10020000);
    assert((next.impact_valid) && (!next.possible_impact)
           && (0 == next.impact_count));
    tick(10040000);
    buddy4_task_snapshot_t out;
    assert((E_OK == buddy4_task_get(&out)) && (out.possible_impact)
           && (1 == out.impact_count));
    assert(fabsf(out.impact_deviation_g - 0.45f) < 0.001f);
    assert((out.magnitude_g < 1.10f) && (10040000 == out.impact_time_us));
    assert((E_OK == buddy4_task_get(&out)) && (1 == out.impact_count));
    tick(10060000);
    assert((!next.possible_impact) && (1 == next.impact_count));
    raw_vector(&bus_sensor, INT16_MAX, 0, 0);
    tick(10080000);
    assert((!next.impact_valid) && (!next.possible_impact)
           && (1 == next.impact_count));
    task_horizontal(0);

    for (uint64_t t = 10100000; t <= 10140000; t += 20000)
    {
        tick(t);
    }

    task_horizontal(0.45f);
    tick(10160000);
    tick(10180000);
    /* Cooldown survives. */
    assert((!next.possible_impact) && (1 == next.impact_count));
    task_horizontal(0);

    for (uint64_t t = 10200000; t <= 10800000; t += 20000)
    {
        tick(t);
    }

    task_horizontal(0.45f);
    tick(10820000);
    tick(10840000);
    assert((next.possible_impact) && (2 == next.impact_count));
}

int main (void)
{
    buddy4_task_snapshot_t out;
    assert(E_OBJ == buddy4_task_get(&out));
    assert(E_OBJ == buddy4_task_start());
    open_failure = -33;
    assert((-33 == buddy4_task_init()) && (1 == deleted_mutex));
    open_failure = 0;
    assert((0 == buddy4_task_init()) && (E_OBJ == buddy4_task_init()));
    assert(E_PAR == buddy4_task_get(NULL));
    start_failure = -34;
    assert((-34 == buddy4_task_start()) && (1 == deleted_task));
    assert((1 == deleted_flag) && (1 == deleted_cyclic));
    start_failure = 0;
    flag_failure = -35;
    assert(-35 == buddy4_task_start());
    flag_failure = 0;
    cyclic_failure = -36;
    assert(-36 == buddy4_task_start());
    cyclic_failure = 0;
    cyclic_start_failure = -37;
    assert(-37 == buddy4_task_start());
    cyclic_start_failure = 0;
    assert((0 == buddy4_task_start()) && (E_OBJ == buddy4_task_start()));
    assert((0.61f == imu.config.roll_offset_deg)
           && (11.3f == imu.config.centre_spacing_cm));
    bus_sensor.regs[15] = 0x49;
    bus_sensor.regs[0x27] = 8;
    vector(&bus_sensor, 0.04f, 0.04f, -0.9f);
    tick(20000);
    assert((0 == buddy4_task_get(&out)) && (out.valid));
    assert((!out.motion_valid) && (!out.yaw_valid) && (!out.possible_impact));
    buddy4_task_speed_t speed = {true, 40000, 3, 3};
    assert(0 == buddy4_task_submit_speed(&speed));
    tick(40000);
    assert((yaw.output.valid) && (!next.motion_valid));
    tick(60000);
    assert(!next.motion_valid); /* Duplicate is not another observation. */
    for (uint64_t t = 80000; t <= 140000; t += 20000)
    {
        speed.sample_us = t;
        buddy4_task_submit_speed(&speed);
        tick(t);
    }

    assert((next.motion_valid) && (BUDDY4_MOTION_FORWARD == next.motion_state));
    tick(239000);
    assert(next.motion_valid); /* Just below the 100 ms boundary. */
    tick(240000);
    assert(!next.motion_valid); /* Boundary is stale in the core API. */
    tick(260000);
    assert((!next.motion_valid) && (!next.yaw_valid));
    speed.sample_us = 100000;
    buddy4_task_submit_speed(&speed);
    tick(280000);
    assert((!next.motion_valid) && (!next.yaw_valid));
    bus_sensor.regs[0x27] = 0;
    tick(300000);
    assert((!next.valid) && (!next.impact_valid));
    bus_sensor.regs[0x27] = 8;
    short_device = 1;
    tick(320000);
    assert((!next.valid) && (1 == next.io_errors) && (!next.side_event_ended));
    assert((next.saved_error_present) && (E_IO == next.saved_error_code));
    assert((next.saved_error_read) && (0x27 == next.saved_error_register)
           && (320000 == next.saved_error_us));
    int calls = device_calls;
    tick(340000);
    assert(device_calls == calls);
    short_device = 0;
    tick(1020000);
    assert((next.valid) && (1 == next.io_errors));
    assert((next.saved_error_present) && (E_IO == next.saved_error_code)
           && (320000 == next.saved_error_us));
    fake_clock = 1061000;
    assert((0 == buddy4_task_get(&out)) && (!out.valid) && (!out.impact_valid));
    assert(next.valid); /* Getter must expire a copy only. */
    calibrated_roll(6);

    for (uint64_t t = 1040000; t <= 1640000; t += 20000)
    {
        tick(t);
    }

    assert((1 == next.side) && (imu.output.event.peak_confirmed));
    calibrated_roll(0);

    for (uint64_t t = 1660000; t <= 2100000; t += 20000)
    {
        tick(t);
    }

    assert((1 == next.completed_events) && (1 == next.last_event.side));
    assert((next.last_event.stable_valid) && (next.last_event.median_valid));
    assert((next.last_event.stable_cm > 1) && (next.last_event.median_cm > 1));
    assert((0 == buddy4_task_get(&out)) && (1 == out.completed_events));
    assert((0 == buddy4_task_get(&out)) && (1 == out.completed_events));
    calls = device_calls;
    clock_failure = -5;
    tick(2120000);
    assert((device_calls == calls) && (!next.valid)
           && (!next.side_event_ended));
    assert((1 == next.completed_events) && (-5 == next.device_error));
    clock_failure = 0;
    assert((0 == buddy4_task_get(&out)) && (!out.valid));
    assert((1 == out.completed_events) && (1 == out.last_event.side));
    assert((out.last_event.stable_valid) && (out.last_event.median_valid));
    assert((out.last_event.stable_cm > 1) && (out.last_event.median_cm > 1));
    lock_failure = E_TMOUT;
    assert(E_TMOUT == buddy4_task_get(&out));
    lock_failure = 0;
    next.skipped_slots = 0;
    uint64_t target = 0;
    bool running = false;
    assert((claim_release(&target, &running, 30000)) && (50000 == target));

    for (uint64_t t = 50000; t <= 230000; t += 20000)
    {
        assert(claim_release(&target, &running, t));
    }

    /* Exact-boundary 20ms work loses no release. */
    assert(0 == next.skipped_slots);
    /* Duplicate cannot burst. */
    assert(!claim_release(&target, &running, 230000));
    assert((claim_release(&target, &running, 310000))
           && (3 == next.skipped_slots)
           && (330000 == target));
    next.skipped_slots = UINT32_MAX - 1;
    assert((claim_release(&target, &running, 1000000))
           && (UINT32_MAX == next.skipped_slots));
    UINT bits = 0;
    release_handler(NULL);
    release_handler(NULL);
    release_handler(NULL);
    assert((0 == tk_wai_flg(release_flag, 1, TWF_ORW | TWF_BITCLR, &bits,
                            TMO_FEVR)) && (1 == bits));
    assert(E_TMOUT == tk_wai_flg(release_flag, 1, TWF_ORW | TWF_BITCLR, &bits,
                                 TMO_FEVR));
    /* Combined bridge never consumes an old or mismatched register pointer. */
    buddy4_bus_t b = buddy4_device_bus(&device);
    uint8_t reg = 15;
    uint8_t value;
    assert(-1 == b.read(b.context, 0x1d, &value, 1, false));
    assert(1 == b.write(b.context, 0x1d, &reg, 1, true));
    assert(-1 == b.read(b.context, 0x1e, &value, 1, false));
    assert(-1 == b.read(b.context, 0x1d, &value, 1, false));
    /* Diagnostics expose delayed I/O without altering sample timestamps. */
    fake_transfer_us = 10000;
    fail_device = 0;
    short_device = 0;
    bus_sensor.regs[0x27] = 8;
    imu.ready = true;
    tick(3000000);
    assert((20 == next.poll_duration_ms) && (next.timing_valid));
    assert((3000000 == next.poll_us) && (3000000 == next.sample_us));
    assert((0 == buddy4_task_get(&out)) && (20 == out.sample_age_ms)
           && (out.valid));
    fake_clock = 3041000;
    assert((0 == buddy4_task_get(&out)) && (41 == out.sample_age_ms)
           && (!out.valid));
    assert(next.valid);
    fake_transfer_us = 0;
    assert(UINT32_MAX == elapsed_ms(1000, 2000));
    assert(UINT32_MAX == elapsed_ms(UINT64_MAX, 0));
    fail_device = E_TMOUT;
    tick(3500000);
    assert((!next.valid) && (E_TMOUT == next.saved_error_code)
           && (3500000 == next.saved_error_us));
    /* Periodic identity read. */
    assert((0x0f == next.saved_error_register) && (next.saved_error_read));

    for (uint32_t idx = 0; idx < 32; idx++)
    {
        assert(next.saved_timeout_trace[idx] == 100 + idx);
    }

    fail_device = 0;
    tick(4520000);
    assert((next.valid) && (E_OK == device.last_error));
    assert((E_TMOUT == next.saved_error_code)
           && (3500000 == next.saved_error_us));

    for (uint32_t idx = 0; idx < 32; idx++)
    {
        assert(next.saved_timeout_trace[idx] == 100 + idx);
    }

    assert((0 == buddy4_task_get(&out)) && (E_TMOUT == out.saved_error_code));

    for (uint32_t idx = 0; idx < 32; idx++)
    {
        assert(out.saved_timeout_trace[idx] == 100 + idx);
    }

    /* Exercise mailbox recovery independently of the earlier retained sample.
     * A nonfinite observation invalidates both consumers. Re-reading one
     * mailbox observation cannot complete motion confirmation, and the public
     * getter must expire motion/yaw without waiting for another task poll. */
    assert(buddy4_motion_init(&motion, buddy4_motion_default_config()));
    assert(buddy4_encoder_yaw_init(&yaw, buddy4_encoder_yaw_default_config()));
    has_speed = false;
    last_speed_us = 0;
    memset(&mailbox, 0, sizeof(mailbox));
    speed = (buddy4_task_speed_t){true, 4600000, NAN, 3};
    assert(0 == buddy4_task_submit_speed(&speed));
    tick(4600000);
    assert((!next.motion_valid) && (!next.yaw_valid));
    speed = (buddy4_task_speed_t){true, 4620000, 3, 3};
    assert(0 == buddy4_task_submit_speed(&speed));
    tick(4620000);
    assert((!next.motion_valid) && (next.yaw_valid));

    for (uint64_t t = 4640000; t <= 4720000; t += 20000)
    {
        tick(t);
        assert(!next.motion_valid); /* Same sample cannot confirm. */
    }

    assert(!next.yaw_valid); /* The unchanged observation reached 100 ms. */
    for (uint64_t t = 4740000; t <= 4840000; t += 20000)
    {
        speed.sample_us = t;
        assert(0 == buddy4_task_submit_speed(&speed));
        tick(t);
    }

    assert((next.motion_valid) && (BUDDY4_MOTION_FORWARD == next.motion_state));
    assert((next.yaw_valid) && (4840000 == next.encoder_sample_us));
    fake_clock = 4939999;
    assert((0 == buddy4_task_get(&out)) && (out.motion_valid)
           && (out.yaw_valid));
    fake_clock = 4940000;
    assert((0 == buddy4_task_get(&out)) && (!out.motion_valid)
           && (!out.yaw_valid));
    assert(BUDDY4_MOTION_UNKNOWN == out.motion_state);

    /* Actual owner task, successful native device requests, clipped input.
     * Retain completed tilt/impact history, clear pending evidence, recover. */
    uint32_t completed = next.completed_events;
    buddy4_task_event_t completed_result = next.last_event;
    assert(buddy4_impact_init(&impact, buddy4_impact_default_config()));
    calibrated_roll(6);
    tick(6000000);

    for (uint64_t t = 6020000; t <= 6100000; t += 20000)
    {
        tick(t);
    }

    assert((imu.output.event.side) && (!imu.output.event.peak_confirmed));

    for (int idx = 0; idx < 3; ++idx)
    {
        float mag = idx ? 1.6f : 1.0f;
        vector(&bus_sensor, imu.config.accel_bias_g[0],
               imu.config.accel_bias_g[1],
               -mag / imu.config.accel_scale[2] + imu.config.accel_bias_g[2]);
        tick(6120000 + idx * 20000);
    }

    assert(1 == next.impact_count);
    uint64_t impact_time = next.impact_time_us;
    uint32_t errors = next.io_errors;
    raw_vector(&bus_sensor, INT16_MIN, INT16_MIN, INT16_MIN);

    for (uint64_t t = 6180000; t <= 6500000; t += 20000)
    {
        tick(t);
        assert(E_OK == buddy4_task_get(&out));
        assert((out.valid) && (!out.tilt_valid) && (!out.acceleration_valid));
        assert((BUDDY4_QUALITY_CLIPPED == out.measurement_quality)
               && (7 == out.clipped_axes));
        assert((!out.impact_valid) && (!out.possible_impact)
               && (!impact.pending_samples));
        assert((1 == out.impact_count) && (out.impact_time_us == impact_time));
        assert((out.io_errors == errors) && (!out.side_event_ended)
               && (!imu.output.event.side));
        assert(out.completed_events == completed);
        assert(0 == memcmp(&out.last_event, &completed_result,
                           sizeof(completed_result)));
    }

    assert((17 == out.clipped_samples) && (out.unsuitable_samples >= 17));
    assert((0x49 == out.who_am_i) && (out.sensor_config_valid)
           && (0x6f == out.sensor_ctrl[1]));
    calibrated_roll(6);

    for (uint64_t t = 6520000; t <= 6700000; t += 20000)
    {
        tick(t);
    }

    assert((next.valid) && (next.tilt_valid) && (next.impact_valid)
           && (!next.possible_impact));
    assert((!imu.output.event.peak_confirmed)
           && (!imu.output.event.median_candidate_valid));
    tick(6720000);
    assert((imu.output.event.peak_confirmed)
           && (imu.output.event.median_candidate_valid));
    assert((17 == next.clipped_samples) && (1 == next.impact_count)
           && (next.completed_events == completed));
    /* A runtime configuration mismatch is explicitly a verification failure. */
    bus_sensor.regs[0x21] ^= 0x08;
    tick(8000000);
    assert((!next.valid) && (!next.sensor_config_valid)
           && (next.io_errors == errors + 1));
    assert((next.sensor_ctrl[2] != 0xc0) && (next.completed_events == completed)
           && (1 == next.impact_count));
    tick(9020000);
    assert((next.valid) && (next.sensor_config_valid) && (next.tilt_valid));

    fail_device = 1;
    assert(-1 == b.write(b.context, 0x1d, (uint8_t[]){0x20, 0x6f}, 2, false));
    test_impact_release_faults();
    fail_device = 0;
    test_horizontal_impact_task();
    puts("Native MTK task/device bridge tests passed: lifecycle, combined "
         "I/O, invalid/stale/reversed/duplicate input, recovery and snapshots");
    return 0;
}

/*** end of file ***/

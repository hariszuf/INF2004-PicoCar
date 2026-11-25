#include "../../barrc_types.h"
#include "buddy4_obstruction_sdk.h"
#include "scheduler.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static buddy4_obstruction_sdk_t a;
static drive_control_t control;
static motion_status_t motor;
static buddy4_imu_t imu;
static uint64_t t;
static uint32_t skipped;
static void reset (void)
{
    buddy4_obstruction_sdk_init(&a);
    t = 0;
    skipped = 0;
    control = (drive_control_t){.running = true, .mode = DRIVE_MANUAL,
        .started_us = 1,
        .selected_left_cm_s = 3, .selected_right_cm_s = 3};
    motor = (motion_status_t){.state = MOTION_STATE_EXECUTING,
        .terrain_speed_scale = 1};
    imu = (buddy4_imu_t){.ready = true};
    imu.output.valid = imu.output.acceleration_valid = imu.output.tilt_valid =
            true;
}

static void note (float32_t left, float32_t right)
{
    t += 20000;
    motor.left_speed_cm_s = left;
    motor.right_speed_cm_s = right;
    imu.output.timestamp_us = t + 500;
    buddy4_obstruction_sdk_note_update(&a, &motor, &control, t, t + 100,
                                       skipped);
}

static buddy4_obstruction_output_t poll (void)
{
    return buddy4_obstruction_sdk_poll(&a, &control, &imu, true, t + 1000);
}

static buddy4_obstruction_output_t step (float32_t l, float32_t r)
{
    note(l, r);
    return poll();
}

static void establish (void)
{
    for (int32_t idx = 0; idx < 27; idx++)
    {
        assert(!step(3, 3).possible_obstruction);
    }

    assert((a.observer.output.valid) && (a.observer.output.established));
}

static void pending (void)
{
    for (int32_t idx = 0; idx < 5; idx++)
    {
        assert(!step(0.12f, 0.12f).possible_obstruction);
    }
}

static void ordinary_and_blocked (void)
{
    reset();
    establish();

    for (int32_t idx = 0; idx < 100; idx++)
    {
        assert(!step(3.1f, 2.9f).possible_obstruction);
    }

    pending();
    assert(step(0.004f, 0.004f).possible_obstruction);
    assert(1 == a.observer.output.event_count);

    for (int32_t idx = 0; idx < 20; idx++)
    {
        assert(1 == poll().event_count); /* Repeated getters are not updates. */
    }

    for (int32_t idx = 0; idx < 100; idx++)
    {
        assert(!step(0, 0).possible_obstruction);
    }

    establish();
    pending();
    assert(step(0, 0).possible_obstruction);
    assert(2 == a.observer.output.event_count);
    reset();
    establish();

    for (int32_t idx = 0; idx < 5; idx++)
    {
        step(3.18f, 3.09f); /* Normal pre-stop comparison. */
    }

    control.running = false;

    for (int32_t idx = 0; idx < 20; idx++)
    {
        assert(!step(0, 0).possible_obstruction);
    }

    assert(0 == a.observer.output.event_count);
}

static void command_cancellation (void)
{
    for (int32_t kind = 0; kind < 6; kind++)
    {
        reset();
        establish();
        pending();
        note(0, 0); /* Would confirm this observation. */
        if (0 == kind)
        {
            control.running = false; /* S processed AFTER motor update. */
        }

        if (1 == kind)
        {
            control.mode = DRIVE_TURN_LEFT;
        }

        if (2 == kind)
        {
            control.mode = DRIVE_LINE;
        }

        if (3 == kind)
        {
            ++control.started_us; /* New drive/session. */
        }

        if (4 == kind)
        {
            control.selected_left_cm_s = 2;
        }

        if (5 == kind)
        {
            control.stop_fault = MOTION_FAULT_STALL;
        }

        assert((!poll().possible_obstruction)
               && (0 == a.observer.output.event_count));
        assert(!a.observer.output.established);
    }

    reset();
    control.running = false;
    note(3, 3);
    control.running = true;
    ++control.started_us;
    assert(!poll().valid); /* G cannot reuse a pre-command motor observation. */
}

static void ramps_turns_faults (void)
{
    for (int32_t kind = 0; kind < 12; kind++)
    {
        reset();
        establish();
        pending();

        if (0 == kind)
        {
            imu.output.terrain = 1;
        }

        if (1 == kind)
        {
            imu.output.side = -1;
        }

        if (2 == kind)
        {
            /* Before 6 deg terrain entry. */
            imu.output.filtered_pitch_deg = 3.01f;
        }

        if (3 == kind)
        {
            /* Before 4 deg side entry. */
            imu.output.filtered_roll_deg = -2.01f;
        }

        if (4 == kind)
        {
            imu.output.filtered_pitch_deg = 4;
        }

        if (5 == kind)
        {
            motor.terrain = MOTION_TERRAIN_DOWNHILL;
        }

        if (6 == kind)
        {
            motor.terrain_speed_scale = 0.8f;
        }

        if (7 == kind)
        {
            motor.left_terrain_duty = 0.05f;
        }

        if (8 == kind)
        {
            motor.fault = MOTION_FAULT_STALL;
        }

        if (9 == kind)
        {
            motor.state = MOTION_STATE_STOPPING;
        }

        if (10 == kind)
        {
            control.mode = DRIVE_TURN_RIGHT;
        }

        if (11 == kind)
        {
            control.selected_right_cm_s = 2;
        }

        for (int32_t idx = 0; idx < 20; idx++)
        {
            assert(!step(0, 0).possible_obstruction);
        }

        assert((!a.observer.output.valid)
               && (0 == a.observer.output.event_count));
    }

    reset();
    establish();

    for (int32_t idx = 0; idx < 50; idx++)
    {
        assert(!step(0, 3).possible_obstruction);
    }

    assert((0 == a.observer.output.event_count)
           && (!a.observer.output.established));
}

static void timing_and_invalid (void)
{
    for (int32_t kind = 0; kind < 18; kind++)
    {
        reset();
        establish();
        pending();
        uint64_t imu_before = imu.output.timestamp_us;

        if (0 == kind)
        {
            skipped++;
        }

        if (1 == kind)
        {
            t += 40000; /* Missing update: cadence no longer qualified. */
        }

        if (2 == kind)
        {
            t -= 15000; /* Too-short interval, Buddy 2 would clamp dt. */
        }

        if (3 == kind)
        {
            imu.ready = false;
        }

        if (4 == kind)
        {
            imu.output.valid = false;
        }

        if (5 == kind)
        {
            imu.output.acceleration_valid = false;
        }

        if (6 == kind)
        {
            imu.output.tilt_valid = false;
        }

        if (7 == kind)
        {
            imu.output.clipped_axes = 1;
        }

        if (8 == kind)
        {
            imu.output.roll_deg = NAN;
        }

        note(9 == kind ? NAN : 0, 10 == kind ? INFINITY : 0);

        if (11 == kind)
        {
            imu.output.timestamp_us = t - 40001;
        }

        if (12 == kind)
        {
            imu.output.timestamp_us = t + 1001; /* Future relative to poll. */
        }

        if (13 == kind)
        {
            /* Fresh age, excess cross-source skew. */
            imu.output.timestamp_us = t - 31000;
        }

        if (14 == kind)
        {
            /* Reversed IMU time is a fault. */
            imu.output.timestamp_us = imu_before - 1;
        }

        if (15 == kind)
        {
            /* Duplicate callback. */
            buddy4_obstruction_sdk_note_update(&a, &motor, &control, t, t + 100,
                                               skipped);
        }

        if (16 == kind)
        {
            /* Long update. */
            buddy4_obstruction_sdk_note_update(&a, &motor, &control, t,
                                               t + 6000, skipped);
        }

        if (17 == kind)
        {
            a.end_us = t + 1001; /* Future update end. */
        }

        assert((!poll().possible_obstruction)
               && (0 == a.observer.output.event_count));
        assert((!a.observer.output.valid) && (!a.observer.output.established));
    }

    reset();
    establish();
    pending();
    assert(!buddy4_obstruction_sdk_poll(&a, &control, &imu, true,
                                        t + 41000).valid);
    t += 60000;

    for (int32_t idx = 0; idx < 30; idx++)
    {
        step(0, 0);
    }

    assert((0 == a.observer.output.event_count)
           && (!a.observer.output.established));
    establish();
    pending();
    assert(step(0, 0).possible_obstruction);
    /* Context failure retains a completed event; recovery cannot bypass
     * cooldown. */
    control.running = false;
    assert(!poll().possible_obstruction);
    assert(1 == a.observer.output.event_count);
    control.running = true;
    ++control.started_us;

    for (int32_t idx = 0; idx < 27; idx++)
    {
        step(3, 3);
    }

    assert(!a.observer.output.established);

    for (int32_t idx = 0; idx < 30; idx++)
    {
        assert(!step(0, 0).possible_obstruction);
    }

    establish();
    pending();
    assert(step(0, 0).possible_obstruction);
    assert(2 == a.observer.output.event_count);
}

static void phased_20ms_updates (void)
{
    reset();
    /* Both sources update every 20 ms, but the motor snapshot sometimes reaches
     * the main-loop poll before that cycle's IMU poll. The previous IMU remains
     * valid/fresh. Polling it again must neither advance nor reset evidence. */
    for (int32_t idx = 1; idx <= 200; idx++)
    {
        t = (uint64_t)idx * 20000;
        motor.left_speed_cm_s = 3.18f;
        motor.right_speed_cm_s = 3.09f;
        buddy4_obstruction_sdk_note_update(&a, &motor, &control, t, t + 100,
                                           skipped);
        uint64_t previous = a.observer.output.observation_us;
        bool b_delayed = idx % 2;

        if (!b_delayed)
        {
            imu.output.timestamp_us = t + 500;
        }

        buddy4_obstruction_output_t out = poll();

        if (b_delayed)
        {
            if (idx > 2)
            {
                /* No duplicate evidence. */
                assert(out.observation_us == previous);
            }

            imu.output.timestamp_us = t + 2000;
            out = buddy4_obstruction_sdk_poll(&a, &control, &imu, true,
                                              t + 3000);
        }

        if (idx > 1)
        {
            assert(out.valid);
        }

        if (idx >= 28)
        {
            assert(out.established);
        }

        assert((!out.possible_obstruction) && (0 == out.event_count));
    }

    puts("Phased 20 ms motor/IMU updates: established throughout steady 4 s "
         "drive after warmup");
}

static void waiting_is_not_evidence (void)
{
    reset();
    establish();
    pending();
    uint64_t previous_imu = imu.output.timestamp_us;
    uint64_t previous_motor = a.observer.output.observation_us;
    note(0, 0);
    imu.output.timestamp_us = previous_imu;
    buddy4_obstruction_output_t out = poll();
    assert((a.pending) && (out.valid) && (out.established));
    assert((out.observation_us == previous_motor) && (80 == out.slow_ms)
           && (0 == out.event_count));

    for (uint64_t now = t + 1100; now < t + 3000; now += 100)
    {
        out = buddy4_obstruction_sdk_poll(&a, &control, &imu, true, now);
        assert((a.pending) && (!out.possible_obstruction)
               && (0 == out.event_count));
        assert((out.observation_us == previous_motor) && (80 == out.slow_ms));
    }

    imu.output.timestamp_us = t + 3000;
    out = buddy4_obstruction_sdk_poll(&a, &control, &imu, true, t + 4000);
    assert((!a.pending) && (out.possible_obstruction)
           && (1 == out.event_count));
    /* Motor observation time is not rewritten. */
    assert(out.event_us == t + 100);

    /* The same would-be confirmation must still lose to S while awaiting IMU.
     */
    reset();
    establish();
    pending();
    previous_imu = imu.output.timestamp_us;
    note(0, 0);
    imu.output.timestamp_us = previous_imu;
    poll();
    assert(a.pending);
    control.running = false;
    imu.output.timestamp_us = t + 3000;
    out = buddy4_obstruction_sdk_poll(&a, &control, &imu, true, t + 4000);
    assert((!out.valid) && (!out.established) && (!out.possible_obstruction)
           && (0 == out.event_count));

    /* No IMU progress: pending evidence expires without confirming. */
    reset();
    establish();
    pending();
    previous_imu = imu.output.timestamp_us;
    note(0, 0);
    imu.output.timestamp_us = previous_imu;
    poll();
    out = buddy4_obstruction_sdk_poll(&a, &control, &imu, true, t + 41001);
    assert((!out.valid) && (!out.established) && (!out.possible_obstruction)
           && (0 == out.event_count));
    /* Fresh IMU data cannot substitute for missing motor updates either. */
    reset();
    establish();

    for (int32_t idx = 0; idx < 3; idx++)
    {
        t += 20000;
        imu.output.timestamp_us = t + 500;
        out = poll();
        assert((!out.possible_obstruction) && (0 == out.event_count));

        if (idx > 0)
        {
            assert((!out.valid) && (!out.established));
        }
    }

    /* A missing motor/IMU pairing may not be overwritten into continuous data.
     */
    reset();
    establish();
    previous_imu = imu.output.timestamp_us;
    note(3, 3);
    imu.output.timestamp_us = previous_imu;
    poll();
    assert(a.pending);
    note(3, 3); /* Still pending at a second actual motor update. */
    assert((!poll().valid) && (!a.observer.output.established));
    assert(0 == a.observer.output.event_count);
    /* Known nonfinite encoder input cancels even before a NEW IMU is ready. */
    reset();
    establish();
    previous_imu = imu.output.timestamp_us;
    note(NAN, 3);
    imu.output.timestamp_us = previous_imu;
    assert((!poll().valid) && (!a.observer.output.established));
}

typedef struct
{
    float32_t pitch;
    float32_t roll;
    float32_t filtered_pitch;
    float32_t filtered_roll;
    float32_t left;
    float32_t right;
} drive_row_t;
static const drive_row_t recorded_drive[] = {
#include "fixtures/obstruction_drive_20261102_1346.inc"
};
typedef struct
{
    uint64_t sample_us;
    drive_row_t row;
} contact_row_t;
static const contact_row_t recorded_contact[] = {
#include "fixtures/obstruction_foam_20261102_1424.inc"
};
static void actual_foam_transition (void)
{
    reset();
    uint64_t event = 0;
    bool b_established_before_asymmetry = false;

    for (uint32_t idx = 0; idx < sizeof(recorded_contact)
         / sizeof(recorded_contact[0]); idx++)
    {
        const contact_row_t * p_sample = &recorded_contact[idx];
        const drive_row_t * p_r = &p_sample->row;
        /* Retain recorded gaps, simulate update phase. */
        t = p_sample->sample_us - 20000;
        imu.output.pitch_deg = p_r->pitch;
        imu.output.roll_deg = p_r->roll;
        imu.output.filtered_pitch_deg = p_r->filtered_pitch;
        imu.output.filtered_roll_deg = p_r->filtered_roll;
        buddy4_obstruction_output_t out = step(p_r->left, p_r->right);

        if ((p_sample->sample_us >= 87480295)
            && (p_sample->sample_us < 87600302)
            && (out.established))
        {
            b_established_before_asymmetry = true;
        }

        if (out.possible_obstruction)
        {
            event = out.event_us;
        }
    }

    assert(b_established_before_asymmetry);
    assert((event >= 87840000) && (event < 88224602)
           && (1 == a.observer.output.event_count));
    control.running = false;
    assert(!buddy4_obstruction_sdk_poll(&a, &control, &imu, true,
                                        88224602).possible_obstruction);
    assert(1 == a.observer.output.event_count);
    printf("Actual foam speed/angle replay: possible obstruction at %.6f s "
           "before recorded S at 88.224602 s (callback phase simulated)\n",
           event / 1e6);
}

static void recorded_angles_and_cooperative_schedule (void)
{
    /* Use the ACTUAL recorded angles/speeds with the real phase-locked
     * scheduler
     * and SDK call order. Loop/I2C/log durations are synthetic: the CSV does
     * not
     * contain every intervening motor callback/main-loop poll. This isolates
     * the
     * eligibility gates; it is not an exact reconstruction of hidden callbacks.
     */
    uint32_t pitch_rejections = 0;
    uint32_t roll_rejections = 0;
    uint32_t angle_rejections = 0;

    for (uint32_t idx = 0; idx < sizeof(recorded_drive)
         / sizeof(recorded_drive[0]); idx++)
    {
        const drive_row_t * p_r = &recorded_drive[idx];
        bool b_pitch = fabsf(p_r->pitch) > 3, roll = fabsf(p_r->roll) > 2;
        pitch_rejections += b_pitch;
        roll_rejections += roll;
        angle_rejections += b_pitch || roll;
        assert((fabsf(p_r->filtered_pitch) <= 3)
               && (fabsf(p_r->filtered_roll) <= 2));
    }

    assert((60 == pitch_rejections) && (69 == roll_rejections)
           && (108 == angle_rejections));

    for (int32_t flat_control = 0; flat_control < 2; flat_control++)
    {
        reset();
        task_t motor_release = {.period = 20000};
        task_t imu_release = {.period = 20000};
        uint64_t clock = 0;
        uint64_t previous_observation = 0;
        uint64_t first_established = 0;
        uint32_t row = 0;
        uint32_t repeated = 0;
        uint32_t established = 0;

        while (row < sizeof(recorded_drive) / sizeof(recorded_drive[0]))
        {
            const drive_row_t * p_r = &recorded_drive[row];
            uint64_t loop_now = clock;
            clock += 60; /* Safety/status work after main captures its 'now'. */
            if (due(&motor_release, loop_now))
            {
                uint64_t begin = clock;
                clock += 100;
                motor.left_speed_cm_s = p_r->left;
                motor.right_speed_cm_s = p_r->right;
                buddy4_obstruction_sdk_note_update(&a, &motor, &control, begin,
                        clock, motor_release.skipped);
            }

            bool b_sampled = due(&imu_release, clock);

            if (b_sampled)
            {
                imu.output.timestamp_us = clock;
                imu.output.pitch_deg = flat_control ? 0 : p_r->pitch;
                imu.output.roll_deg = flat_control ? 0 : p_r->roll;
                imu.output.filtered_pitch_deg = p_r->filtered_pitch;
                imu.output.filtered_roll_deg = p_r->filtered_roll;
                /* Bounded acquisition duration similar to capture. */
                clock += 1500;
            }

            /* Serial/safety decisions always precede assessment. */
            clock += 30;
            buddy4_obstruction_output_t out =
                    buddy4_obstruction_sdk_poll(&a, &control, &imu, true,
                                                clock);

            if (b_sampled)
            {
                repeated += out.observation_us == previous_observation;
                previous_observation = out.observation_us;
                established += out.established;

                if ((out.established) && (!first_established))
                {
                    first_established = clock;
                }

                assert((0 == out.event_count) && (!out.possible_obstruction));
                /* Repeated CSV/getter observations are not new encoder inputs.
                 */
                uint64_t since = a.observer.moving_since_us;

                for (int32_t read = 0; read < 3; read++)
                {
                    clock += 10;
                    buddy4_obstruction_output_t again =
                            buddy4_obstruction_sdk_poll(&a, &control, &imu,
                            true, clock);
                    assert(again.observation_us == out.observation_us);
                    assert((again.established == out.established)
                           && (0 == again.event_count));
                    assert(a.observer.moving_since_us == since);
                }

                row++;
                clock += 1500 + (row % 3) * 800; /* CSV formatting/USB work. */
            }

            clock += 100;
        }

        if (!flat_control)
        {
            assert((established > 100) && (a.observer.output.established));
            printf("Actual normal-driving vectors: first establishment at "
                   "%.3f s of synthetic replay, %u/230 established rows, no "
                   "obstruction events\n", first_established / 1e6,
                   established);
        }
        else
        {
            /* Angle-control TEST INPUT only: same speeds/scheduling/filtered
             * angles, with raw angles set to zero. Finite raw amplitude must
             * not decide filtered-orientation eligibility. */
            assert((established > 100) && (a.observer.output.established));
            puts("Eligible-angle control with same speeds/scheduling "
                 "establishes without changing thresholds");
        }
    }
}

static void recorded_ramp (const char * name, uint32_t expected_rows,
                           bool b_held)
{
    /* Historical 69-column outputs already include their original calibration.
     * Retain actual sample gaps. Callback/I2C phase is synthetic, not a replay
     * of unlogged task releases. Legacy data lacks modern quality flags and
     * motor terrain-compensation fields: derive only finite/rail/magnitude
     * suitability, leaving motor terrain FLAT to exercise the IMU guard alone.
     */
    char path[1024];
    size_t prefix = 0;

    for (size_t idx = 0; __FILE__[idx]; idx++)
    {
        if (('/' == __FILE__[idx]) || ('\\' == __FILE__[idx]))
        {
            prefix = idx + 1;
        }
    }

    snprintf(path, sizeof(path), "%.*s../evidence/2026-10-28/raw/%s",
             (int32_t)prefix, __FILE__, name);
    FILE *p_file = fopen(path, "r");
    assert(p_file);
    reset();
    establish(); /* Stress the ramp against an already armed observer. */
    char line[8192];
    uint32_t rows = 0;
    uint32_t guarded = 0;
    uint64_t first = 0;
    uint64_t base = t + 20000;

    while (fgets(line, sizeof(line), p_file))
    {
        if ((line[0]<'0') || (line[0]>'9'))
        {
            continue;
        }

        char * field[70];
        uint32_t n = 0;

        for (char * p_v = strtok(line, ","); p_v
             && (n < 70); p_v = strtok(NULL, ","))
        {
            field[n++] = p_v;
        }

        if (69 != n)
        {
            continue;
        }

        uint64_t stamp = strtoull(field[4], NULL, 10);

        if (!rows)
        {
            first = stamp;
        }

        t = base + (stamp - first) - 20000;
        control.running = b_held ? true : 0 != atoi(field[28]);
        control.mode = (drive_mode_t)atoi(field[29]);
        motor.state =
                b_held ? MOTION_STATE_EXECUTING
                        : (motion_state_t)atoi(field[39]);
        motor.fault =
                b_held ? MOTION_FAULT_NONE : (motion_fault_t)atoi(field[40]);
        imu.output.valid = (0 != atoi(field[3])) && (0 == atoi(field[2]));
        float32_t x = strtof(field[9], NULL), y = strtof(field[10], NULL),
        z = strtof(field[11], NULL);
        float32_t mag = hypotf(hypotf(x, y), z);
        bool b_clipped = false;

        for (int32_t idx = 6; idx <= 8; idx++)
        {
            if ((atoi(field[idx]) <= INT16_MIN)
                || (atoi(field[idx]) >= INT16_MAX))
            {
                b_clipped = true;
            }
        }

        imu.output.acceleration_valid = (!b_clipped) && (isfinite(mag));
        imu.output.tilt_valid = (imu.output.acceleration_valid) && (mag >= 0.8f)
                && (mag <= 1.2f);
        imu.output.clipped_axes = b_clipped ? 7 : 0;
        imu.output.pitch_deg = strtof(field[12], NULL);
        imu.output.roll_deg = strtof(field[13], NULL);
        imu.output.filtered_pitch_deg = strtof(field[14], NULL);
        imu.output.filtered_roll_deg = strtof(field[15], NULL);
        imu.output.terrain = atoi(field[16]);
        imu.output.side = atoi(field[17]);
        buddy4_obstruction_output_t out =
                step(strtof(field[36], NULL), strtof(field[37], NULL));
        assert((!out.possible_obstruction) && (0 == out.event_count));

        if ((imu.output.terrain) || (imu.output.side)
            || (fabsf(imu.output.filtered_pitch_deg) > 3) ||
            (fabsf(imu.output.filtered_roll_deg) > 2))
        {
            assert((!out.valid) && (!out.established));
            guarded++;
        }

        rows++;
    }

    fclose(p_file);
    assert((rows == expected_rows) && (guarded > 0));

    if (b_held)
    {
        /* Artificial continuing demand, recorded held tilt/speeds. */
        assert(guarded == rows);
    }

    printf("Recorded %s: %u rows, %u tilt-guarded, no possible obstruction\n",
           name, rows, guarded);
}

static void filtered_onset_and_recovery (void)
{
    /* Representative sustained steps under the existing alpha=.15 filter:
     * filtered tilt crosses the unchanged exit-angle guard before the 100 ms
     * slowdown could confirm. The terrain/side flags may still be level. */
    const float32_t roll_step[] = {0.750f, 1.388f, 1.929f, 2.390f};
    const float32_t pitch_step[] = {1.200f, 2.220f, 3.087f};

    for (int32_t axis = 0; axis < 2; axis++)
    {
        for (int32_t sign = -1; sign <= 1; sign += 2)
        {
            reset();
            establish();
            uint32_t n = axis ? 4 : 3;

            for (uint32_t idx = 0; idx < n; idx++)
            {
                imu.output.pitch_deg = axis ? 0 : sign * 8;
                imu.output.roll_deg = axis ? sign * 5 : 0;
                imu.output.filtered_pitch_deg =
                        axis ? 0 : sign * pitch_step[idx];
                imu.output.filtered_roll_deg = axis ? sign * roll_step[idx] : 0;
                assert(!step(0.1f, 0.1f).possible_obstruction);
            }

            assert((!a.observer.output.valid)
                   && (!a.observer.output.established));

            for (int32_t idx = 0; idx < 20; idx++)
            {
                assert(!step(0, 0).possible_obstruction);
            }

            assert(0 == a.observer.output.event_count);
            imu.output.pitch_deg = imu.output.roll_deg = 0;
            imu.output.filtered_pitch_deg = imu.output.filtered_roll_deg = 0;

            for (int32_t idx = 0; idx < 15; idx++)
            {
                step(3, 3);
            }

            /* Return to flat still needs full warmup. */
            assert(!a.observer.output.established);

            for (int32_t idx = 0; idx < 15; idx++)
            {
                step(3, 3);
            }

            assert(a.observer.output.established);
            pending();
            assert(step(0, 0).possible_obstruction);
        }
    }

    reset();
    establish();
    pending();
    imu.output.filtered_roll_deg = 2.01f;
    assert((!step(0, 0).possible_obstruction)
           && (0 == a.observer.output.event_count));
    /* Hysteresis state still suppresses even at the angle guard boundary. */
    reset();
    establish();
    imu.output.filtered_roll_deg = 2;
    imu.output.side = 1;
    assert(!step(0, 0).valid);
    reset();
    establish();
    imu.output.filtered_pitch_deg = -3;
    imu.output.terrain = -1;
    assert(!step(0, 0).valid);
}

static void reported_contact_confirmation_limits (void)
{
    /* Independent synthetic sequences from the operator's summary; actual
     * recorded-vector replays below exercise the separate 20 ms recovery. */
    reset();
    establish();
    pending(); /* Five low-speed observations span 80 ms. */
    assert((a.observer.output.established)
           && (80 == a.observer.output.slow_ms));
    note(0, 0);
    control.running = false; /* S wins on the would-confirm release. */
    assert((!poll().possible_obstruction)
           && (0 == a.observer.output.event_count));
    assert(!a.observer.output.established);

    for (int32_t transient = 0; transient < 2; transient++)
    {
        reset();
        establish();
        /* -4.41 degrees is below terrain's 6-degree entry, but outside the
         * adapter's stricter 3-degree ramp-onset guard. Both wheels still turn.
         */
        imu.output.filtered_pitch_deg = -4.41f;
        assert((0 == imu.output.terrain) && (0 == imu.output.side));
        assert(!step(2.5f, 2.5f).valid);
        assert(!a.observer.output.established);

        for (int32_t idx = 0; idx < 25; idx++)
        {
            if (transient)
            {
                imu.output.filtered_pitch_deg *= 0.85f;
            }

            buddy4_obstruction_output_t out = step(0, 0);
            assert((!out.established) && (!out.possible_obstruction)
                   && (0 == out.event_count));
        }

        if (transient)
        {
            /* Level recovery while stationary cannot re-establish prior motion.
             * The normal full 500 ms moving window is required again. */
            assert(fabsf(imu.output.filtered_pitch_deg) < 3);

            for (int32_t idx = 0; idx < 25; idx++)
            {
                assert(!step(3, 3).established);
            }

            assert(step(3, 3).established);
            pending();
            assert(step(0, 0).possible_obstruction);
        }
        else
        {
            /* The same filtered pitch and speed loss can be a shallow ramp
             * load: bypassing the guard would accept this ambiguous sequence.
             */
            assert(!a.observer.output.valid);
        }
    }

    /* Isolate the pitch gate: unchanged level input and the same slowdown
     * confirm after the original 100 ms, without changing any thresholds. */
    reset();
    establish();
    step(2.5f, 2.5f);
    pending();
    assert(step(0, 0).possible_obstruction);
    puts("Reported contact synthetic cases: S cancels 80 ms candidate; "
         "prolonged -4.41deg/recovery cancels history; flat 100 ms control "
         "confirms");
}

static void new_recorded_contact (const char * name, uint32_t expected_rows,
                                  bool b_should_detect)
{
    /* Original CSV vectors and sensor gaps, with a synthetic motor/IMU callback
     * phase. Unlogged callbacks and encoder acquisition times are not invented
     * as hardware evidence. Original files remain byte-identical. */
    char path[1024];
    size_t prefix = 0;

    for (size_t idx = 0; __FILE__[idx]; idx++)
    {
        if (('/' == __FILE__[idx]) || ('\\' == __FILE__[idx]))
        {
            prefix = idx + 1;
        }
    }

    snprintf(path, sizeof(path), "%.*s../evidence/2026-11-02/%s",
             (int32_t)prefix,
             __FILE__, name);
    FILE *p_file = fopen(path, "r");
    assert(p_file);
    reset();
    char line[8192];
    uint32_t rows = 0;
    uint64_t event = 0;

    while (fgets(line, sizeof(line), p_file))
    {
        char * data = strstr(line, " -> ");
        data = data ? data + 4 : line;

        if ((*data<'0') || (*data>'9'))
        {
            continue;
        }

        char * field[87];
        uint32_t n = 0;

        for (char * p_v = strtok(data, ","); p_v
             && (n < 87); p_v = strtok(NULL, ","))
        {
            field[n++] = p_v;
        }

        if (86 != n)
        {
            continue;
        }

        t = strtoull(field[10], NULL, 10) - 20000;
        control.running = 0 != atoi(field[34]);
        control.mode = (drive_mode_t)atoi(field[35]);
        motor.state = (motion_state_t)atoi(field[45]);
        motor.fault = (motion_fault_t)atoi(field[46]);
        motor.terrain = (motion_terrain_t)atoi(field[48]);
        motor.terrain_speed_scale = strtof(field[49], NULL);
        motor.left_terrain_duty = strtof(field[50], NULL);
        motor.right_terrain_duty = strtof(field[51], NULL);
        imu.output.valid = 0 != atoi(field[3]);
        imu.output.tilt_valid = 0 != atoi(field[4]);
        imu.output.acceleration_valid = 0 != atoi(field[5]);
        imu.output.clipped_axes = (uint8_t)atoi(field[7]);
        imu.output.pitch_deg = strtof(field[18], NULL);
        imu.output.roll_deg = strtof(field[19], NULL);
        imu.output.filtered_pitch_deg = strtof(field[20], NULL);
        imu.output.filtered_roll_deg = strtof(field[21], NULL);
        imu.output.terrain = atoi(field[22]);
        imu.output.side = atoi(field[23]);
        buddy4_obstruction_output_t out =
                step(strtof(field[42], NULL), strtof(field[43], NULL));

        if (out.possible_obstruction)
        {
            assert(control.running);
            event = out.event_us;
        }

        rows++;
    }

    fclose(p_file);
    assert(rows == expected_rows);

    if (b_should_detect)
    {
        assert((event > 9180000) && (event < 9498713)
               && (1 == a.observer.output.event_count));
    }
    else
    {
        assert((!event) && (0 == a.observer.output.event_count));
    }

    printf("Recorded %s: %u rows, event_us=%llu (callback phase simulated)\n",
           name, rows, (uint64_t)event);
}

static void brief_pitch_pause_safety (void)
{
    reset();
    establish();
    pending();
    imu.output.filtered_pitch_deg = -4.41f;
    assert((!step(0, 0).valid) && (a.observer.paused));
    assert((!a.observer.output.established)
           && (0 == a.observer.output.slow_ms));
    imu.output.filtered_pitch_deg = -2.834f;
    /* The pre-pause 80 ms never contributes: need another full 100 ms. */
    for (int32_t idx = 0; idx < 5; idx++)
    {
        buddy4_obstruction_output_t out = step(0, 0);
        assert((out.valid) && (out.established) && (!out.possible_obstruction));
    }

    assert(step(0, 0).possible_obstruction);
    assert(1 == a.observer.output.event_count);

    reset(); /* A pitch excursion cannot create established history. */
    imu.output.filtered_pitch_deg = -4.41f;
    step(3, 3);
    imu.output.filtered_pitch_deg = 0;

    for (int32_t idx = 0; idx < 30; idx++)
    {
        assert(!step(0, 0).possible_obstruction);
    }

    assert(!a.observer.output.established);

    reset();
    establish();
    imu.output.filtered_pitch_deg = 4.41f;
    step(0, 0);
    step(0, 0);
    step(0, 0); /* Persistent for 40 ms cancels. */
    assert((!a.observer.paused) && (!a.observer.output.established));
    imu.output.filtered_pitch_deg = 0;

    for (int32_t idx = 0; idx < 20; idx++)
    {
        assert(!step(0, 0).possible_obstruction);
    }

    assert(!a.observer.output.event_count);

    /* Every hard cancellation still wins while history is quarantined. */
    for (int32_t kind = 0; kind < 13; kind++)
    {
        reset();
        establish();
        imu.output.filtered_pitch_deg = -4.41f;
        step(0, 0);
        assert(a.observer.paused);
        imu.output.filtered_pitch_deg = 0;

        if (0 == kind)
        {
            control.running = false;
        }

        if (1 == kind)
        {
            control.mode = DRIVE_TURN_LEFT;
        }

        if (2 == kind)
        {
            motor.fault = MOTION_FAULT_STALL;
        }

        if (3 == kind)
        {
            ++control.started_us;
        }

        if (4 == kind)
        {
            imu.output.valid = false;
        }

        if (5 == kind)
        {
            imu.output.tilt_valid = false;
        }

        if (6 == kind)
        {
            imu.output.acceleration_valid = false;
        }

        if (7 == kind)
        {
            imu.output.clipped_axes = 1;
        }

        if (8 == kind)
        {
            imu.output.filtered_roll_deg = 2.01f;
        }

        if (9 == kind)
        {
            imu.output.terrain = 1;
        }

        if (10 == kind)
        {
            motor.terrain_speed_scale = 0.8f;
        }

        if (11 == kind)
        {
            skipped++;
        }

        if (12 == kind)
        {
            t += 40001;
        }

        step(0, 0);
        assert((!a.observer.paused) && (!a.observer.output.established));

        for (int32_t idx = 0; idx < 10; idx++)
        {
            assert(!step(0, 0).possible_obstruction);
        }

        assert(!a.observer.output.event_count);
    }

    /* Pauses and unequal-wheel grace cannot be chained in either order. */
    reset();
    establish();
    step(0, 3);
    imu.output.filtered_pitch_deg = 4.41f;
    step(0, 0);
    assert((!a.observer.paused) && (!a.observer.output.established));
    reset();
    establish();
    imu.output.filtered_pitch_deg = 4.41f;
    step(0, 3);
    assert((!a.observer.paused) && (!a.observer.output.established));
    reset();
    establish();
    imu.output.filtered_pitch_deg = 4.41f;
    step(0, 0);
    imu.output.filtered_pitch_deg = 0;
    step(0, 3);
    assert((!a.observer.output.established) && (!a.observer.asymmetric));
    puts("Brief pitch pause: no evidence bridging; full confirmation after "
         "recovery; persistent tilt, faults, turns and missing updates cancel");
}

int main (void)
{
    brief_pitch_pause_safety();
    new_recorded_contact("foam_contact_1510.txt", 773, false);
    new_recorded_contact("foam_contact_1513.txt", 669, true);
    new_recorded_contact("normal_driving_1346.txt", 996, false);
    reported_contact_confirmation_limits();
    actual_foam_transition();
    recorded_ramp("left_moving_after.txt", 458, false);
    recorded_ramp("right_moving_after.txt", 394, false);
    recorded_ramp("left_held_after.txt", 373, true);
    recorded_ramp("right_held_after.txt", 574, true);
    phased_20ms_updates();
    waiting_is_not_evidence();
    recorded_angles_and_cooperative_schedule();
    filtered_onset_and_recovery();
    ordinary_and_blocked();
    command_cancellation();
    ramps_turns_faults();
    timing_and_invalid();
    puts("Buddy 4 SDK obstruction adapter tests passed: command ordering, "
         "timing qualification, terrain/quality exclusions and history");
    return 0;
}

/*** end of file ***/

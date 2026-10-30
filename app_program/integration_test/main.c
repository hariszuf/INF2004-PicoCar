#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/i2c.h"
#include "motion_config.h"
#include "motion_control.h"
#include "line_config.h"
#include "line_follow.h"
#include "line_sensors.h"
#include "buddy4_imu.h"
#include "buddy4_motion.h"
#include "buddy4_encoder_yaw.h"
#include "buddy4_impact.h"
#include "scheduler.h"
#include "drive_control.h"
#include "terrain_link.h"

static drive_control_t control;
static int bus_write(void *ctx, uint8_t a, const uint8_t *p, size_t n, bool ns) {
    (void)ctx;
    return i2c_write_timeout_us(i2c0, a, p, n, ns, BUDDY4_I2C_TIMEOUT_US);
}
static int bus_read(void *ctx, uint8_t a, uint8_t *p, size_t n, bool ns) {
    (void)ctx;
    return i2c_read_timeout_us(i2c0, a, p, n, ns, BUDDY4_I2C_TIMEOUT_US);
}
static void runtime(task_t *t, uint64_t start) {
    uint32_t elapsed = (uint32_t)(time_us_64() - start);
    if (elapsed > t->runtime_max_us) t->runtime_max_us = elapsed;
}
int main(void) {
    /* Brake before USB or sensor setup. Boot never starts the line follower. */
    (void)motion_init();
    line_follow_init();
    drive_init(&control);
    stdio_init_all();
    i2c_init(i2c0, 100000);
    gpio_set_function(16, GPIO_FUNC_I2C);
    gpio_set_function(17, GPIO_FUNC_I2C);
    gpio_pull_up(16); gpio_pull_up(17);
    buddy4_encoder_yaw_t encoder_yaw;
    buddy4_encoder_yaw_init(&encoder_yaw, buddy4_encoder_yaw_default_config());
    buddy4_motion_t motion_class;
    buddy4_motion_init(&motion_class, buddy4_motion_default_config());
    buddy4_impact_t impact;
    buddy4_impact_init(&impact, buddy4_impact_default_config());
    buddy4_imu_t imu;
    buddy4_bus_t bus = {NULL, bus_write, bus_read};
    buddy4_init(&imu, bus, buddy4_default_config());
    task_t motor = {.period = MOTION_UPDATE_PERIOD_MS * 1000u};
    task_t line = {.period = LINE_UPDATE_PERIOD_MS * 1000u};
    task_t sensor = {.period = BUDDY4_PERIOD_US};
    uint64_t retry = 0, last_fresh = 0;
    bool has_fresh = false, was_usb = false, header = false;
    uint32_t seq = 0;
    buddy4_result_t result = BUDDY4_NOT_READY;
    for (;;) {
        bool usb = stdio_usb_connected();
        
        if (usb && !was_usb) header = true;
        was_usb = usb;
        uint64_t now = time_us_64();
        drive_safety(&control, usb, has_fresh, now, last_fresh);
        if (due(&motor, now)) {
            uint64_t start = time_us_64();
            motion_update();
            motion_status_t observed; motion_get_status(&observed);
            uint64_t speed_sample_us = time_us_64();
            buddy4_encoder_yaw_update(&encoder_yaw, observed.left_speed_cm_s,
                observed.right_speed_cm_s, true, speed_sample_us, speed_sample_us);
            /* Buddy 2 snapshot has no timestamp: stamp only actual update releases. */
            buddy4_motion_update(&motion_class, observed.left_speed_cm_s,
                observed.right_speed_cm_s, true, speed_sample_us, speed_sample_us);
            drive_safety(&control, usb, has_fresh, time_us_64(), last_fresh);
            runtime(&motor, start);
        }
        if (due(&line, time_us_64())) {
            uint64_t start = time_us_64();
            drive_line_update(&control);
            runtime(&line, start);
        }
        bool sampled = false;
        if (due(&sensor, time_us_64())) {
            uint64_t start = time_us_64();
            /* Configuration has multiple bounded transfers; only retry stopped. */
            if (!control.running && !imu.ready && start >= retry) {
                (void)buddy4_configure(&imu);
                retry = time_us_64() + 1000000u;
            }
            result = buddy4_poll(&imu, time_us_64());
            /* Buddy 4 -> Buddy 2: slope / hump decides speed and power. */
            if (result == BUDDY4_FRESH && !imu.output.tilt_valid) {
                /* Quality failure is not a bus error. Publish invalid terrain. */
                const motion_terrain_input_t unavailable = {0};
                (void)motion_set_terrain(&unavailable);
            } else terrain_link_feed(&imu, result);
            if (result == BUDDY4_FRESH && imu.output.valid) {
                last_fresh = imu.output.timestamp_us; has_fresh = true;
            }
            buddy4_impact_update(&impact, imu.output.accel_g[0], imu.output.accel_g[1],
                imu.output.accel_g[2], result == BUDDY4_FRESH && imu.output.valid && imu.output.acceleration_valid,
                imu.output.timestamp_us, time_us_64());
            runtime(&sensor, start);
            sampled = true;
        }
        /* At most one character per pass, zero timeout. No USB reads in updates.
           Uppercase G starts; S/space stops; R explicitly clears a stopped latch. */
        int c = usb ? getchar_timeout_us(0) : PICO_ERROR_TIMEOUT;
        drive_command(&control, c, usb, imu.ready && imu.output.valid &&
            has_fresh && time_us_64() - last_fresh < 100000u, time_us_64());
        drive_safety(&control, usb, has_fresh, time_us_64(), last_fresh);
        /* Bounded best-effort output outside updates. Slow readers may lose data. */
        if (usb && header) {
            printf("# standalone_Pico_SDK,commands=M_manual_L_line_A_left_test_D_right_test_R_then_G_stop_S_or_space,serial_stop_is_not_hardware_estop\n");
            printf("# calibration=buddy4_default_config,mounting_offsets_need_recalibration=%d,geometry_confirmed=%d\n",
                   imu.config.mounting_offsets_need_recalibration, imu.config.centre_spacing_confirmed);
            printf("seq,time_us,result,valid,tilt_valid,acceleration_valid,measurement_quality,clipped_axes,clipped_samples,unsuitable_samples,sample_us,imu_errors,raw_x,raw_y,raw_z,ax_g,ay_g,az_g,pitch_deg,roll_deg,pitch_filtered,roll_filtered,terrain,side,transient_g,bump,height_cm,event_side,peak_confirmed,peak_roll,peak_height,median_candidate_valid,median_candidate_roll_deg,median_candidate_height_cm,running,mode,line_state,line_mask,left_cmd,right_cmd,left_count,right_count,left_speed,right_speed,distance_cm,motor_state,motor_fault,stop_fault,terrain_mode,terrain_speed_scale,left_terrain_duty,right_terrain_duty,motor_skipped,line_skipped,imu_skipped,motor_late_us,line_late_us,imu_late_us,motor_runtime_us,line_runtime_us,imu_runtime_us,encoder_motion_state,encoder_motion_valid,encoder_motion_sample_us,possible_impact_valid,possible_impact_event,possible_impact_count,possible_impact_sample_us,possible_impact_event_us,possible_impact_magnitude_g,possible_impact_baseline_g,possible_impact_deviation_g,encoder_yaw_rate_estimate_rad_s,encoder_yaw_rate_estimate_deg_s,encoder_yaw_rate_estimate_valid,encoder_yaw_rate_estimate_sample_us,selected_target_left_cm_s,selected_target_right_cm_s,control_reason\n");
            header = false;
        }
        if (usb && control.event_pending) {
            printf("# CONTROL,time_us=%llu,event=%s,original_fault=%lu,mode=%d,manual_speed_cm_s=%.2f,selected_target_left_cm_s=%.2f,selected_target_right_cm_s=%.2f,turn_test_limit_us=%lu\n",
                   (unsigned long long)time_us_64(), control.reason, (unsigned long)control.stop_fault, control.mode, (double)MANUAL_SPEED_CM_S,
                   control.selected_left_cm_s,control.selected_right_cm_s,(unsigned long)TURN_TEST_LIMIT_US);
            control.event_pending = false;
        }
        if (usb && sampled) {
            const buddy4_output_t *o = &imu.output;
            motion_status_t m; motion_get_status(&m);
            line_status_t l; line_follow_get_status(&l);
            buddy4_motion_output_t mc = buddy4_motion_get(&motion_class, time_us_64());
            buddy4_encoder_yaw_output_t ey = buddy4_encoder_yaw_get(&encoder_yaw, time_us_64());
            buddy4_impact_output_t pi = buddy4_impact_get(&impact, time_us_64());
            if (o->event_ended) printf("# SIDE_EVENT_END,start_us=%llu,end_us=%llu,side=%d,confirmed=%d,peak_roll=%.3f,peak_height_cm=%.3f,median_candidate_valid=%d,median_candidate_roll_deg=%.3f,median_candidate_height_cm=%.3f\n",
                (unsigned long long)o->ended_event.start_us, (unsigned long long)o->timestamp_us,
                o->ended_event.side, o->ended_event.peak_confirmed,
                o->ended_event.peak_roll_deg, o->ended_event.peak_height_cm,
                o->ended_event.median_candidate_valid,o->ended_event.median_candidate_roll_deg,o->ended_event.median_candidate_height_cm);
            printf("%lu,%llu,%d,%d,%d,%d,%s,%u,%lu,%lu,%llu,%lu,%d,%d,%d,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.4f,%d,%.3f,%d,%d,%.3f,%.3f,%d,%.3f,%.3f,%d,%d,%d,%u,%.3f,%.3f,%ld,%ld,%.3f,%.3f,%.3f,%d,%d,%lu,%d,%.3f,%.3f,%.3f,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%s,%d,%llu,%d,%d,%lu,%llu,%llu,%.4f,%.4f,%.4f,%.5f,%.4f,%d,%llu,%.3f,%.3f,%s\n",
                (unsigned long)seq++, (unsigned long long)time_us_64(), result,o->valid,
                o->tilt_valid,o->acceleration_valid,buddy4_quality_name(o->quality),
                (unsigned)o->clipped_axes,(unsigned long)o->clipped_samples,(unsigned long)o->unsuitable_samples,
                (unsigned long long)o->timestamp_us,(unsigned long)o->error_count,
                o->raw[0],o->raw[1],o->raw[2],o->accel_g[0],o->accel_g[1],o->accel_g[2],
                o->pitch_deg,o->roll_deg,o->filtered_pitch_deg,o->filtered_roll_deg,
                o->terrain,o->side,o->transient_g,o->transient_event,o->wheel_height_cm,
                o->event.side,o->event.peak_confirmed,o->event.peak_roll_deg,o->event.peak_height_cm,
                o->event.median_candidate_valid,o->event.median_candidate_roll_deg,o->event.median_candidate_height_cm,
                control.running,control.mode,l.state,(unsigned)line_sensors_read(),control.mode != DRIVE_LINE ? (control.running ? control.selected_left_cm_s : 0) : l.left_cmd_cm_s,control.mode != DRIVE_LINE ? (control.running ? control.selected_right_cm_s : 0) : l.right_cmd_cm_s,
                (long)m.left_count,(long)m.right_count,m.left_speed_cm_s,m.right_speed_cm_s,m.distance_cm,
                m.state,m.fault,(unsigned long)control.stop_fault,
                (int)m.terrain,m.terrain_speed_scale,m.left_terrain_duty,m.right_terrain_duty,
                (unsigned long)motor.skipped,(unsigned long)line.skipped,(unsigned long)sensor.skipped,
                (unsigned long)motor.late_max_us,(unsigned long)line.late_max_us,(unsigned long)sensor.late_max_us,
                (unsigned long)motor.runtime_max_us,(unsigned long)line.runtime_max_us,(unsigned long)sensor.runtime_max_us,
                buddy4_motion_name(mc.state),mc.valid,(unsigned long long)mc.sample_us,
                pi.valid,pi.possible_impact,(unsigned long)pi.event_count,
                (unsigned long long)pi.sample_us,(unsigned long long)pi.event_us,
                pi.magnitude_g,pi.baseline_g,pi.deviation_g,
                ey.encoder_yaw_rate_estimate_rad_s,ey.encoder_yaw_rate_estimate_deg_s,
                ey.valid,(unsigned long long)ey.sample_us,
                control.selected_left_cm_s,control.selected_right_cm_s,control.reason);
        }
        tight_loop_contents();
    }
}




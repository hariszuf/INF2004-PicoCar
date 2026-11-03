#include "buddy4_obstruction_sdk.h"
#include <math.h>

void buddy4_obstruction_sdk_init(buddy4_obstruction_sdk_t *a) {
    *a=(buddy4_obstruction_sdk_t){0};buddy4_obstruction_init(&a->observer);
}
void buddy4_obstruction_sdk_note_update(buddy4_obstruction_sdk_t *a,
    const motion_status_t *status,const drive_control_t *demand,
    uint64_t begin,uint64_t end,uint32_t skipped) {
    a->timing_ok=a->have_update && !a->pending && begin>a->begin_us &&
        begin-a->begin_us>=10000u && begin-a->begin_us<=30000u &&
        end>=begin && end-begin<=5000u && skipped==a->skipped;
    a->status=*status;a->demand=*demand;
    a->begin_us=begin;a->end_us=end;a->skipped=skipped;
    a->have_update=a->pending=true;
}
buddy4_obstruction_output_t buddy4_obstruction_sdk_poll(buddy4_obstruction_sdk_t *a,
    const drive_control_t *c,const buddy4_imu_t *imu,bool usb,uint64_t now) {
    const buddy4_output_t *o=&imu->output;
    const motion_status_t *m=&a->status;
    bool usable=a->have_update && usb && c->running && c->mode==DRIVE_MANUAL &&
        c->stop_fault==MOTION_FAULT_NONE && a->demand.running &&
        a->demand.mode==c->mode && a->demand.started_us==c->started_us &&
        a->demand.selected_left_cm_s==c->selected_left_cm_s &&
        a->demand.selected_right_cm_s==c->selected_right_cm_s &&
        m->state==MOTION_STATE_EXECUTING && m->fault==MOTION_FAULT_NONE &&
        isfinite(m->left_speed_cm_s) && isfinite(m->right_speed_cm_s) &&
        m->terrain==MOTION_TERRAIN_FLAT && isfinite(m->terrain_speed_scale) &&
        fabsf(m->terrain_speed_scale-1)<0.001f &&
        isfinite(m->left_terrain_duty) && isfinite(m->right_terrain_duty) &&
        fabsf(m->left_terrain_duty)<0.001f && fabsf(m->right_terrain_duty)<0.001f &&
        imu->ready && o->valid && o->acceleration_valid && o->tilt_valid &&
        !o->clipped_axes && o->terrain==0 && o->side==0 &&
        isfinite(o->pitch_deg) && isfinite(o->roll_deg) &&
        isfinite(o->filtered_pitch_deg) && isfinite(o->filtered_roll_deg) &&
        /* Roll/terrain/quality guards cancel immediately. A brief pitch-only
         * excursion is passed as an explicit pause, never as level evidence.
         * Invalid tilt is never treated as flat, even if state flags are zero. */
        fabsf(o->filtered_roll_deg)<=2 &&
        o->timestamp_us<=now && now-o->timestamp_us<=40000u &&
        a->end_us<=now && now-a->end_us<=40000u;
    if(usable) {
        uint64_t skew=o->timestamp_us>a->end_us?o->timestamp_us-a->end_us:
                                                     a->end_us-o->timestamp_us;
        usable=skew<=30000u;
    }
    if(!usable) {
        buddy4_obstruction_cancel(&a->observer,now);
        a->pending=false;
    } else if(a->pending) {
        bool imu_new=!a->has_imu || o->timestamp_us>a->last_imu_us;
        if(!a->timing_ok || (a->has_imu && o->timestamp_us<a->last_imu_us)) {
            buddy4_obstruction_cancel(&a->observer,now);a->pending=false;
        } else if(imu_new) {
            buddy4_obstruction_input_t in={
                .valid=true,.eligible=true,.observation_us=a->end_us,
                .pause_motion=fabsf(o->filtered_pitch_deg)>3,
                .demand_id=c->started_us,
                .command_left_cm_s=c->selected_left_cm_s,
                .command_right_cm_s=c->selected_right_cm_s,
                .left_cm_s=m->left_speed_cm_s,.right_cm_s=m->right_speed_cm_s};
            buddy4_obstruction_update(&a->observer,&in,now);a->pending=false;
            a->has_imu=true;a->last_imu_us=o->timestamp_us;
        }
        /* Equal IMU timestamp is ordinary phase skew between two 20 ms tasks.
         * Keep the motor snapshot pending, without advancing evidence, until a
         * NEW IMU arrives. All eligibility/age/skew checks above and the core's
         * getter expiry still apply on every poll. A second motor callback while
         * waiting is unqualified by note_update(), never silently combined. */
    }
    return buddy4_obstruction_get(&a->observer,now);
}

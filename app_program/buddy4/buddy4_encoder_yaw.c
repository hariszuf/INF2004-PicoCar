#include "buddy4_encoder_yaw.h"
#include <float.h>
#include <math.h>
static bool clock_ok(buddy4_encoder_yaw_t *d, uint64_t now) {
    if (d->has_clock && now < d->clock_us) {
        d->output.valid=false; return false;
    }
    d->has_clock=true; d->clock_us=now; return true;
}
static bool geometry_ok(buddy4_encoder_yaw_config_t cfg) {
    return isfinite(cfg.centre_spacing_cm) && cfg.centre_spacing_cm > 0 && cfg.stale_us > 0;
}
buddy4_encoder_yaw_config_t buddy4_encoder_yaw_default_config(void) {
    return (buddy4_encoder_yaw_config_t){11.3f,100000u};
}
bool buddy4_encoder_yaw_init(buddy4_encoder_yaw_t *d,buddy4_encoder_yaw_config_t cfg) {
    *d=(buddy4_encoder_yaw_t){.config=cfg,.configured=geometry_ok(cfg)};
    return d->configured;
}
void buddy4_encoder_yaw_update(buddy4_encoder_yaw_t *d,float left,float right,
                              bool valid,uint64_t sample,uint64_t now) {
    d->output.valid=false;
    if (!clock_ok(d,now) || !d->configured || !geometry_ok(d->config) || !valid ||
        !isfinite(left) || !isfinite(right) || sample > now ||
        now-sample >= d->config.stale_us ||
        (d->has_sample_time && sample <= d->last_sample_us)) return;
    /* Double intermediates avoid overflow in right-left for finite float inputs. */
    double rad=((double)right-(double)left)/(double)d->config.centre_spacing_cm;
    double deg=rad*(180.0/3.14159265358979323846);
    if (!isfinite(rad) || !isfinite(deg) || fabs(rad)>FLT_MAX || fabs(deg)>FLT_MAX) return;
    d->has_sample_time=true; d->last_sample_us=sample;
    d->output=(buddy4_encoder_yaw_output_t){true,sample,(float)rad,(float)deg};
}
buddy4_encoder_yaw_output_t buddy4_encoder_yaw_get(buddy4_encoder_yaw_t *d,uint64_t now) {
    if (!clock_ok(d,now) || !d->configured || !geometry_ok(d->config) ||
        !d->has_sample_time || now < d->output.sample_us ||
        now-d->output.sample_us >= d->config.stale_us) d->output.valid=false;
    return d->output;
}

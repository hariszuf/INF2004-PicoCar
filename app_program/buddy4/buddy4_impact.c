#include "buddy4_impact.h"
#include <math.h>
static void invalidate(buddy4_impact_t *d) {
    d->output.valid = d->output.possible_impact = false;
    d->primed = d->armed = false;
    d->pending_samples = d->calm_samples = 0;
}
static bool clock_ok(buddy4_impact_t *d, uint64_t now) {
    if (d->has_clock && now < d->clock_us) { invalidate(d); return false; }
    d->has_clock = true; d->clock_us = now; return true;
}
buddy4_impact_config_t buddy4_impact_default_config(void) {
    return (buddy4_impact_config_t){0.30f,0.10f,20000u,40000u,500000u,500000u};
}
bool buddy4_impact_init(buddy4_impact_t *d, buddy4_impact_config_t cfg) {
    *d = (buddy4_impact_t){.config=cfg};
    d->configured = isfinite(cfg.trigger_g) && isfinite(cfg.release_g) &&
        cfg.release_g >= 0 && cfg.trigger_g > cfg.release_g &&
        cfg.confirm_us > 0 && cfg.max_gap_us > 0 && cfg.cooldown_us > 0 &&
        cfg.baseline_tau_us > 0;
    return d->configured;
}
void buddy4_impact_update(buddy4_impact_t *d, float x,float y,float z,
                         bool valid,uint64_t sample,uint64_t now) {
    d->output.possible_impact = false;
    if (!clock_ok(d,now)) return;
    float mag = hypotf(hypotf(x,y),z);
    if (!d->configured || !valid || !isfinite(mag) ||
        sample > now || now-sample > d->config.max_gap_us ||
        (d->has_time && sample <= d->last_time_us)) {
        invalidate(d); return;
    }
    uint64_t dt = d->has_time ? sample-d->last_time_us : 0;
    if (d->has_time && dt > d->config.max_gap_us) invalidate(d);
    d->last_time_us=sample; d->has_time=true;
    d->output.sample_us=sample; d->output.magnitude_g=mag;
    d->output.valid=true;
    if (!d->primed) {
        d->baseline_g=d->output.baseline_g=mag; d->output.deviation_g=0;
        d->primed=true; /* Startup can arm; recovery must observe calm samples. */
        d->armed = d->output.event_count == 0;
        return;
    }
    d->output.baseline_g=d->baseline_g; /* Reference used for this sample. */
    float signal=fabsf(mag-d->baseline_g);
    d->output.deviation_g=signal;
    if (signal <= d->config.release_g) {
        if (d->calm_samples < 2) ++d->calm_samples;
        if (d->calm_samples >= 2 && (d->output.event_count == 0 ||
            sample-d->output.event_us >= d->config.cooldown_us)) d->armed=true;
    } else d->calm_samples=0;
    if (signal >= d->config.trigger_g && d->armed) {
        if (!d->pending_samples) d->pending_since_us=sample;
        if (d->pending_samples < 2) ++d->pending_samples;
        if (d->pending_samples >= 2 && sample-d->pending_since_us >= d->config.confirm_us) {
            d->output.possible_impact=true;
            if (d->output.event_count != UINT32_MAX) ++d->output.event_count;
            d->output.event_us=sample;
            d->armed=false; d->pending_samples=0;
        }
    } else d->pending_samples=0;
    /* Freeze baseline during elevated transients; track gravity/nonzero DC gently. */
    if (signal < d->config.trigger_g) {
        float alpha=(float)dt/((float)d->config.baseline_tau_us+(float)dt);
        d->baseline_g += alpha*(mag-d->baseline_g);
    }
}
buddy4_impact_output_t buddy4_impact_get(buddy4_impact_t *d,uint64_t now) {
    if (!clock_ok(d,now) || !d->primed || now < d->output.sample_us ||
        now-d->output.sample_us > d->config.max_gap_us) invalidate(d);
    return d->output;
}


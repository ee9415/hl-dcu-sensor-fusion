#include "pbv_dcu/safety_supervisor.h"

#include <math.h>
#include <string.h>

static PbvBehaviorGate max_gate(PbvBehaviorGate a, PbvBehaviorGate b) {
    return (a > b) ? a : b;
}

void pbv_safety_init(PbvSafetyContext *ctx) {
    if (ctx != NULL) {
        memset(ctx, 0, sizeof(*ctx));
    }
}

PbvSafetyOutput pbv_safety_update(
    PbvSafetyContext *ctx,
    const PbvDcuConfig *cfg,
    const PbvPerceptionSnapshot *snapshot,
    const PbvFaultOutput *fault,
    const PbvStateOutput *state
) {
    PbvSafetyOutput out;
    float lane_conf = 0.0f;

    memset(&out, 0, sizeof(out));
    if (cfg == NULL || snapshot == NULL || fault == NULL || state == NULL) {
        out.enforced_gate = PBV_GATE_STOP;
        out.override_stop = true;
        return out;
    }

    out.enforced_gate = max_gate(fault->requested_gate, state->behavior_gate);
    out.max_speed_mps = cfg->nominal_speed_mps;
    out.max_steer_norm = cfg->max_steer_norm;

    if (snapshot->frame_age_ms > cfg->max_frame_age_ms) {
        out.enforced_gate = max_gate(out.enforced_gate, PBV_GATE_SLOW);
        out.intervention_bits |= PBV_INTERVENTION_STALE_DATA;
    }

    if (snapshot->traffic_light_state == PBV_TL_RED &&
        snapshot->stop_line_distance_m >= 0.0f &&
        snapshot->stop_line_distance_m < cfg->stop_line_trigger_m) {
        out.enforced_gate = PBV_GATE_STOP;
        out.intervention_bits |= PBV_INTERVENTION_RED_LIGHT | PBV_INTERVENTION_STOP_LINE;
    } else if (snapshot->traffic_light_state == PBV_TL_YELLOW &&
               snapshot->stop_line_distance_m >= 0.0f &&
               snapshot->stop_line_distance_m < (cfg->stop_line_trigger_m * 1.8f)) {
        out.enforced_gate = max_gate(out.enforced_gate, PBV_GATE_SLOW);
        out.intervention_bits |= PBV_INTERVENTION_STOP_LINE;
    }

    if (snapshot->target_valid && snapshot->target_distance_m > 0.0f) {
        if (snapshot->target_distance_m < cfg->target_brake_distance_m) {
            out.enforced_gate = PBV_GATE_STOP;
            out.intervention_bits |= PBV_INTERVENTION_TARGET_NEAR;
        } else if (snapshot->target_distance_m < cfg->target_slow_distance_m) {
            out.enforced_gate = max_gate(out.enforced_gate, PBV_GATE_SLOW);
            out.intervention_bits |= PBV_INTERVENTION_TARGET_NEAR;
        }
    }

    lane_conf = snapshot->lane_valid ? snapshot->lane_confidence : 0.0f;
    if (lane_conf < cfg->min_lane_conf) {
        out.max_steer_norm = fminf(out.max_steer_norm, 0.35f);
        out.enforced_gate = max_gate(out.enforced_gate, PBV_GATE_SLOW);
        out.intervention_bits |= PBV_INTERVENTION_DEGRADED_LANE;
    }

    if (fault->fail_safe) {
        out.enforced_gate = PBV_GATE_STOP;
        out.intervention_bits |= PBV_INTERVENTION_FAILSAFE;
    }

    switch (out.enforced_gate) {
        case PBV_GATE_ALLOW:
            out.max_speed_mps = cfg->nominal_speed_mps;
            break;
        case PBV_GATE_SLOW:
            out.max_speed_mps = cfg->slow_speed_mps;
            break;
        case PBV_GATE_STOP:
        default:
            out.max_speed_mps = 0.0f;
            out.override_stop = true;
            break;
    }

    if (ctx != NULL) {
        ctx->last_intervention_bits = out.intervention_bits;
    }
    return out;
}

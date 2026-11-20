#include "pbv_dcu/decision_control.h"

#include <math.h>
#include <string.h>

static float clampf(float v, float lo, float hi) {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

void pbv_decision_init(PbvDecisionContext *ctx) {
    if (ctx != NULL) {
        memset(ctx, 0, sizeof(*ctx));
    }
}

PbvControlCommand pbv_decision_update(
    PbvDecisionContext *ctx,
    const PbvDcuConfig *cfg,
    const PbvPerceptionSnapshot *snapshot,
    const PbvFaultOutput *fault,
    const PbvStateOutput *state,
    const PbvSafetyOutput *safety
) {
    PbvControlCommand cmd;
    float steering = 0.0f;
    float target_speed = 0.0f;
    float brake = 0.0f;

    memset(&cmd, 0, sizeof(cmd));
    if (cfg == NULL || snapshot == NULL || fault == NULL || state == NULL || safety == NULL) {
        cmd.source = PBV_CMD_SOURCE_FAULT;
        cmd.brake_norm = 0.7f;
        return cmd;
    }

    if (ctx != NULL) {
        ctx->heartbeat += 1u;
        cmd.heartbeat = ctx->heartbeat;
    }

    if (snapshot->lane_valid) {
        steering = -(cfg->k_offset * snapshot->lane_offset_m + cfg->k_heading * snapshot->lane_heading_rad);
        steering = clampf(steering, -safety->max_steer_norm, safety->max_steer_norm);
    }

    target_speed = safety->max_speed_mps;

    if (snapshot->target_valid && snapshot->target_distance_m > 0.0f) {
        if (snapshot->target_distance_m < cfg->target_brake_distance_m) {
            target_speed = 0.0f;
            brake = 0.6f;
        } else if (snapshot->target_distance_m < cfg->target_slow_distance_m) {
            float span = cfg->target_slow_distance_m - cfg->target_brake_distance_m;
            float alpha = (snapshot->target_distance_m - cfg->target_brake_distance_m) / fmaxf(0.01f, span);
            target_speed = fminf(target_speed, cfg->slow_speed_mps * clampf(alpha, 0.0f, 1.0f));
        }
    }

    if (snapshot->stop_line_distance_m >= 0.0f && state->drive_state == PBV_DRIVE_APPROACH_STOP) {
        float alpha = clampf(snapshot->stop_line_distance_m / fmaxf(0.01f, cfg->stop_line_trigger_m), 0.0f, 1.0f);
        target_speed = fminf(target_speed, cfg->slow_speed_mps * alpha);
        brake = fmaxf(brake, (1.0f - alpha) * 0.3f);
    }

    if (safety->enforced_gate == PBV_GATE_STOP || state->drive_state == PBV_DRIVE_STOPPED || fault->fail_safe) {
        target_speed = 0.0f;
        brake = fmaxf(brake, 0.5f);
        cmd.source = fault->fail_safe ? PBV_CMD_SOURCE_FAULT : PBV_CMD_SOURCE_SAFETY;
    } else {
        cmd.source = (safety->enforced_gate == PBV_GATE_SLOW) ? PBV_CMD_SOURCE_SAFETY : PBV_CMD_SOURCE_NORMAL;
    }

    if (!snapshot->lane_valid) {
        steering = 0.0f;
    }

    cmd.target_speed_mps = clampf(target_speed, 0.0f, cfg->nominal_speed_mps);
    cmd.steering_norm = clampf(steering, -cfg->max_steer_norm, cfg->max_steer_norm);
    cmd.brake_norm = clampf(brake, 0.0f, 1.0f);
    return cmd;
}

#include "pbv_dcu/fault_management.h"

#include <string.h>

void pbv_fault_init(PbvFaultContext *ctx) {
    if (ctx != NULL) {
        memset(ctx, 0, sizeof(*ctx));
    }
}

static PbvBehaviorGate max_gate(PbvBehaviorGate a, PbvBehaviorGate b) {
    return (a > b) ? a : b;
}

PbvFaultOutput pbv_fault_update(
    PbvFaultContext *ctx,
    const PbvDcuConfig *cfg,
    const PbvPerceptionSnapshot *snapshot,
    uint64_t now_ms,
    bool new_frame
) {
    PbvFaultOutput out;
    uint32_t age_ms = 0u;

    memset(&out, 0, sizeof(out));
    out.now_ms = now_ms;
    out.sensor_health = PBV_SENSOR_OK;
    out.requested_gate = PBV_GATE_ALLOW;
    out.effective_lane_confidence = (snapshot != NULL) ? snapshot->lane_confidence : 0.0f;

    if (ctx == NULL || cfg == NULL || snapshot == NULL) {
        out.fail_safe = true;
        out.sensor_health = PBV_SENSOR_FAILSAFE;
        out.requested_gate = PBV_GATE_STOP;
        out.fault_bits = PBV_FAULT_CAMERA_MISSING | PBV_FAULT_FRAME_TIMEOUT;
        return out;
    }

    age_ms = (now_ms > snapshot->stamp_ms) ? (uint32_t)(now_ms - snapshot->stamp_ms) : snapshot->frame_age_ms;

    if (!ctx->initialized) {
        ctx->initialized = true;
        ctx->last_seq = snapshot->seq;
        ctx->last_rx_ms = now_ms;
    }

    if (new_frame) {
        if (snapshot->seq > ctx->last_seq + 1u) {
            out.lost_counter = snapshot->seq - ctx->last_seq - 1u;
            out.fault_bits |= PBV_FAULT_PACKET_LOSS;
        }
        ctx->last_seq = snapshot->seq;
        ctx->last_rx_ms = now_ms;
    }

    out.timeout_fault = (age_ms > cfg->max_frame_age_ms);
    out.camera_fault = !snapshot->camera_ok;
    out.fps_fault = (snapshot->frame_fps > 0.1f && snapshot->frame_fps < 4.0f);
    out.lane_fault = (!snapshot->lane_valid || snapshot->lane_confidence < cfg->min_lane_conf);

    if (out.camera_fault) {
        out.fault_bits |= PBV_FAULT_CAMERA_MISSING;
    }
    if (out.timeout_fault) {
        out.fault_bits |= PBV_FAULT_FRAME_TIMEOUT;
    }
    if (out.fps_fault) {
        out.fault_bits |= PBV_FAULT_LOW_FPS;
    }
    if (out.lane_fault) {
        out.fault_bits |= PBV_FAULT_LANE_INVALID;
    }
    if (!snapshot->target_valid) {
        out.fault_bits |= PBV_FAULT_TARGET_INVALID;
    }

    if (out.camera_fault || out.timeout_fault || out.lane_fault || out.fps_fault) {
        ctx->bad_frame_streak += 1u;
    } else {
        ctx->bad_frame_streak = 0u;
    }
    out.consecutive_bad_frames = ctx->bad_frame_streak;

    if (age_ms > cfg->hard_timeout_ms || out.camera_fault || ctx->bad_frame_streak >= 10u) {
        out.fail_safe = true;
    }

    if (out.fail_safe) {
        out.sensor_health = PBV_SENSOR_FAILSAFE;
        out.requested_gate = PBV_GATE_STOP;
    } else if (out.lane_fault || out.fps_fault || out.timeout_fault) {
        out.sensor_health = PBV_SENSOR_DEGRADED;
        out.requested_gate = PBV_GATE_SLOW;
    }

    if (out.timeout_fault) {
        out.stale_counter += 1u;
        out.requested_gate = max_gate(out.requested_gate, PBV_GATE_SLOW);
    }

    if (age_ms > cfg->hard_timeout_ms) {
        out.requested_gate = PBV_GATE_STOP;
        out.sensor_health = PBV_SENSOR_FAILSAFE;
        out.fail_safe = true;
    }

    out.effective_lane_confidence = snapshot->lane_valid ? snapshot->lane_confidence : 0.0f;
    return out;
}

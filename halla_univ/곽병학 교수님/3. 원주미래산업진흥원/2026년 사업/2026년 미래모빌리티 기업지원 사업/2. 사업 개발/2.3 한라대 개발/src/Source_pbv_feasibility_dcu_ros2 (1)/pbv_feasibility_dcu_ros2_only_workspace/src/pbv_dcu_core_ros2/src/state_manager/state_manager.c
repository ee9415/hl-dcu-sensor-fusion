#include "pbv_dcu/state_manager.h"

#include <stdbool.h>
#include <string.h>

static PbvBehaviorGate max_gate(PbvBehaviorGate a, PbvBehaviorGate b) {
    return (a > b) ? a : b;
}

static void transition_to(PbvStateContext *ctx, PbvDriveState new_state, uint64_t now_ms) {
    if (ctx->state.drive_state != new_state) {
        ctx->state.drive_state = new_state;
        ctx->state.last_transition_ms = now_ms;
        ctx->state.hold_timer_ms = 0u;
    }
}

void pbv_state_init(PbvStateContext *ctx, uint64_t now_ms) {
    if (ctx == NULL) {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
    ctx->state.drive_state = PBV_DRIVE_INIT;
    ctx->state.behavior_gate = PBV_GATE_STOP;
    ctx->state.last_transition_ms = now_ms;
}

PbvStateOutput pbv_state_update(
    PbvStateContext *ctx,
    const PbvDcuConfig *cfg,
    const PbvPerceptionSnapshot *snapshot,
    const PbvFaultOutput *fault,
    uint64_t now_ms
) {
    PbvStateOutput out;
    uint32_t elapsed_ms = 0u;
    bool red_stop = false;
    bool green_release = false;
    bool target_near = false;

    if (ctx == NULL || cfg == NULL || snapshot == NULL || fault == NULL) {
        memset(&out, 0, sizeof(out));
        out.drive_state = PBV_DRIVE_FAILSAFE;
        out.behavior_gate = PBV_GATE_STOP;
        return out;
    }

    red_stop = (snapshot->traffic_light_state == PBV_TL_RED &&
                snapshot->stop_line_distance_m >= 0.0f &&
                snapshot->stop_line_distance_m < cfg->stop_line_trigger_m);
    green_release = (snapshot->traffic_light_state == PBV_TL_GREEN);
    target_near = (snapshot->target_valid &&
                   snapshot->target_distance_m > 0.0f &&
                   snapshot->target_distance_m < cfg->target_brake_distance_m);

    if (fault->fail_safe) {
        transition_to(ctx, PBV_DRIVE_FAILSAFE, now_ms);
    } else if (fault->sensor_health == PBV_SENSOR_DEGRADED) {
        transition_to(ctx, PBV_DRIVE_DEGRADED, now_ms);
    } else {
        switch (ctx->state.drive_state) {
            case PBV_DRIVE_INIT:
                if (snapshot->camera_ok) {
                    transition_to(ctx, PBV_DRIVE_READY, now_ms);
                }
                break;

            case PBV_DRIVE_READY:
                if (red_stop || target_near) {
                    transition_to(ctx, PBV_DRIVE_APPROACH_STOP, now_ms);
                } else {
                    transition_to(ctx, PBV_DRIVE_CRUISE, now_ms);
                }
                break;

            case PBV_DRIVE_CRUISE:
                if (red_stop || target_near) {
                    transition_to(ctx, PBV_DRIVE_APPROACH_STOP, now_ms);
                }
                break;

            case PBV_DRIVE_APPROACH_STOP:
                if ((snapshot->stop_line_distance_m >= 0.0f &&
                     snapshot->stop_line_distance_m <= cfg->hard_stop_line_m) ||
                    target_near) {
                    transition_to(ctx, PBV_DRIVE_STOPPED, now_ms);
                } else if (!red_stop && !target_near) {
                    transition_to(ctx, PBV_DRIVE_CRUISE, now_ms);
                }
                break;

            case PBV_DRIVE_STOPPED:
                elapsed_ms = (uint32_t)(now_ms - ctx->state.last_transition_ms);
                ctx->state.hold_timer_ms = elapsed_ms;
                if (green_release && elapsed_ms >= cfg->stop_hold_ms) {
                    transition_to(ctx, PBV_DRIVE_CRUISE, now_ms);
                }
                break;

            case PBV_DRIVE_DEGRADED:
                if (fault->sensor_health == PBV_SENSOR_OK) {
                    transition_to(ctx, PBV_DRIVE_READY, now_ms);
                }
                break;

            case PBV_DRIVE_FAILSAFE:
                if (!fault->fail_safe && fault->sensor_health != PBV_SENSOR_FAILSAFE) {
                    transition_to(ctx, PBV_DRIVE_READY, now_ms);
                }
                break;

            default:
                transition_to(ctx, PBV_DRIVE_INIT, now_ms);
                break;
        }
    }

    ctx->state.ready = (ctx->state.drive_state != PBV_DRIVE_INIT);
    ctx->state.stop_requested = red_stop || target_near || fault->requested_gate == PBV_GATE_STOP;

    switch (ctx->state.drive_state) {
        case PBV_DRIVE_INIT:
        case PBV_DRIVE_STOPPED:
        case PBV_DRIVE_FAILSAFE:
            ctx->state.behavior_gate = PBV_GATE_STOP;
            break;

        case PBV_DRIVE_DEGRADED:
        case PBV_DRIVE_APPROACH_STOP:
            ctx->state.behavior_gate = PBV_GATE_SLOW;
            break;

        case PBV_DRIVE_READY:
        case PBV_DRIVE_CRUISE:
        default:
            ctx->state.behavior_gate = PBV_GATE_ALLOW;
            break;
    }

    ctx->state.behavior_gate = max_gate(ctx->state.behavior_gate, fault->requested_gate);
    out = ctx->state;
    return out;
}

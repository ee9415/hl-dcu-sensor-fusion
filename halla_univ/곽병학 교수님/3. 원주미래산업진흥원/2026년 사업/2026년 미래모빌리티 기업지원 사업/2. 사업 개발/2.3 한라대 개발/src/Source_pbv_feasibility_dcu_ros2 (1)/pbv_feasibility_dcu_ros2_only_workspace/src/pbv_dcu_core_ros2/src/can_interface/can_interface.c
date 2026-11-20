#include "pbv_dcu/can_interface.h"

#include <stdio.h>
#include <string.h>

#include "pbv_dcu/socketcan_io.h"
#include "pbv_dcu/time_utils.h"

static int16_t scale_signed(float value) {
    if (value > 1.0f) {
        value = 1.0f;
    }
    if (value < -1.0f) {
        value = -1.0f;
    }
    return (int16_t)(value * 1000.0f);
}

static void encode_speed_frame(
    PbvCanFrame *frame,
    const PbvControlCommand *cmd,
    const PbvFaultOutput *fault,
    const PbvSafetyOutput *safety
) {
    int16_t speed_mmps = (int16_t)(cmd->target_speed_mps * 1000.0f);
    uint16_t brake_scaled = (uint16_t)(cmd->brake_norm * 1000.0f);

    memset(frame, 0, sizeof(*frame));
    frame->id = PBV_CAN_ID_SPEED_CMD;
    frame->dlc = 8u;
    frame->data[0] = (uint8_t)(speed_mmps & 0xff);
    frame->data[1] = (uint8_t)((speed_mmps >> 8) & 0xff);
    frame->data[2] = (uint8_t)(brake_scaled & 0xff);
    frame->data[3] = (uint8_t)((brake_scaled >> 8) & 0xff);
    frame->data[4] = (uint8_t)safety->enforced_gate;
    frame->data[5] = cmd->heartbeat;
    frame->data[6] = (uint8_t)(fault->fault_bits & 0xffu);
    frame->data[7] = (uint8_t)((fault->fault_bits >> 8) & 0xffu);
}

static void encode_steer_frame(
    PbvCanFrame *frame,
    const PbvControlCommand *cmd,
    const PbvStateOutput *state,
    uint32_t tx_counter
) {
    int16_t steer_scaled = scale_signed(cmd->steering_norm);

    memset(frame, 0, sizeof(*frame));
    frame->id = PBV_CAN_ID_STEER_CMD;
    frame->dlc = 8u;
    frame->data[0] = (uint8_t)(steer_scaled & 0xff);
    frame->data[1] = (uint8_t)((steer_scaled >> 8) & 0xff);
    frame->data[2] = (uint8_t)state->drive_state;
    frame->data[3] = (uint8_t)cmd->source;
    frame->data[4] = (uint8_t)(tx_counter & 0xffu);
    frame->data[5] = (uint8_t)((tx_counter >> 8) & 0xffu);
    frame->data[6] = (uint8_t)((tx_counter >> 16) & 0xffu);
    frame->data[7] = (uint8_t)((tx_counter >> 24) & 0xffu);
}

static void encode_diag_frame(
    PbvCanFrame *frame,
    const PbvFaultOutput *fault,
    const PbvSafetyOutput *safety
) {
    memset(frame, 0, sizeof(*frame));
    frame->id = PBV_CAN_ID_DIAG_TX;
    frame->dlc = 8u;
    frame->data[0] = (uint8_t)fault->sensor_health;
    frame->data[1] = (uint8_t)safety->enforced_gate;
    frame->data[2] = (uint8_t)(safety->intervention_bits & 0xffu);
    frame->data[3] = (uint8_t)((safety->intervention_bits >> 8) & 0xffu);
}

void pbv_can_init(PbvCanInterfaceContext *ctx, bool dry_run) {
    if (ctx != NULL) {
        memset(ctx, 0, sizeof(*ctx));
        ctx->dry_run = dry_run;
        ctx->socket_fd = -1;
    }
}

int pbv_can_open(PbvCanInterfaceContext *ctx, const PbvDcuConfig *cfg) {
    if (ctx == NULL || cfg == NULL) {
        return -1;
    }

    ctx->use_socketcan = cfg->use_socketcan;
    if (!cfg->use_socketcan) {
        return 0;
    }

    ctx->socket_fd = pbv_socketcan_open(cfg->can_if, true);
    if (ctx->socket_fd < 0) {
        return -1;
    }
    return 0;
}

void pbv_can_close(PbvCanInterfaceContext *ctx) {
    if (ctx != NULL && ctx->socket_fd >= 0) {
        pbv_socketcan_close(ctx->socket_fd);
        ctx->socket_fd = -1;
    }
}

int pbv_can_build_command_frames(
    PbvCanInterfaceContext *ctx,
    const PbvControlCommand *cmd,
    const PbvFaultOutput *fault,
    const PbvStateOutput *state,
    const PbvSafetyOutput *safety,
    PbvCanFrame *speed_out,
    PbvCanFrame *steer_out,
    PbvCanFrame *diag_out
) {
    if (ctx == NULL || cmd == NULL || fault == NULL || state == NULL || safety == NULL ||
        speed_out == NULL || steer_out == NULL || diag_out == NULL) {
        return -1;
    }

    ctx->tx_counter += 1u;
    encode_speed_frame(speed_out, cmd, fault, safety);
    encode_steer_frame(steer_out, cmd, state, ctx->tx_counter);
    encode_diag_frame(diag_out, fault, safety);
    return 0;
}

int pbv_can_format_frame_payload(const PbvCanFrame *frame, char *buffer, size_t buffer_size) {
    int written = 0;
    unsigned int i = 0u;

    if (frame == NULL || buffer == NULL || buffer_size == 0u) {
        return -1;
    }

    written = snprintf(buffer, buffer_size, "0x%03x", frame->id);
    if (written < 0 || (size_t)written >= buffer_size) {
        return -1;
    }

    for (i = 0u; i < frame->dlc; ++i) {
        int rc = snprintf(buffer + written, buffer_size - (size_t)written, " %02x", frame->data[i]);
        if (rc < 0 || (size_t)rc >= buffer_size - (size_t)written) {
            return -1;
        }
        written += rc;
    }
    return written;
}

static int send_frame(PbvCanInterfaceContext *ctx, const PbvCanFrame *frame) {
    unsigned int i = 0u;
    if (ctx->dry_run || !ctx->use_socketcan) {
        printf("CAN TX 0x%03x [%u] ", frame->id, frame->dlc);
        for (i = 0u; i < frame->dlc; ++i) {
            printf("%02x ", frame->data[i]);
        }
        printf("\n");
        return 0;
    }

    return pbv_socketcan_send_frame(ctx->socket_fd, frame);
}

int pbv_can_send_command(
    PbvCanInterfaceContext *ctx,
    const PbvControlCommand *cmd,
    const PbvFaultOutput *fault,
    const PbvStateOutput *state,
    const PbvSafetyOutput *safety,
    uint64_t now_ms
) {
    PbvCanFrame speed;
    PbvCanFrame steer;
    PbvCanFrame diag;
    int rc = 0;
    (void)now_ms;

    if (pbv_can_build_command_frames(ctx, cmd, fault, state, safety, &speed, &steer, &diag) != 0) {
        return -1;
    }

    rc |= send_frame(ctx, &speed);
    rc |= send_frame(ctx, &steer);
    rc |= send_frame(ctx, &diag);
    return rc;
}

bool pbv_can_poll_feedback(PbvCanInterfaceContext *ctx, PbvCanFeedback *feedback_out) {
    PbvCanFrame rx;
    int16_t speed_mmps = 0;
    int16_t steer_scaled = 0;

    if (ctx == NULL || feedback_out == NULL || !ctx->use_socketcan || ctx->socket_fd < 0) {
        return false;
    }

    if (!pbv_socketcan_read_frame(ctx->socket_fd, &rx)) {
        return false;
    }

    if (rx.id != PBV_CAN_ID_ECU_FEEDBACK || rx.dlc < 8u) {
        return false;
    }

    memset(feedback_out, 0, sizeof(*feedback_out));
    speed_mmps = (int16_t)((uint16_t)rx.data[0] | ((uint16_t)rx.data[1] << 8));
    steer_scaled = (int16_t)((uint16_t)rx.data[2] | ((uint16_t)rx.data[3] << 8));

    feedback_out->valid = true;
    feedback_out->measured_speed_mps = (float)speed_mmps / 1000.0f;
    feedback_out->measured_steering_norm = (float)steer_scaled / 1000.0f;
    feedback_out->rx_counter = (uint16_t)rx.data[4] | ((uint16_t)rx.data[5] << 8);
    feedback_out->status_bits = (uint16_t)rx.data[6] | ((uint16_t)rx.data[7] << 8);
    feedback_out->stamp_ms = pbv_now_monotonic_ms();

    ctx->last_feedback = *feedback_out;
    return true;
}

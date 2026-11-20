#ifndef PBV_DCU_CAN_INTERFACE_H
#define PBV_DCU_CAN_INTERFACE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pbv_dcu/config.h"
#include "pbv_dcu/data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    PBV_CAN_ID_SPEED_CMD = 0x200,
    PBV_CAN_ID_STEER_CMD = 0x201,
    PBV_CAN_ID_DIAG_TX = 0x210,
    PBV_CAN_ID_ECU_FEEDBACK = 0x310
};

typedef struct {
    bool dry_run;
    bool use_socketcan;
    int socket_fd;
    uint32_t tx_counter;
    PbvCanFeedback last_feedback;
} PbvCanInterfaceContext;

void pbv_can_init(PbvCanInterfaceContext *ctx, bool dry_run);
int pbv_can_open(PbvCanInterfaceContext *ctx, const PbvDcuConfig *cfg);
void pbv_can_close(PbvCanInterfaceContext *ctx);
int pbv_can_send_command(
    PbvCanInterfaceContext *ctx,
    const PbvControlCommand *cmd,
    const PbvFaultOutput *fault,
    const PbvStateOutput *state,
    const PbvSafetyOutput *safety,
    uint64_t now_ms
);
bool pbv_can_poll_feedback(PbvCanInterfaceContext *ctx, PbvCanFeedback *feedback_out);
int pbv_can_build_command_frames(
    PbvCanInterfaceContext *ctx,
    const PbvControlCommand *cmd,
    const PbvFaultOutput *fault,
    const PbvStateOutput *state,
    const PbvSafetyOutput *safety,
    PbvCanFrame *speed_out,
    PbvCanFrame *steer_out,
    PbvCanFrame *diag_out
);
int pbv_can_format_frame_payload(const PbvCanFrame *frame, char *buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif  /* PBV_DCU_CAN_INTERFACE_H */

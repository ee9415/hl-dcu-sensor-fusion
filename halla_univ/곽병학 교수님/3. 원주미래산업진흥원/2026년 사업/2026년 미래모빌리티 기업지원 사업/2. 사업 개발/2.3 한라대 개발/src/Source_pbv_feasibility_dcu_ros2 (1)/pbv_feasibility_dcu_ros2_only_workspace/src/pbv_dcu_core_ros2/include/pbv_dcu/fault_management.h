#ifndef PBV_DCU_FAULT_MANAGEMENT_H
#define PBV_DCU_FAULT_MANAGEMENT_H

#include <stdbool.h>
#include <stdint.h>

#include "pbv_dcu/config.h"
#include "pbv_dcu/data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool initialized;
    uint32_t last_seq;
    uint64_t last_rx_ms;
    uint32_t bad_frame_streak;
} PbvFaultContext;

void pbv_fault_init(PbvFaultContext *ctx);
PbvFaultOutput pbv_fault_update(
    PbvFaultContext *ctx,
    const PbvDcuConfig *cfg,
    const PbvPerceptionSnapshot *snapshot,
    uint64_t now_ms,
    bool new_frame
);

#ifdef __cplusplus
}
#endif

#endif  /* PBV_DCU_FAULT_MANAGEMENT_H */

#ifndef PBV_DCU_STATE_MANAGER_H
#define PBV_DCU_STATE_MANAGER_H

#include <stdint.h>

#include "pbv_dcu/config.h"
#include "pbv_dcu/data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    PbvStateOutput state;
} PbvStateContext;

void pbv_state_init(PbvStateContext *ctx, uint64_t now_ms);
PbvStateOutput pbv_state_update(
    PbvStateContext *ctx,
    const PbvDcuConfig *cfg,
    const PbvPerceptionSnapshot *snapshot,
    const PbvFaultOutput *fault,
    uint64_t now_ms
);

#ifdef __cplusplus
}
#endif

#endif  /* PBV_DCU_STATE_MANAGER_H */

#ifndef PBV_DCU_DECISION_CONTROL_H
#define PBV_DCU_DECISION_CONTROL_H

#include <stdint.h>

#include "pbv_dcu/config.h"
#include "pbv_dcu/data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t heartbeat;
} PbvDecisionContext;

void pbv_decision_init(PbvDecisionContext *ctx);
PbvControlCommand pbv_decision_update(
    PbvDecisionContext *ctx,
    const PbvDcuConfig *cfg,
    const PbvPerceptionSnapshot *snapshot,
    const PbvFaultOutput *fault,
    const PbvStateOutput *state,
    const PbvSafetyOutput *safety
);

#ifdef __cplusplus
}
#endif

#endif  /* PBV_DCU_DECISION_CONTROL_H */

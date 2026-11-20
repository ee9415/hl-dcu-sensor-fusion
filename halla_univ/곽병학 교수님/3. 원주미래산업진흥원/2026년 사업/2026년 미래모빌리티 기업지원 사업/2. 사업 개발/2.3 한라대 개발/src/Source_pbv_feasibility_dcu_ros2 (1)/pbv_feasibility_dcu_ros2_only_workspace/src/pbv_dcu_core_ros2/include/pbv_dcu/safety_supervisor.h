#ifndef PBV_DCU_SAFETY_SUPERVISOR_H
#define PBV_DCU_SAFETY_SUPERVISOR_H

#include <stdint.h>

#include "pbv_dcu/config.h"
#include "pbv_dcu/data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t last_intervention_bits;
} PbvSafetyContext;

void pbv_safety_init(PbvSafetyContext *ctx);
PbvSafetyOutput pbv_safety_update(
    PbvSafetyContext *ctx,
    const PbvDcuConfig *cfg,
    const PbvPerceptionSnapshot *snapshot,
    const PbvFaultOutput *fault,
    const PbvStateOutput *state
);

#ifdef __cplusplus
}
#endif

#endif  /* PBV_DCU_SAFETY_SUPERVISOR_H */

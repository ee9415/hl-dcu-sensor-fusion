#ifndef PBV_DCU_LOG_H
#define PBV_DCU_LOG_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pbv_dcu/data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    FILE *fp;
    bool header_written;
} PbvCsvLogger;

int pbv_log_open(PbvCsvLogger *logger, const char *path);
void pbv_log_close(PbvCsvLogger *logger);
void pbv_log_write(
    PbvCsvLogger *logger,
    uint64_t now_ms,
    const PbvPerceptionSnapshot *snapshot,
    const PbvFaultOutput *fault,
    const PbvStateOutput *state,
    const PbvSafetyOutput *safety,
    const PbvControlCommand *cmd,
    const PbvCanFeedback *feedback
);

#ifdef __cplusplus
}
#endif

#endif  /* PBV_DCU_LOG_H */

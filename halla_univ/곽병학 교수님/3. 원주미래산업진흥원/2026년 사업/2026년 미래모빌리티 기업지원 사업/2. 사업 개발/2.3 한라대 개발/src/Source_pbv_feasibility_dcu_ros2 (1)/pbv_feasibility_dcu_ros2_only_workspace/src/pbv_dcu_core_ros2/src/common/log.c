#include "pbv_dcu/log.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

static void ensure_parent_dir(const char *path) {
    char tmp[512];
    char *slash = NULL;

    if (path == NULL) {
        return;
    }

    memset(tmp, 0, sizeof(tmp));
    strncpy(tmp, path, sizeof(tmp) - 1u);
    slash = strrchr(tmp, '/');
    if (slash == NULL) {
        return;
    }

    *slash = '\0';
    if (*tmp != '\0') {
        (void)mkdir(tmp, 0775);
    }
}

int pbv_log_open(PbvCsvLogger *logger, const char *path) {
    if (logger == NULL || path == NULL) {
        return -1;
    }

    memset(logger, 0, sizeof(*logger));
    ensure_parent_dir(path);
    logger->fp = fopen(path, "w");
    if (logger->fp == NULL) {
        return -1;
    }

    return 0;
}

void pbv_log_close(PbvCsvLogger *logger) {
    if (logger != NULL && logger->fp != NULL) {
        fclose(logger->fp);
        logger->fp = NULL;
    }
}

void pbv_log_write(
    PbvCsvLogger *logger,
    uint64_t now_ms,
    const PbvPerceptionSnapshot *snapshot,
    const PbvFaultOutput *fault,
    const PbvStateOutput *state,
    const PbvSafetyOutput *safety,
    const PbvControlCommand *cmd,
    const PbvCanFeedback *feedback
) {
    FILE *fp = NULL;
    if (logger == NULL || logger->fp == NULL || snapshot == NULL || fault == NULL || state == NULL || safety == NULL || cmd == NULL) {
        return;
    }

    fp = logger->fp;
    if (!logger->header_written) {
        fprintf(
            fp,
            "now_ms,seq,frame_age_ms,lane_offset_m,lane_heading_rad,lane_confidence,target_distance_m,stop_line_distance_m,traffic_light_state,"
            "fault_bits,sensor_health,requested_gate,drive_state,behavior_gate,enforced_gate,override_stop,max_speed_mps,max_steer_norm,"
            "target_speed_mps,steering_norm,brake_norm,feedback_valid,measured_speed_mps,measured_steering_norm\n"
        );
        logger->header_written = true;
    }

    fprintf(
        fp,
        "%llu,%u,%u,%.4f,%.4f,%.4f,%.4f,%.4f,%u,%u,%u,%u,%u,%u,%u,%u,%.4f,%.4f,%.4f,%.4f,%.4f,%u,%.4f,%.4f\n",
        (unsigned long long)now_ms,
        snapshot->seq,
        snapshot->frame_age_ms,
        snapshot->lane_offset_m,
        snapshot->lane_heading_rad,
        snapshot->lane_confidence,
        snapshot->target_distance_m,
        snapshot->stop_line_distance_m,
        (unsigned int)snapshot->traffic_light_state,
        fault->fault_bits,
        (unsigned int)fault->sensor_health,
        (unsigned int)fault->requested_gate,
        (unsigned int)state->drive_state,
        (unsigned int)state->behavior_gate,
        (unsigned int)safety->enforced_gate,
        safety->override_stop ? 1u : 0u,
        safety->max_speed_mps,
        safety->max_steer_norm,
        cmd->target_speed_mps,
        cmd->steering_norm,
        cmd->brake_norm,
        (feedback != NULL && feedback->valid) ? 1u : 0u,
        (feedback != NULL && feedback->valid) ? feedback->measured_speed_mps : 0.0f,
        (feedback != NULL && feedback->valid) ? feedback->measured_steering_norm : 0.0f
    );
    fflush(fp);
}

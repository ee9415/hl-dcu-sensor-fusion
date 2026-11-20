#include "pbv_dcu/config.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char *trim(char *s) {
    char *end = NULL;
    while (*s != '\0' && isspace((unsigned char)*s)) {
        ++s;
    }
    if (*s == '\0') {
        return s;
    }
    end = s + strlen(s) - 1;
    while (end > s && isspace((unsigned char)*end)) {
        *end = '\0';
        --end;
    }
    return s;
}

void pbv_config_set_defaults(PbvDcuConfig *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->udp_port = 39001u;
    cfg->use_socketcan = false;
    strncpy(cfg->can_if, "vcan0", sizeof(cfg->can_if) - 1u);
    cfg->heartbeat_period_ms = 50u;
    cfg->nominal_speed_mps = 0.55f;
    cfg->slow_speed_mps = 0.25f;
    cfg->max_frame_age_ms = 400u;
    cfg->hard_timeout_ms = 900u;
    cfg->min_lane_conf = 0.45f;
    cfg->stop_line_trigger_m = 1.20f;
    cfg->hard_stop_line_m = 0.50f;
    cfg->target_slow_distance_m = 3.0f;
    cfg->target_brake_distance_m = 1.8f;
    cfg->k_offset = 0.90f;
    cfg->k_heading = 0.70f;
    cfg->max_steer_norm = 0.80f;
    cfg->stop_hold_ms = 1200u;
    strncpy(cfg->log_path, "logs/feasibility_dcu.csv", sizeof(cfg->log_path) - 1u);
}

static int set_key_value(PbvDcuConfig *cfg, const char *key, const char *value) {
    if (strcmp(key, "udp_port") == 0) {
        cfg->udp_port = (uint16_t)strtoul(value, NULL, 10);
    } else if (strcmp(key, "use_socketcan") == 0) {
        cfg->use_socketcan = (strtoul(value, NULL, 10) != 0u);
    } else if (strcmp(key, "can_if") == 0) {
        strncpy(cfg->can_if, value, sizeof(cfg->can_if) - 1u);
    } else if (strcmp(key, "heartbeat_period_ms") == 0) {
        cfg->heartbeat_period_ms = (uint32_t)strtoul(value, NULL, 10);
    } else if (strcmp(key, "nominal_speed_mps") == 0) {
        cfg->nominal_speed_mps = strtof(value, NULL);
    } else if (strcmp(key, "slow_speed_mps") == 0) {
        cfg->slow_speed_mps = strtof(value, NULL);
    } else if (strcmp(key, "max_frame_age_ms") == 0) {
        cfg->max_frame_age_ms = (uint32_t)strtoul(value, NULL, 10);
    } else if (strcmp(key, "hard_timeout_ms") == 0) {
        cfg->hard_timeout_ms = (uint32_t)strtoul(value, NULL, 10);
    } else if (strcmp(key, "min_lane_conf") == 0) {
        cfg->min_lane_conf = strtof(value, NULL);
    } else if (strcmp(key, "stop_line_trigger_m") == 0) {
        cfg->stop_line_trigger_m = strtof(value, NULL);
    } else if (strcmp(key, "hard_stop_line_m") == 0) {
        cfg->hard_stop_line_m = strtof(value, NULL);
    } else if (strcmp(key, "target_slow_distance_m") == 0) {
        cfg->target_slow_distance_m = strtof(value, NULL);
    } else if (strcmp(key, "target_brake_distance_m") == 0) {
        cfg->target_brake_distance_m = strtof(value, NULL);
    } else if (strcmp(key, "k_offset") == 0) {
        cfg->k_offset = strtof(value, NULL);
    } else if (strcmp(key, "k_heading") == 0) {
        cfg->k_heading = strtof(value, NULL);
    } else if (strcmp(key, "max_steer_norm") == 0) {
        cfg->max_steer_norm = strtof(value, NULL);
    } else if (strcmp(key, "stop_hold_ms") == 0) {
        cfg->stop_hold_ms = (uint32_t)strtoul(value, NULL, 10);
    } else if (strcmp(key, "log_path") == 0) {
        strncpy(cfg->log_path, value, sizeof(cfg->log_path) - 1u);
    } else {
        return -1;
    }
    return 0;
}

int pbv_config_load_file(PbvDcuConfig *cfg, const char *path) {
    FILE *fp = NULL;
    char line[512];

    pbv_config_set_defaults(cfg);

    if (path == NULL || *path == '\0') {
        return 0;
    }

    fp = fopen(path, "r");
    if (fp == NULL) {
        return -1;
    }

    while (fgets(line, sizeof(line), fp) != NULL) {
        char *eq = NULL;
        char *key = NULL;
        char *value = NULL;

        key = trim(line);
        if (*key == '\0' || *key == '#') {
            continue;
        }

        eq = strchr(key, '=');
        if (eq == NULL) {
            continue;
        }

        *eq = '\0';
        value = trim(eq + 1);
        key = trim(key);
        (void)set_key_value(cfg, key, value);
    }

    fclose(fp);
    return 0;
}

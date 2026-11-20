#ifndef PBV_DCU_CONFIG_H
#define PBV_DCU_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t udp_port;
    bool use_socketcan;
    char can_if[32];
    uint32_t heartbeat_period_ms;
    float nominal_speed_mps;
    float slow_speed_mps;
    uint32_t max_frame_age_ms;
    uint32_t hard_timeout_ms;
    float min_lane_conf;
    float stop_line_trigger_m;
    float hard_stop_line_m;
    float target_slow_distance_m;
    float target_brake_distance_m;
    float k_offset;
    float k_heading;
    float max_steer_norm;
    uint32_t stop_hold_ms;
    char log_path[256];
} PbvDcuConfig;

void pbv_config_set_defaults(PbvDcuConfig *cfg);
int pbv_config_load_file(PbvDcuConfig *cfg, const char *path);

#ifdef __cplusplus
}
#endif

#endif  /* PBV_DCU_CONFIG_H */

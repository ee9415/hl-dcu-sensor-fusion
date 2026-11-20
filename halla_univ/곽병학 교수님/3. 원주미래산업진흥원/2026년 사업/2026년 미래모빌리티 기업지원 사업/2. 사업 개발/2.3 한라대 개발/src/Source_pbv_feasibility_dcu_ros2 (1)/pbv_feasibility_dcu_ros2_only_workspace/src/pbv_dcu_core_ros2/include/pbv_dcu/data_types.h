#ifndef PBV_DCU_DATA_TYPES_H
#define PBV_DCU_DATA_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PBV_TL_UNKNOWN = 0,
    PBV_TL_RED = 1,
    PBV_TL_YELLOW = 2,
    PBV_TL_GREEN = 3
} PbvTrafficLightState;

typedef enum {
    PBV_SENSOR_OK = 0,
    PBV_SENSOR_DEGRADED = 1,
    PBV_SENSOR_FAILSAFE = 2
} PbvSensorHealth;

typedef enum {
    PBV_GATE_ALLOW = 0,
    PBV_GATE_SLOW = 1,
    PBV_GATE_STOP = 2
} PbvBehaviorGate;

typedef enum {
    PBV_DRIVE_INIT = 0,
    PBV_DRIVE_READY = 1,
    PBV_DRIVE_CRUISE = 2,
    PBV_DRIVE_APPROACH_STOP = 3,
    PBV_DRIVE_STOPPED = 4,
    PBV_DRIVE_DEGRADED = 5,
    PBV_DRIVE_FAILSAFE = 6
} PbvDriveState;

typedef enum {
    PBV_CMD_SOURCE_NORMAL = 0,
    PBV_CMD_SOURCE_SAFETY = 1,
    PBV_CMD_SOURCE_FAULT = 2
} PbvCommandSource;

enum {
    PBV_FAULT_NONE = 0u,
    PBV_FAULT_CAMERA_MISSING = 1u << 0,
    PBV_FAULT_FRAME_TIMEOUT = 1u << 1,
    PBV_FAULT_LOW_FPS = 1u << 2,
    PBV_FAULT_LANE_INVALID = 1u << 3,
    PBV_FAULT_TARGET_INVALID = 1u << 4,
    PBV_FAULT_CAN_TX = 1u << 5,
    PBV_FAULT_PACKET_LOSS = 1u << 6
};

enum {
    PBV_INTERVENTION_NONE = 0u,
    PBV_INTERVENTION_STALE_DATA = 1u << 0,
    PBV_INTERVENTION_RED_LIGHT = 1u << 1,
    PBV_INTERVENTION_TARGET_NEAR = 1u << 2,
    PBV_INTERVENTION_DEGRADED_LANE = 1u << 3,
    PBV_INTERVENTION_FAILSAFE = 1u << 4,
    PBV_INTERVENTION_STOP_LINE = 1u << 5
};

typedef struct {
    uint64_t stamp_ms;
    uint32_t seq;
    float lane_offset_m;
    float lane_heading_rad;
    float lane_confidence;
    float target_distance_m;
    float stop_line_distance_m;
    float traffic_light_confidence;
    float frame_fps;
    uint32_t frame_age_ms;
    bool camera_ok;
    bool lane_valid;
    bool target_valid;
    PbvTrafficLightState traffic_light_state;
    uint32_t flags;
} PbvPerceptionSnapshot;

typedef struct {
    uint64_t now_ms;
    uint32_t stale_counter;
    uint32_t lost_counter;
    uint32_t consecutive_bad_frames;
    uint32_t fault_bits;
    PbvSensorHealth sensor_health;
    PbvBehaviorGate requested_gate;
    bool fail_safe;
    bool timeout_fault;
    bool camera_fault;
    bool lane_fault;
    bool fps_fault;
    float effective_lane_confidence;
} PbvFaultOutput;

typedef struct {
    PbvDriveState drive_state;
    PbvBehaviorGate behavior_gate;
    uint64_t last_transition_ms;
    uint32_t hold_timer_ms;
    bool ready;
    bool stop_requested;
} PbvStateOutput;

typedef struct {
    PbvBehaviorGate enforced_gate;
    bool override_stop;
    float max_speed_mps;
    float max_steer_norm;
    uint32_t intervention_bits;
} PbvSafetyOutput;

typedef struct {
    float target_speed_mps;
    float steering_norm;
    float brake_norm;
    PbvCommandSource source;
    uint8_t heartbeat;
} PbvControlCommand;

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} PbvCanFrame;

typedef struct {
    bool valid;
    float measured_speed_mps;
    float measured_steering_norm;
    uint16_t rx_counter;
    uint16_t status_bits;
    uint64_t stamp_ms;
} PbvCanFeedback;

const char *pbv_gate_to_string(PbvBehaviorGate gate);
const char *pbv_drive_state_to_string(PbvDriveState state);
const char *pbv_sensor_health_to_string(PbvSensorHealth health);
const char *pbv_traffic_light_to_string(PbvTrafficLightState state);

#ifdef __cplusplus
}
#endif

#endif  /* PBV_DCU_DATA_TYPES_H */

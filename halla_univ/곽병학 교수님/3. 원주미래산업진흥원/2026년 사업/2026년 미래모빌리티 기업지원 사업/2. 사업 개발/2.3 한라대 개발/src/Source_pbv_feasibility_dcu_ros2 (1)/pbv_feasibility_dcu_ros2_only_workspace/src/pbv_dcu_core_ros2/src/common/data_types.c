#include "pbv_dcu/data_types.h"

const char *pbv_gate_to_string(PbvBehaviorGate gate) {
    switch (gate) {
        case PBV_GATE_ALLOW: return "ALLOW";
        case PBV_GATE_SLOW: return "SLOW";
        case PBV_GATE_STOP: return "STOP";
        default: return "UNKNOWN";
    }
}

const char *pbv_drive_state_to_string(PbvDriveState state) {
    switch (state) {
        case PBV_DRIVE_INIT: return "INIT";
        case PBV_DRIVE_READY: return "READY";
        case PBV_DRIVE_CRUISE: return "CRUISE";
        case PBV_DRIVE_APPROACH_STOP: return "APPROACH_STOP";
        case PBV_DRIVE_STOPPED: return "STOPPED";
        case PBV_DRIVE_DEGRADED: return "DEGRADED";
        case PBV_DRIVE_FAILSAFE: return "FAILSAFE";
        default: return "UNKNOWN";
    }
}

const char *pbv_sensor_health_to_string(PbvSensorHealth health) {
    switch (health) {
        case PBV_SENSOR_OK: return "OK";
        case PBV_SENSOR_DEGRADED: return "DEGRADED";
        case PBV_SENSOR_FAILSAFE: return "FAILSAFE";
        default: return "UNKNOWN";
    }
}

const char *pbv_traffic_light_to_string(PbvTrafficLightState state) {
    switch (state) {
        case PBV_TL_UNKNOWN: return "UNKNOWN";
        case PBV_TL_RED: return "RED";
        case PBV_TL_YELLOW: return "YELLOW";
        case PBV_TL_GREEN: return "GREEN";
        default: return "UNKNOWN";
    }
}

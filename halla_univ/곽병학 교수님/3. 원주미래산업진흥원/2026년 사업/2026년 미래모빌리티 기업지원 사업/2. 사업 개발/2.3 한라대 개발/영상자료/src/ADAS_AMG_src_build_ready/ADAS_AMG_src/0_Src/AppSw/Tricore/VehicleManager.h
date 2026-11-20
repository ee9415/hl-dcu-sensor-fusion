#ifndef VEHICLE_MANAGER_H_
#define VEHICLE_MANAGER_H_

#include "Ifx_Types.h"

/* Vehicle dimensions confirmed for this Ackermann platform. */
#define VM_WHEELBASE_M                    (0.44f)
#define VM_FRONT_TRACK_M                  (0.51f)

/* TC275 is the final authority for both CAN and manual speed requests. */
#define VM_CAN_MAX_SPEED_KPH              (13.0f)
#define VM_PS2_MAX_SPEED_KPH              (13.0f)
#define VM_PS2_MAX_REVERSE_SPEED_KPH      (1.0f)
#define VM_CAN_MAX_REVERSE_SPEED_KPH      (0.7f)
#define VM_MAX_STEERING_DEG               (27.0f)
#define VM_MAX_LATERAL_ACCEL_MPS2         (1.50f)
#define VM_STEERING_CENTER_DEADBAND_DEG   (0.5f)

/* Fixed execution characteristics of the CPU0 control path. */
#define VM_PERIOD_SEC                     (0.005f)
#define VM_TURN_LIMIT_SEARCH_ITERATIONS   (12u)

typedef enum
{
    VehicleManagerOwner_None = 0,
    VehicleManagerOwner_Ps2 = 1,
    VehicleManagerOwner_Can = 2,
    VehicleManagerOwner_ServiceTest = 3
} VehicleManagerOwner;

typedef enum
{
    VehicleManagerState_Init = 0,
    VehicleManagerState_Standby = 1,
    VehicleManagerState_Centering = 2,
    VehicleManagerState_ManualPs2 = 3,
    VehicleManagerState_AutoCan = 4,
    VehicleManagerState_CanRearm = 5,
    VehicleManagerState_FaultTorqueOff = 6,
    VehicleManagerState_Estop = 7,
    VehicleManagerState_ServiceSpeedTest = 8,
    VehicleManagerState_Validation = 9
} VehicleManagerState;

#define VM_FAULT_NONE                    (0x00000000u)
#define VM_FAULT_ESTOP_ACTIVE            (0x00000001u)
#define VM_FAULT_STEERING_FEEDBACK       (0x00000002u)
#define VM_FAULT_SPEED_FEEDBACK          (0x00000004u)
#define VM_FAULT_CAN_INVALID             (0x00000008u)
#define VM_FAULT_CAN_REARM_REQUIRED      (0x00000010u)
#define VM_FAULT_CAN_SEQUENCE            (0x00000020u)
#define VM_FAULT_PS2_NEUTRAL_REQUIRED    (0x00000040u)

typedef struct
{
    boolean estopActive;
    boolean speedFeedbackValid;
    boolean steeringFeedbackValid;
    float32 measuredSpeedKph;
    float32 measuredSteeringDeg;

    boolean ps2Available;
    float32 ps2RequestedSpeedKph;
    float32 ps2RequestedSteeringDeg;

    boolean canModeSeen;
    boolean canCommandValid;
    boolean canCommandEnabled;
    uint8 canSequence;
    uint32 canValidCount;
    float32 canRequestedSpeedKph;
    float32 canRequestedSteeringDeg;

    boolean serviceTestRequested;
    boolean serviceValidationRequested;
    float32 serviceRequestedSpeedKph;
    float32 serviceRequestedSteeringDeg;
} VehicleManagerInput;

typedef struct
{
    VehicleManagerOwner owner;
    VehicleManagerState state;
    uint32 faultMask;

    float32 requestedSpeedKph;
    float32 requestedSteeringDeg;
    float32 targetSpeedKph;
    float32 targetSteeringDeg;
    float32 effectiveSteeringDeg;
    float32 steeringLimitDeg;
    float32 curvatureInvM;
    float32 turnSpeedLimitKph;
    float32 predictedLateralAccelMps2;

    boolean driveEnable;
    boolean steeringTargetValid;
    boolean lateralLimitActive;
    boolean steeringLimitActive;
    boolean canRearmRequired;
    boolean invalidateCanShadow;
    boolean allowServiceTests;
} VehicleManagerOutput;

void VehicleManager_Init(void);
void VehicleManager_Step5ms(const VehicleManagerInput *input,
                            VehicleManagerOutput *output);

/* Debugger-visible Vehicle Manager trace. */
extern volatile uint8 g_vehicleManagerState;
extern volatile uint8 g_vehicleManagerOwner;
extern volatile uint32 g_vehicleManagerFaultMask;
extern volatile float32 g_vehicleManagerRequestedSpeedKph;
extern volatile float32 g_vehicleManagerTargetSpeedKph;
extern volatile float32 g_vehicleManagerRequestedSteeringDeg;
extern volatile float32 g_vehicleManagerTargetSteeringDeg;
extern volatile float32 g_vehicleManagerEffectiveSteeringDeg;
extern volatile float32 g_vehicleManagerSteeringLimitDeg;
extern volatile float32 g_vehicleManagerCurvatureInvM;
extern volatile float32 g_vehicleManagerTurnSpeedLimitKph;
extern volatile float32 g_vehicleManagerPredictedLateralAccelMps2;
extern volatile uint8 g_vehicleManagerLateralLimitActive;
extern volatile uint8 g_vehicleManagerSteeringLimitActive;
extern volatile uint8 g_vehicleManagerCanRearmRequired;
extern volatile uint32 g_vehicleManagerStepCount;

#endif /* VEHICLE_MANAGER_H_ */

#ifndef GRAPH_TEST_H_
#define GRAPH_TEST_H_

#include "Ifx_Types.h"

/* GraphTest_Step5ms() must be called once per 5 ms control-loop cycle. */
#define GRAPH_TEST_STEP_MS                         (5u)
#define GRAPH_TEST_PHASE_DURATION_MS               (5000u)
#define GRAPH_TEST_MODE1_MAX_PHASE                 (13u)
#define GRAPH_TEST_MODE2_MAX_PHASE                 (7u)

#define GRAPH_TEST_START_MAX_RPM                   (5.0f)
#define GRAPH_TEST_ABSOLUTE_OVERSPEED_KPH          (13.5f)

/* Deliberately independent from VehicleManager.h. */
#define GRAPH_TEST_MODE1_EXPECTED_VM_OWNER         (3u)
#define GRAPH_TEST_MODE1_EXPECTED_VM_STATE         (8u)
#define GRAPH_TEST_MODE2_EXPECTED_VM_OWNER         (3u)
#define GRAPH_TEST_MODE2_EXPECTED_VM_STATE         (9u)

typedef enum
{
    GraphTestMode_None = 0,
    GraphTestMode_SpeedTracking = 1,
    GraphTestMode_VmValidation = 2
} GraphTestMode;

typedef enum
{
    GraphTestState_Idle = 0,
    GraphTestState_Running = 1,
    GraphTestState_Complete = 2,
    GraphTestState_Aborted = 3,
    GraphTestState_Rejected = 4
} GraphTestState;

typedef enum
{
    GraphTestAbort_None = 0,
    GraphTestAbort_InvalidMode = 1,
    GraphTestAbort_StartGate = 2,
    GraphTestAbort_UserCancel = 3,
    GraphTestAbort_Estop = 4,
    GraphTestAbort_SpeedFeedback = 5,
    GraphTestAbort_SteeringFeedback = 6,
    GraphTestAbort_SourceConflict = 7,
    GraphTestAbort_LegacyTest = 8,
    GraphTestAbort_VmOwner = 9,
    GraphTestAbort_UnexpectedDrive = 10,
    GraphTestAbort_Overspeed = 11,
    GraphTestAbort_Internal = 12
} GraphTestAbortReason;

typedef struct
{
    boolean estopActive;
    boolean speedFeedbackHealthy;
    boolean steeringFeedbackHealthy;
    boolean sourceConflict;
    boolean legacyTestActive;
    boolean canRearmRequired;
    uint32 safetyFaultMask;
    float32 actualVehicleRpm;
    float32 actualVehicleKph;
    float32 actualSteeringDeg;
} GraphTestInput;

typedef struct
{
    uint8 mode;
    uint8 state;
    uint8 phase;
    uint8 abortReason;
    boolean active;

    /* TRUE in both active modes. The caller offers these values to VehicleManager. */
    boolean actuatorRequestValid;
    float32 requestedSpeedKph;
    float32 requestedSteeringDeg;

    uint32 phaseElapsedMs;
    uint32 totalElapsedMs;
    uint32 runSequence;
    uint32 completedCount;
    uint32 abortedCount;
} GraphTestOutput;

/* Expression commands/configuration. START and CANCEL are consumed to zero. */
extern volatile uint8 g_graphTestMode;
extern volatile uint8 g_graphTestStart;
extern volatile uint8 g_graphTestCancel;

/* Expression state/trace. */
extern volatile uint8 g_graphTestActive;
extern volatile uint8 g_graphTestState;
extern volatile uint8 g_graphTestPhase;
extern volatile uint8 g_graphTestAbortReason;
extern volatile uint8 g_graphTestActiveMode;
extern volatile uint8 g_graphTestActuatorRequestValid;
extern volatile uint32 g_graphTestPhaseElapsedMs;
extern volatile uint32 g_graphTestTotalElapsedMs;
extern volatile float32 g_graphTestRequestedSpeedKph;
extern volatile float32 g_graphTestRequestedSteeringDeg;
extern volatile uint32 g_graphTestRunSequence;
extern volatile uint32 g_graphTestCompletedCount;
extern volatile uint32 g_graphTestAbortedCount;

/* Last actual VehicleManager result supplied through the notification API. */
extern volatile uint8 g_graphTestVmOwner;
extern volatile uint8 g_graphTestVmState;
extern volatile uint8 g_graphTestVmDriveEnable;
extern volatile uint32 g_graphTestVmFaultMask;
extern volatile uint32 g_graphTestVmNotifyCount;

void GraphTest_Init(void);
void GraphTest_Step5ms(const GraphTestInput *input, GraphTestOutput *output);
void GraphTest_GetOutput(GraphTestOutput *output);

/*
 * Call after VehicleManager has stepped and before its result is applied.
 * If the actual owner/state/drive contract is wrong, this call immediately
 * invalidates the GraphTest request and latches Aborted.
 */
void GraphTest_NotifyVehicleManager(uint8 owner,
                                    uint8 state,
                                    uint32 faultMask,
                                    boolean driveEnable);

#endif /* GRAPH_TEST_H_ */

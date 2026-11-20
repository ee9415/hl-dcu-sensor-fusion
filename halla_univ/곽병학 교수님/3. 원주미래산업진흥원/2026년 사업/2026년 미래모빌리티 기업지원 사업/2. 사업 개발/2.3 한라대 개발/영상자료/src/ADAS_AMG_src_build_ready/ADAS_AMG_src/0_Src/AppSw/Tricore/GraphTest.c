#include "GraphTest.h"

#define GRAPH_TEST_PHASE_DURATION_LOOPS \
    (GRAPH_TEST_PHASE_DURATION_MS / GRAPH_TEST_STEP_MS)

volatile uint8 g_graphTestMode = (uint8)GraphTestMode_None;
volatile uint8 g_graphTestStart = 0u;
volatile uint8 g_graphTestCancel = 0u;

volatile uint8 g_graphTestActive = 0u;
volatile uint8 g_graphTestState = (uint8)GraphTestState_Idle;
volatile uint8 g_graphTestPhase = 0u;
volatile uint8 g_graphTestAbortReason = (uint8)GraphTestAbort_None;
volatile uint8 g_graphTestActiveMode = (uint8)GraphTestMode_None;
volatile uint8 g_graphTestActuatorRequestValid = 0u;
volatile uint32 g_graphTestPhaseElapsedMs = 0u;
volatile uint32 g_graphTestTotalElapsedMs = 0u;
volatile float32 g_graphTestRequestedSpeedKph = 0.0f;
volatile float32 g_graphTestRequestedSteeringDeg = 0.0f;
volatile uint32 g_graphTestRunSequence = 0u;
volatile uint32 g_graphTestCompletedCount = 0u;
volatile uint32 g_graphTestAbortedCount = 0u;

volatile uint8 g_graphTestVmOwner = 0u;
volatile uint8 g_graphTestVmState = 0u;
volatile uint8 g_graphTestVmDriveEnable = 0u;
volatile uint32 g_graphTestVmFaultMask = 0u;
volatile uint32 g_graphTestVmNotifyCount = 0u;

static uint8 g_graphTestRunningMode = (uint8)GraphTestMode_None;
static uint32 g_graphTestPhaseLoopCount = 0u;

static float32 GraphTest_AbsFloat(float32 value)
{
    return (value < 0.0f) ? -value : value;
}

static void GraphTest_ClearRequests(void)
{
    g_graphTestActuatorRequestValid = 0u;
    g_graphTestRequestedSpeedKph = 0.0f;
    g_graphTestRequestedSteeringDeg = 0.0f;
}

static void GraphTest_Reject(uint8 reason)
{
    g_graphTestActive = 0u;
    g_graphTestState = (uint8)GraphTestState_Rejected;
    g_graphTestPhase = 0u;
    g_graphTestAbortReason = reason;
    g_graphTestActiveMode = (uint8)GraphTestMode_None;
    g_graphTestRunningMode = (uint8)GraphTestMode_None;
    g_graphTestPhaseLoopCount = 0u;
    g_graphTestPhaseElapsedMs = 0u;
    g_graphTestTotalElapsedMs = 0u;
    GraphTest_ClearRequests();
}

static void GraphTest_Abort(uint8 reason)
{
    if (g_graphTestActive == 0u)
    {
        return;
    }

    g_graphTestActive = 0u;
    g_graphTestState = (uint8)GraphTestState_Aborted;
    g_graphTestAbortReason = reason;
    g_graphTestPhaseLoopCount = 0u;
    g_graphTestAbortedCount++;
    GraphTest_ClearRequests();
}

static void GraphTest_Complete(void)
{
    if (g_graphTestActive == 0u)
    {
        return;
    }

    g_graphTestActive = 0u;
    g_graphTestState = (uint8)GraphTestState_Complete;
    g_graphTestAbortReason = (uint8)GraphTestAbort_None;
    g_graphTestPhaseLoopCount = 0u;
    g_graphTestCompletedCount++;
    GraphTest_ClearRequests();
}

static boolean GraphTest_StartGatePassed(const GraphTestInput *input)
{
    if (input == 0)
    {
        return FALSE;
    }

    if ((input->estopActive != FALSE) ||
        (input->speedFeedbackHealthy == FALSE) ||
        (input->steeringFeedbackHealthy == FALSE) ||
        (input->sourceConflict != FALSE) ||
        (input->legacyTestActive != FALSE) ||
        (input->canRearmRequired != FALSE) ||
        (input->safetyFaultMask != 0u) ||
        (GraphTest_AbsFloat(input->actualVehicleRpm) >
         GRAPH_TEST_START_MAX_RPM))
    {
        return FALSE;
    }

    return TRUE;
}

static void GraphTest_SetMode2Case(uint8 phase)
{
    g_graphTestActuatorRequestValid = 1u;

    switch (phase)
    {
    case 1u:
        g_graphTestRequestedSpeedKph = 3.0f;
        g_graphTestRequestedSteeringDeg = 0.0f;
        break;

    case 2u:
        g_graphTestRequestedSpeedKph = 3.0f;
        g_graphTestRequestedSteeringDeg = 20.0f;
        break;

    case 3u:
        g_graphTestRequestedSpeedKph = 5.0f;
        g_graphTestRequestedSteeringDeg = 20.0f;
        break;

    case 4u:
        g_graphTestRequestedSpeedKph = 8.0f;
        g_graphTestRequestedSteeringDeg = 10.0f;
        break;

    case 5u:
        g_graphTestRequestedSpeedKph = 13.0f;
        g_graphTestRequestedSteeringDeg = 0.0f;
        break;

    case 6u:
        g_graphTestRequestedSpeedKph = 13.0f;
        g_graphTestRequestedSteeringDeg = 5.0f;
        break;

    case 7u:
    default:
        g_graphTestRequestedSpeedKph = 0.0f;
        g_graphTestRequestedSteeringDeg = 0.0f;
        break;
    }
}

static void GraphTest_SetCurrentPhaseRequests(void)
{
    if (g_graphTestRunningMode == (uint8)GraphTestMode_SpeedTracking)
    {
        g_graphTestActuatorRequestValid = 1u;
        g_graphTestRequestedSpeedKph = (float32)g_graphTestPhase;
        g_graphTestRequestedSteeringDeg = 0.0f;
    }
    else if (g_graphTestRunningMode == (uint8)GraphTestMode_VmValidation)
    {
        GraphTest_SetMode2Case(g_graphTestPhase);
    }
    else
    {
        GraphTest_Abort((uint8)GraphTestAbort_Internal);
    }
}

static void GraphTest_Start(uint8 mode)
{
    g_graphTestRunningMode = mode;
    g_graphTestActiveMode = mode;
    g_graphTestActive = 1u;
    g_graphTestState = (uint8)GraphTestState_Running;
    g_graphTestPhase = 1u;
    g_graphTestAbortReason = (uint8)GraphTestAbort_None;
    g_graphTestPhaseLoopCount = 0u;
    g_graphTestPhaseElapsedMs = 0u;
    g_graphTestTotalElapsedMs = 0u;
    g_graphTestRunSequence++;
    GraphTest_SetCurrentPhaseRequests();
}

static uint8 GraphTest_GetMaximumPhase(void)
{
    if (g_graphTestRunningMode == (uint8)GraphTestMode_SpeedTracking)
    {
        return GRAPH_TEST_MODE1_MAX_PHASE;
    }

    if (g_graphTestRunningMode == (uint8)GraphTestMode_VmValidation)
    {
        return GRAPH_TEST_MODE2_MAX_PHASE;
    }

    return 0u;
}

static boolean GraphTest_CheckContinuousAbort(const GraphTestInput *input)
{
    if (input == 0)
    {
        GraphTest_Abort((uint8)GraphTestAbort_Internal);
        return TRUE;
    }

    if (input->estopActive != FALSE)
    {
        GraphTest_Abort((uint8)GraphTestAbort_Estop);
        return TRUE;
    }

    if (input->speedFeedbackHealthy == FALSE)
    {
        GraphTest_Abort((uint8)GraphTestAbort_SpeedFeedback);
        return TRUE;
    }

    if (input->steeringFeedbackHealthy == FALSE)
    {
        GraphTest_Abort((uint8)GraphTestAbort_SteeringFeedback);
        return TRUE;
    }

    if ((input->sourceConflict != FALSE) ||
        (input->canRearmRequired != FALSE))
    {
        GraphTest_Abort((uint8)GraphTestAbort_SourceConflict);
        return TRUE;
    }

    if (input->legacyTestActive != FALSE)
    {
        GraphTest_Abort((uint8)GraphTestAbort_LegacyTest);
        return TRUE;
    }

    if (input->safetyFaultMask != 0u)
    {
        GraphTest_Abort((uint8)GraphTestAbort_Internal);
        return TRUE;
    }

    if (g_graphTestMode != g_graphTestRunningMode)
    {
        GraphTest_Abort((uint8)GraphTestAbort_Internal);
        return TRUE;
    }

    if (GraphTest_AbsFloat(input->actualVehicleKph) >
        GRAPH_TEST_ABSOLUTE_OVERSPEED_KPH)
    {
        GraphTest_Abort((uint8)GraphTestAbort_Overspeed);
        return TRUE;
    }

    return FALSE;
}

static void GraphTest_AdvanceTimeAndPhase(void)
{
    uint8 maximumPhase = GraphTest_GetMaximumPhase();

    if (maximumPhase == 0u)
    {
        GraphTest_Abort((uint8)GraphTestAbort_Internal);
        return;
    }

    g_graphTestPhaseLoopCount++;
    g_graphTestPhaseElapsedMs =
        g_graphTestPhaseLoopCount * (uint32)GRAPH_TEST_STEP_MS;
    g_graphTestTotalElapsedMs += (uint32)GRAPH_TEST_STEP_MS;

    if (g_graphTestPhaseLoopCount <
        (uint32)GRAPH_TEST_PHASE_DURATION_LOOPS)
    {
        return;
    }

    if (g_graphTestPhase >= maximumPhase)
    {
        GraphTest_Complete();
        return;
    }

    g_graphTestPhase++;
    g_graphTestPhaseLoopCount = 0u;
    g_graphTestPhaseElapsedMs = 0u;
    GraphTest_SetCurrentPhaseRequests();
}

void GraphTest_Init(void)
{
    g_graphTestMode = (uint8)GraphTestMode_None;
    g_graphTestStart = 0u;
    g_graphTestCancel = 0u;
    g_graphTestActive = 0u;
    g_graphTestState = (uint8)GraphTestState_Idle;
    g_graphTestPhase = 0u;
    g_graphTestAbortReason = (uint8)GraphTestAbort_None;
    g_graphTestActiveMode = (uint8)GraphTestMode_None;
    g_graphTestPhaseElapsedMs = 0u;
    g_graphTestTotalElapsedMs = 0u;
    g_graphTestRunSequence = 0u;
    g_graphTestCompletedCount = 0u;
    g_graphTestAbortedCount = 0u;
    g_graphTestVmOwner = 0u;
    g_graphTestVmState = 0u;
    g_graphTestVmDriveEnable = 0u;
    g_graphTestVmFaultMask = 0u;
    g_graphTestVmNotifyCount = 0u;
    g_graphTestRunningMode = (uint8)GraphTestMode_None;
    g_graphTestPhaseLoopCount = 0u;
    GraphTest_ClearRequests();
}

void GraphTest_Step5ms(const GraphTestInput *input, GraphTestOutput *output)
{
    boolean startedThisStep = FALSE;

    if (g_graphTestCancel != 0u)
    {
        g_graphTestCancel = 0u;
        g_graphTestStart = 0u;
        GraphTest_Abort((uint8)GraphTestAbort_UserCancel);
        GraphTest_GetOutput(output);
        return;
    }

    if (g_graphTestStart != 0u)
    {
        uint8 requestedMode = g_graphTestMode;

        g_graphTestStart = 0u;
        if (g_graphTestActive == 0u)
        {
            if ((requestedMode != (uint8)GraphTestMode_SpeedTracking) &&
                (requestedMode != (uint8)GraphTestMode_VmValidation))
            {
                g_graphTestMode = (uint8)GraphTestMode_None;
                GraphTest_Reject((uint8)GraphTestAbort_InvalidMode);
                GraphTest_GetOutput(output);
                return;
            }

            if (GraphTest_StartGatePassed(input) == FALSE)
            {
                GraphTest_Reject((uint8)GraphTestAbort_StartGate);
                GraphTest_GetOutput(output);
                return;
            }

            GraphTest_Start(requestedMode);
            startedThisStep = TRUE;
        }
    }

    if (g_graphTestActive != 0u)
    {
        if (GraphTest_CheckContinuousAbort(input) == FALSE)
        {
            if (startedThisStep == FALSE)
            {
                GraphTest_AdvanceTimeAndPhase();
            }
        }
    }

    GraphTest_GetOutput(output);
}

void GraphTest_GetOutput(GraphTestOutput *output)
{
    if (output == 0)
    {
        return;
    }

    output->mode = g_graphTestActiveMode;
    output->state = g_graphTestState;
    output->phase = g_graphTestPhase;
    output->abortReason = g_graphTestAbortReason;
    output->active = (g_graphTestActive != 0u) ? TRUE : FALSE;
    output->actuatorRequestValid =
        (g_graphTestActuatorRequestValid != 0u) ? TRUE : FALSE;
    if (g_graphTestActuatorRequestValid != 0u)
    {
        output->requestedSpeedKph = g_graphTestRequestedSpeedKph;
        output->requestedSteeringDeg = g_graphTestRequestedSteeringDeg;
    }
    else
    {
        output->requestedSpeedKph = 0.0f;
        output->requestedSteeringDeg = 0.0f;
    }
    output->phaseElapsedMs = g_graphTestPhaseElapsedMs;
    output->totalElapsedMs = g_graphTestTotalElapsedMs;
    output->runSequence = g_graphTestRunSequence;
    output->completedCount = g_graphTestCompletedCount;
    output->abortedCount = g_graphTestAbortedCount;
}

void GraphTest_NotifyVehicleManager(uint8 owner,
                                    uint8 state,
                                    uint32 faultMask,
                                    boolean driveEnable)
{
    g_graphTestVmOwner = owner;
    g_graphTestVmState = state;
    g_graphTestVmFaultMask = faultMask;
    g_graphTestVmDriveEnable = (driveEnable != FALSE) ? 1u : 0u;
    g_graphTestVmNotifyCount++;

    if (g_graphTestActive == 0u)
    {
        return;
    }

    if (faultMask != 0u)
    {
        GraphTest_Abort((uint8)GraphTestAbort_Internal);
        return;
    }

    if (g_graphTestRunningMode == (uint8)GraphTestMode_SpeedTracking)
    {
        if ((owner != GRAPH_TEST_MODE1_EXPECTED_VM_OWNER) ||
            (state != GRAPH_TEST_MODE1_EXPECTED_VM_STATE))
        {
            GraphTest_Abort((uint8)GraphTestAbort_VmOwner);
            return;
        }

        if (driveEnable == FALSE)
        {
            GraphTest_Abort((uint8)GraphTestAbort_UnexpectedDrive);
        }
    }
    else if (g_graphTestRunningMode ==
             (uint8)GraphTestMode_VmValidation)
    {
        if ((owner != GRAPH_TEST_MODE2_EXPECTED_VM_OWNER) ||
            (state != GRAPH_TEST_MODE2_EXPECTED_VM_STATE))
        {
            GraphTest_Abort((uint8)GraphTestAbort_VmOwner);
            return;
        }

        if (driveEnable == FALSE)
        {
            GraphTest_Abort((uint8)GraphTestAbort_UnexpectedDrive);
        }
    }
    else
    {
        GraphTest_Abort((uint8)GraphTestAbort_Internal);
    }
}

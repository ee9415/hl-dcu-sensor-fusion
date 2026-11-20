#include "VehicleManager.h"

#define VM_DEG_TO_RAD                  (0.01745329251994329577f)
#define VM_KPH_TO_MPS                  (0.27777777777777777778f)
#define VM_CURVATURE_MIN_INV_M         (0.000001f)
#define VM_TAN_CONSERVATIVE_FACTOR     (1.001f)
#define VM_MIN_TURN_RADIUS_DENOMINATOR_M (0.05f)

volatile uint8 g_vehicleManagerState = (uint8)VehicleManagerState_Init;
volatile uint8 g_vehicleManagerOwner = (uint8)VehicleManagerOwner_None;
volatile uint32 g_vehicleManagerFaultMask = VM_FAULT_NONE;
volatile float32 g_vehicleManagerRequestedSpeedKph = 0.0f;
volatile float32 g_vehicleManagerTargetSpeedKph = 0.0f;
volatile float32 g_vehicleManagerRequestedSteeringDeg = 0.0f;
volatile float32 g_vehicleManagerTargetSteeringDeg = 0.0f;
volatile float32 g_vehicleManagerEffectiveSteeringDeg = 0.0f;
volatile float32 g_vehicleManagerSteeringLimitDeg = 0.0f;
volatile float32 g_vehicleManagerCurvatureInvM = 0.0f;
volatile float32 g_vehicleManagerTurnSpeedLimitKph = 0.0f;
volatile float32 g_vehicleManagerPredictedLateralAccelMps2 = 0.0f;
volatile uint8 g_vehicleManagerLateralLimitActive = 0u;
volatile uint8 g_vehicleManagerSteeringLimitActive = 0u;
volatile uint8 g_vehicleManagerCanRearmRequired = 0u;
volatile uint32 g_vehicleManagerStepCount = 0u;

static boolean g_vmPreviousPs2Available = FALSE;
static boolean g_vmCanRearmRequired = FALSE;
static boolean g_vmCanDisableSeen = FALSE;
static uint32 g_vmCanRearmBaselineCount = 0u;
static uint32 g_vmCanDisableCount = 0u;
static uint8 g_vmCanDisableSequence = 0u;
static boolean g_vmCanAcceptedSequenceValid = FALSE;
static uint32 g_vmCanAcceptedCount = 0u;
static uint8 g_vmCanAcceptedSequence = 0u;
static boolean g_vmPreviousEstopActive = FALSE;
static boolean g_vmFeedbackFaultActive = FALSE;
static boolean g_vmPs2NeutralRequired = FALSE;

static float32 VehicleManager_AbsFloat(float32 value)
{
    return (value < 0.0f) ? -value : value;
}

static float32 VehicleManager_ClampFloat(float32 value,
                                         float32 minValue,
                                         float32 maxValue)
{
    if (value < minValue)
    {
        return minValue;
    }

    if (value > maxValue)
    {
        return maxValue;
    }

    return value;
}

static boolean VehicleManager_IsNewCount(uint32 count, uint32 reference)
{
    uint32 delta = count - reference;
    return ((delta != 0u) && (delta < 0x80000000u)) ? TRUE : FALSE;
}

static boolean VehicleManager_IsForwardSequence(uint8 previous, uint8 current)
{
    uint8 delta = (uint8)(current - previous);
    return ((delta != 0u) && (delta <= 127u)) ? TRUE : FALSE;
}

/*
 * Deterministic tan approximation for |steering| <= 27 degrees.  The series
 * through x^9 is slightly inflated so the resulting speed envelope remains
 * conservative without adding a libm dependency to the 5 ms path.
 */
static float32 VehicleManager_TanAbsDeg(float32 angleDeg)
{
    float32 x = VehicleManager_AbsFloat(angleDeg) * VM_DEG_TO_RAD;
    float32 x2 = x * x;
    float32 x3 = x * x2;
    float32 x5 = x3 * x2;
    float32 x7 = x5 * x2;
    float32 x9 = x7 * x2;
    float32 tanValue = x +
                       (x3 / 3.0f) +
                       ((2.0f * x5) / 15.0f) +
                       ((17.0f * x7) / 315.0f) +
                       ((62.0f * x9) / 2835.0f);

    return tanValue * VM_TAN_CONSERVATIVE_FACTOR;
}

static float32 VehicleManager_CurvatureFromSteering(float32 steeringDeg)
{
    float32 limitedDeg = VehicleManager_ClampFloat(steeringDeg,
                                                    -VM_MAX_STEERING_DEG,
                                                    VM_MAX_STEERING_DEG);
    float32 tanValue = VehicleManager_TanAbsDeg(limitedDeg);
    float32 denominator = VM_WHEELBASE_M - ((VM_FRONT_TRACK_M * 0.5f) * tanValue);

    /*
     * The potentiometer angle has not yet been identified as inner, outer, or
     * bicycle-equivalent.  Treat it as the outer front-wheel angle, which is
     * the worst case for center-path curvature.  A turn-radius/yaw calibration
     * can later replace this conservative conversion without changing callers.
     */
    if (denominator < VM_MIN_TURN_RADIUS_DENOMINATOR_M)
    {
        denominator = VM_MIN_TURN_RADIUS_DENOMINATOR_M;
    }

    return tanValue / denominator;
}

static float32 VehicleManager_LateralAccel(float32 speedKph,
                                           float32 curvatureInvM)
{
    float32 speedMps = VehicleManager_AbsFloat(speedKph) * VM_KPH_TO_MPS;
    return speedMps * speedMps * curvatureInvM;
}

static float32 VehicleManager_FindTurnSpeedLimitKph(float32 curvatureInvM,
                                                    float32 sourceMaxKph)
{
    float32 lowKph = 0.0f;
    float32 highKph = sourceMaxKph;
    float32 midKph;
    uint8 iteration;

    if (curvatureInvM <= VM_CURVATURE_MIN_INV_M)
    {
        return sourceMaxKph;
    }

    if (VehicleManager_LateralAccel(highKph, curvatureInvM) <=
        VM_MAX_LATERAL_ACCEL_MPS2)
    {
        return sourceMaxKph;
    }

    for (iteration = 0u; iteration < VM_TURN_LIMIT_SEARCH_ITERATIONS; iteration++)
    {
        midKph = (lowKph + highKph) * 0.5f;
        if (VehicleManager_LateralAccel(midKph, curvatureInvM) <=
            VM_MAX_LATERAL_ACCEL_MPS2)
        {
            lowKph = midKph;
        }
        else
        {
            highKph = midKph;
        }
    }

    return lowKph;
}

static float32 VehicleManager_FindSteeringLimitDeg(float32 measuredSpeedKph)
{
    float32 lowDeg = 0.0f;
    float32 highDeg = VM_MAX_STEERING_DEG;
    float32 midDeg;
    float32 speedMagnitudeKph = VehicleManager_AbsFloat(measuredSpeedKph);
    uint8 iteration;

    if (VehicleManager_LateralAccel(speedMagnitudeKph,
                                    VehicleManager_CurvatureFromSteering(highDeg)) <=
        VM_MAX_LATERAL_ACCEL_MPS2)
    {
        return VM_MAX_STEERING_DEG;
    }

    for (iteration = 0u; iteration < VM_TURN_LIMIT_SEARCH_ITERATIONS; iteration++)
    {
        midDeg = (lowDeg + highDeg) * 0.5f;
        if (VehicleManager_LateralAccel(speedMagnitudeKph,
                                        VehicleManager_CurvatureFromSteering(midDeg)) <=
            VM_MAX_LATERAL_ACCEL_MPS2)
        {
            lowDeg = midDeg;
        }
        else
        {
            highDeg = midDeg;
        }
    }

    return lowDeg;
}

static void VehicleManager_ClearOutputFields(VehicleManagerOutput *output)
{
    output->owner = VehicleManagerOwner_None;
    output->state = VehicleManagerState_Standby;
    output->faultMask = VM_FAULT_NONE;
    output->requestedSpeedKph = 0.0f;
    output->requestedSteeringDeg = 0.0f;
    output->targetSpeedKph = 0.0f;
    output->targetSteeringDeg = 0.0f;
    output->effectiveSteeringDeg = 0.0f;
    output->steeringLimitDeg = 0.0f;
    output->curvatureInvM = 0.0f;
    output->turnSpeedLimitKph = 0.0f;
    output->predictedLateralAccelMps2 = 0.0f;
    output->driveEnable = FALSE;
    output->steeringTargetValid = FALSE;
    output->lateralLimitActive = FALSE;
    output->steeringLimitActive = FALSE;
    output->canRearmRequired = FALSE;
    output->invalidateCanShadow = FALSE;
    output->allowServiceTests = FALSE;
}

static void VehicleManager_ClearOutput(VehicleManagerOutput *output)
{
    VehicleManager_ClearOutputFields(output);
    output->canRearmRequired = g_vmCanRearmRequired;
}

static void VehicleManager_StartCanRearm(const VehicleManagerInput *input,
                                         VehicleManagerOutput *output,
                                         boolean invalidateShadow)
{
    g_vmCanRearmRequired = TRUE;
    g_vmCanDisableSeen = FALSE;
    g_vmCanRearmBaselineCount = input->canValidCount;
    g_vmCanAcceptedSequenceValid = FALSE;
    output->canRearmRequired = TRUE;

    if (invalidateShadow != FALSE)
    {
        output->invalidateCanShadow = TRUE;
    }
}

static void VehicleManager_LatchSafetyRearm(const VehicleManagerInput *input,
                                            VehicleManagerOutput *output)
{
    /* There is no stale CAN command to re-arm until CAN has been used. */
    if ((input->canModeSeen != FALSE) ||
        (g_vmCanAcceptedSequenceValid != FALSE) ||
        (g_vmCanRearmRequired != FALSE))
    {
        VehicleManager_StartCanRearm(input, output, TRUE);
    }

    /* A held speed button must not restart the car after a safety stop. */
    g_vmPs2NeutralRequired = TRUE;
}

static void VehicleManager_RequestCenter(const VehicleManagerInput *input,
                                         VehicleManagerOutput *output)
{
    output->targetSpeedKph = 0.0f;
    output->targetSteeringDeg = 0.0f;
    output->driveEnable = FALSE;

    if (input->steeringFeedbackValid != FALSE)
    {
        if (VehicleManager_AbsFloat(input->measuredSteeringDeg) >
            VM_STEERING_CENTER_DEADBAND_DEG)
        {
            output->steeringTargetValid = TRUE;
            if (output->state == VehicleManagerState_Standby)
            {
                output->state = VehicleManagerState_Centering;
            }
        }
    }
}

static boolean VehicleManager_TryCompleteCanRearm(const VehicleManagerInput *input,
                                                  VehicleManagerOutput *output)
{
    if ((input->canCommandValid != FALSE) &&
        (VehicleManager_IsNewCount(input->canValidCount,
                                   g_vmCanRearmBaselineCount) != FALSE))
    {
        if (input->canCommandEnabled == FALSE)
        {
            g_vmCanDisableSeen = TRUE;
            g_vmCanDisableCount = input->canValidCount;
            g_vmCanDisableSequence = input->canSequence;
        }
        else if ((g_vmCanDisableSeen != FALSE) &&
                 (VehicleManager_IsNewCount(input->canValidCount,
                                            g_vmCanDisableCount) != FALSE) &&
                 (VehicleManager_IsForwardSequence(g_vmCanDisableSequence,
                                                   input->canSequence) != FALSE))
        {
            g_vmCanRearmRequired = FALSE;
            g_vmCanDisableSeen = FALSE;
            g_vmCanAcceptedSequenceValid = TRUE;
            g_vmCanAcceptedCount = input->canValidCount;
            g_vmCanAcceptedSequence = input->canSequence;
            output->canRearmRequired = FALSE;
            return TRUE;
        }
    }

    output->state = VehicleManagerState_CanRearm;
    output->faultMask |= VM_FAULT_CAN_REARM_REQUIRED;
    output->canRearmRequired = TRUE;
    return FALSE;
}

static boolean VehicleManager_CheckActiveCanSequence(const VehicleManagerInput *input,
                                                     VehicleManagerOutput *output)
{
    if (g_vmCanAcceptedSequenceValid == FALSE)
    {
        g_vmCanAcceptedSequenceValid = TRUE;
        g_vmCanAcceptedCount = input->canValidCount;
        g_vmCanAcceptedSequence = input->canSequence;
        return TRUE;
    }

    if (VehicleManager_IsNewCount(input->canValidCount, g_vmCanAcceptedCount) != FALSE)
    {
        if (VehicleManager_IsForwardSequence(g_vmCanAcceptedSequence,
                                             input->canSequence) == FALSE)
        {
            VehicleManager_StartCanRearm(input, output, FALSE);
            output->state = VehicleManagerState_FaultTorqueOff;
            output->faultMask |= VM_FAULT_CAN_SEQUENCE;
            return FALSE;
        }

        g_vmCanAcceptedCount = input->canValidCount;
        g_vmCanAcceptedSequence = input->canSequence;
    }

    return TRUE;
}

static void VehicleManager_ApplyMotionEnvelope(VehicleManagerOutput *output,
                                               float32 sourceMaxKph,
                                               float32 measuredSpeedKph,
                                               float32 measuredSteeringDeg)
{
    float32 requestedMagnitude;
    float32 targetMagnitude;
    float32 speedSign;

    output->requestedSteeringDeg = VehicleManager_ClampFloat(output->requestedSteeringDeg,
                                                             -VM_MAX_STEERING_DEG,
                                                             VM_MAX_STEERING_DEG);
    output->targetSteeringDeg = output->requestedSteeringDeg;
    output->steeringLimitDeg = VehicleManager_FindSteeringLimitDeg(measuredSpeedKph);
    if (VehicleManager_AbsFloat(output->requestedSteeringDeg) >
        output->steeringLimitDeg)
    {
        output->targetSteeringDeg = (output->requestedSteeringDeg < 0.0f) ?
            -output->steeringLimitDeg : output->steeringLimitDeg;
        output->steeringLimitActive = TRUE;
    }
    output->effectiveSteeringDeg = VehicleManager_AbsFloat(output->requestedSteeringDeg);
    if (VehicleManager_AbsFloat(measuredSteeringDeg) > output->effectiveSteeringDeg)
    {
        output->effectiveSteeringDeg = VehicleManager_AbsFloat(measuredSteeringDeg);
    }
    output->curvatureInvM = VehicleManager_CurvatureFromSteering(output->effectiveSteeringDeg);
    output->turnSpeedLimitKph = VehicleManager_FindTurnSpeedLimitKph(output->curvatureInvM,
                                                                    sourceMaxKph);

    requestedMagnitude = VehicleManager_AbsFloat(output->requestedSpeedKph);
    speedSign = (output->requestedSpeedKph < 0.0f) ? -1.0f : 1.0f;
    targetMagnitude = requestedMagnitude;
    if (targetMagnitude > sourceMaxKph)
    {
        targetMagnitude = sourceMaxKph;
    }
    if (targetMagnitude > output->turnSpeedLimitKph)
    {
        targetMagnitude = output->turnSpeedLimitKph;
        output->lateralLimitActive = TRUE;
    }

    output->targetSpeedKph = speedSign * targetMagnitude;
    output->predictedLateralAccelMps2 =
        VehicleManager_LateralAccel(output->targetSpeedKph, output->curvatureInvM);
}

static void VehicleManager_PublishTrace(const VehicleManagerOutput *output)
{
    g_vehicleManagerState = (uint8)output->state;
    g_vehicleManagerOwner = (uint8)output->owner;
    g_vehicleManagerFaultMask = output->faultMask;
    g_vehicleManagerRequestedSpeedKph = output->requestedSpeedKph;
    g_vehicleManagerTargetSpeedKph = output->targetSpeedKph;
    g_vehicleManagerRequestedSteeringDeg = output->requestedSteeringDeg;
    g_vehicleManagerTargetSteeringDeg = output->targetSteeringDeg;
    g_vehicleManagerEffectiveSteeringDeg = output->effectiveSteeringDeg;
    g_vehicleManagerSteeringLimitDeg = output->steeringLimitDeg;
    g_vehicleManagerCurvatureInvM = output->curvatureInvM;
    g_vehicleManagerTurnSpeedLimitKph = output->turnSpeedLimitKph;
    g_vehicleManagerPredictedLateralAccelMps2 = output->predictedLateralAccelMps2;
    g_vehicleManagerLateralLimitActive = (output->lateralLimitActive != FALSE) ? 1u : 0u;
    g_vehicleManagerSteeringLimitActive = (output->steeringLimitActive != FALSE) ? 1u : 0u;
    g_vehicleManagerCanRearmRequired = (output->canRearmRequired != FALSE) ? 1u : 0u;
    g_vehicleManagerStepCount++;
}

void VehicleManager_Init(void)
{
    g_vmPreviousPs2Available = FALSE;
    g_vmCanRearmRequired = FALSE;
    g_vmCanDisableSeen = FALSE;
    g_vmCanRearmBaselineCount = 0u;
    g_vmCanDisableCount = 0u;
    g_vmCanDisableSequence = 0u;
    g_vmCanAcceptedSequenceValid = FALSE;
    g_vmCanAcceptedCount = 0u;
    g_vmCanAcceptedSequence = 0u;
    g_vmPreviousEstopActive = FALSE;
    g_vmFeedbackFaultActive = FALSE;
    g_vmPs2NeutralRequired = FALSE;

    g_vehicleManagerState = (uint8)VehicleManagerState_Init;
    g_vehicleManagerOwner = (uint8)VehicleManagerOwner_None;
    g_vehicleManagerFaultMask = VM_FAULT_NONE;
    g_vehicleManagerRequestedSpeedKph = 0.0f;
    g_vehicleManagerTargetSpeedKph = 0.0f;
    g_vehicleManagerRequestedSteeringDeg = 0.0f;
    g_vehicleManagerTargetSteeringDeg = 0.0f;
    g_vehicleManagerEffectiveSteeringDeg = 0.0f;
    g_vehicleManagerSteeringLimitDeg = 0.0f;
    g_vehicleManagerCurvatureInvM = 0.0f;
    g_vehicleManagerTurnSpeedLimitKph = 0.0f;
    g_vehicleManagerPredictedLateralAccelMps2 = 0.0f;
    g_vehicleManagerLateralLimitActive = 0u;
    g_vehicleManagerSteeringLimitActive = 0u;
    g_vehicleManagerCanRearmRequired = 0u;
    g_vehicleManagerStepCount = 0u;
}

void VehicleManager_Step5ms(const VehicleManagerInput *input,
                            VehicleManagerOutput *output)
{
    boolean ps2Rising;
    boolean ps2Falling;
    boolean speedFeedbackRequired;
    uint32 feedbackFaultMask;

    if ((input == 0) || (output == 0))
    {
        return;
    }

    VehicleManager_ClearOutput(output);

    ps2Rising = ((input->ps2Available != FALSE) &&
                 (g_vmPreviousPs2Available == FALSE)) ? TRUE : FALSE;
    ps2Falling = ((input->ps2Available == FALSE) &&
                  (g_vmPreviousPs2Available != FALSE)) ? TRUE : FALSE;

    if (ps2Rising != FALSE)
    {
        VehicleManager_StartCanRearm(input, output, TRUE);
        g_vmPs2NeutralRequired = TRUE;
    }

    if (ps2Falling != FALSE)
    {
        /* Discard CAN frames received while manual control owned the vehicle. */
        VehicleManager_StartCanRearm(input, output, TRUE);
    }

    g_vmPreviousPs2Available = input->ps2Available;
    output->canRearmRequired = g_vmCanRearmRequired;

    if (input->estopActive != FALSE)
    {
        if ((g_vmPreviousEstopActive == FALSE) ||
            ((input->canModeSeen != FALSE) &&
             (g_vmCanRearmRequired == FALSE)))
        {
            VehicleManager_LatchSafetyRearm(input, output);
        }
        g_vmPreviousEstopActive = TRUE;
        output->state = VehicleManagerState_Estop;
        output->faultMask |= VM_FAULT_ESTOP_ACTIVE;
        VehicleManager_RequestCenter(input, output);
        VehicleManager_PublishTrace(output);
        return;
    }
    g_vmPreviousEstopActive = FALSE;

    speedFeedbackRequired =
        ((input->ps2Available != FALSE) ||
         (input->serviceTestRequested != FALSE) ||
         ((input->canModeSeen != FALSE) &&
          (input->canCommandValid != FALSE) &&
          (input->canCommandEnabled != FALSE))) ? TRUE : FALSE;

    feedbackFaultMask = VM_FAULT_NONE;
    if (input->steeringFeedbackValid == FALSE)
    {
        feedbackFaultMask |= VM_FAULT_STEERING_FEEDBACK;
    }
    if ((speedFeedbackRequired != FALSE) &&
        (input->speedFeedbackValid == FALSE))
    {
        feedbackFaultMask |= VM_FAULT_SPEED_FEEDBACK;
    }

    if (feedbackFaultMask != VM_FAULT_NONE)
    {
        if ((g_vmFeedbackFaultActive == FALSE) ||
            ((input->canModeSeen != FALSE) &&
             (g_vmCanRearmRequired == FALSE)))
        {
            VehicleManager_LatchSafetyRearm(input, output);
        }
        g_vmFeedbackFaultActive = TRUE;
        output->state = VehicleManagerState_FaultTorqueOff;
        output->faultMask |= feedbackFaultMask;
        VehicleManager_RequestCenter(input, output);
        VehicleManager_PublishTrace(output);
        return;
    }
    g_vmFeedbackFaultActive = FALSE;

    if (input->ps2Available != FALSE)
    {
        if (g_vmPs2NeutralRequired != FALSE)
        {
            if (VehicleManager_AbsFloat(input->ps2RequestedSpeedKph) > 0.0f)
            {
                output->state = VehicleManagerState_FaultTorqueOff;
                output->faultMask |= VM_FAULT_PS2_NEUTRAL_REQUIRED;
                VehicleManager_RequestCenter(input, output);
                VehicleManager_PublishTrace(output);
                return;
            }
            g_vmPs2NeutralRequired = FALSE;
        }

        output->owner = VehicleManagerOwner_Ps2;
        output->state = VehicleManagerState_ManualPs2;
        output->requestedSpeedKph = VehicleManager_ClampFloat(input->ps2RequestedSpeedKph,
                                                              -VM_PS2_MAX_REVERSE_SPEED_KPH,
                                                              VM_PS2_MAX_SPEED_KPH);
        output->requestedSteeringDeg = input->ps2RequestedSteeringDeg;
        VehicleManager_ApplyMotionEnvelope(output,
                                           VM_PS2_MAX_SPEED_KPH,
                                           input->measuredSpeedKph,
                                           input->measuredSteeringDeg);
    }
    else if (input->canModeSeen != FALSE)
    {
        if (g_vmCanRearmRequired != FALSE)
        {
            if (VehicleManager_TryCompleteCanRearm(input, output) == FALSE)
            {
                VehicleManager_RequestCenter(input, output);
                VehicleManager_PublishTrace(output);
                return;
            }
        }

        if (input->canCommandValid == FALSE)
        {
            VehicleManager_StartCanRearm(input, output, FALSE);
            output->state = VehicleManagerState_FaultTorqueOff;
            output->faultMask |= VM_FAULT_CAN_INVALID;
            VehicleManager_RequestCenter(input, output);
            VehicleManager_PublishTrace(output);
            return;
        }

        if (input->canCommandEnabled == FALSE)
        {
            g_vmCanRearmRequired = TRUE;
            g_vmCanDisableSeen = TRUE;
            g_vmCanRearmBaselineCount = input->canValidCount;
            g_vmCanAcceptedSequenceValid = FALSE;
            g_vmCanDisableCount = input->canValidCount;
            g_vmCanDisableSequence = input->canSequence;
            output->state = VehicleManagerState_Standby;
            output->canRearmRequired = TRUE;
            VehicleManager_RequestCenter(input, output);
            VehicleManager_PublishTrace(output);
            return;
        }

        if (VehicleManager_CheckActiveCanSequence(input, output) == FALSE)
        {
            VehicleManager_RequestCenter(input, output);
            VehicleManager_PublishTrace(output);
            return;
        }

        output->owner = VehicleManagerOwner_Can;
        output->state = VehicleManagerState_AutoCan;
        output->requestedSpeedKph = VehicleManager_ClampFloat(input->canRequestedSpeedKph,
                                                              -VM_CAN_MAX_REVERSE_SPEED_KPH,
                                                              VM_CAN_MAX_SPEED_KPH);
        output->requestedSteeringDeg = input->canRequestedSteeringDeg;
        VehicleManager_ApplyMotionEnvelope(output,
                                           VM_CAN_MAX_SPEED_KPH,
                                           input->measuredSpeedKph,
                                           input->measuredSteeringDeg);
    }
    else if (input->serviceTestRequested != FALSE)
    {
        output->owner = VehicleManagerOwner_ServiceTest;
        output->state = (input->serviceValidationRequested != FALSE) ?
                        VehicleManagerState_Validation :
                        VehicleManagerState_ServiceSpeedTest;
        output->requestedSpeedKph = VehicleManager_ClampFloat(
            input->serviceRequestedSpeedKph,
            0.0f,
            VM_CAN_MAX_SPEED_KPH);
        /* Mode 1 supplies zero; Mode 2 supplies the validation steering pattern. */
        output->requestedSteeringDeg = input->serviceRequestedSteeringDeg;
        VehicleManager_ApplyMotionEnvelope(output,
                                           VM_CAN_MAX_SPEED_KPH,
                                           input->measuredSpeedKph,
                                           input->measuredSteeringDeg);
    }
    else
    {
        output->state = VehicleManagerState_Standby;
        VehicleManager_RequestCenter(input, output);
        output->allowServiceTests =
            ((input->speedFeedbackValid != FALSE) &&
             (input->steeringFeedbackValid != FALSE) &&
             (output->state == VehicleManagerState_Standby) &&
             (g_vmCanRearmRequired == FALSE)) ? TRUE : FALSE;
        VehicleManager_PublishTrace(output);
        return;
    }

    output->driveEnable = TRUE;
    output->steeringTargetValid = TRUE;
    output->allowServiceTests = FALSE;
    output->canRearmRequired = g_vmCanRearmRequired;
    VehicleManager_PublishTrace(output);
}

#include "ControlLoop.h"

#include "Adas_Manager.h"
#include "Board_Pins.h"
#include "CCU6_PWM_Generation.h"
#include "Can_SpeedTx.h"
#include "Can_TrajectoryFd.h"
#include "Drive_PWM.h"
#include "ERU_Interrupt.h"
#include "GraphTest.h"
#include "GPT12_Timer_Interrupt.h"
#include "IfxPort.h"
#include "Ps2_Controller.h"
#include "Speed_Calculate.h"
#include "Steering_ADC.h"
#include "VehicleManager.h"

#define CONTROL_LOOP_PERIOD_SEC       (0.005f)
#define STEERING_KP_US_PER_DEG        (12.0f)
#define STEERING_KI_US_PER_DEG_SEC    (0.0f)
#define STEERING_INTEGRAL_LIMIT_US    (150.0f)
#define STEERING_DEADBAND_DEG         (0.5f)
#define STEERING_MIN_MOVE_OFFSET_US   (120.0f)
#define STEERING_CONTROL_SIGN         (1.0f)
#define STEERING_TARGET_LIMIT_DEG     (STEERING_MAX_ANGLE_DEG)
#define STEERING_PWM_ONLY_TEST_ENABLE (0u)
#define STEERING_PWM_TEST_STEP_PERIOD_LOOPS (100u)
#define STEERING_PWM_TEST_STEP_US     (2u)
#define STEERING_POT_PID_ENABLE       (1u)
#define STEERING_POT_PID_KP_US_PER_RAW (0.06f)
#define STEERING_POT_PID_KI_US_PER_RAW_SEC (0.0f)
#define STEERING_POT_PID_KD_US_PER_RAW_PER_SEC (0.0f)
#define STEERING_POT_PID_INTEGRAL_LIMIT_US (120.0f)
#define STEERING_POT_PID_CORRECTION_LIMIT_US (220.0f)
#define STEERING_POT_PID_DEADBAND_RAW (20u)
#define STEERING_POT_PID_MAX_STEP_US  (5u)
#define SPEED_TARGET_KMH              (0.0f)
#define SPEED_TARGET_MPS              (0.0f)
#define SPEED_TARGET_LIMIT_MPS        (4.0f)
#define SPEED_CONTROL_START_DUTY      (10.0f)
/* Least-squares fit of duty vs. rpm from the actual converged holding duty
 * at 12 logged targets spanning 40-260 rpm (2-13 km/h), each read from a
 * settled 1 s window under real driving load. A single straight line fits
 * all 12 points to within ~1 %duty -- e.g. at 260 rpm the fit predicts
 * 97.8% and the measured value was 97.8%. That means the true motor+load
 * duty/rpm relationship really is linear (slope 0.368 %duty/rpm, plus a
 * ~2.2% static-friction intercept); it was never a local anomaly at any
 * particular rpm. The previous single-constant form (rpm/336*100, i.e.
 * slope 0.298 with zero intercept) was simply the wrong line -- optimistic
 * everywhere, and increasingly so at higher rpm (gap grew from ~5% at 40
 * rpm to ~20% at 260 rpm), which is exactly the "PID has to make up more
 * and more of the gap as target rises" pattern chased across several
 * tuning rounds before this was measured directly. Re-measure by holding
 * GraphTest steps and reading the settled SpeedControlStatus duty at each
 * level if the drivetrain/load changes. */
#define SPEED_CONTROL_FEEDFORWARD_SLOPE_DUTY_PER_RPM (0.368f)
#define SPEED_CONTROL_FEEDFORWARD_INTERCEPT_DUTY     (2.2f)
/* Confirmed on two independent lifted-wheel 1-13 km/h sweeps: duty stddev
 * during the last settled second was ~1.9-2.0 %duty at 140 rpm (7 km/h)
 * both times, vs ~0.0-0.5 everywhere else (100/160 rpm each looked bad on
 * only one of the two runs -- not repeatable, so left alone). That's
 * consistent with motor/gearbox torque ripple (cogging, gear-mesh
 * harmonic) specific to that output speed, not a PID tuning gap. Nudge
 * feedforward up in a triangular band around it so the PID has less
 * correcting to do there; band edges taper to zero so there's no sudden
 * duty step entering/leaving the band. This is a first-pass magnitude --
 * verify with a repeat log and adjust. */
/* This is a variance finding, not a mean-duty one -- separate from the
 * slope/intercept fit above. Confirmed on two independent lifted-wheel
 * 1-13 km/h sweeps: duty stddev during the last settled second was
 * ~1.9-2.0 %duty at 140 rpm (7 km/h) both times, vs ~0.0-0.5 everywhere
 * else, consistent with motor/gearbox torque ripple (cogging, gear-mesh
 * harmonic) specific to that output speed -- a real vibration, not a mean
 * duty gap the corrected feedforward line above already explains. Keep this
 * small triangular boost on top of the corrected base line to damp it. */
#define SPEED_RESONANCE_BAND_CENTER_RPM        (140.0f)
#define SPEED_RESONANCE_BAND_HALFWIDTH_RPM     (15.0f)
#define SPEED_RESONANCE_FEEDFORWARD_BOOST_DUTY (3.0f)
#define SPEED_PID_KP_DUTY_PER_RPM     (0.35f)
/* Was raised 0.60 -> 0.90 (clamp widened 10 -> 18) to kill a steady-state
 * RPM offset -- but that offset was actually the raw-error-vs-deadbanded-
 * error integrator bug fixed above, not a lack of gain. With that structural
 * fix in place, 0.90 + an 18 %duty clamp turned out to be more authority
 * than the 40-120 rpm band (2, 5, 6 km/h) needs: duty was non-saturated
 * there (15-50%), so the integrator had room to swing, and it did -- a
 * sustained multi-second period oscillation (duty kicks up ~8-10 %duty then
 * decays, rpm swinging +/-10-25 rpm around target), confirmed target-flat/
 * current-oscillating in logged data, not present at 160+ rpm where duty is
 * closer to saturation. First revert (0.90 -> 0.60, clamp 18 -> 12) only
 * slowed the oscillation's period (~3.5-4 s -> ~5-6 s) without removing it,
 * confirmed by a repeat log -- so the same lever needed to go further, not a
 * different lever. */
#define SPEED_PID_KI_DUTY_PER_RPM_SEC (0.40f)
/* Kd raised 0.001 -> 0.005 (5x) to actually damp the rise/settle overshoot
 * instead of sitting near-idle. At 0.001, a fast 2000 rpm/s error swing
 * only produced ~2% duty of correction, well under the existing +/-5%
 * derivative clamp (SPEED_CONTROL_DERIVATIVE_LIMIT_DUTY, left unchanged --
 * it's the safety bound that keeps this low-risk even if Kd needs another
 * bump later). This is meant to trim the ~10-20% transient overshoot seen
 * on every step without touching final settling accuracy, which is
 * already good and is I's job, not D's. */
#define SPEED_PID_KD_DUTY_PER_RPM_PER_SEC (0.005f)
/* This deadband now only shapes the P/D error (chatter suppression from
 * encoder noise near the setpoint). The integrator uses the raw error
 * (see ControlLoop_UpdateDriveSpeedControl), so it is no longer the thing
 * capping how close the loop can settle -- it used to be, before I was
 * switched to integrate the raw error instead of this deadbanded one. */
#define SPEED_CONTROL_ERROR_DEADBAND_RPM (1.0f)
/* One encoder count of sample-to-sample jitter is ~0.3 rpm; below that it's
 * measurement quantization, not real error, so don't integrate it. */
#define SPEED_CONTROL_INTEGRAL_DEADBAND_RPM (0.3f)
/* Tightened 18 -> 12 -> 8 alongside the Ki revert above, for the same
 * reason each time: less room for the integrator to swing in the
 * non-saturated 2/5/6 km/h band where the oscillation was observed.
 * NOTE: logged data at 8 later showed this clamp is too tight to let I
 * fully close the gap at high targets (13 km/h needs ~14 %duty of I, but
 * it pins at 8, leaving a flat ~7% steady-state undershoot -- confirmed
 * not-still-converging), and at 80 rpm the needed I (~7-9%) sits almost
 * exactly on this ceiling, which is likely what the oscillation there
 * actually was (pin/release, not classic windup). A wider clamp (~20)
 * fixes both without reintroducing the earlier fast oscillation, since Ki
 * (the integration rate) is what drives oscillation speed, not this
 * ceiling -- kept at 8 for now per explicit choice, revisit if the
 * high-target undershoot or 80 rpm behavior needs fixing later. */
#define SPEED_CONTROL_INTEGRAL_LIMIT_DUTY (8.0f)
/* Raised 5 -> 12 %duty. Kd was already raised 5x (0.001 -> 0.005) last
 * round and it barely changed the overshoot -- because D was hitting this
 * clamp almost immediately on any real error swing, so more Kd just meant
 * hitting the same ceiling sooner, not applying more damping. Widening the
 * ceiling itself is what actually lets the larger Kd do anything. */
#define SPEED_CONTROL_DERIVATIVE_LIMIT_DUTY (12.0f)
#define SPEED_CONTROL_MAX_DUTY        (100.0f)
#define SPEED_CONTROL_DUTY_RAMP_PER_LOOP (1.0f)
/* Soft-start the SETPOINT itself, not just the duty output. A stepped
 * target (e.g. 0 -> 13 km/h instantly) forces a large instantaneous error,
 * which is exactly what drives the overshoot spike on every big step --
 * no amount of P/I/duty-ramp tuning removes that without also either
 * slowing the whole loop down or undershooting at high targets (both
 * tried and reverted). Ramping the target instead means the error the PID
 * ever sees stays small and the measured RPM can track the ramp almost the
 * whole way up, so there is nothing left to overshoot.
 * Slowed 1.5 -> 1.0 m/s^2: at 1.5, overshoot stayed ~8-11% at every level
 * regardless of Ki/Kd/anti-windup changes, which points at the ramp still
 * outrunning how fast the drivetrain + 100 ms speed-average filter can
 * actually keep up -- the measured RPM is inherently reacting to the
 * setpoint of ~100 ms ago no matter how the PID is tuned. Giving the ramp
 * more time (13 km/h now takes ~3.6 s) leaves the filter's lag a smaller
 * fraction of the rise, which is the one lever untried so far. 3.6 s still
 * fits every hold length seen in testing (shortest was ~2.2 s, but that
 * was the 1 km/h step, which only needs ~0.3 s to ramp regardless of this
 * rate). */
#define SPEED_TARGET_RAMP_MPS_PER_SEC (1.0f)
#define SPEED_TARGET_RAMP_MPS_PER_LOOP (SPEED_TARGET_RAMP_MPS_PER_SEC * CONTROL_LOOP_PERIOD_SEC)
/* Low-pass the measured RPM feedback itself before it ever reaches the
 * error calc (and therefore before it reaches CAN telemetry too, since
 * g_currentVehicleRpm is assigned from this filtered value below). Gain
 * tuning (Ki/Kd/anti-windup/ramp) mainly moved the transient overshoot
 * around without settling the ripple; this trades a bit of responsiveness
 * for a visibly flatter plateau, which is what was actually asked for.
 * alpha = dt / (RC + dt) for a single-pole filter at the 5 ms loop rate.
 * Was pushed down to 0.02 (~0.65 Hz corner) to flatten the 140 rpm
 * resonance wobble. That's now handled directly by the resonance
 * feedforward band above, so this no longer needs to carry that job --
 * and at 0.02 it was adding ~0.2-0.3 s of feedback lag, which combined
 * with Ki was the other half of the 4-6 km/h oscillation (see Ki comment
 * above): lag pushes the loop's phase margin down, so the same Ki that's
 * stable without the lag can self-oscillate with it. 0.06 (~1.9 Hz corner)
 * still meaningfully smooths encoder noise while cutting most of that lag
 * back out. */
#define SPEED_FEEDBACK_FILTER_ALPHA (0.06f)
#define SPEED_BRAKE_MAX_DUTY          (70.0f)
#define SPEED_OVERSPEED_BRAKE_RPM     (1.0f)
#define SPEED_OVERSPEED_BRAKE_GAIN    (2.0f)
#define SPEED_CALIBRATION_FIXED_DUTY_ENABLE (0u)
#define SPEED_CALIBRATION_FIXED_DUTY_PERCENT (100.0f)
#define SPEED_BUFFER_AUTO_TEST_DUTY_PERCENT (100u)
#define SPEED_BUFFER_AUTO_TEST_START_SIZE   (SPEED_BUFFER_FIXED_COUNT)
#define SPEED_BUFFER_AUTO_TEST_END_SIZE     (SPEED_BUFFER_FIXED_COUNT)
#define SPEED_BUFFER_AUTO_TEST_STEP_SIZE    (SPEED_BUFFER_FIXED_COUNT)
#define SPEED_BUFFER_AUTO_TEST_STAGE_MS     (2000u)
#define SPEED_BUFFER_AUTO_TEST_STATUS_IDLE  (0u)
#define SPEED_BUFFER_AUTO_TEST_STATUS_RUN   (1u)
#define SPEED_BUFFER_AUTO_TEST_STATUS_DONE  (2u)
#define UNIT_TEST_CASE_NONE                 (0u)
#define UNIT_TEST_CASE_MAX_SPEED            (1u)
#define UNIT_TEST_CASE_DRIVE_DEADBAND       (2u)
#define UNIT_TEST_CASE_CANCEL               (255u)
#define UNIT_TEST_STATUS_IDLE               (0u)
#define UNIT_TEST_STATUS_RUN                (1u)
#define UNIT_TEST_STATUS_DONE               (2u)
#define UNIT_TEST_STATUS_NO_MOTION          (3u)
#define UNIT_TEST_STATUS_CANCELLED          (4u)
#define MAX_SPEED_TEST_PHASE_IDLE           (0u)
#define MAX_SPEED_TEST_PHASE_ZERO_DUTY      (1u)
#define MAX_SPEED_TEST_PHASE_FULL_DUTY      (2u)
#define MAX_SPEED_TEST_PHASE_DONE           (3u)
#define MAX_SPEED_TEST_DUTY_PERCENT         (100u)
#define MAX_SPEED_TEST_ZERO_DUTY_MS         (1000u)
#define MAX_SPEED_TEST_RUN_MS               (6000u)
#define MAX_SPEED_TEST_BUFFER_SIZE          (SPEED_BUFFER_FIXED_COUNT)
#define DRIVE_DEADBAND_TEST_START_DUTY_PERCENT (0u)
#define DRIVE_DEADBAND_TEST_END_DUTY_PERCENT   (12u)
#define DRIVE_DEADBAND_TEST_STEP_DUTY_PERCENT  (1u)
#define DRIVE_DEADBAND_TEST_STAGE_MS           (1000u)
#define DRIVE_DEADBAND_TEST_MOVE_STABLE_MS     (500u)
#define DRIVE_DEADBAND_TEST_MOVE_THRESHOLD_RPM (20.0f)
#define DRIVE_DEADBAND_TEST_BUFFER_SIZE        (SPEED_BUFFER_FIXED_COUNT)
#define UNIT_TEST_DONE_HOLD_MS                 (2000u)
#define JOYSTICK_PRIORITY_TIMEOUT_MS           (200u)
#define JOYSTICK_DRIVE_BLOCK_NONE              (0u)
#define JOYSTICK_DRIVE_BLOCK_DISABLED          (1u)
#define JOYSTICK_DRIVE_BLOCK_NO_PAD            (2u)
#define DRIVE_DIRECTION_STOP                   (0u)
#define DRIVE_DIRECTION_FORWARD                (1u)
#define DRIVE_DIRECTION_REVERSE                (2u)
#define AUTO_CAN_SOURCE_NONE                   (0u)
#define AUTO_CAN_SOURCE_LEGACY_PC              (1u)
#define AUTO_CAN_SOURCE_ADAS_MANAGER           (2u)

volatile float32 g_targetSteeringDeg = 0.0f;
volatile float32 g_limitedSteeringTargetDeg = 0.0f;
volatile float32 g_currentSteeringDeg = 0.0f;
volatile float32 g_dbgSteeringErrorDeg = 0.0f;
volatile float32 g_dbgSteeringCorrectionUs = 0.0f;
volatile float32 g_currentVehicleMps = 0.0f;
volatile float32 g_targetVehicleKph = SPEED_TARGET_KMH;
volatile float32 g_targetVehicleMps = SPEED_TARGET_MPS;
volatile float32 g_currentVehicleRpm = 0.0f;
volatile float32 g_targetVehicleRpm = 0.0f;
volatile float32 g_speedControlErrorMps = 0.0f;
volatile float32 g_speedControlErrorRpm = 0.0f;
volatile float32 g_speedControlFeedforwardDuty = 0.0f;
volatile float32 g_speedControlProportionalDuty = 0.0f;
volatile float32 g_speedControlDutyCommand = 0.0f;
volatile float32 g_motorAppliedDutyPct = 0.0f;
volatile float32 g_speedControlIntegralDuty = 0.0f;
volatile float32 g_speedControlDerivativeDuty = 0.0f;
volatile float32 g_speedControlPidDuty = 0.0f;
volatile uint8   g_speedControlEnabled = 1u;
volatile uint8   g_speedBrakeActive = 0u;
volatile uint8   g_speedCalibrationFixedDutyEnable = SPEED_CALIBRATION_FIXED_DUTY_ENABLE;
volatile float32 g_speedCalibrationFixedDutyPercent = SPEED_CALIBRATION_FIXED_DUTY_PERCENT;
volatile uint8   g_manualDutyTestEnable = 0u;
volatile float32 g_manualDutyTestPercent = 0.0f;
volatile float32 g_manualDutyTestCurrentRpm = 0.0f;
volatile float32 g_manualDutyTestCurrentKph = 0.0f;
volatile uint8   g_joystickDriveModeEnabled = 1u;
volatile uint8   g_joystickDriveModeActive = 0u;
volatile uint8   g_joystickDriveModeBlockedReason = JOYSTICK_DRIVE_BLOCK_NO_PAD;
volatile uint8   g_joystickPriorityActive = 0u;
volatile float32 g_joystickTargetRpm = 0.0f;
volatile uint8   g_pcCommandControlActive = 0u;
volatile uint8   g_pcCommandFailsafeActive = 0u;
volatile uint16  g_pcCommandAppliedSteeringRaw = STEERING_ADC_RAW_CENTER;
volatile uint8   g_speedBufferAutoTestStart = 0u;
volatile uint8   g_speedBufferAutoTestActive = 0u;
volatile uint8   g_speedBufferAutoTestDone = 0u;
volatile uint8   g_speedBufferAutoTestStatus = SPEED_BUFFER_AUTO_TEST_STATUS_IDLE;
volatile uint8   g_speedBufferAutoTestCurrentSize = SPEED_BUFFER_AUTO_TEST_START_SIZE;
volatile uint8   g_speedBufferAutoTestStageIndex = 0u;
volatile uint8   g_speedBufferAutoTestDutyPercent = SPEED_BUFFER_AUTO_TEST_DUTY_PERCENT;
volatile uint16  g_speedBufferAutoTestStageMs = SPEED_BUFFER_AUTO_TEST_STAGE_MS;
volatile uint16  g_speedBufferAutoTestElapsedMs = 0u;
volatile float32 g_speedBufferAutoTestAvgVehicleRpm = 0.0f;
volatile uint32  g_speedBufferAutoTestHoldZeroCount = 0u;
volatile uint8   g_unitTestCaseCommand = UNIT_TEST_CASE_NONE;
volatile uint8   g_unitTestCaseActive = 0u;
volatile uint8   g_unitTestCaseDone = 0u;
volatile uint8   g_unitTestCaseId = UNIT_TEST_CASE_NONE;
volatile uint8   g_unitTestCaseStatus = UNIT_TEST_STATUS_IDLE;
volatile uint8   g_maxSpeedTestPhase = MAX_SPEED_TEST_PHASE_IDLE;
volatile uint8   g_maxSpeedTestDutyPercent = MAX_SPEED_TEST_DUTY_PERCENT;
volatile uint16  g_maxSpeedTestZeroDutyMs = MAX_SPEED_TEST_ZERO_DUTY_MS;
volatile uint16  g_maxSpeedTestRunMs = MAX_SPEED_TEST_RUN_MS;
volatile uint16  g_maxSpeedTestElapsedMs = 0u;
volatile float32 g_maxSpeedTestCurrentRpm = 0.0f;
volatile float32 g_maxSpeedTestCurrentKph = 0.0f;
volatile float32 g_maxSpeedTestAvgRpm = 0.0f;
volatile float32 g_maxSpeedTestAvgKph = 0.0f;
volatile float32 g_maxSpeedTestMaxRpm = 0.0f;
volatile float32 g_maxSpeedTestMaxKph = 0.0f;
volatile uint32  g_maxSpeedTestSampleCount = 0u;
volatile uint8   g_driveDeadbandTestCurrentDutyPercent = 0u;
volatile uint8   g_driveDeadbandTestLastStoppedDutyPercent = 0u;
volatile uint8   g_driveDeadbandTestFirstMovingDutyPercent = 0u;
volatile uint8   g_driveDeadbandTestMaxDutyPercent = DRIVE_DEADBAND_TEST_END_DUTY_PERCENT;
volatile uint16  g_driveDeadbandTestStageMs = DRIVE_DEADBAND_TEST_STAGE_MS;
volatile uint16  g_driveDeadbandTestElapsedMs = 0u;
volatile uint16  g_driveDeadbandTestMoveStableMs = 0u;
volatile float32 g_driveDeadbandTestMoveThresholdRpm = DRIVE_DEADBAND_TEST_MOVE_THRESHOLD_RPM;
volatile float32 g_driveDeadbandTestCurrentRpm = 0.0f;
volatile float32 g_driveDeadbandTestCurrentKph = 0.0f;
volatile float32 g_driveDeadbandTestFinalRpm = 0.0f;
volatile float32 g_driveDeadbandTestFirstMovingRpm = 0.0f;
volatile float32 g_driveDeadbandTestFirstMovingKph = 0.0f;
volatile uint16  g_currentSteeringPulseUs = STEERING_SERVO_NO_JOYSTICK_US;
volatile uint16  g_manualSteeringPulseUs = STEERING_SERVO_NO_JOYSTICK_US;
volatile boolean g_manualSteeringPulseValid = FALSE;
volatile uint8   g_steeringPotPidEnabled = STEERING_POT_PID_ENABLE;
volatile uint8   g_steeringPotTargetValid = 0u;
volatile uint16  g_steeringPotTargetRaw = STEERING_ADC_RAW_CENTER;
volatile sint32  g_steeringPotErrorRaw = 0;
volatile float32 g_steeringPotPidKpUsPerRaw = STEERING_POT_PID_KP_US_PER_RAW;
volatile float32 g_steeringPotPidKiUsPerRawSec = STEERING_POT_PID_KI_US_PER_RAW_SEC;
volatile float32 g_steeringPotPidKdUsPerRawPerSec = STEERING_POT_PID_KD_US_PER_RAW_PER_SEC;
volatile float32 g_steeringPotPidIntegralUs = 0.0f;
volatile float32 g_steeringPotPidCorrectionUs = 0.0f;
volatile uint16  g_steeringPotPidFeedforwardPulseUs = STEERING_SERVO_CENTER_US;
volatile uint16  g_steeringPotPidCommandPulseUs = STEERING_SERVO_CENTER_US;
volatile uint16  g_steeringPotPidDeadbandRaw = STEERING_POT_PID_DEADBAND_RAW;
volatile uint16  g_steeringPotPidMaxStepUs = STEERING_POT_PID_MAX_STEP_US;
volatile uint8   g_steeringPotPidAtTarget = 0u;
volatile uint8   g_currentDriveDutyPercent = 0u;
volatile uint8   g_driveForwardEnabled = 0u;
volatile uint8   g_driveReverseEnabled = 0u;
volatile uint8   g_driveDirection = DRIVE_DIRECTION_STOP;
volatile uint8   g_ackermannState = (uint8)AckermannState_Init;
volatile uint8   g_estopPinHigh = 1u;
volatile uint8   g_estopActive = 0u;
volatile uint8   g_speedFeedbackHealthy = 0u;
volatile uint8   g_steeringFeedbackHealthy = 0u;
volatile uint8   g_driveDirectionInterlockActive = 0u;
volatile uint16  g_driveDirectionInterlockHoldLoops = 0u;
volatile uint32  g_encoderNoMotionLoopCount = 0u;
volatile uint32  g_steeringNoResponseLoopCount = 0u;
volatile uint32  g_dbgControlLoopInitCount = 0u;
volatile uint8   g_steeringPwmOnlyTestEnabled = STEERING_PWM_ONLY_TEST_ENABLE;
volatile sint32  g_steeringPwmTestDirection = 0;
volatile uint16  g_steeringPwmTestPulseUs = STEERING_SERVO_CENTER_US;
volatile uint8   g_steeringPwmTestAtLimit = 0u;
volatile uint8   g_steeringCenterTuneEnabled = 0u;
volatile uint16  g_steeringCenterTunePulseUs = STEERING_SERVO_CENTER_US;
volatile sint32  g_steeringCenterTuneDirection = 0;
volatile uint16  g_steeringCenterTuneStepUs = 1u;
volatile uint8   g_steeringCenterTuneAtLimit = 0u;

static float32 g_steeringIntegralUs = 0.0f;
static sint32 g_steeringPotPidPreviousErrorRaw = 0;
static boolean g_steeringPotPidHasPreviousError = FALSE;
static VehicleSpeedValue g_lastSpeed;
static float32 g_driveDutyCommandFloat = 0.0f;
static float32 g_speedTargetRampedMpsSigned = 0.0f;
static float32 g_speedFeedbackFilteredRpm = 0.0f;
static boolean g_speedFeedbackFilterInitialized = FALSE;
static float32 g_speedControlPreviousErrorRpm = 0.0f;
static boolean g_speedControlHasPreviousError = FALSE;
static uint16 g_encoderDebugPrevLeftCount = 0u;
static uint16 g_encoderDebugPrevRightCount = 0u;
static boolean g_encoderDebugHasPrevious = FALSE;
static uint16 g_steeringPwmTestLoopCount = 0u;
static uint32 g_speedBufferAutoTestStageLoopCount = 0u;
static uint32 g_maxSpeedTestPhaseLoopCount = 0u;
static uint32 g_maxSpeedTestRunLoopCount = 0u;
static float32 g_maxSpeedTestRpmSum = 0.0f;
static float32 g_maxSpeedTestKphSum = 0.0f;
static uint32 g_driveDeadbandTestStageLoopCount = 0u;
static uint32 g_driveDeadbandTestMoveLoopCount = 0u;
static uint32 g_driveDeadbandTestTotalLoopCount = 0u;
static uint32 g_unitTestDoneHoldLoopCount = 0u;
static uint8 g_graphTestTelemetryPreviousMode = (uint8)GraphTestMode_None;
static uint8 g_graphTestTelemetryPreviousState = (uint8)GraphTestState_Idle;
static uint8 g_graphTestTelemetryPreviousPhase = 0u;
/* VehicleManager sees one monotonic CAN command domain even when ownership
 * moves from legacy 0x300 to the ADAS Manager.  Raw application sequence
 * counters are used for E2E validation in their own receivers and are never
 * compared across sources. */
static uint8 g_autoCanSource = AUTO_CAN_SOURCE_NONE;
static uint8 g_autoCanRawSequence = 0u;
static uint32 g_autoCanRawValidCount = 0u;
static boolean g_autoCanRawCommandValid = FALSE;
static boolean g_autoCanRawCommandEnabled = FALSE;
static sint16 g_autoCanRawSpeedKphX100 = 0;
static sint16 g_autoCanRawSteeringDegX10 = 0;
static uint8 g_autoCanMuxSequence = 0u;
static uint32 g_autoCanMuxValidCount = 0u;

static void ControlLoop_ResetSpeedPid(void);

static float32 ControlLoop_ClampFloat(float32 value, float32 minValue, float32 maxValue)
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

static float32 ControlLoop_AbsFloat(float32 value)
{
    return (value < 0.0f) ? -value : value;
}

static sint32 ControlLoop_AbsSint32(sint32 value)
{
    return (value < 0) ? -value : value;
}

static boolean ControlLoop_IsSpeedFeedbackHealthy(void)
{
    /* Keep the proven version-16 traction feedback contract.  The VM still
     * owns/arbitrates the command, while the lower speed loop uses the same
     * left-encoder validity that the working vehicle used. */
    g_encoderNoMotionLoopCount = 0u;
    return (g_lastSpeed.left.valid != FALSE) ? TRUE : FALSE;
}

static boolean ControlLoop_IsSteeringFeedbackHealthy(SteeringAdcValue steering)
{
    /* Keep VM feedback gating, but avoid the former 500 ms false stop caused
     * by slow linkage motion or a small persistent center offset. */
    g_steeringNoResponseLoopCount = 0u;
    return (steering.valid != FALSE) ? TRUE : FALSE;
}

static uint16 ControlLoop_ClampSteeringPotTargetRaw(uint16 targetRaw)
{
    uint16 rawMin = g_steeringPotRawMin;
    uint16 rawCenter = g_steeringPotRawCenter;
    uint16 rawMax = g_steeringPotRawMax;

    if ((rawMin >= rawCenter) || (rawCenter >= rawMax))
    {
        return STEERING_ADC_RAW_CENTER;
    }

    if (targetRaw < rawMin)
    {
        return rawMin;
    }

    if (targetRaw > rawMax)
    {
        return rawMax;
    }

    return targetRaw;
}

static uint16 ControlLoop_MapSteeringDegX10ToRaw(sint16 steeringDegX10)
{
    uint16 rawMin = g_steeringPotRawMin;
    uint16 rawCenter = g_steeringPotRawCenter;
    uint16 rawMax = g_steeringPotRawMax;
    uint32 magnitude;
    uint32 offset;

    if ((rawMin >= rawCenter) || (rawCenter >= rawMax))
    {
        rawMin = STEERING_ADC_RAW_MIN;
        rawCenter = STEERING_ADC_RAW_CENTER;
        rawMax = STEERING_ADC_RAW_MAX;
    }

    if (steeringDegX10 >= 0)
    {
        magnitude = (uint32)steeringDegX10;
        if (magnitude > (uint32)ACKERMANN_CAN_PC_COMMAND_MAX_STEER_X10)
        {
            magnitude = (uint32)ACKERMANN_CAN_PC_COMMAND_MAX_STEER_X10;
        }

        offset = ((magnitude * (uint32)(rawMax - rawCenter)) +
                  ((uint32)ACKERMANN_CAN_PC_COMMAND_MAX_STEER_X10 / 2u)) /
                 (uint32)ACKERMANN_CAN_PC_COMMAND_MAX_STEER_X10;
        return ControlLoop_ClampSteeringPotTargetRaw((uint16)((uint32)rawCenter + offset));
    }

    magnitude = (uint32)(-(sint32)steeringDegX10);
    if (magnitude > (uint32)ACKERMANN_CAN_PC_COMMAND_MAX_STEER_X10)
    {
        magnitude = (uint32)ACKERMANN_CAN_PC_COMMAND_MAX_STEER_X10;
    }

    offset = ((magnitude * (uint32)(rawCenter - rawMin)) +
              ((uint32)ACKERMANN_CAN_PC_COMMAND_MAX_STEER_X10 / 2u)) /
             (uint32)ACKERMANN_CAN_PC_COMMAND_MAX_STEER_X10;
    return ControlLoop_ClampSteeringPotTargetRaw((uint16)((uint32)rawCenter - offset));
}

static uint16 ControlLoop_MapSteeringPotRawToPulse(uint16 targetRaw)
{
    uint32 deltaRaw;
    uint32 deltaPulse;
    uint16 rawMin = g_steeringPotRawMin;
    uint16 rawCenter = g_steeringPotRawCenter;
    uint16 rawMax = g_steeringPotRawMax;

    targetRaw = ControlLoop_ClampSteeringPotTargetRaw(targetRaw);

    if ((rawMin >= rawCenter) || (rawCenter >= rawMax))
    {
        return STEERING_SERVO_CENTER_US;
    }

    if (targetRaw <= rawMin)
    {
        return STEERING_SERVO_RIGHT_US;
    }

    if (targetRaw >= rawMax)
    {
        return STEERING_SERVO_LEFT_US;
    }

    if (targetRaw <= rawCenter)
    {
        deltaRaw = (uint32)(rawCenter - targetRaw);
        deltaPulse = (deltaRaw * (uint32)(STEERING_SERVO_RIGHT_US - STEERING_SERVO_CENTER_US)) /
                     (uint32)(rawCenter - rawMin);
        return (uint16)(STEERING_SERVO_CENTER_US + deltaPulse);
    }

    deltaRaw = (uint32)(targetRaw - rawCenter);
    deltaPulse = (deltaRaw * (uint32)(STEERING_SERVO_CENTER_US - STEERING_SERVO_LEFT_US)) /
                 (uint32)(rawMax - rawCenter);
    return (uint16)(STEERING_SERVO_CENTER_US - deltaPulse);
}

static float32 ControlLoop_MapSteeringPotRawToAngle(uint16 raw)
{
    uint16 rawMin = g_steeringPotRawMin;
    uint16 rawCenter = g_steeringPotRawCenter;
    uint16 rawMax = g_steeringPotRawMax;

    if ((rawMin >= rawCenter) || (rawCenter >= rawMax))
    {
        return 0.0f;
    }

    if (raw >= rawCenter)
    {
        return ((float32)(raw - rawCenter) * STEERING_MAX_ANGLE_DEG) /
               (float32)(rawMax - rawCenter);
    }

    return -((float32)(rawCenter - raw) * STEERING_MAX_ANGLE_DEG) /
           (float32)(rawCenter - rawMin);
}

static uint16 ControlLoop_ApplySteeringPulseSlew(uint16 desiredPulse)
{
    uint16 currentPulse = g_currentSteeringPulseUs;
    uint16 stepUs = g_steeringPotPidMaxStepUs;

    if (currentPulse < STEERING_SERVO_MIN_US)
    {
        currentPulse = STEERING_SERVO_MIN_US;
    }
    else if (currentPulse > STEERING_SERVO_MAX_US)
    {
        currentPulse = STEERING_SERVO_MAX_US;
    }

    if (stepUs == 0u)
    {
        stepUs = 1u;
    }
    else if (stepUs > 50u)
    {
        stepUs = 50u;
    }

    g_steeringPotPidMaxStepUs = stepUs;

    if (desiredPulse > currentPulse)
    {
        if ((uint16)(desiredPulse - currentPulse) > stepUs)
        {
            return (uint16)(currentPulse + stepUs);
        }
    }
    else if (currentPulse > desiredPulse)
    {
        if ((uint16)(currentPulse - desiredPulse) > stepUs)
        {
            return (uint16)(currentPulse - stepUs);
        }
    }

    return desiredPulse;
}

static uint16 ControlLoop_ComputeSteeringPotPidPulse(SteeringAdcValue steering)
{
    uint16 targetRaw = ControlLoop_ClampSteeringPotTargetRaw(g_steeringPotTargetRaw);
    uint16 feedforwardPulse = ControlLoop_MapSteeringPotRawToPulse(targetRaw);
    sint32 errorRaw = (sint32)targetRaw - (sint32)steering.raw;
    sint32 derivativeRaw = 0;
    float32 correctionUs;
    float32 desiredPulse;

    g_steeringPotTargetRaw = targetRaw;
    g_steeringPotErrorRaw = errorRaw;
    g_steeringPotPidFeedforwardPulseUs = feedforwardPulse;
    g_limitedSteeringTargetDeg = ControlLoop_MapSteeringPotRawToAngle(targetRaw);

    if (g_steeringPotPidHasPreviousError != FALSE)
    {
        derivativeRaw = (sint32)(errorRaw - g_steeringPotPidPreviousErrorRaw);
    }

    g_steeringPotPidPreviousErrorRaw = errorRaw;
    g_steeringPotPidHasPreviousError = TRUE;

    if (ControlLoop_AbsSint32(errorRaw) <= (sint32)g_steeringPotPidDeadbandRaw)
    {
        g_steeringPotPidAtTarget = 1u;
        g_steeringPotPidIntegralUs = 0.0f;
        g_steeringPotPidCorrectionUs = 0.0f;
        g_dbgSteeringErrorDeg = g_limitedSteeringTargetDeg - steering.angleDeg;
        g_dbgSteeringCorrectionUs = 0.0f;
        /*
         * Preserve the last stable corrective output while the potentiometer
         * remains inside the deadband.  Returning to an imperfect feed-forward
         * pulse here repeatedly re-creates the error and causes hunting.
         */
        g_steeringPotPidCommandPulseUs = g_currentSteeringPulseUs;
        if (g_steeringPotPidCommandPulseUs < STEERING_SERVO_MIN_US)
        {
            g_steeringPotPidCommandPulseUs = STEERING_SERVO_MIN_US;
        }
        else if (g_steeringPotPidCommandPulseUs > STEERING_SERVO_MAX_US)
        {
            g_steeringPotPidCommandPulseUs = STEERING_SERVO_MAX_US;
        }
        return g_steeringPotPidCommandPulseUs;
    }

    g_steeringPotPidAtTarget = 0u;
    g_steeringPotPidIntegralUs +=
        (-(float32)errorRaw) * g_steeringPotPidKiUsPerRawSec * CONTROL_LOOP_PERIOD_SEC;
    g_steeringPotPidIntegralUs = ControlLoop_ClampFloat(g_steeringPotPidIntegralUs,
                                                        -STEERING_POT_PID_INTEGRAL_LIMIT_US,
                                                        STEERING_POT_PID_INTEGRAL_LIMIT_US);

    correctionUs =
        (-(float32)errorRaw * g_steeringPotPidKpUsPerRaw) +
        g_steeringPotPidIntegralUs +
        (-(float32)derivativeRaw * g_steeringPotPidKdUsPerRawPerSec / CONTROL_LOOP_PERIOD_SEC);
    correctionUs = ControlLoop_ClampFloat(correctionUs,
                                          -STEERING_POT_PID_CORRECTION_LIMIT_US,
                                          STEERING_POT_PID_CORRECTION_LIMIT_US);

    desiredPulse = (float32)feedforwardPulse + correctionUs;
    desiredPulse = ControlLoop_ClampFloat(desiredPulse,
                                          (float32)STEERING_SERVO_MIN_US,
                                          (float32)STEERING_SERVO_MAX_US);

    g_steeringPotPidCorrectionUs = correctionUs;
    g_dbgSteeringErrorDeg = g_limitedSteeringTargetDeg - steering.angleDeg;
    g_dbgSteeringCorrectionUs = correctionUs;
    g_steeringPotPidCommandPulseUs = ControlLoop_ApplySteeringPulseSlew((uint16)desiredPulse);

    return g_steeringPotPidCommandPulseUs;
}

static float32 ControlLoop_GetClampedSteeringTarget(void)
{
    float32 targetDeg = ControlLoop_ClampFloat(g_targetSteeringDeg,
                                               -STEERING_TARGET_LIMIT_DEG,
                                               STEERING_TARGET_LIMIT_DEG);

    g_limitedSteeringTargetDeg = targetDeg;
    return targetDeg;
}

static uint16 ControlLoop_ComputeSteeringPulse(float32 targetDeg, float32 measuredDeg)
{
    float32 errorDeg = targetDeg - measuredDeg;
    float32 correctionUs;
    float32 pulse;

    if (ControlLoop_AbsFloat(errorDeg) <= STEERING_DEADBAND_DEG)
    {
        g_steeringIntegralUs = 0.0f;
        g_dbgSteeringErrorDeg = errorDeg;
        g_dbgSteeringCorrectionUs = 0.0f;
        return STEERING_SERVO_CENTER_US;
    }

    g_steeringIntegralUs += errorDeg * STEERING_KI_US_PER_DEG_SEC * CONTROL_LOOP_PERIOD_SEC;
    g_steeringIntegralUs = ControlLoop_ClampFloat(g_steeringIntegralUs,
                                                  -STEERING_INTEGRAL_LIMIT_US,
                                                  STEERING_INTEGRAL_LIMIT_US);

    correctionUs = STEERING_CONTROL_SIGN *
                   ((STEERING_KP_US_PER_DEG * errorDeg) + g_steeringIntegralUs);

    if (ControlLoop_AbsFloat(correctionUs) < STEERING_MIN_MOVE_OFFSET_US)
    {
        correctionUs = (correctionUs < 0.0f) ?
                       -STEERING_MIN_MOVE_OFFSET_US :
                       STEERING_MIN_MOVE_OFFSET_US;
    }

    g_dbgSteeringErrorDeg = errorDeg;
    g_dbgSteeringCorrectionUs = correctionUs;

    pulse = (float32)STEERING_SERVO_CENTER_US + correctionUs;

    pulse = ControlLoop_ClampFloat(pulse,
                                   (float32)STEERING_SERVO_MIN_US,
                                   (float32)STEERING_SERVO_MAX_US);

    return (uint16)pulse;
}

#if (DRIVE_ONLY_TEST == 0u)
static uint16 ControlLoop_ClampSteeringPulseUs(uint16 pulseUs)
{
    if (pulseUs < STEERING_SERVO_MIN_US)
    {
        return STEERING_SERVO_MIN_US;
    }

    if (pulseUs > STEERING_SERVO_MAX_US)
    {
        return STEERING_SERVO_MAX_US;
    }

    return pulseUs;
}

static void ControlLoop_UpdateSteeringPwmOnlyTest(void)
{
    uint16 pulseUs = ControlLoop_ClampSteeringPulseUs(g_steeringPwmTestPulseUs);
    uint16 stepUs = STEERING_PWM_TEST_STEP_US;

    g_steeringPwmTestLoopCount++;
    if (g_steeringPwmTestLoopCount >= STEERING_PWM_TEST_STEP_PERIOD_LOOPS)
    {
        g_steeringPwmTestLoopCount = 0u;

        if (g_steeringPwmTestDirection > 0)
        {
            if (pulseUs <= (uint16)(STEERING_SERVO_MAX_US - stepUs))
            {
                pulseUs = (uint16)(pulseUs + stepUs);
                g_steeringPwmTestAtLimit = 0u;
            }
            else
            {
                pulseUs = STEERING_SERVO_MAX_US;
                g_steeringPwmTestAtLimit = 1u;
            }
        }
        else if (g_steeringPwmTestDirection < 0)
        {
            if (pulseUs >= (uint16)(STEERING_SERVO_MIN_US + stepUs))
            {
                pulseUs = (uint16)(pulseUs - stepUs);
                g_steeringPwmTestAtLimit = 0u;
            }
            else
            {
                pulseUs = STEERING_SERVO_MIN_US;
                g_steeringPwmTestAtLimit = 1u;
            }
        }
        else
        {
            g_steeringPwmTestAtLimit = 0u;
        }
    }

    pulseUs = ControlLoop_ClampSteeringPulseUs(pulseUs);
    g_steeringPwmTestPulseUs = pulseUs;
    g_currentSteeringPulseUs = pulseUs;
    g_dbgSteeringCorrectionUs = (float32)pulseUs - (float32)STEERING_SERVO_CENTER_US;
    SteeringPwm_SetPulseUs(pulseUs);
}

static void ControlLoop_UpdateSteeringCenterTune(void)
{
    uint16 pulseUs = ControlLoop_ClampSteeringPulseUs(g_steeringCenterTunePulseUs);
    uint16 stepUs = g_steeringCenterTuneStepUs;

    if (stepUs == 0u)
    {
        stepUs = 1u;
    }
    else if (stepUs > 50u)
    {
        stepUs = 50u;
    }

    g_steeringCenterTuneStepUs = stepUs;

    if (g_steeringCenterTuneDirection > 0)
    {
        if (pulseUs <= (uint16)(STEERING_SERVO_MAX_US - stepUs))
        {
            pulseUs = (uint16)(pulseUs + stepUs);
            g_steeringCenterTuneAtLimit = 0u;
        }
        else
        {
            pulseUs = STEERING_SERVO_MAX_US;
            g_steeringCenterTuneAtLimit = 1u;
        }
    }
    else if (g_steeringCenterTuneDirection < 0)
    {
        if (pulseUs >= (uint16)(STEERING_SERVO_MIN_US + stepUs))
        {
            pulseUs = (uint16)(pulseUs - stepUs);
            g_steeringCenterTuneAtLimit = 0u;
        }
        else
        {
            pulseUs = STEERING_SERVO_MIN_US;
            g_steeringCenterTuneAtLimit = 1u;
        }
    }
    else
    {
        g_steeringCenterTuneAtLimit = 0u;
    }

    pulseUs = ControlLoop_ClampSteeringPulseUs(pulseUs);
    g_steeringCenterTunePulseUs = pulseUs;
    g_currentSteeringPulseUs = pulseUs;
    g_manualSteeringPulseUs = pulseUs;
    g_manualSteeringPulseValid = TRUE;
    g_dbgSteeringCorrectionUs = (float32)pulseUs - (float32)STEERING_SERVO_CENTER_US;
    SteeringPwm_SetPulseUs(pulseUs);
}
#endif

static sint32 ControlLoop_ComputeU16Delta(uint16 previousCount, uint16 currentCount)
{
    return (sint32)((sint16)(currentCount - previousCount));
}

static void ControlLoop_SendEncoderDebug(void)
{
    uint16 leftCount = dbgEncoderLeftHwCount;
    uint16 rightCount = dbgEncoderRightHwCount;
    sint32 leftDelta = 0;
    sint32 rightDelta = 0;

    if (g_encoderDebugHasPrevious != FALSE)
    {
        leftDelta = ControlLoop_ComputeU16Delta(g_encoderDebugPrevLeftCount, leftCount);
        rightDelta = ControlLoop_ComputeU16Delta(g_encoderDebugPrevRightCount, rightCount);
        leftDelta *= (sint32)ENCODER_LEFT_COUNT_SIGN;
        rightDelta *= (sint32)ENCODER_RIGHT_COUNT_SIGN;
    }

    g_encoderDebugPrevLeftCount = leftCount;
    g_encoderDebugPrevRightCount = rightCount;
    g_encoderDebugHasPrevious = TRUE;

    CanSpeedTx_SendEncoderDebug(leftCount, rightCount, leftDelta, rightDelta);
}

static void ControlLoop_ApplyDriveDutyDirected(uint8 dutyPercent, uint8 driveDirection)
{
    /* Direction/PWM application intentionally matches version 16. */
    g_driveDirectionInterlockActive = 0u;
    g_driveDirectionInterlockHoldLoops = 0u;

    if ((dutyPercent == 0u) || (driveDirection == DRIVE_DIRECTION_STOP))
    {
        DrivePwm_Stop();
        g_currentDriveDutyPercent = 0u;
        g_motorAppliedDutyPct = 0.0f;
        g_driveForwardEnabled = 0u;
        g_driveReverseEnabled = 0u;
        g_driveDirection = DRIVE_DIRECTION_STOP;
        SpeedCalc_SetBufferScanDutyPercent(0u);
        return;
    }

    if ((g_driveDirection != DRIVE_DIRECTION_STOP) &&
        (g_driveDirection != driveDirection))
    {
        DrivePwm_Stop();
        g_currentDriveDutyPercent = 0u;
        g_motorAppliedDutyPct = 0.0f;
        g_driveForwardEnabled = 0u;
        g_driveReverseEnabled = 0u;
        g_driveDirection = DRIVE_DIRECTION_STOP;
        SpeedCalc_SetBufferScanDutyPercent(0u);
        return;
    }

    if (driveDirection == DRIVE_DIRECTION_REVERSE)
    {
        DrivePwm_Reverse(dutyPercent);
        g_driveForwardEnabled = 0u;
        g_driveReverseEnabled = 1u;
        g_driveDirection = DRIVE_DIRECTION_REVERSE;
    }
    else
    {
        DrivePwm_Forward(dutyPercent);
        g_driveForwardEnabled = 1u;
        g_driveReverseEnabled = 0u;
        g_driveDirection = DRIVE_DIRECTION_FORWARD;
    }

    g_currentDriveDutyPercent = dutyPercent;
    g_motorAppliedDutyPct = (float32)dutyPercent;
    SpeedCalc_SetBufferScanDutyPercent(dutyPercent);
}

static void ControlLoop_ApplyDriveDuty(uint8 dutyPercent)
{
    ControlLoop_ApplyDriveDutyDirected(dutyPercent, DRIVE_DIRECTION_FORWARD);
}

static uint8 ControlLoop_DutyFloatToPercent(float32 duty)
{
    duty = ControlLoop_ClampFloat(duty, 0.0f, 100.0f);
    return (uint8)(duty + 0.5f);
}

static uint8 ControlLoop_ClampDutyPercent(uint8 dutyPercent)
{
    return (dutyPercent > 100u) ? 100u : dutyPercent;
}

static float32 ControlLoop_MpsToRpm(float32 mps)
{
    return (mps * 60.0f) / (3.14159265f * WHEEL_DIAMETER_M);
}

static float32 ControlLoop_RpmToMps(float32 rpm)
{
    return (rpm * 3.14159265f * WHEEL_DIAMETER_M) / 60.0f;
}

static float32 ControlLoop_RpmToKph(float32 rpm)
{
    return ControlLoop_RpmToMps(rpm) * 3.6f;
}

static float32 ControlLoop_ComputeFeedforwardDuty(float32 targetRpm)
{
    float32 duty;
    float32 distanceFromResonanceRpm;

    if (targetRpm <= 0.0f)
    {
        return 0.0f;
    }

    duty = (SPEED_CONTROL_FEEDFORWARD_SLOPE_DUTY_PER_RPM * targetRpm) +
           SPEED_CONTROL_FEEDFORWARD_INTERCEPT_DUTY;

    if (duty < SPEED_CONTROL_START_DUTY)
    {
        duty = SPEED_CONTROL_START_DUTY;
    }

    /* See SPEED_RESONANCE_BAND_CENTER_RPM comment: extra torque ripple
     * repeatably observed near 140 rpm. Triangular boost, zero at the band
     * edges, full SPEED_RESONANCE_FEEDFORWARD_BOOST_DUTY at the center. */
    distanceFromResonanceRpm = ControlLoop_AbsFloat(targetRpm - SPEED_RESONANCE_BAND_CENTER_RPM);
    if (distanceFromResonanceRpm < SPEED_RESONANCE_BAND_HALFWIDTH_RPM)
    {
        duty += SPEED_RESONANCE_FEEDFORWARD_BOOST_DUTY *
               (1.0f - (distanceFromResonanceRpm / SPEED_RESONANCE_BAND_HALFWIDTH_RPM));
    }

    return ControlLoop_ClampFloat(duty, 0.0f, SPEED_CONTROL_MAX_DUTY);
}

static void ControlLoop_ResetSpeedPid(void)
{
    g_speedControlProportionalDuty = 0.0f;
    g_speedControlIntegralDuty = 0.0f;
    g_speedControlDerivativeDuty = 0.0f;
    g_speedControlPidDuty = 0.0f;
    g_speedControlPreviousErrorRpm = 0.0f;
    g_speedControlHasPreviousError = FALSE;
    g_speedBrakeActive = 0u;
    /* Every reset path (e-stop, disable, fault, test-mode entry) should also
     * restart the target soft-start from zero, so the next launch always
     * ramps up cleanly instead of jumping from a stale ramped value. */
    g_speedTargetRampedMpsSigned = 0.0f;
    /* Force the feedback filter to reseed from the next real sample instead
     * of crawling up from a stale filtered value after a stop. */
    g_speedFeedbackFilterInitialized = FALSE;
}

static void ControlLoop_UpdateEStopState(void)
{
#if (ESTOP_ENABLE != 0u)
    g_estopPinHigh = IfxPort_getPinState(ESTOP_PORT, ESTOP_PIN) ? 1u : 0u;
    g_estopActive = (g_estopPinHigh == 0u) ? 1u : 0u;
#else
    g_estopPinHigh = 1u;
    g_estopActive = 0u;
#endif
}

void ControlLoop_StopDriveNow(void)
{
    g_targetVehicleKph = 0.0f;
    g_targetVehicleMps = 0.0f;
    g_targetVehicleRpm = 0.0f;
    g_speedControlErrorMps = 0.0f;
    g_speedControlErrorRpm = 0.0f;
    g_speedControlFeedforwardDuty = 0.0f;
    g_speedControlDutyCommand = 0.0f;
    g_driveDutyCommandFloat = 0.0f;
    ControlLoop_ResetSpeedPid();
    ControlLoop_ApplyDriveDutyDirected(0u, DRIVE_DIRECTION_STOP);
}

static float32 ControlLoop_ApplyDutyRamp(float32 currentDuty, float32 targetDuty, float32 ratePerLoop)
{
    float32 delta = targetDuty - currentDuty;

    if (delta > ratePerLoop)
    {
        return currentDuty + ratePerLoop;
    }

    if (delta < -ratePerLoop)
    {
        return currentDuty - ratePerLoop;
    }

    return targetDuty;
}

static void ControlLoop_UpdateDriveSpeedControl(boolean speedValid, float32 measuredMps, float32 measuredRpm)
{
    float32 targetMpsSigned;
    float32 targetMps;
    float32 targetRpm;
    float32 targetRpmSigned;
    float32 measuredSignedMps;
    float32 measuredSignedRpm;
    float32 controlErrorRpm;
    float32 feedforwardDuty;
    float32 derivativeRpmPerSec;
    float32 desiredDuty;
    float32 desiredDutySigned;
    float32 rampRate;
    uint8 requestedDriveDirection;
    uint8 driveDirection;

    if (g_estopActive != 0u)
    {
        ControlLoop_ResetSpeedPid();
        g_speedControlDutyCommand = 0.0f;
        g_driveDutyCommandFloat = 0.0f;
        ControlLoop_ApplyDriveDuty(0u);
        return;
    }

    if (g_speedControlEnabled == 0u)
    {
        ControlLoop_ResetSpeedPid();
        g_speedControlDutyCommand = 0.0f;
        g_driveDutyCommandFloat = 0.0f;
        ControlLoop_ApplyDriveDuty(0u);
        return;
    }

    g_speedBrakeActive = 0u;
    rampRate = SPEED_CONTROL_DUTY_RAMP_PER_LOOP;

    /* Smooth the measured RPM before anything downstream (error calc, P/I/D,
     * and the g_currentVehicleRpm telemetry assigned from it below) ever
     * sees it. Seed from the first real sample after a stop instead of
     * ramping up from zero, so a fresh launch doesn't start from a fake
     * low reading. */
    if (g_speedFeedbackFilterInitialized == FALSE)
    {
        g_speedFeedbackFilteredRpm = measuredRpm;
        g_speedFeedbackFilterInitialized = TRUE;
    }
    else
    {
        g_speedFeedbackFilteredRpm += SPEED_FEEDBACK_FILTER_ALPHA *
                                      (measuredRpm - g_speedFeedbackFilteredRpm);
    }
    measuredRpm = g_speedFeedbackFilteredRpm;
    measuredMps = ControlLoop_RpmToMps(measuredRpm);

    targetMpsSigned = ControlLoop_ClampFloat(g_targetVehicleMps,
                                             -SPEED_TARGET_LIMIT_MPS,
                                             SPEED_TARGET_LIMIT_MPS);
    requestedDriveDirection = (targetMpsSigned < 0.0f) ? DRIVE_DIRECTION_REVERSE : DRIVE_DIRECTION_FORWARD;
    driveDirection = requestedDriveDirection;

    if (ControlLoop_AbsFloat(targetMpsSigned) <= 0.0f)
    {
        driveDirection = DRIVE_DIRECTION_STOP;
        requestedDriveDirection = DRIVE_DIRECTION_STOP;
        /* A commanded stop must cut torque immediately, not coast down the
         * ramp -- reset it so the next launch always starts a fresh
         * soft-start from zero instead of resuming mid-ramp. */
        g_speedTargetRampedMpsSigned = 0.0f;
    }
    else if (g_speedTargetRampedMpsSigned < targetMpsSigned)
    {
        g_speedTargetRampedMpsSigned += SPEED_TARGET_RAMP_MPS_PER_LOOP;
        if (g_speedTargetRampedMpsSigned > targetMpsSigned)
        {
            g_speedTargetRampedMpsSigned = targetMpsSigned;
        }
    }
    else if (g_speedTargetRampedMpsSigned > targetMpsSigned)
    {
        g_speedTargetRampedMpsSigned -= SPEED_TARGET_RAMP_MPS_PER_LOOP;
        if (g_speedTargetRampedMpsSigned < targetMpsSigned)
        {
            g_speedTargetRampedMpsSigned = targetMpsSigned;
        }
    }

    /* Everything below tracks this ramped setpoint, not the raw step, so
     * the PID error never jumps to a big instantaneous value in the first
     * place -- there is nothing left to overshoot on. */
    targetMpsSigned = (driveDirection == DRIVE_DIRECTION_STOP) ? 0.0f : g_speedTargetRampedMpsSigned;
    targetMps = ControlLoop_AbsFloat(targetMpsSigned);

    targetRpm = ControlLoop_MpsToRpm(targetMps);
    targetRpmSigned = (driveDirection == DRIVE_DIRECTION_REVERSE) ? -targetRpm : targetRpm;
    measuredSignedMps = (driveDirection == DRIVE_DIRECTION_REVERSE) ? -measuredMps : measuredMps;
    measuredSignedRpm = (driveDirection == DRIVE_DIRECTION_REVERSE) ? -measuredRpm : measuredRpm;

    if (g_speedCalibrationFixedDutyEnable != 0u)
    {
        g_targetVehicleRpm = targetRpmSigned;
        g_currentVehicleMps = measuredSignedMps;
        g_currentVehicleRpm = measuredSignedRpm;
        g_speedControlErrorMps = targetMps - measuredMps;
        g_speedControlErrorRpm = targetRpm - measuredRpm;
        g_speedControlFeedforwardDuty = 0.0f;
        ControlLoop_ResetSpeedPid();
        desiredDuty = ControlLoop_ClampFloat(g_speedCalibrationFixedDutyPercent, 0.0f, 100.0f);
        g_driveDutyCommandFloat = ControlLoop_ApplyDutyRamp(g_driveDutyCommandFloat,
                                                            desiredDuty,
                                                            rampRate);
        g_speedControlDutyCommand = g_driveDutyCommandFloat;
        ControlLoop_ApplyDriveDutyDirected(ControlLoop_DutyFloatToPercent(g_driveDutyCommandFloat),
                                           driveDirection);
        return;
    }

    feedforwardDuty = ControlLoop_ComputeFeedforwardDuty(targetRpm);

    g_targetVehicleRpm = targetRpmSigned;
    g_currentVehicleMps = measuredSignedMps;
    g_currentVehicleRpm = measuredSignedRpm;
    g_speedControlFeedforwardDuty = feedforwardDuty;
    g_speedControlErrorMps = targetMps - measuredMps;
    g_speedControlErrorRpm = targetRpm - measuredRpm;

    /* Remove encoder/PID chatter around the requested speed without hiding
     * the raw error used by telemetry. */
    controlErrorRpm = g_speedControlErrorRpm;
    if (ControlLoop_AbsFloat(controlErrorRpm) <= SPEED_CONTROL_ERROR_DEADBAND_RPM)
    {
        controlErrorRpm = 0.0f;
    }
    else if (controlErrorRpm > 0.0f)
    {
        controlErrorRpm -= SPEED_CONTROL_ERROR_DEADBAND_RPM;
    }
    else
    {
        controlErrorRpm += SPEED_CONTROL_ERROR_DEADBAND_RPM;
    }

    if (targetMps <= 0.0f)
    {
        g_targetVehicleRpm = 0.0f;
        g_currentVehicleMps = g_lastSpeed.vehicleMpsSigned;
        g_currentVehicleRpm = g_lastSpeed.vehicleRpmSigned;
        g_speedControlFeedforwardDuty = 0.0f;
        g_speedControlErrorMps = -g_lastSpeed.vehicleMpsSigned;
        g_speedControlErrorRpm = -g_lastSpeed.vehicleRpmSigned;
        ControlLoop_ResetSpeedPid();
        g_driveDutyCommandFloat = 0.0f;
        g_speedControlDutyCommand = 0.0f;
        ControlLoop_ApplyDriveDutyDirected(0u, DRIVE_DIRECTION_STOP);
        return;
    }
    else if (speedValid == FALSE)
    {
        ControlLoop_ResetSpeedPid();
        desiredDuty = feedforwardDuty;
    }
    else
    {
        g_speedControlProportionalDuty = SPEED_PID_KP_DUTY_PER_RPM * controlErrorRpm;

        /* Integrate the RAW error (g_speedControlErrorRpm), not the
         * deadbanded controlErrorRpm. Feeding the integrator a deadbanded
         * error structurally freezes it as soon as |error| <= deadband, so
         * the loop can never settle closer than the deadband width -- that
         * was the source of the persistent steady-state RPM/kph offset.
         * P/D still use the deadbanded error so encoder noise near the
         * setpoint doesn't chatter the duty output; only I needs the raw
         * error so it keeps working until the true error is zero.
         *
         * But "raw" still needs its own tiny deadband. One encoder count
         * of jitter between consecutive 5 ms samples is ~0.3 rpm (60 /
         * (COUNTS_PER_REV * 0.005s)), and with zero deadband I was
         * integrating that single-count noise every single loop, never
         * resting -- producing a small continuous duty hunt/limit-cycle at
         * steady state, worst at low target speeds where 0.3 rpm is a
         * larger fraction of the target. This deadband is far smaller than
         * the old P/D one, so it only swallows sub-count measurement noise
         * and does not reintroduce the steady-state offset problem. */
        {
            float32 integralErrorRpm = g_speedControlErrorRpm;

            if (ControlLoop_AbsFloat(integralErrorRpm) <= SPEED_CONTROL_INTEGRAL_DEADBAND_RPM)
            {
                integralErrorRpm = 0.0f;
            }

            g_speedControlIntegralDuty += integralErrorRpm *
                                          SPEED_PID_KI_DUTY_PER_RPM_SEC *
                                          CONTROL_LOOP_PERIOD_SEC;
        }
        g_speedControlIntegralDuty = ControlLoop_ClampFloat(g_speedControlIntegralDuty,
                                                            -SPEED_CONTROL_INTEGRAL_LIMIT_DUTY,
                                                            SPEED_CONTROL_INTEGRAL_LIMIT_DUTY);

        if (g_speedControlHasPreviousError == FALSE)
        {
            derivativeRpmPerSec = 0.0f;
            g_speedControlHasPreviousError = TRUE;
        }
        else
        {
            derivativeRpmPerSec = (controlErrorRpm - g_speedControlPreviousErrorRpm) /
                                  CONTROL_LOOP_PERIOD_SEC;
        }

        g_speedControlPreviousErrorRpm = controlErrorRpm;
        g_speedControlDerivativeDuty = SPEED_PID_KD_DUTY_PER_RPM_PER_SEC * derivativeRpmPerSec;
        g_speedControlDerivativeDuty = ControlLoop_ClampFloat(g_speedControlDerivativeDuty,
                                                              -SPEED_CONTROL_DERIVATIVE_LIMIT_DUTY,
                                                              SPEED_CONTROL_DERIVATIVE_LIMIT_DUTY);
        g_speedControlPidDuty = g_speedControlProportionalDuty +
                                g_speedControlIntegralDuty +
                                g_speedControlDerivativeDuty;

        if (g_speedControlErrorRpm < -SPEED_OVERSPEED_BRAKE_RPM)
        {
            desiredDutySigned = feedforwardDuty +
                                (g_speedControlPidDuty * SPEED_OVERSPEED_BRAKE_GAIN);
        }
        else
        {
            desiredDutySigned = feedforwardDuty + g_speedControlPidDuty;
        }
        desiredDutySigned = ControlLoop_ClampFloat(desiredDutySigned,
                                                  -SPEED_BRAKE_MAX_DUTY,
                                                  SPEED_CONTROL_MAX_DUTY);
        if (desiredDutySigned < 0.0f)
        {
            /* A speed overshoot must never reverse the motor.  Reduce drive
             * torque in the requested direction and let the vehicle coast. */
            desiredDuty = 0.0f;
            g_speedBrakeActive = 0u;
            driveDirection = requestedDriveDirection;
        }
        else
        {
            desiredDuty = desiredDutySigned;
            driveDirection = requestedDriveDirection;
        }
    }

    {
        float32 dutyBeforeRamp = desiredDuty;

        g_driveDutyCommandFloat = ControlLoop_ApplyDutyRamp(g_driveDutyCommandFloat,
                                                            desiredDuty,
                                                            rampRate);

        /*
         * Anti-windup. The duty ramp caps how fast the applied duty can move
         * (SPEED_CONTROL_DUTY_RAMP_PER_LOOP), but that cap is the same
         * absolute %/loop regardless of target speed. A high-speed step
         * needs a much bigger feedforward duty, so it takes proportionally
         * longer for the ramp to catch up -- and while it is still catching
         * up, the measured RPM lags the commanded duty, so the raw-error
         * integral (see above) keeps accumulating on an error the actuator
         * is not yet allowed to correct. By the time the ramp catches up,
         * far more integral has piled up than the loop actually needs,
         * producing the growing overshoot seen at higher target speeds.
         * Roll back this loop's integral step whenever the ramp actually
         * clipped the commanded duty, so I only keeps growing while the
         * actuator is free to act on it.
         */
        if (g_driveDutyCommandFloat != dutyBeforeRamp)
        {
            g_speedControlIntegralDuty -= g_speedControlErrorRpm *
                                          SPEED_PID_KI_DUTY_PER_RPM_SEC *
                                          CONTROL_LOOP_PERIOD_SEC;
            g_speedControlIntegralDuty = ControlLoop_ClampFloat(g_speedControlIntegralDuty,
                                                                -SPEED_CONTROL_INTEGRAL_LIMIT_DUTY,
                                                                SPEED_CONTROL_INTEGRAL_LIMIT_DUTY);
        }
    }
    g_speedControlDutyCommand = g_driveDutyCommandFloat;
    ControlLoop_ApplyDriveDutyDirected(ControlLoop_DutyFloatToPercent(g_driveDutyCommandFloat),
                                       driveDirection);
}

static void ControlLoop_StartSpeedBufferAutoTest(void)
{
    g_speedBufferAutoTestStart = 0u;
    g_speedBufferAutoTestActive = 1u;
    g_speedBufferAutoTestDone = 0u;
    g_speedBufferAutoTestStatus = SPEED_BUFFER_AUTO_TEST_STATUS_RUN;
    g_speedBufferAutoTestCurrentSize = SPEED_BUFFER_AUTO_TEST_START_SIZE;
    g_speedBufferAutoTestStageIndex = 0u;
    g_speedBufferAutoTestElapsedMs = 0u;
    g_speedBufferAutoTestAvgVehicleRpm = 0.0f;
    g_speedBufferAutoTestHoldZeroCount = 0u;
    g_speedBufferAutoTestStageLoopCount = 0u;

    g_targetVehicleKph = 0.0f;
    g_targetVehicleMps = 0.0f;
    g_speedCalibrationFixedDutyEnable = 0u;
    ControlLoop_ResetSpeedPid();
    SpeedCalc_SetAverageBufferSize(SPEED_BUFFER_AUTO_TEST_START_SIZE, TRUE);
}

static void ControlLoop_StopSpeedBufferAutoTest(void)
{
    g_speedBufferAutoTestActive = 0u;
    g_speedBufferAutoTestDone = 1u;
    g_speedBufferAutoTestStatus = SPEED_BUFFER_AUTO_TEST_STATUS_DONE;
    g_speedBufferAutoTestElapsedMs = g_speedBufferAutoTestStageMs;
    g_speedBufferAutoTestStageLoopCount = 0u;

    g_targetVehicleKph = 0.0f;
    g_targetVehicleMps = 0.0f;
    g_speedControlDutyCommand = 0.0f;
    g_driveDutyCommandFloat = 0.0f;
    ControlLoop_ResetSpeedPid();
    ControlLoop_ApplyDriveDuty(0u);
    SpeedCalc_SetAverageBufferSize(SPEED_BUFFER_FIXED_COUNT, TRUE);
}

static void ControlLoop_AdvanceSpeedBufferAutoTest(void)
{
    uint8 nextSize;

    if (g_speedBufferAutoTestCurrentSize >= SPEED_BUFFER_AUTO_TEST_END_SIZE)
    {
        ControlLoop_StopSpeedBufferAutoTest();
        return;
    }

    nextSize = (uint8)(g_speedBufferAutoTestCurrentSize + SPEED_BUFFER_AUTO_TEST_STEP_SIZE);
    if (nextSize > SPEED_BUFFER_AUTO_TEST_END_SIZE)
    {
        nextSize = SPEED_BUFFER_AUTO_TEST_END_SIZE;
    }

    g_speedBufferAutoTestCurrentSize = nextSize;
    g_speedBufferAutoTestStageIndex++;
    g_speedBufferAutoTestElapsedMs = 0u;
    g_speedBufferAutoTestStageLoopCount = 0u;
    SpeedCalc_SetAverageBufferSize((uint16)nextSize, TRUE);
}

static void ControlLoop_UpdateSpeedBufferAutoTest(void)
{
    uint8 dutyPercent;
    uint32 elapsedMs;
    uint16 stageMs;

    if (g_speedBufferAutoTestStart != 0u)
    {
        ControlLoop_StartSpeedBufferAutoTest();
    }

    if (g_speedBufferAutoTestActive == 0u)
    {
        return;
    }

    dutyPercent = ControlLoop_ClampDutyPercent(g_speedBufferAutoTestDutyPercent);
    g_speedBufferAutoTestDutyPercent = dutyPercent;

    if ((g_lastSpeed.valid == TRUE) && (g_lastSpeed.vehicleRpm > 0.1f))
    {
        g_speedBufferAutoTestAvgVehicleRpm = g_lastSpeed.vehicleRpm;
    }
    else if (g_speedBufferAutoTestAvgVehicleRpm > 0.1f)
    {
        g_speedBufferAutoTestHoldZeroCount++;
    }

    g_speedControlFeedforwardDuty = 0.0f;
    g_speedControlDutyCommand = (float32)dutyPercent;
    g_driveDutyCommandFloat = (float32)dutyPercent;
    ControlLoop_ResetSpeedPid();
    ControlLoop_ApplyDriveDuty(dutyPercent);

    stageMs = (g_speedBufferAutoTestStageMs == 0u) ?
              SPEED_BUFFER_AUTO_TEST_STAGE_MS :
              g_speedBufferAutoTestStageMs;

    g_speedBufferAutoTestStageLoopCount++;
    elapsedMs = g_speedBufferAutoTestStageLoopCount * 5u;
    g_speedBufferAutoTestElapsedMs = (elapsedMs > 65535u) ? 65535u : (uint16)elapsedMs;

    if (elapsedMs >= (uint32)stageMs)
    {
        ControlLoop_AdvanceSpeedBufferAutoTest();
    }
}

static void ControlLoop_SendSpeedBufferAutoTestTelemetry(void)
{
    VehicleSpeedValue txSpeed = g_lastSpeed;

    if ((g_speedBufferAutoTestAvgVehicleRpm > 0.1f) &&
        ((txSpeed.valid == FALSE) || (txSpeed.vehicleRpm <= 0.1f)))
    {
        txSpeed.left.rawRpm = g_speedBufferAutoTestAvgVehicleRpm;
        txSpeed.left.avgRpm = g_speedBufferAutoTestAvgVehicleRpm;
        txSpeed.right.rawRpm = g_speedBufferAutoTestAvgVehicleRpm;
        txSpeed.right.avgRpm = g_speedBufferAutoTestAvgVehicleRpm;
        txSpeed.vehicleRpm = g_speedBufferAutoTestAvgVehicleRpm;
        txSpeed.vehicleMps = ControlLoop_RpmToMps(g_speedBufferAutoTestAvgVehicleRpm);
        txSpeed.valid = TRUE;
    }

    CanSpeedTx_SendWheelSpeed(&txSpeed);
    CanSpeedTx_SendRpmDebug(&txSpeed);
    CanSpeedTx_SendDriveStatus(g_currentDriveDutyPercent,
                               g_driveDirection,
                               g_estopActive,
                               g_speedBufferAutoTestCurrentSize,
                               g_dbgSpeedLeftDeltaCount,
                               g_dbgSpeedRightDeltaCount);
    CanSpeedTx_SendSpeedControlStatus(g_targetVehicleRpm,
                                      g_currentVehicleRpm,
                                      g_speedControlDutyCommand,
                                      g_speedControlFeedforwardDuty);
    CanSpeedTx_SendBufferTestStatus(g_speedBufferAutoTestCurrentSize,
                                    g_speedBufferAutoTestStageIndex,
                                    g_currentDriveDutyPercent,
                                    g_speedBufferAutoTestStatus,
                                    g_speedBufferAutoTestAvgVehicleRpm,
                                    g_speedBufferAutoTestElapsedMs);
    CanSpeedTx_SendEncoderCount(g_dbgSpeedLeftCount, g_dbgSpeedRightCount);
    ControlLoop_SendEncoderDebug();
}

static void ControlLoop_StartMaxSpeedTest(void)
{
    g_unitTestCaseCommand = UNIT_TEST_CASE_NONE;
    g_unitTestCaseActive = 1u;
    g_unitTestCaseDone = 0u;
    g_unitTestCaseId = UNIT_TEST_CASE_MAX_SPEED;
    g_unitTestCaseStatus = UNIT_TEST_STATUS_RUN;
    g_unitTestDoneHoldLoopCount = 0u;

    g_maxSpeedTestPhase = MAX_SPEED_TEST_PHASE_ZERO_DUTY;
    g_maxSpeedTestDutyPercent = MAX_SPEED_TEST_DUTY_PERCENT;
    g_maxSpeedTestElapsedMs = 0u;
    g_maxSpeedTestCurrentRpm = 0.0f;
    g_maxSpeedTestCurrentKph = 0.0f;
    g_maxSpeedTestAvgRpm = 0.0f;
    g_maxSpeedTestAvgKph = 0.0f;
    g_maxSpeedTestMaxRpm = 0.0f;
    g_maxSpeedTestMaxKph = 0.0f;
    g_maxSpeedTestSampleCount = 0u;
    g_maxSpeedTestPhaseLoopCount = 0u;
    g_maxSpeedTestRunLoopCount = 0u;
    g_maxSpeedTestRpmSum = 0.0f;
    g_maxSpeedTestKphSum = 0.0f;

    g_targetVehicleKph = 0.0f;
    g_targetVehicleMps = 0.0f;
    g_speedCalibrationFixedDutyEnable = 0u;
    ControlLoop_ResetSpeedPid();
    ControlLoop_ApplyDriveDuty(0u);
    SpeedCalc_SetAverageBufferSize(MAX_SPEED_TEST_BUFFER_SIZE, TRUE);
}

static void ControlLoop_FinishMaxSpeedTest(void)
{
    g_unitTestCaseActive = 0u;
    g_unitTestCaseDone = 1u;
    g_unitTestCaseStatus = UNIT_TEST_STATUS_DONE;
    g_maxSpeedTestPhase = MAX_SPEED_TEST_PHASE_DONE;
    g_unitTestDoneHoldLoopCount = UNIT_TEST_DONE_HOLD_MS / 5u;

    g_targetVehicleKph = 0.0f;
    g_targetVehicleMps = 0.0f;
    g_speedControlDutyCommand = 0.0f;
    g_driveDutyCommandFloat = 0.0f;
    ControlLoop_ResetSpeedPid();
    ControlLoop_ApplyDriveDuty(0u);
    SpeedCalc_SetAverageBufferSize(SPEED_BUFFER_FIXED_COUNT, TRUE);
}

static uint16 ControlLoop_GetMaxSpeedZeroDutyMs(void)
{
    return (g_maxSpeedTestZeroDutyMs == 0u) ?
           MAX_SPEED_TEST_ZERO_DUTY_MS :
           g_maxSpeedTestZeroDutyMs;
}

static uint16 ControlLoop_GetMaxSpeedRunMs(void)
{
    return (g_maxSpeedTestRunMs == 0u) ?
           MAX_SPEED_TEST_RUN_MS :
           g_maxSpeedTestRunMs;
}

static void ControlLoop_UpdateMaxSpeedTest(void)
{
    uint32 phaseElapsedMs;
    uint8 dutyPercent;

    if ((g_unitTestCaseCommand == UNIT_TEST_CASE_MAX_SPEED) &&
        (g_unitTestCaseActive == 0u))
    {
        ControlLoop_StartMaxSpeedTest();
    }

    if (g_unitTestCaseActive == 0u)
    {
        if (g_unitTestDoneHoldLoopCount > 0u)
        {
            g_unitTestDoneHoldLoopCount--;
            ControlLoop_ApplyDriveDuty(0u);
        }
        return;
    }

    if (g_unitTestCaseId != UNIT_TEST_CASE_MAX_SPEED)
    {
        return;
    }

    g_maxSpeedTestCurrentRpm = (g_lastSpeed.valid == TRUE) ? g_lastSpeed.vehicleRpm : 0.0f;
    g_maxSpeedTestCurrentKph = ControlLoop_RpmToKph(g_maxSpeedTestCurrentRpm);

    if (g_maxSpeedTestPhase == MAX_SPEED_TEST_PHASE_ZERO_DUTY)
    {
        g_speedControlFeedforwardDuty = 0.0f;
        g_speedControlDutyCommand = 0.0f;
        g_driveDutyCommandFloat = 0.0f;
        ControlLoop_ResetSpeedPid();
        ControlLoop_ApplyDriveDuty(0u);

        g_maxSpeedTestPhaseLoopCount++;
        phaseElapsedMs = g_maxSpeedTestPhaseLoopCount * 5u;
        g_maxSpeedTestElapsedMs = (phaseElapsedMs > 65535u) ? 65535u : (uint16)phaseElapsedMs;

        if (phaseElapsedMs >= (uint32)ControlLoop_GetMaxSpeedZeroDutyMs())
        {
            g_maxSpeedTestPhase = MAX_SPEED_TEST_PHASE_FULL_DUTY;
            g_maxSpeedTestPhaseLoopCount = 0u;
            SpeedCalc_SetAverageBufferSize(MAX_SPEED_TEST_BUFFER_SIZE, TRUE);
        }
        return;
    }

    dutyPercent = ControlLoop_ClampDutyPercent(g_maxSpeedTestDutyPercent);
    g_maxSpeedTestDutyPercent = dutyPercent;
    g_speedControlFeedforwardDuty = 0.0f;
    g_speedControlDutyCommand = (float32)dutyPercent;
    g_driveDutyCommandFloat = (float32)dutyPercent;
    ControlLoop_ResetSpeedPid();
    ControlLoop_ApplyDriveDuty(dutyPercent);

    if ((g_lastSpeed.valid == TRUE) && (g_maxSpeedTestCurrentRpm > 0.1f))
    {
        g_maxSpeedTestSampleCount++;
        g_maxSpeedTestRpmSum += g_maxSpeedTestCurrentRpm;
        g_maxSpeedTestKphSum += g_maxSpeedTestCurrentKph;
        g_maxSpeedTestAvgRpm = g_maxSpeedTestRpmSum / (float32)g_maxSpeedTestSampleCount;
        g_maxSpeedTestAvgKph = g_maxSpeedTestKphSum / (float32)g_maxSpeedTestSampleCount;

        if (g_maxSpeedTestCurrentRpm > g_maxSpeedTestMaxRpm)
        {
            g_maxSpeedTestMaxRpm = g_maxSpeedTestCurrentRpm;
            g_maxSpeedTestMaxKph = g_maxSpeedTestCurrentKph;
        }
    }

    g_maxSpeedTestPhaseLoopCount++;
    g_maxSpeedTestRunLoopCount++;
    phaseElapsedMs = g_maxSpeedTestPhaseLoopCount * 5u;
    g_maxSpeedTestElapsedMs =
        (ControlLoop_GetMaxSpeedZeroDutyMs() + phaseElapsedMs > 65535u) ?
        65535u :
        (uint16)(ControlLoop_GetMaxSpeedZeroDutyMs() + phaseElapsedMs);

    if (phaseElapsedMs >= (uint32)ControlLoop_GetMaxSpeedRunMs())
    {
        ControlLoop_FinishMaxSpeedTest();
    }
}

static uint8 ControlLoop_GetMaxSpeedElapsedSec(void)
{
    uint32 elapsedSec = ((uint32)g_maxSpeedTestElapsedMs) / 1000u;

    return (elapsedSec > 255u) ? 255u : (uint8)elapsedSec;
}

static void ControlLoop_SendMaxSpeedTestTelemetry(void)
{
    CanSpeedTx_SendWheelSpeed(&g_lastSpeed);
    CanSpeedTx_SendRpmDebug(&g_lastSpeed);
    CanSpeedTx_SendDriveStatus(g_currentDriveDutyPercent,
                               g_driveDirection,
                               g_estopActive,
                               g_dbgSpeedBufferCount,
                               g_dbgSpeedLeftDeltaCount,
                               g_dbgSpeedRightDeltaCount);
    CanSpeedTx_SendSpeedControlStatus(g_targetVehicleRpm,
                                      g_currentVehicleRpm,
                                      g_speedControlDutyCommand,
                                      g_speedControlFeedforwardDuty);
    CanSpeedTx_SendBufferTestStatus(0u, 0u, 0u, SPEED_BUFFER_AUTO_TEST_STATUS_IDLE, 0.0f, 0u);
    CanSpeedTx_SendMaxSpeedTestStatus(g_unitTestCaseId,
                                      g_unitTestCaseStatus,
                                      g_maxSpeedTestPhase,
                                      g_currentDriveDutyPercent,
                                      g_maxSpeedTestCurrentRpm,
                                      g_maxSpeedTestCurrentKph);
    CanSpeedTx_SendMaxSpeedResult(g_unitTestCaseId,
                                  g_unitTestCaseStatus,
                                  g_maxSpeedTestDutyPercent,
                                  ControlLoop_GetMaxSpeedElapsedSec(),
                                  g_maxSpeedTestAvgRpm,
                                  g_maxSpeedTestAvgKph);
    CanSpeedTx_SendEncoderCount(g_dbgSpeedLeftCount, g_dbgSpeedRightCount);
    ControlLoop_SendEncoderDebug();
}

static uint8 ControlLoop_GetDeadbandMaxDuty(void)
{
    uint8 maxDuty = ControlLoop_ClampDutyPercent(g_driveDeadbandTestMaxDutyPercent);

    g_driveDeadbandTestMaxDutyPercent = maxDuty;
    return maxDuty;
}

static void ControlLoop_StartDriveDeadbandTest(void)
{
    g_unitTestCaseCommand = UNIT_TEST_CASE_NONE;
    g_unitTestCaseActive = 1u;
    g_unitTestCaseDone = 0u;
    g_unitTestCaseId = UNIT_TEST_CASE_DRIVE_DEADBAND;
    g_unitTestCaseStatus = UNIT_TEST_STATUS_RUN;
    g_unitTestDoneHoldLoopCount = 0u;

    g_driveDeadbandTestCurrentDutyPercent = DRIVE_DEADBAND_TEST_START_DUTY_PERCENT;
    g_driveDeadbandTestLastStoppedDutyPercent = 0u;
    g_driveDeadbandTestFirstMovingDutyPercent = 0u;
    g_driveDeadbandTestElapsedMs = 0u;
    g_driveDeadbandTestMoveStableMs = 0u;
    g_driveDeadbandTestCurrentRpm = 0.0f;
    g_driveDeadbandTestCurrentKph = 0.0f;
    g_driveDeadbandTestFinalRpm = 0.0f;
    g_driveDeadbandTestFirstMovingRpm = 0.0f;
    g_driveDeadbandTestFirstMovingKph = 0.0f;
    g_driveDeadbandTestStageLoopCount = 0u;
    g_driveDeadbandTestMoveLoopCount = 0u;
    g_driveDeadbandTestTotalLoopCount = 0u;

    g_targetVehicleKph = 0.0f;
    g_targetVehicleMps = 0.0f;
    g_speedCalibrationFixedDutyEnable = 0u;
    ControlLoop_ResetSpeedPid();
    SpeedCalc_SetAverageBufferSize(DRIVE_DEADBAND_TEST_BUFFER_SIZE, TRUE);
}

static void ControlLoop_FinishDriveDeadbandTest(uint8 statusCode)
{
    g_unitTestCaseActive = 0u;
    g_unitTestCaseDone = 1u;
    g_unitTestCaseStatus = statusCode;
    g_driveDeadbandTestFinalRpm = g_driveDeadbandTestCurrentRpm;
    g_driveDeadbandTestCurrentKph = ControlLoop_RpmToKph(g_driveDeadbandTestCurrentRpm);
    g_unitTestDoneHoldLoopCount = UNIT_TEST_DONE_HOLD_MS / 5u;

    g_targetVehicleKph = 0.0f;
    g_targetVehicleMps = 0.0f;
    g_speedControlDutyCommand = 0.0f;
    g_driveDutyCommandFloat = 0.0f;
    ControlLoop_ResetSpeedPid();
    ControlLoop_ApplyDriveDuty(0u);
    SpeedCalc_SetAverageBufferSize(SPEED_BUFFER_FIXED_COUNT, TRUE);
}

static void ControlLoop_CancelUnitTest(void)
{
    g_unitTestCaseCommand = UNIT_TEST_CASE_NONE;
    g_unitTestCaseActive = 0u;
    g_unitTestCaseDone = 1u;
    g_unitTestCaseStatus = UNIT_TEST_STATUS_CANCELLED;
    g_maxSpeedTestPhase = MAX_SPEED_TEST_PHASE_DONE;
    g_unitTestDoneHoldLoopCount = UNIT_TEST_DONE_HOLD_MS / 5u;

    g_targetVehicleKph = 0.0f;
    g_targetVehicleMps = 0.0f;
    g_speedControlDutyCommand = 0.0f;
    g_driveDutyCommandFloat = 0.0f;
    ControlLoop_ResetSpeedPid();
    ControlLoop_ApplyDriveDuty(0u);
    SpeedCalc_SetAverageBufferSize(SPEED_BUFFER_FIXED_COUNT, TRUE);
}

static void ControlLoop_AdvanceDriveDeadbandDuty(void)
{
    uint8 maxDuty = ControlLoop_GetDeadbandMaxDuty();

    g_driveDeadbandTestLastStoppedDutyPercent = g_driveDeadbandTestCurrentDutyPercent;
    if (g_driveDeadbandTestCurrentDutyPercent >= maxDuty)
    {
        ControlLoop_FinishDriveDeadbandTest(UNIT_TEST_STATUS_NO_MOTION);
        return;
    }

    g_driveDeadbandTestCurrentDutyPercent =
        (uint8)(g_driveDeadbandTestCurrentDutyPercent + DRIVE_DEADBAND_TEST_STEP_DUTY_PERCENT);
    if (g_driveDeadbandTestCurrentDutyPercent > maxDuty)
    {
        g_driveDeadbandTestCurrentDutyPercent = maxDuty;
    }

    g_driveDeadbandTestStageLoopCount = 0u;
    g_driveDeadbandTestMoveLoopCount = 0u;
    g_driveDeadbandTestMoveStableMs = 0u;
    SpeedCalc_SetAverageBufferSize(DRIVE_DEADBAND_TEST_BUFFER_SIZE, TRUE);
}

static void ControlLoop_UpdateDriveDeadbandTest(void)
{
    uint16 stageMs;
    uint16 stableMs;
    uint32 stageElapsedMs;

    if ((g_unitTestCaseCommand == UNIT_TEST_CASE_DRIVE_DEADBAND) &&
        (g_unitTestCaseActive == 0u))
    {
        ControlLoop_StartDriveDeadbandTest();
    }

    if (g_unitTestCaseActive == 0u)
    {
        if (g_unitTestDoneHoldLoopCount > 0u)
        {
            g_unitTestDoneHoldLoopCount--;
            ControlLoop_ApplyDriveDuty(0u);
        }
        return;
    }

    if (g_unitTestCaseId != UNIT_TEST_CASE_DRIVE_DEADBAND)
    {
        return;
    }

    g_driveDeadbandTestCurrentRpm = (g_lastSpeed.valid == TRUE) ? g_lastSpeed.vehicleRpm : 0.0f;
    g_driveDeadbandTestCurrentKph = ControlLoop_RpmToKph(g_driveDeadbandTestCurrentRpm);

    g_speedControlFeedforwardDuty = 0.0f;
    g_speedControlDutyCommand = (float32)g_driveDeadbandTestCurrentDutyPercent;
    g_driveDutyCommandFloat = (float32)g_driveDeadbandTestCurrentDutyPercent;
    ControlLoop_ResetSpeedPid();
    ControlLoop_ApplyDriveDuty(g_driveDeadbandTestCurrentDutyPercent);

    if ((g_driveDeadbandTestCurrentDutyPercent > 0u) &&
        (g_driveDeadbandTestCurrentRpm >= g_driveDeadbandTestMoveThresholdRpm))
    {
        g_driveDeadbandTestMoveLoopCount++;
    }
    else
    {
        g_driveDeadbandTestMoveLoopCount = 0u;
    }

    stableMs = (g_driveDeadbandTestMoveLoopCount * 5u > 65535u) ?
               65535u :
               (uint16)(g_driveDeadbandTestMoveLoopCount * 5u);
    g_driveDeadbandTestMoveStableMs = stableMs;

    if (stableMs >= DRIVE_DEADBAND_TEST_MOVE_STABLE_MS)
    {
        g_driveDeadbandTestFirstMovingDutyPercent = g_driveDeadbandTestCurrentDutyPercent;
        g_driveDeadbandTestFirstMovingRpm = g_driveDeadbandTestCurrentRpm;
        g_driveDeadbandTestFirstMovingKph = g_driveDeadbandTestCurrentKph;
        ControlLoop_FinishDriveDeadbandTest(UNIT_TEST_STATUS_DONE);
        return;
    }

    stageMs = (g_driveDeadbandTestStageMs == 0u) ?
              DRIVE_DEADBAND_TEST_STAGE_MS :
              g_driveDeadbandTestStageMs;

    g_driveDeadbandTestStageLoopCount++;
    g_driveDeadbandTestTotalLoopCount++;
    stageElapsedMs = g_driveDeadbandTestStageLoopCount * 5u;
    g_driveDeadbandTestElapsedMs =
        (g_driveDeadbandTestTotalLoopCount * 5u > 65535u) ?
        65535u :
        (uint16)(g_driveDeadbandTestTotalLoopCount * 5u);

    if (stageElapsedMs >= (uint32)stageMs)
    {
        ControlLoop_AdvanceDriveDeadbandDuty();
    }
}

static uint8 ControlLoop_GetUnitTestElapsedSec(void)
{
    uint32 elapsedSec = ((uint32)g_driveDeadbandTestElapsedMs) / 1000u;

    return (elapsedSec > 255u) ? 255u : (uint8)elapsedSec;
}

static void ControlLoop_SendDriveDeadbandTestTelemetry(void)
{
    CanSpeedTx_SendWheelSpeed(&g_lastSpeed);
    CanSpeedTx_SendRpmDebug(&g_lastSpeed);
    CanSpeedTx_SendDriveStatus(g_currentDriveDutyPercent,
                               g_driveDirection,
                               g_estopActive,
                               g_dbgSpeedBufferCount,
                               g_dbgSpeedLeftDeltaCount,
                               g_dbgSpeedRightDeltaCount);
    CanSpeedTx_SendSpeedControlStatus(g_targetVehicleRpm,
                                      g_currentVehicleRpm,
                                      g_speedControlDutyCommand,
                                      g_speedControlFeedforwardDuty);
    CanSpeedTx_SendBufferTestStatus(0u, 0u, 0u, SPEED_BUFFER_AUTO_TEST_STATUS_IDLE, 0.0f, 0u);
    CanSpeedTx_SendDriveDeadbandTestStatus(g_unitTestCaseId,
                                           g_unitTestCaseStatus,
                                           g_driveDeadbandTestCurrentDutyPercent,
                                           g_driveDeadbandTestLastStoppedDutyPercent,
                                           g_driveDeadbandTestFirstMovingDutyPercent,
                                           g_driveDeadbandTestCurrentRpm,
                                           ControlLoop_GetUnitTestElapsedSec());
    CanSpeedTx_SendDriveDeadbandResult(g_unitTestCaseId,
                                       g_unitTestCaseStatus,
                                       g_driveDeadbandTestLastStoppedDutyPercent,
                                       g_driveDeadbandTestFirstMovingDutyPercent,
                                       g_driveDeadbandTestFirstMovingRpm,
                                       g_driveDeadbandTestFirstMovingKph);
    CanSpeedTx_SendEncoderCount(g_dbgSpeedLeftCount, g_dbgSpeedRightCount);
    ControlLoop_SendEncoderDebug();
}

static boolean ControlLoop_IsUnitTestRequestedOrActive(void)
{
    return ((g_unitTestCaseCommand == UNIT_TEST_CASE_MAX_SPEED) ||
            (g_unitTestCaseCommand == UNIT_TEST_CASE_DRIVE_DEADBAND) ||
            (g_unitTestCaseCommand == UNIT_TEST_CASE_CANCEL) ||
            (g_unitTestCaseActive != 0u) ||
            (g_unitTestDoneHoldLoopCount > 0u)) ? TRUE : FALSE;
}

static void ControlLoop_UpdateUnitTestCase(void)
{
    if (g_unitTestCaseCommand == UNIT_TEST_CASE_CANCEL)
    {
        ControlLoop_CancelUnitTest();
        return;
    }

    if ((g_unitTestCaseCommand == UNIT_TEST_CASE_MAX_SPEED) ||
        ((g_unitTestCaseCommand == UNIT_TEST_CASE_NONE) &&
         (g_unitTestCaseId == UNIT_TEST_CASE_MAX_SPEED)))
    {
        ControlLoop_UpdateMaxSpeedTest();
        return;
    }

    if ((g_unitTestCaseCommand == UNIT_TEST_CASE_DRIVE_DEADBAND) ||
        ((g_unitTestCaseCommand == UNIT_TEST_CASE_NONE) &&
         (g_unitTestCaseId == UNIT_TEST_CASE_DRIVE_DEADBAND)))
    {
        ControlLoop_UpdateDriveDeadbandTest();
        return;
    }

    if (g_unitTestDoneHoldLoopCount > 0u)
    {
        g_unitTestDoneHoldLoopCount--;
        ControlLoop_ApplyDriveDuty(0u);
    }
}

static void ControlLoop_SendUnitTestTelemetry(void)
{
    if (g_unitTestCaseId == UNIT_TEST_CASE_MAX_SPEED)
    {
        ControlLoop_SendMaxSpeedTestTelemetry();
    }
    else
    {
        ControlLoop_SendDriveDeadbandTestTelemetry();
    }
}

static float32 ControlLoop_GetJoystickTargetRpm(void)
{
    float32 targetKph = (float32)g_ps2AckermannTargetKphX10 / 10.0f;

    if (g_ps2AckermannPresent == 0u)
    {
        return 0.0f;
    }

    return ControlLoop_MpsToRpm(targetKph / 3.6f);
}

static void ControlLoop_SendJoystickCommandTelemetry(void)
{
    float32 targetRpm = ControlLoop_GetJoystickTargetRpm();

    g_joystickTargetRpm = targetRpm;
    CanSpeedTx_SendJoystickCommand(g_ps2AckermannPresent,
                                   g_ps2AckermannReceiverAlive,
                                   g_ps2AckermannButtons,
                                   targetRpm,
                                   g_ps2AckermannSteeringPulseUs);
}

static void ControlLoop_UpdateManualDutyTest(void)
{
    uint8 dutyPercent = ControlLoop_DutyFloatToPercent(g_manualDutyTestPercent);

    g_manualDutyTestPercent = ControlLoop_ClampFloat(g_manualDutyTestPercent, 0.0f, 100.0f);
    g_manualDutyTestCurrentRpm = (g_lastSpeed.valid == TRUE) ? g_lastSpeed.vehicleRpm : 0.0f;
    g_manualDutyTestCurrentKph = ControlLoop_RpmToKph(g_manualDutyTestCurrentRpm);

    g_unitTestCaseCommand = UNIT_TEST_CASE_NONE;
    g_unitTestCaseActive = 0u;
    g_unitTestDoneHoldLoopCount = 0u;
    g_speedBufferAutoTestStart = 0u;
    g_speedBufferAutoTestActive = 0u;
    g_speedCalibrationFixedDutyEnable = 0u;

    g_speedControlFeedforwardDuty = 0.0f;
    g_speedControlDutyCommand = (float32)dutyPercent;
    g_driveDutyCommandFloat = (float32)dutyPercent;
    ControlLoop_ResetSpeedPid();
    ControlLoop_ApplyDriveDuty(dutyPercent);
}

static void ControlLoop_SendManualDutyTestTelemetry(void)
{
    CanSpeedTx_SendWheelSpeed(&g_lastSpeed);
    CanSpeedTx_SendRpmDebug(&g_lastSpeed);
    CanSpeedTx_SendDriveStatus(g_currentDriveDutyPercent,
                               g_driveDirection,
                               g_estopActive,
                               g_dbgSpeedBufferCount,
                               g_dbgSpeedLeftDeltaCount,
                               g_dbgSpeedRightDeltaCount);
    CanSpeedTx_SendSpeedControlStatus(g_targetVehicleRpm,
                                      g_currentVehicleRpm,
                                      g_speedControlDutyCommand,
                                      g_speedControlFeedforwardDuty);
    CanSpeedTx_SendBufferTestStatus(0u, 0u, 0u, SPEED_BUFFER_AUTO_TEST_STATUS_IDLE, 0.0f, 0u);
    CanSpeedTx_SendEncoderCount(g_dbgSpeedLeftCount, g_dbgSpeedRightCount);
    ControlLoop_SendEncoderDebug();
}

static void ControlLoop_DisableTestModesForRemoteControl(void)
{
    boolean wasTestActive =
        ((g_manualDutyTestEnable != 0u) ||
         (g_unitTestCaseCommand != UNIT_TEST_CASE_NONE) ||
         (g_unitTestCaseActive != 0u) ||
         (g_unitTestDoneHoldLoopCount != 0u) ||
         (g_speedBufferAutoTestStart != 0u) ||
         (g_speedBufferAutoTestActive != 0u) ||
         (g_steeringCenterTuneEnabled != 0u) ||
         (g_steeringPwmOnlyTestEnabled != 0u) ||
         (g_speedCalibrationFixedDutyEnable != 0u)) ? TRUE : FALSE;

    if (wasTestActive != FALSE)
    {
        /* A remote owner must always start its duty ramp from zero. */
        ControlLoop_StopDriveNow();
    }

    g_manualDutyTestEnable = 0u;
    g_manualDutyTestPercent = 0.0f;
    g_unitTestCaseCommand = UNIT_TEST_CASE_NONE;
    g_unitTestCaseActive = 0u;
    g_unitTestCaseDone = 0u;
    g_unitTestCaseId = UNIT_TEST_CASE_NONE;
    g_unitTestCaseStatus = UNIT_TEST_STATUS_IDLE;
    g_unitTestDoneHoldLoopCount = 0u;
    g_speedBufferAutoTestStart = 0u;
    g_speedBufferAutoTestActive = 0u;
    g_speedBufferAutoTestDone = 0u;
    g_speedBufferAutoTestStatus = SPEED_BUFFER_AUTO_TEST_STATUS_IDLE;
    g_steeringCenterTuneEnabled = 0u;
    g_steeringPwmOnlyTestEnabled = 0u;
    g_speedCalibrationFixedDutyEnable = 0u;
    g_speedControlEnabled = 1u;
}

static boolean ControlLoop_IsLegacyTestRequestedOrActive(void)
{
    return
        ((g_manualDutyTestEnable != 0u) ||
         (g_unitTestCaseCommand != UNIT_TEST_CASE_NONE) ||
         (g_unitTestCaseActive != 0u) ||
         (g_unitTestDoneHoldLoopCount != 0u) ||
         (g_speedBufferAutoTestStart != 0u) ||
         (g_speedBufferAutoTestActive != 0u) ||
         (g_steeringCenterTuneEnabled != 0u) ||
         (g_steeringPwmOnlyTestEnabled != 0u) ||
         (g_speedCalibrationFixedDutyEnable != 0u)) ? TRUE : FALSE;
}

static sint16 ControlLoop_SteeringDegToX10(float32 steeringDeg)
{
    float32 scaled = steeringDeg * 10.0f;

    if (scaled >= 0.0f)
    {
        scaled += 0.5f;
    }
    else
    {
        scaled -= 0.5f;
    }

    return (sint16)scaled;
}

/* TRAJ_FD(CAN FD 0x320) 제어 파라미터. previewTimeS/게인은 실측 후 튜닝 필요 -
 * 지금은 PDF 4단계(Curvature FF+FB) 예시값 기준의 출발점일 뿐임. */
static const TrajFdControlParam g_trajFdControlParam =
{
    0.6f,           /* previewTimeS (T1) */
    0.3f,           /* previewDistMinM */
    3.0f,           /* previewDistMaxM */
    VM_WHEELBASE_M, /* wheelBaseM - VehicleManager.h의 실측값 재사용 */
    1.0f,           /* stanleyGainK (Stage3 대안용, Stage4에서는 안 씀) */
    0.6f,           /* curvatureFfKy */
    1.0f,           /* curvatureFfKpsi */
    0.10f           /* speedEpsilonMps */
};

/* 카메라/GPS는 이제 각자 독립된 Adas_Manager 슬롯(0x320/0x321)이므로 필터
 * 상태(eyFiltPrev)와 시퀀스 카운터도 채널별로 따로 가져야 한다 - 하나를
 * 공유하면 둘 사이를 오갈 때 필터가 서로의 상태를 오염시킨다. */
static float32 g_trajFdCameraEyFiltPrev = 0.0f;
static uint8   g_trajFdCameraSequence = 0u;
static float32 g_trajFdGpsEyFiltPrev = 0.0f;
static uint8   g_trajFdGpsSequence = 0u;

/* TC275에 헤딩(yaw) 센서가 없어서(코드베이스 전체 검색 결과 psi/heading/yaw
 * 소스 없음) vehicleHeadingRad는 0.0f로 고정 전달한다 - TrajFd_BuildAdasManagerSlotFrame
 * 호출 시 그대로 넘김. 상위가 보내는 c0..c3는 "그 프레임을 계산한 순간의
 * 차량 진행방향"을 x축으로 하는 차량 고정좌표계라고 전제한다
 * (TRAJ_FD_PROTOCOL.md 참고). 상위-하위 사이 CAN 전송 주기만큼의 지연은
 * 보정되지 않으니, 전송 주기를 짧게(20~50Hz) 유지할 것. */

static void ControlLoop_UpdateAutoCanInput(VehicleManagerInput *input,
                                           const AdasManagerOutput *adasOutput)
{
    uint8 source;
    uint8 rawSequence;
    uint32 rawValidCount;
    boolean rawCommandValid;
    boolean rawCommandEnabled;
    sint16 rawSpeedKphX100;
    sint16 rawSteeringDegX10;
    boolean snapshotChanged;

    if ((input == 0) || (adasOutput == 0))
    {
        return;
    }

    if (adasOutput->modeSeen != FALSE)
    {
        source = AUTO_CAN_SOURCE_ADAS_MANAGER;
        rawSequence = adasOutput->sequence;
        rawValidCount = adasOutput->validCount;
        rawCommandValid = adasOutput->commandValid;
        rawCommandEnabled = adasOutput->commandEnabled;
        rawSpeedKphX100 = adasOutput->speedKphX100;
        rawSteeringDegX10 = adasOutput->steeringDegX10;
    }
    else if (g_pcCommandModeSeen != 0u)
    {
        source = AUTO_CAN_SOURCE_LEGACY_PC;
        rawSequence = g_pcCommandSequence;
        rawValidCount = g_pcCommandValidCount;
        rawCommandValid = (g_pcCommandValid != 0u) ? TRUE : FALSE;
        rawCommandEnabled = (g_pcCommandEnabled != 0u) ? TRUE : FALSE;
        rawSpeedKphX100 = g_pcCommandSpeedKphX100;
        rawSteeringDegX10 = g_pcCommandSteeringDegX10;
    }
    else
    {
        input->canModeSeen = FALSE;
        input->canCommandValid = FALSE;
        input->canCommandEnabled = FALSE;
        input->canSequence = g_autoCanMuxSequence;
        input->canValidCount = g_autoCanMuxValidCount;
        input->canRequestedSpeedKph = 0.0f;
        input->canRequestedSteeringDeg = 0.0f;
        return;
    }

    snapshotChanged = ((source != g_autoCanSource) ||
                       (rawSequence != g_autoCanRawSequence) ||
                       (rawValidCount != g_autoCanRawValidCount) ||
                       (rawCommandValid != g_autoCanRawCommandValid) ||
                       (rawCommandEnabled != g_autoCanRawCommandEnabled) ||
                       (rawSpeedKphX100 != g_autoCanRawSpeedKphX100) ||
                       (rawSteeringDegX10 != g_autoCanRawSteeringDegX10)) ?
                      TRUE : FALSE;

    if (snapshotChanged != FALSE)
    {
        g_autoCanMuxSequence++;
        g_autoCanMuxValidCount++;
        g_autoCanSource = source;
        g_autoCanRawSequence = rawSequence;
        g_autoCanRawValidCount = rawValidCount;
        g_autoCanRawCommandValid = rawCommandValid;
        g_autoCanRawCommandEnabled = rawCommandEnabled;
        g_autoCanRawSpeedKphX100 = rawSpeedKphX100;
        g_autoCanRawSteeringDegX10 = rawSteeringDegX10;
    }

    input->canModeSeen = TRUE;
    input->canCommandValid = rawCommandValid;
    input->canCommandEnabled = rawCommandEnabled;
    input->canSequence = g_autoCanMuxSequence;
    input->canValidCount = g_autoCanMuxValidCount;
    input->canRequestedSpeedKph = (float32)rawSpeedKphX100 / 100.0f;
    input->canRequestedSteeringDeg = (float32)rawSteeringDegX10 / 10.0f;
}

static void ControlLoop_ApplyVehicleManager(SteeringAdcValue steering)
{
    VehicleManagerInput input;
    VehicleManagerOutput output;
    AdasManagerOutput adasOutput;
    GraphTestInput graphInput;
    GraphTestOutput graphOutput;
    boolean ps2Available;
    boolean speedFeedbackHealthy;
    boolean steeringFeedbackHealthy;
    boolean graphWasActive;
    boolean suppressActuatorsThisLoop;
    boolean keepSteeringCenterTune;
    uint8 graphStatusFlags;
    uint16 graphPhaseElapsedMs;
    uint16 targetRaw;
    uint8 trajDrainGuard;

    ps2Available = ((g_joystickDriveModeEnabled != 0u) &&
                    (Ps2Controller_IsPadConnected(JOYSTICK_PRIORITY_TIMEOUT_MS) != FALSE)) ?
                   TRUE : FALSE;

    speedFeedbackHealthy = ControlLoop_IsSpeedFeedbackHealthy();
    steeringFeedbackHealthy = ControlLoop_IsSteeringFeedbackHealthy(steering);
    g_speedFeedbackHealthy = (speedFeedbackHealthy != FALSE) ? 1u : 0u;
    g_steeringFeedbackHealthy = (steeringFeedbackHealthy != FALSE) ? 1u : 0u;

    input.estopActive = (g_estopActive != 0u) ? TRUE : FALSE;
    input.speedFeedbackValid = speedFeedbackHealthy;
    input.steeringFeedbackValid = steeringFeedbackHealthy;
    input.measuredSpeedKph = g_lastSpeed.vehicleMps * 3.6f;
    input.measuredSteeringDeg = steering.angleDeg;
    input.ps2Available = ps2Available;
    input.ps2RequestedSpeedKph = (float32)g_ps2AckermannTargetKphX10 / 10.0f;
    input.ps2RequestedSteeringDeg =
        ControlLoop_MapSteeringPotRawToAngle(g_ps2AckermannSteeringTargetRaw);
    /* TRAJ_FD 카메라(0x320)/GPS(0x321) 수신함 비우기 - 5ms 루프당 CAN FD
     * 프레임이 한 개 이상 밀려있을 가능성은 낮지만(20~50Hz 송신 vs 200Hz
     * 루프) 안전하게 채널당 몇 번까지만 드레인. */
    trajDrainGuard = 4u;
    while ((CanTrajectoryFd_PollChannel(&g_canTrajFdCamera, tick100us) != FALSE) && (trajDrainGuard > 0u))
    {
        trajDrainGuard--;
    }
    trajDrainGuard = 4u;
    while ((CanTrajectoryFd_PollChannel(&g_canTrajFdGps, tick100us) != FALSE) && (trajDrainGuard > 0u))
    {
        trajDrainGuard--;
    }

    /* v7: 카메라(0x320)/GPS(0x321)를 각자 별도 슬롯으로 Adas_Manager의 own
     * 6슬롯 우선순위(Mrm>Collision>Parking>Traffic>Camera>Gps)에 직접
     * 주입한다. 둘 중 무엇을 쓸지는 더 이상 여기서 고르지 않고 Adas_Manager
     * 쪽 freshness/priority 로직이 알아서 고른다 (AdasManager_SelectCameraOrGps).
     * AdasManager_Step5ms()가 이번 사이클을 처리하기 전에 먼저 주입해야
     * 하므로 그 호출보다 앞에 와야 함. */
    {
        uint8 cameraFrame[8];
        uint8 gpsFrame[8];

        TrajFd_BuildAdasManagerSlotFrame(&g_canTrajFdCamera, &g_trajFdControlParam,
                                         g_lastSpeed.vehicleMps, 0.0f, VM_PERIOD_SEC,
                                         tick100us, ADAS_MANAGER_CAN_ID_CAMERA,
                                         ADAS_MANAGER_FUNCTION_CAMERA,
                                         &g_trajFdCameraEyFiltPrev, &g_trajFdCameraSequence,
                                         cameraFrame);
        (void)AdasManager_OnCanFrame(ADAS_MANAGER_CAN_ID_CAMERA, cameraFrame, 8u,
                                     FALSE, tick100us);

        TrajFd_BuildAdasManagerSlotFrame(&g_canTrajFdGps, &g_trajFdControlParam,
                                         g_lastSpeed.vehicleMps, 0.0f, VM_PERIOD_SEC,
                                         tick100us, ADAS_MANAGER_CAN_ID_GPS,
                                         ADAS_MANAGER_FUNCTION_GPS,
                                         &g_trajFdGpsEyFiltPrev, &g_trajFdGpsSequence,
                                         gpsFrame);
        (void)AdasManager_OnCanFrame(ADAS_MANAGER_CAN_ID_GPS, gpsFrame, 8u,
                                     FALSE, tick100us);
    }

    AdasManager_Step5ms(tick100us);
    AdasManager_GetOutput(&adasOutput);

    /* A fresh ACTIVE 0x310..0x314 command commissions ADAS for this boot.
     * Until then legacy 0x300 remains usable.  The mux below owns the single
     * monotonic count/sequence domain expected by VehicleManager. */
    ControlLoop_UpdateAutoCanInput(&input, &adasOutput);

    graphInput.estopActive = input.estopActive;
    graphInput.speedFeedbackHealthy = speedFeedbackHealthy;
    graphInput.steeringFeedbackHealthy = steeringFeedbackHealthy;
    /* An explicit Expression START selects a dedicated verification state.
     * While it is active, GraphTest owns the request surface and normal PS2/CAN
     * sources are masked before the VehicleManager step below. */
    graphInput.sourceConflict = FALSE;
    graphInput.legacyTestActive = ControlLoop_IsLegacyTestRequestedOrActive();
    graphInput.canRearmRequired = FALSE;
    graphInput.safetyFaultMask =
        g_vehicleManagerFaultMask &
        (VM_FAULT_ESTOP_ACTIVE |
         VM_FAULT_STEERING_FEEDBACK |
         VM_FAULT_SPEED_FEEDBACK);
    graphInput.actualVehicleRpm = g_lastSpeed.vehicleRpm;
    graphInput.actualVehicleKph = g_lastSpeed.vehicleMps * 3.6f;
    graphInput.actualSteeringDeg = steering.angleDeg;

    graphWasActive = (g_graphTestActive != 0u) ? TRUE : FALSE;
    GraphTest_Step5ms(&graphInput, &graphOutput);

    /* A running graph test owns the service-test surface exclusively. */
    if ((graphWasActive != FALSE) || (graphOutput.active != FALSE))
    {
        ControlLoop_DisableTestModesForRemoteControl();
    }

    if ((graphWasActive != FALSE) &&
        (graphOutput.active == FALSE) &&
        (graphOutput.abortReason == (uint8)GraphTestAbort_SourceConflict) &&
        (input.canModeSeen != FALSE))
    {
        /* A CAN source appearing during a test must complete normal re-arm. */
        CanSpeedTx_InvalidatePcCommandForManualOverride();
        AdasManager_InvalidateForManualOverride();
        input.canCommandValid = FALSE;
        input.canCommandEnabled = FALSE;
    }

    input.serviceTestRequested =
        ((graphOutput.active != FALSE) &&
         (graphOutput.actuatorRequestValid != FALSE)) ? TRUE : FALSE;
    input.serviceValidationRequested =
        ((input.serviceTestRequested != FALSE) &&
         (graphOutput.mode == (uint8)GraphTestMode_VmValidation)) ? TRUE : FALSE;
    input.serviceRequestedSpeedKph =
        (input.serviceTestRequested != FALSE) ? graphOutput.requestedSpeedKph : 0.0f;
    input.serviceRequestedSteeringDeg =
        (input.serviceTestRequested != FALSE) ? graphOutput.requestedSteeringDeg : 0.0f;

    if (input.serviceTestRequested != FALSE)
    {
        /* GraphTest is an explicit debugger-selected VM state.  Suppress the
         * normal command sources for this step without changing their shadows. */
        input.ps2Available = FALSE;
        input.canModeSeen = FALSE;
        input.canCommandValid = FALSE;
        input.canCommandEnabled = FALSE;
    }

    VehicleManager_Step5ms(&input, &output);
    GraphTest_NotifyVehicleManager((uint8)output.owner,
                                   (uint8)output.state,
                                   output.faultMask,
                                   output.driveEnable);
    GraphTest_GetOutput(&graphOutput);

    suppressActuatorsThisLoop =
        ((graphWasActive != FALSE) &&
         (graphOutput.active == FALSE)) ? TRUE : FALSE;

    if (suppressActuatorsThisLoop != FALSE)
    {
        output.driveEnable = FALSE;
        output.steeringTargetValid = FALSE;
        output.targetSpeedKph = 0.0f;
        output.targetSteeringDeg = 0.0f;
    }

    CanSpeedTx_SendVehicleManagerTelemetry(&output);

    graphStatusFlags = 0u;
    if (graphOutput.active != FALSE)
    {
        graphStatusFlags |= ACKERMANN_CAN_GRAPH_TEST_ACTIVE_MASK;
    }
    if (output.owner == VehicleManagerOwner_ServiceTest)
    {
        graphStatusFlags |= ACKERMANN_CAN_GRAPH_TEST_SERVICE_OWNER_MASK;
    }
    if (speedFeedbackHealthy != FALSE)
    {
        graphStatusFlags |= ACKERMANN_CAN_GRAPH_TEST_SPEED_HEALTHY_MASK;
    }
    if (steeringFeedbackHealthy != FALSE)
    {
        graphStatusFlags |= ACKERMANN_CAN_GRAPH_TEST_STEERING_HEALTHY_MASK;
    }
    if (output.lateralLimitActive != FALSE)
    {
        graphStatusFlags |= ACKERMANN_CAN_GRAPH_TEST_LATERAL_LIMIT_MASK;
    }
    if (output.steeringLimitActive != FALSE)
    {
        graphStatusFlags |= ACKERMANN_CAN_GRAPH_TEST_STEERING_LIMIT_MASK;
    }
    if ((graphOutput.mode != g_graphTestTelemetryPreviousMode) ||
        (graphOutput.state != g_graphTestTelemetryPreviousState) ||
        (graphOutput.phase != g_graphTestTelemetryPreviousPhase))
    {
        graphStatusFlags |= ACKERMANN_CAN_GRAPH_TEST_PHASE_EVENT_MASK;
    }
    if (graphOutput.state == (uint8)GraphTestState_Complete)
    {
        graphStatusFlags |= ACKERMANN_CAN_GRAPH_TEST_COMPLETE_MASK;
    }

    graphPhaseElapsedMs =
        (graphOutput.phaseElapsedMs > 65535u) ? 65535u :
        (uint16)graphOutput.phaseElapsedMs;
    CanSpeedTx_SendGraphTestStatus(graphOutput.mode,
                                   graphOutput.state,
                                   graphOutput.phase,
                                   graphOutput.abortReason,
                                   graphPhaseElapsedMs,
                                   graphStatusFlags);
    g_graphTestTelemetryPreviousMode = graphOutput.mode;
    g_graphTestTelemetryPreviousState = graphOutput.state;
    g_graphTestTelemetryPreviousPhase = graphOutput.phase;

    if (output.invalidateCanShadow != FALSE)
    {
        CanSpeedTx_InvalidatePcCommandForManualOverride();
        AdasManager_InvalidateForManualOverride();
    }

    g_joystickPriorityActive = (ps2Available != FALSE) ? 1u : 0u;
    g_joystickDriveModeActive = g_joystickPriorityActive;
    if (g_joystickDriveModeEnabled == 0u)
    {
        g_joystickDriveModeBlockedReason = JOYSTICK_DRIVE_BLOCK_DISABLED;
    }
    else if (ps2Available == FALSE)
    {
        g_joystickDriveModeBlockedReason = JOYSTICK_DRIVE_BLOCK_NO_PAD;
    }
    else
    {
        g_joystickDriveModeBlockedReason = JOYSTICK_DRIVE_BLOCK_NONE;
    }

    g_pcCommandControlActive =
        (output.owner == VehicleManagerOwner_Can) ? 1u : 0u;
    g_pcCommandFailsafeActive =
        (((output.faultMask & (VM_FAULT_CAN_INVALID |
                              VM_FAULT_CAN_REARM_REQUIRED |
                              VM_FAULT_CAN_SEQUENCE)) != 0u) ||
         ((input.canModeSeen != FALSE) &&
          (output.state == VehicleManagerState_FaultTorqueOff))) ? 1u : 0u;

    /* Center calibration must remain usable while VM is Centering or CAN is
     * waiting for re-arm; otherwise the 5 ms loop immediately clears the very
     * flag needed to correct the center.  GraphTest/E-stop/invalid ADC still
     * take precedence, and the tune branch always forces traction to zero. */
    keepSteeringCenterTune =
        ((g_steeringCenterTuneEnabled != 0u) &&
         (graphWasActive == FALSE) &&
         (graphOutput.active == FALSE) &&
         (input.estopActive == FALSE) &&
         (steeringFeedbackHealthy != FALSE)) ? TRUE : FALSE;

    if ((output.allowServiceTests == FALSE) &&
        (keepSteeringCenterTune == FALSE))
    {
        ControlLoop_DisableTestModesForRemoteControl();
    }

    g_targetSteeringDeg = output.targetSteeringDeg;
    targetRaw = ControlLoop_MapSteeringDegX10ToRaw(
        ControlLoop_SteeringDegToX10(output.targetSteeringDeg));
    g_pcCommandAppliedSteeringRaw = targetRaw;

    if (output.steeringTargetValid != FALSE)
    {
        ControlLoop_SetSteeringPotTargetRaw(targetRaw, TRUE);
        ControlLoop_SetManualSteeringPulse(
            ControlLoop_MapSteeringPotRawToPulse(targetRaw), TRUE);
    }
    else
    {
        ControlLoop_SetSteeringPotTargetRaw(g_steeringPotRawCenter, FALSE);
        if (steering.valid != FALSE)
        {
            /* Hold the final centered corrective pulse without hunting. */
            ControlLoop_SetManualSteeringPulse(g_currentSteeringPulseUs, TRUE);
        }
        else
        {
            ControlLoop_SetManualSteeringPulse(STEERING_SERVO_CENTER_US, TRUE);
        }
    }

    if (output.driveEnable != FALSE)
    {
        g_speedControlEnabled = 1u;
        ControlLoop_SetTargetVehicleKph(output.targetSpeedKph);
    }
    else
    {
        ControlLoop_StopDriveNow();
        g_speedControlEnabled = 0u;
    }
}

void ControlLoop_Init(void)
{
    AdasManager_Init();
    VehicleManager_Init();
    GraphTest_Init();
    g_graphTestTelemetryPreviousMode = (uint8)GraphTestMode_None;
    g_graphTestTelemetryPreviousState = (uint8)GraphTestState_Idle;
    g_graphTestTelemetryPreviousPhase = 0u;
    g_autoCanSource = AUTO_CAN_SOURCE_NONE;
    g_autoCanRawSequence = 0u;
    g_autoCanRawValidCount = 0u;
    g_autoCanRawCommandValid = FALSE;
    g_autoCanRawCommandEnabled = FALSE;
    g_autoCanRawSpeedKphX100 = 0;
    g_autoCanRawSteeringDegX10 = 0;
    g_autoCanMuxSequence = 0u;
    g_autoCanMuxValidCount = 0u;
    g_dbgControlLoopInitCount++;
    g_targetSteeringDeg = 0.0f;
    g_limitedSteeringTargetDeg = 0.0f;
    g_currentSteeringDeg = 0.0f;
    g_dbgSteeringErrorDeg = 0.0f;
    g_dbgSteeringCorrectionUs = 0.0f;
    g_currentVehicleMps = 0.0f;
    g_targetVehicleKph = SPEED_TARGET_KMH;
    g_targetVehicleMps = SPEED_TARGET_MPS;
    g_currentVehicleRpm = 0.0f;
    g_targetVehicleRpm = ControlLoop_MpsToRpm(SPEED_TARGET_MPS);
    g_speedControlErrorMps = 0.0f;
    g_speedControlErrorRpm = 0.0f;
    g_speedControlFeedforwardDuty = ControlLoop_ComputeFeedforwardDuty(g_targetVehicleRpm);
    g_speedControlProportionalDuty = 0.0f;
    g_speedControlDutyCommand = 0.0f;
    g_motorAppliedDutyPct = 0.0f;
    g_speedControlIntegralDuty = 0.0f;
    g_speedControlDerivativeDuty = 0.0f;
    g_speedControlPidDuty = 0.0f;
    g_speedControlEnabled = 1u;
    g_speedBrakeActive = 0u;
    g_speedCalibrationFixedDutyEnable = SPEED_CALIBRATION_FIXED_DUTY_ENABLE;
    g_speedCalibrationFixedDutyPercent = SPEED_CALIBRATION_FIXED_DUTY_PERCENT;
    g_manualDutyTestEnable = 0u;
    g_manualDutyTestPercent = 0.0f;
    g_manualDutyTestCurrentRpm = 0.0f;
    g_manualDutyTestCurrentKph = 0.0f;
    g_joystickDriveModeEnabled = 1u;
    g_joystickDriveModeActive = 0u;
    g_joystickDriveModeBlockedReason = JOYSTICK_DRIVE_BLOCK_NO_PAD;
    g_joystickPriorityActive = 0u;
    g_pcCommandControlActive = 0u;
    g_pcCommandFailsafeActive = 0u;
    g_pcCommandAppliedSteeringRaw = STEERING_ADC_RAW_CENTER;
    g_speedBufferAutoTestStart = 0u;
    g_speedBufferAutoTestActive = 0u;
    g_speedBufferAutoTestDone = 0u;
    g_speedBufferAutoTestStatus = SPEED_BUFFER_AUTO_TEST_STATUS_IDLE;
    g_speedBufferAutoTestCurrentSize = SPEED_BUFFER_AUTO_TEST_START_SIZE;
    g_speedBufferAutoTestStageIndex = 0u;
    g_speedBufferAutoTestDutyPercent = SPEED_BUFFER_AUTO_TEST_DUTY_PERCENT;
    g_speedBufferAutoTestStageMs = SPEED_BUFFER_AUTO_TEST_STAGE_MS;
    g_speedBufferAutoTestElapsedMs = 0u;
    g_speedBufferAutoTestAvgVehicleRpm = 0.0f;
    g_speedBufferAutoTestHoldZeroCount = 0u;
    g_speedBufferAutoTestStageLoopCount = 0u;
    g_unitTestCaseCommand = UNIT_TEST_CASE_NONE;
    g_unitTestCaseActive = 0u;
    g_unitTestCaseDone = 0u;
    g_unitTestCaseId = UNIT_TEST_CASE_NONE;
    g_unitTestCaseStatus = UNIT_TEST_STATUS_IDLE;
    g_maxSpeedTestPhase = MAX_SPEED_TEST_PHASE_IDLE;
    g_maxSpeedTestDutyPercent = MAX_SPEED_TEST_DUTY_PERCENT;
    g_maxSpeedTestZeroDutyMs = MAX_SPEED_TEST_ZERO_DUTY_MS;
    g_maxSpeedTestRunMs = MAX_SPEED_TEST_RUN_MS;
    g_maxSpeedTestElapsedMs = 0u;
    g_maxSpeedTestCurrentRpm = 0.0f;
    g_maxSpeedTestCurrentKph = 0.0f;
    g_maxSpeedTestAvgRpm = 0.0f;
    g_maxSpeedTestAvgKph = 0.0f;
    g_maxSpeedTestMaxRpm = 0.0f;
    g_maxSpeedTestMaxKph = 0.0f;
    g_maxSpeedTestSampleCount = 0u;
    g_maxSpeedTestPhaseLoopCount = 0u;
    g_maxSpeedTestRunLoopCount = 0u;
    g_maxSpeedTestRpmSum = 0.0f;
    g_maxSpeedTestKphSum = 0.0f;
    g_driveDeadbandTestCurrentDutyPercent = 0u;
    g_driveDeadbandTestLastStoppedDutyPercent = 0u;
    g_driveDeadbandTestFirstMovingDutyPercent = 0u;
    g_driveDeadbandTestMaxDutyPercent = DRIVE_DEADBAND_TEST_END_DUTY_PERCENT;
    g_driveDeadbandTestStageMs = DRIVE_DEADBAND_TEST_STAGE_MS;
    g_driveDeadbandTestElapsedMs = 0u;
    g_driveDeadbandTestMoveStableMs = 0u;
    g_driveDeadbandTestMoveThresholdRpm = DRIVE_DEADBAND_TEST_MOVE_THRESHOLD_RPM;
    g_driveDeadbandTestCurrentRpm = 0.0f;
    g_driveDeadbandTestCurrentKph = 0.0f;
    g_driveDeadbandTestFinalRpm = 0.0f;
    g_driveDeadbandTestFirstMovingRpm = 0.0f;
    g_driveDeadbandTestFirstMovingKph = 0.0f;
    g_driveDeadbandTestStageLoopCount = 0u;
    g_driveDeadbandTestMoveLoopCount = 0u;
    g_driveDeadbandTestTotalLoopCount = 0u;
    g_unitTestDoneHoldLoopCount = 0u;
    g_currentSteeringPulseUs = STEERING_SERVO_NO_JOYSTICK_US;
    g_manualSteeringPulseUs = STEERING_SERVO_NO_JOYSTICK_US;
    g_manualSteeringPulseValid = TRUE;
    g_steeringPotPidEnabled = STEERING_POT_PID_ENABLE;
    /* Steering feedback control is enabled only by an actual owner. */
    g_steeringPotTargetValid = 0u;
    g_steeringPotTargetRaw = STEERING_ADC_RAW_CENTER;
    g_steeringPotErrorRaw = 0;
    g_steeringPotPidKpUsPerRaw = STEERING_POT_PID_KP_US_PER_RAW;
    g_steeringPotPidKiUsPerRawSec = STEERING_POT_PID_KI_US_PER_RAW_SEC;
    g_steeringPotPidKdUsPerRawPerSec = STEERING_POT_PID_KD_US_PER_RAW_PER_SEC;
    g_steeringPotPidIntegralUs = 0.0f;
    g_steeringPotPidCorrectionUs = 0.0f;
    g_steeringPotPidFeedforwardPulseUs = STEERING_SERVO_CENTER_US;
    g_steeringPotPidCommandPulseUs = STEERING_SERVO_CENTER_US;
    g_steeringPotPidDeadbandRaw = STEERING_POT_PID_DEADBAND_RAW;
    g_steeringPotPidMaxStepUs = STEERING_POT_PID_MAX_STEP_US;
    g_steeringPotPidAtTarget = 0u;
    g_steeringPotPidPreviousErrorRaw = 0;
    g_steeringPotPidHasPreviousError = FALSE;
    g_currentDriveDutyPercent = 0u;
    SpeedCalc_SetBufferScanDutyPercent(0u);
    g_driveForwardEnabled = 0u;
    g_driveReverseEnabled = 0u;
    g_driveDirection = DRIVE_DIRECTION_STOP;
    g_speedFeedbackHealthy = 0u;
    g_steeringFeedbackHealthy = 0u;
    g_driveDirectionInterlockActive = 0u;
    g_driveDirectionInterlockHoldLoops = 0u;
    g_encoderNoMotionLoopCount = 0u;
    g_steeringNoResponseLoopCount = 0u;
    g_ackermannState = (uint8)AckermannState_Init;
    g_steeringIntegralUs = 0.0f;
    g_driveDutyCommandFloat = 0.0f;
    g_speedControlPreviousErrorRpm = 0.0f;
    g_speedControlHasPreviousError = FALSE;
    g_encoderDebugPrevLeftCount = 0u;
    g_encoderDebugPrevRightCount = 0u;
    g_encoderDebugHasPrevious = FALSE;
    g_steeringPwmTestLoopCount = 0u;
    g_steeringPwmTestDirection = 0;
    g_steeringPwmTestPulseUs = STEERING_SERVO_CENTER_US;
    g_steeringPwmTestAtLimit = 0u;
    g_steeringCenterTuneEnabled = 0u;
    g_steeringCenterTunePulseUs = STEERING_SERVO_CENTER_US;
    g_steeringCenterTuneDirection = 0;
    g_steeringCenterTuneStepUs = 1u;
    g_steeringCenterTuneAtLimit = 0u;
#if (ESTOP_ENABLE != 0u)
    IfxPort_setPinModeInput(ESTOP_PORT, ESTOP_PIN, IfxPort_InputMode_pullUp);
#endif
#if (DRIVE_ONLY_TEST == 0u)
    SteeringPwm_SetPulseUs(STEERING_SERVO_NO_JOYSTICK_US);
#endif
    DrivePwm_Stop();
}

void ControlLoop_SetTargetVehicleKph(float32 targetKph)
{
    g_targetVehicleKph = targetKph;
    g_targetVehicleMps = targetKph / 3.6f;
}

void ControlLoop_SetManualSteeringPulse(uint16 pulseUs, boolean valid)
{
    g_manualSteeringPulseUs = ControlLoop_ClampSteeringPulseUs(pulseUs);
    g_manualSteeringPulseValid = valid;
    g_steeringIntegralUs = 0.0f;
}

void ControlLoop_SetSteeringPotTargetRaw(uint16 targetRaw, boolean valid)
{
    uint16 clampedRaw = ControlLoop_ClampSteeringPotTargetRaw(targetRaw);

    if ((g_steeringPotTargetValid == 0u) ||
        (g_steeringPotTargetRaw != clampedRaw) ||
        (valid == FALSE))
    {
        g_steeringPotPidIntegralUs = 0.0f;
        g_steeringPotPidHasPreviousError = FALSE;
    }

    g_steeringPotTargetRaw = clampedRaw;
    g_steeringPotTargetValid = (valid != FALSE) ? 1u : 0u;
}

void ControlLoop(void)
{
    SteeringAdcValue steering;
#if (DRIVE_ONLY_TEST == 0u)
    float32 targetSteeringDeg;
    uint16 pulseUs;
#endif

    (void)SpeedCalc_Get(&g_lastSpeed);
    if (g_lastSpeed.valid == TRUE)
    {
        g_currentVehicleMps = g_lastSpeed.vehicleMps;
        g_currentVehicleRpm = g_lastSpeed.vehicleRpm;
    }

    ControlLoop_UpdateEStopState();
#if (DRIVE_ONLY_TEST == 0u)
    (void)SteeringAdc_Update();
    steering = SteeringAdc_Get();
#else
    steering.raw = STEERING_ADC_RAW_CENTER;
    steering.millivolt = 0.0f;
    steering.angleDeg = 0.0f;
    steering.valid = TRUE;
#endif
    ControlLoop_ApplyVehicleManager(steering);

#if (DRIVE_ONLY_TEST == 0u)
    if ((g_steeringCenterTuneEnabled != 0u) ||
        ((g_steeringPwmOnlyTestEnabled != 0u) && (g_joystickPriorityActive == 0u)))
    {
        ControlLoop_ResetSpeedPid();
        g_speedControlEnabled = 0u;
        g_speedControlDutyCommand = 0.0f;
        g_driveDutyCommandFloat = 0.0f;
        ControlLoop_ApplyDriveDuty(0u);

        g_ackermannState = (uint8)AckermannState_Run;
        g_currentSteeringDeg = steering.angleDeg;
        g_limitedSteeringTargetDeg = 0.0f;
        g_dbgSteeringErrorDeg = 0.0f;
        if (g_steeringCenterTuneEnabled != 0u)
        {
            ControlLoop_UpdateSteeringCenterTune();
        }
        else
        {
            ControlLoop_UpdateSteeringPwmOnlyTest();
        }

        CanSpeedTx_SendSensorFeedback(&g_lastSpeed,
                                      steering.angleDeg,
                                      steering.raw,
                                      (steering.valid != FALSE) ? 1u : 0u,
                                      g_pcCommandControlActive,
                                      g_estopActive);
        CanSpeedTx_SendWheelSpeed(&g_lastSpeed);
        CanSpeedTx_SendRpmDebug(&g_lastSpeed);
        CanSpeedTx_SendDriveStatus(g_currentDriveDutyPercent,
                                   g_driveDirection,
                                   g_estopActive,
                                   g_dbgSpeedBufferCount,
                                   g_dbgSpeedLeftDeltaCount,
                                   g_dbgSpeedRightDeltaCount);
        CanSpeedTx_SendSpeedControlStatus(g_targetVehicleRpm,
                                          g_currentVehicleRpm,
                                          g_speedControlDutyCommand,
                                          g_speedControlFeedforwardDuty);
        ControlLoop_SendJoystickCommandTelemetry();
        CanSpeedTx_SendEncoderCount(g_dbgSpeedLeftCount, g_dbgSpeedRightCount);
        ControlLoop_SendEncoderDebug();
        CanSpeedTx_SendSteeringStatus(0.0f,
                                      steering.angleDeg,
                                      steering.raw,
                                      g_currentSteeringPulseUs,
                                      g_ackermannState);
        CanSpeedTx_SendSteeringFeedback(steering.angleDeg,
                                        steering.raw,
                                        steering.millivolt,
                                        0.0f);
        IfxPort_togglePin(LOOP_CHECK_PORT, LOOP_CHECK_PIN);
        return;
    }
#endif

    if (g_manualDutyTestEnable != 0u)
    {
        ControlLoop_UpdateManualDutyTest();
        g_ackermannState = (uint8)AckermannState_Run;
        ControlLoop_SendManualDutyTestTelemetry();
        IfxPort_togglePin(LOOP_CHECK_PORT, LOOP_CHECK_PIN);
        return;
    }

    if (ControlLoop_IsUnitTestRequestedOrActive() != FALSE)
    {
        ControlLoop_UpdateUnitTestCase();
        g_ackermannState = (uint8)AckermannState_Run;
        ControlLoop_SendUnitTestTelemetry();
        IfxPort_togglePin(LOOP_CHECK_PORT, LOOP_CHECK_PIN);
        return;
    }

    if ((g_speedBufferAutoTestStart != 0u) || (g_speedBufferAutoTestActive != 0u))
    {
        ControlLoop_UpdateSpeedBufferAutoTest();
        g_ackermannState = (uint8)AckermannState_Run;
        ControlLoop_SendSpeedBufferAutoTestTelemetry();
        IfxPort_togglePin(LOOP_CHECK_PORT, LOOP_CHECK_PIN);
        return;
    }

    ControlLoop_UpdateDriveSpeedControl(g_lastSpeed.valid, g_currentVehicleMps, g_currentVehicleRpm);

#if (DRIVE_ONLY_TEST != 0u)
    g_ackermannState = (uint8)AckermannState_Run;
    CanSpeedTx_SendSensorFeedback(&g_lastSpeed,
                                  0.0f,
                                  0u,
                                  0u,
                                  g_pcCommandControlActive,
                                  g_estopActive);
    CanSpeedTx_SendWheelSpeed(&g_lastSpeed);
    CanSpeedTx_SendRpmDebug(&g_lastSpeed);
    CanSpeedTx_SendDriveStatus(g_currentDriveDutyPercent,
                               g_driveDirection,
                               g_estopActive,
                               g_dbgSpeedBufferCount,
                               g_dbgSpeedLeftDeltaCount,
                               g_dbgSpeedRightDeltaCount);
    CanSpeedTx_SendSpeedControlStatus(g_targetVehicleRpm,
                                      g_currentVehicleRpm,
                                      g_speedControlDutyCommand,
                                      g_speedControlFeedforwardDuty);
    ControlLoop_SendJoystickCommandTelemetry();
    CanSpeedTx_SendEncoderCount(g_dbgSpeedLeftCount, g_dbgSpeedRightCount);
    ControlLoop_SendEncoderDebug();
    IfxPort_togglePin(LOOP_CHECK_PORT, LOOP_CHECK_PIN);
#else
    targetSteeringDeg = ControlLoop_GetClampedSteeringTarget();

    if (steering.valid == FALSE)
    {
        g_steeringPotPidAtTarget = 0u;
        g_steeringPotPidIntegralUs = 0.0f;
        g_steeringPotPidHasPreviousError = FALSE;
        pulseUs = (g_manualSteeringPulseValid != FALSE) ?
                  ControlLoop_ClampSteeringPulseUs(g_manualSteeringPulseUs) :
                  STEERING_SERVO_CENTER_US;
        g_ackermannState = (uint8)AckermannState_AdcMissing;
        SteeringPwm_SetPulseUs(pulseUs);
        g_currentSteeringDeg = 0.0f;
        g_currentSteeringPulseUs = pulseUs;

        CanSpeedTx_SendSensorFeedback(&g_lastSpeed,
                                      0.0f,
                                      0u,
                                      0u,
                                      g_pcCommandControlActive,
                                      g_estopActive);
        CanSpeedTx_SendWheelSpeed(&g_lastSpeed);
        CanSpeedTx_SendRpmDebug(&g_lastSpeed);
        CanSpeedTx_SendDriveStatus(g_currentDriveDutyPercent,
                                   g_driveDirection,
                                   g_estopActive,
                                   g_dbgSpeedBufferCount,
                                   g_dbgSpeedLeftDeltaCount,
                                   g_dbgSpeedRightDeltaCount);
        CanSpeedTx_SendSpeedControlStatus(g_targetVehicleRpm,
                                          g_currentVehicleRpm,
                                          g_speedControlDutyCommand,
                                          g_speedControlFeedforwardDuty);
        ControlLoop_SendJoystickCommandTelemetry();
        CanSpeedTx_SendEncoderCount(g_dbgSpeedLeftCount, g_dbgSpeedRightCount);
        ControlLoop_SendEncoderDebug();
        CanSpeedTx_SendSteeringStatus(targetSteeringDeg,
                                      0.0f,
                                      0u,
                                      pulseUs,
                                      g_ackermannState);
        CanSpeedTx_SendSteeringFeedback(0.0f,
                                        0u,
                                        0.0f,
                                        0.0f);
        IfxPort_togglePin(LOOP_CHECK_PORT, LOOP_CHECK_PIN);
        return;
    }

    g_ackermannState = (uint8)AckermannState_Run;
    g_currentSteeringDeg = steering.angleDeg;
    if ((g_steeringPotPidEnabled != 0u) && (g_steeringPotTargetValid != 0u))
    {
        pulseUs = ControlLoop_ComputeSteeringPotPidPulse(steering);
        targetSteeringDeg = g_limitedSteeringTargetDeg;
    }
    else
    {
        pulseUs = (g_manualSteeringPulseValid != FALSE) ?
                  ControlLoop_ClampSteeringPulseUs(g_manualSteeringPulseUs) :
                  ControlLoop_ComputeSteeringPulse(targetSteeringDeg, steering.angleDeg);
    }
    SteeringPwm_SetPulseUs(pulseUs);
    g_currentSteeringPulseUs = pulseUs;

    CanSpeedTx_SendSensorFeedback(&g_lastSpeed,
                                  steering.angleDeg,
                                  steering.raw,
                                  (steering.valid != FALSE) ? 1u : 0u,
                                  g_pcCommandControlActive,
                                  g_estopActive);
    CanSpeedTx_SendWheelSpeed(&g_lastSpeed);
    CanSpeedTx_SendRpmDebug(&g_lastSpeed);
    CanSpeedTx_SendDriveStatus(g_currentDriveDutyPercent,
                               g_driveDirection,
                               g_estopActive,
                               g_dbgSpeedBufferCount,
                               g_dbgSpeedLeftDeltaCount,
                               g_dbgSpeedRightDeltaCount);
    CanSpeedTx_SendSpeedControlStatus(g_targetVehicleRpm,
                                      g_currentVehicleRpm,
                                      g_speedControlDutyCommand,
                                      g_speedControlFeedforwardDuty);
    ControlLoop_SendJoystickCommandTelemetry();
    CanSpeedTx_SendEncoderCount(g_dbgSpeedLeftCount, g_dbgSpeedRightCount);
    ControlLoop_SendEncoderDebug();
    CanSpeedTx_SendSteeringStatus(targetSteeringDeg,
                                  steering.angleDeg,
                                  steering.raw,
                                  pulseUs,
                                  g_ackermannState);
    CanSpeedTx_SendSteeringFeedback(steering.angleDeg,
                                    steering.raw,
                                    steering.millivolt,
                                    g_dbgSteeringErrorDeg);
    IfxPort_togglePin(LOOP_CHECK_PORT, LOOP_CHECK_PIN);
#endif
}

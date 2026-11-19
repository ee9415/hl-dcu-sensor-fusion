#ifndef CONTROL_LOOP_H_
#define CONTROL_LOOP_H_

#include "Ifx_Types.h"

typedef enum
{
    AckermannState_Init = 0,
    AckermannState_Run = 1,
    AckermannState_AdcMissing = 2
} AckermannState;

extern volatile float32 g_targetSteeringDeg;
extern volatile float32 g_limitedSteeringTargetDeg;
extern volatile float32 g_currentSteeringDeg;
extern volatile float32 g_dbgSteeringErrorDeg;
extern volatile float32 g_dbgSteeringCorrectionUs;
extern volatile float32 g_currentVehicleMps;
extern volatile float32 g_targetVehicleKph;
extern volatile float32 g_targetVehicleMps;
extern volatile float32 g_currentVehicleRpm;
extern volatile float32 g_targetVehicleRpm;
extern volatile float32 g_speedControlErrorMps;
extern volatile float32 g_speedControlErrorRpm;
extern volatile float32 g_speedControlFeedforwardDuty;
extern volatile float32 g_speedControlProportionalDuty;
extern volatile float32 g_speedControlDutyCommand;
extern volatile float32 g_motorAppliedDutyPct;
extern volatile float32 g_speedControlIntegralDuty;
extern volatile float32 g_speedControlDerivativeDuty;
extern volatile float32 g_speedControlPidDuty;
extern volatile uint8   g_speedControlEnabled;
extern volatile uint8   g_speedBrakeActive;
extern volatile uint8   g_speedCalibrationFixedDutyEnable;
extern volatile float32 g_speedCalibrationFixedDutyPercent;
extern volatile uint8   g_manualDutyTestEnable;
extern volatile float32 g_manualDutyTestPercent;
extern volatile float32 g_manualDutyTestCurrentRpm;
extern volatile float32 g_manualDutyTestCurrentKph;
extern volatile uint8   g_joystickDriveModeEnabled;
extern volatile uint8   g_joystickDriveModeActive;
extern volatile uint8   g_joystickDriveModeBlockedReason;
extern volatile uint8   g_joystickPriorityActive;
extern volatile float32 g_joystickTargetRpm;
extern volatile uint8   g_pcCommandControlActive;
extern volatile uint8   g_pcCommandFailsafeActive;
extern volatile uint16  g_pcCommandAppliedSteeringRaw;
extern volatile uint8   g_speedBufferAutoTestStart;
extern volatile uint8   g_speedBufferAutoTestActive;
extern volatile uint8   g_speedBufferAutoTestDone;
extern volatile uint8   g_speedBufferAutoTestStatus;
extern volatile uint8   g_speedBufferAutoTestCurrentSize;
extern volatile uint8   g_speedBufferAutoTestStageIndex;
extern volatile uint8   g_speedBufferAutoTestDutyPercent;
extern volatile uint16  g_speedBufferAutoTestStageMs;
extern volatile uint16  g_speedBufferAutoTestElapsedMs;
extern volatile float32 g_speedBufferAutoTestAvgVehicleRpm;
extern volatile uint32  g_speedBufferAutoTestHoldZeroCount;
extern volatile uint8   g_unitTestCaseCommand;
extern volatile uint8   g_unitTestCaseActive;
extern volatile uint8   g_unitTestCaseDone;
extern volatile uint8   g_unitTestCaseId;
extern volatile uint8   g_unitTestCaseStatus;
extern volatile uint8   g_maxSpeedTestPhase;
extern volatile uint8   g_maxSpeedTestDutyPercent;
extern volatile uint16  g_maxSpeedTestZeroDutyMs;
extern volatile uint16  g_maxSpeedTestRunMs;
extern volatile uint16  g_maxSpeedTestElapsedMs;
extern volatile float32 g_maxSpeedTestCurrentRpm;
extern volatile float32 g_maxSpeedTestCurrentKph;
extern volatile float32 g_maxSpeedTestAvgRpm;
extern volatile float32 g_maxSpeedTestAvgKph;
extern volatile float32 g_maxSpeedTestMaxRpm;
extern volatile float32 g_maxSpeedTestMaxKph;
extern volatile uint32  g_maxSpeedTestSampleCount;
extern volatile uint8   g_driveDeadbandTestCurrentDutyPercent;
extern volatile uint8   g_driveDeadbandTestLastStoppedDutyPercent;
extern volatile uint8   g_driveDeadbandTestFirstMovingDutyPercent;
extern volatile uint8   g_driveDeadbandTestMaxDutyPercent;
extern volatile uint16  g_driveDeadbandTestStageMs;
extern volatile uint16  g_driveDeadbandTestElapsedMs;
extern volatile uint16  g_driveDeadbandTestMoveStableMs;
extern volatile float32 g_driveDeadbandTestMoveThresholdRpm;
extern volatile float32 g_driveDeadbandTestCurrentRpm;
extern volatile float32 g_driveDeadbandTestCurrentKph;
extern volatile float32 g_driveDeadbandTestFinalRpm;
extern volatile float32 g_driveDeadbandTestFirstMovingRpm;
extern volatile float32 g_driveDeadbandTestFirstMovingKph;
extern volatile uint16  g_currentSteeringPulseUs;
extern volatile uint16  g_manualSteeringPulseUs;
extern volatile boolean g_manualSteeringPulseValid;
extern volatile uint8   g_steeringPotPidEnabled;
extern volatile uint8   g_steeringPotTargetValid;
extern volatile uint16  g_steeringPotTargetRaw;
extern volatile sint32  g_steeringPotErrorRaw;
extern volatile float32 g_steeringPotPidKpUsPerRaw;
extern volatile float32 g_steeringPotPidKiUsPerRawSec;
extern volatile float32 g_steeringPotPidKdUsPerRawPerSec;
extern volatile float32 g_steeringPotPidIntegralUs;
extern volatile float32 g_steeringPotPidCorrectionUs;
extern volatile uint16  g_steeringPotPidFeedforwardPulseUs;
extern volatile uint16  g_steeringPotPidCommandPulseUs;
extern volatile uint16  g_steeringPotPidDeadbandRaw;
extern volatile uint16  g_steeringPotPidMaxStepUs;
extern volatile uint8   g_steeringPotPidAtTarget;
extern volatile uint8   g_currentDriveDutyPercent;
extern volatile uint8   g_driveForwardEnabled;
extern volatile uint8   g_driveReverseEnabled;
extern volatile uint8   g_driveDirection;
extern volatile uint8   g_ackermannState;
extern volatile uint8   g_estopPinHigh;
extern volatile uint8   g_estopActive;
extern volatile uint8   g_speedFeedbackHealthy;
extern volatile uint8   g_steeringFeedbackHealthy;
extern volatile uint8   g_driveDirectionInterlockActive;
extern volatile uint16  g_driveDirectionInterlockHoldLoops;
extern volatile uint32  g_encoderNoMotionLoopCount;
extern volatile uint32  g_steeringNoResponseLoopCount;
extern volatile uint32  g_dbgControlLoopInitCount;
extern volatile uint8   g_steeringPwmOnlyTestEnabled;
extern volatile sint32  g_steeringPwmTestDirection;
extern volatile uint16  g_steeringPwmTestPulseUs;
extern volatile uint8   g_steeringPwmTestAtLimit;
extern volatile uint8   g_steeringCenterTuneEnabled;
extern volatile uint16  g_steeringCenterTunePulseUs;
extern volatile sint32  g_steeringCenterTuneDirection;
extern volatile uint16  g_steeringCenterTuneStepUs;
extern volatile uint8   g_steeringCenterTuneAtLimit;

void ControlLoop_Init(void);
void ControlLoop(void);
void ControlLoop_StopDriveNow(void);
void ControlLoop_SetTargetVehicleKph(float32 targetKph);
void ControlLoop_SetManualSteeringPulse(uint16 pulseUs, boolean valid);
void ControlLoop_SetSteeringPotTargetRaw(uint16 targetRaw, boolean valid);

#endif /* CONTROL_LOOP_H_ */

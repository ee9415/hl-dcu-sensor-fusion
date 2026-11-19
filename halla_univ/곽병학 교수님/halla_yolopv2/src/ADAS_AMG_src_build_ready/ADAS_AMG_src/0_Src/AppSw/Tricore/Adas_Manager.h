#ifndef ADAS_MANAGER_H_
#define ADAS_MANAGER_H_

#include "Ifx_Types.h"

/* One standard-CAN identifier is owned by each ADAS application. */
#define ADAS_MANAGER_CAN_ID_MRM         (0x310u)
#define ADAS_MANAGER_CAN_ID_COLLISION   (0x311u)
#define ADAS_MANAGER_CAN_ID_PARKING     (0x312u)
#define ADAS_MANAGER_CAN_ID_TRAFFIC     (0x313u)
#define ADAS_MANAGER_CAN_ID_CAMERA      (0x320u)
#define ADAS_MANAGER_CAN_ID_GPS         (0x321u)
#define ADAS_MANAGER_CAN_DLC            (8u)

/* Packet-safety limits are intentionally aligned with the CAN_FIXED
 * 0x300 command path that has been verified on the vehicle. */
#define ADAS_MANAGER_MIN_SPEED_X100     (-70)
#define ADAS_MANAGER_MAX_SPEED_X100     (1300)
#define ADAS_MANAGER_MAX_STEERING_X10   (270)

/* CRC-8/SAE-J1850 over CAN-ID high, CAN-ID low, then B0..B6. */
#define ADAS_MANAGER_CRC8_POLY          (0x1Du)
#define ADAS_MANAGER_CRC8_INIT          (0xFFu)
#define ADAS_MANAGER_CRC8_XOROUT        (0xFFu)

/* Common byte-0 layout. */
#define ADAS_MANAGER_HEADER_VERSION_MASK       (0x80u)
#define ADAS_MANAGER_HEADER_ENABLE_MASK        (0x40u)
#define ADAS_MANAGER_HEADER_ATOMIC_MASK        (0x20u)
#define ADAS_MANAGER_HEADER_LATERAL_VALID_MASK (0x10u)
#define ADAS_MANAGER_HEADER_LONG_VALID_MASK    (0x08u)
#define ADAS_MANAGER_HEADER_FUNCTION_MASK      (0x07u)

/* FunctionId is deliberately independent of CAN-ID arbitration priority.
 * Camera keeps the legacy Nominal functionId (1) so it inherits that role's
 * validation rules; GPS is the newly added function. */
#define ADAS_MANAGER_FUNCTION_CAMERA     (1u)
#define ADAS_MANAGER_FUNCTION_TRAFFIC    (2u)
#define ADAS_MANAGER_FUNCTION_COLLISION  (3u)
#define ADAS_MANAGER_FUNCTION_PARKING    (4u)
#define ADAS_MANAGER_FUNCTION_MRM        (5u)
#define ADAS_MANAGER_FUNCTION_GPS        (6u)

/* During integration, the PPT v0 headers (bit7=0) remain accepted. Both v0
 * and v1 require the CRC-8 profile defined in Adas_Manager.c. */
#define ADAS_MANAGER_ALLOW_LEGACY_V0     (1u)

/* Byte-6 upper three bits. */
typedef enum
{
    AdasManagerRequest_Inactive = 0,
    AdasManagerRequest_Ready = 1,
    AdasManagerRequest_ActiveGood = 2,
    AdasManagerRequest_ActiveDegraded = 3,
    AdasManagerRequest_Completed = 4,
    AdasManagerRequest_Aborted = 5,
    AdasManagerRequest_Fault = 6,
    AdasManagerRequest_Invalid = 7
} AdasManagerRequestStatus;

typedef enum
{
    AdasManagerSegment_Straight = 0,
    AdasManagerSegment_StaticObstacle = 1,
    AdasManagerSegment_Hill = 2,
    AdasManagerSegment_DynamicObstacleAcceleration = 3,
    AdasManagerSegment_Parking = 4,
    AdasManagerSegment_Intersection = 5,
    AdasManagerSegment_SelectedLane = 6,
    AdasManagerSegment_Invalid = 7
} AdasManagerSegment;

typedef enum
{
    AdasManagerSource_None = 0,
    AdasManagerSource_Camera = ADAS_MANAGER_FUNCTION_CAMERA,
    AdasManagerSource_Traffic = ADAS_MANAGER_FUNCTION_TRAFFIC,
    AdasManagerSource_Collision = ADAS_MANAGER_FUNCTION_COLLISION,
    AdasManagerSource_Parking = ADAS_MANAGER_FUNCTION_PARKING,
    AdasManagerSource_Mrm = ADAS_MANAGER_FUNCTION_MRM,
    AdasManagerSource_Gps = ADAS_MANAGER_FUNCTION_GPS
} AdasManagerSource;

typedef enum
{
    AdasManagerSlot_NeverReceived = 0,
    AdasManagerSlot_Invalid = 1,
    AdasManagerSlot_Inactive = 2,
    AdasManagerSlot_Active = 3,
    AdasManagerSlot_Stale = 4
} AdasManagerSlotState;

/* Last packet rejection reason per application.  These values are for
 * debugger/telemetry use; they never take part in arbitration. */
typedef enum
{
    AdasManagerReject_None = 0,
    AdasManagerReject_Dlc = 1,
    AdasManagerReject_MessageLost = 2,
    AdasManagerReject_Crc = 3,
    AdasManagerReject_Header = 4,
    AdasManagerReject_Range = 5,
    AdasManagerReject_Sequence = 6
} AdasManagerRejectReason;

typedef struct
{
    boolean modeSeen;
    boolean commandValid;
    boolean commandEnabled;
    boolean degraded;
    uint8 sequence;
    uint32 validCount;
    sint16 speedKphX100;
    sint16 steeringDegX10;
    uint8 selectedSource;
    uint8 longitudinalOwner;
    uint8 lateralOwner;
    uint8 appContext;
    uint8 segment;
    uint8 parkingType;
    uint8 trafficSignal;
    uint8 selectedLane;
} AdasManagerOutput;

void AdasManager_Init(void);
boolean AdasManager_OnCanFrame(uint16 canId,
                               const uint8 data[ADAS_MANAGER_CAN_DLC],
                               uint8 dlc,
                               boolean messageLost,
                               uint32 nowTick100us);
void AdasManager_Step5ms(uint32 nowTick100us);
void AdasManager_GetOutput(AdasManagerOutput *output);
void AdasManager_InvalidateForManualOverride(void);

/* Debugger-visible state. Slot order is MRM, Collision, Parking, Traffic,
 * Camera, Gps. A never-received application remains INVALID diagnostically
 * but does not block a received valid application. */
extern volatile uint8  g_adasManagerModeSeen;
extern volatile uint8  g_adasManagerOutputValid;
extern volatile uint8  g_adasManagerOutputEnabled;
extern volatile uint8  g_adasManagerSelectedSource;
extern volatile uint8  g_adasManagerLongitudinalOwner;
extern volatile uint8  g_adasManagerLateralOwner;
extern volatile uint8  g_adasManagerAppContext;
extern volatile uint8  g_adasManagerSegment;
extern volatile sint16 g_adasManagerSpeedKphX100;
extern volatile sint16 g_adasManagerSteeringDegX10;
extern volatile uint8  g_adasManagerAggregateSequence;
extern volatile uint32 g_adasManagerAggregateValidCount;
extern volatile uint8  g_adasManagerReceivedMask;
extern volatile uint8  g_adasManagerValidatedMask;
extern volatile uint8  g_adasManagerFreshMask;
extern volatile uint8  g_adasManagerActiveMask;
extern volatile uint8  g_adasManagerManualRearmBlocked;
extern volatile uint32 g_adasManagerManualBlockCount;
extern volatile uint8  g_adasManagerSlotState[6];
extern volatile uint32 g_adasManagerSlotAgeMs[6];
extern volatile uint32 g_adasManagerSlotValidCount[6];
extern volatile uint32 g_adasManagerSlotInvalidCount[6];
extern volatile uint32 g_adasManagerSlotDuplicateCount[6];
extern volatile uint32 g_adasManagerSlotSequenceGapCount[6];
extern volatile uint32 g_adasManagerSlotMessageLostCount[6];
extern volatile uint8  g_adasManagerSlotLastRejectReason[6];
extern volatile uint32 g_adasManagerPacketRejectCount;
extern volatile uint32 g_adasManagerCrcErrorCount;
extern volatile uint32 g_adasManagerHeaderErrorCount;
extern volatile uint32 g_adasManagerRangeErrorCount;
extern volatile uint32 g_adasManagerSequenceErrorCount;

#endif /* ADAS_MANAGER_H_ */

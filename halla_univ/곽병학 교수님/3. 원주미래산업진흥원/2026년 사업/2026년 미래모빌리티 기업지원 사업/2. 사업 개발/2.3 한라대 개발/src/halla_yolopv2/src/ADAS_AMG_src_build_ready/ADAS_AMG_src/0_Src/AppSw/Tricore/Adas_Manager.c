#include "Adas_Manager.h"

#define ADAS_MANAGER_SLOT_COUNT                 (6u)
#define ADAS_MANAGER_SLOT_MRM                   (0u)
#define ADAS_MANAGER_SLOT_COLLISION             (1u)
#define ADAS_MANAGER_SLOT_PARKING               (2u)
#define ADAS_MANAGER_SLOT_TRAFFIC               (3u)
#define ADAS_MANAGER_SLOT_CAMERA                (4u)
#define ADAS_MANAGER_SLOT_GPS                   (5u)

#define ADAS_MANAGER_STATUS_SHIFT               (5u)
#define ADAS_MANAGER_STATUS_MASK                (0xE0u)
#define ADAS_MANAGER_SEQUENCE_MASK              (0x1Fu)
#define ADAS_MANAGER_CONTEXT_SEGMENT_MASK       (0x07u)
#define ADAS_MANAGER_CONTEXT_PARKING_MASK       (0x03u)
#define ADAS_MANAGER_CONTEXT_TRAFFIC_MASK       (0x07u)
#define ADAS_MANAGER_CONTEXT_TRAFFIC_LANE_MASK  (0x18u)
#define ADAS_MANAGER_CONTEXT_TRAFFIC_LANE_SHIFT (3u)

/* Collision is specified at 10 ms. The other applications may initially use
 * the PPT's 50 ms development period. Missing applications are never treated
 * as a fault; these timeouts only invalidate a slot that has supplied data. */
#define ADAS_MANAGER_MRM_TIMEOUT_TICK100US       (600u)
#define ADAS_MANAGER_COLLISION_TIMEOUT_TICK100US (300u)
#define ADAS_MANAGER_PARKING_TIMEOUT_TICK100US   (1500u)
#define ADAS_MANAGER_TRAFFIC_TIMEOUT_TICK100US   (1500u)
/* Matches TRAJ_FD_WATCHDOG_MS (500 ms / 10 missed 50 ms frames). */
#define ADAS_MANAGER_CAMERA_TIMEOUT_TICK100US    (5000u)
#define ADAS_MANAGER_GPS_TIMEOUT_TICK100US       (5000u)

typedef struct
{
    boolean seen;
    boolean hasAcceptedFrame;
    boolean packetValid;
    boolean fresh;
    boolean active;
    boolean enabled;
    boolean atomic;
    boolean longitudinalValid;
    boolean lateralValid;
    boolean degraded;
    boolean hasSequence;
    boolean manualRearmReady;
    boolean manualEligible;
    boolean sequenceResyncRequired;
    uint8 functionId;
    uint8 status;
    uint8 sequence;
    uint8 appContext;
    uint8 segment;
    uint8 parkingType;
    uint8 trafficSignal;
    uint8 selectedLane;
    sint16 speedKphX100;
    sint16 steeringDegX10;
    uint32 lastValidTick100us;
    uint32 validCount;
    uint32 invalidCount;
    uint32 duplicateCount;
    uint32 sequenceGapCount;
    uint32 messageLostCount;
    uint32 manualBaselineCount;
    uint32 manualDisableCount;
} AdasManagerSlot;

volatile uint8  g_adasManagerModeSeen = 0u;
volatile uint8  g_adasManagerOutputValid = 0u;
volatile uint8  g_adasManagerOutputEnabled = 0u;
volatile uint8  g_adasManagerSelectedSource = (uint8)AdasManagerSource_None;
volatile uint8  g_adasManagerLongitudinalOwner = (uint8)AdasManagerSource_None;
volatile uint8  g_adasManagerLateralOwner = (uint8)AdasManagerSource_None;
volatile uint8  g_adasManagerAppContext = 0u;
volatile uint8  g_adasManagerSegment = (uint8)AdasManagerSegment_Invalid;
volatile sint16 g_adasManagerSpeedKphX100 = 0;
volatile sint16 g_adasManagerSteeringDegX10 = 0;
volatile uint8  g_adasManagerAggregateSequence = 0u;
volatile uint32 g_adasManagerAggregateValidCount = 0u;
volatile uint8  g_adasManagerReceivedMask = 0u;
volatile uint8  g_adasManagerValidatedMask = 0u;
volatile uint8  g_adasManagerFreshMask = 0u;
volatile uint8  g_adasManagerActiveMask = 0u;
volatile uint8  g_adasManagerManualRearmBlocked = 0u;
volatile uint32 g_adasManagerManualBlockCount = 0u;
volatile uint8  g_adasManagerSlotState[ADAS_MANAGER_SLOT_COUNT] = {0u};
volatile uint32 g_adasManagerSlotAgeMs[ADAS_MANAGER_SLOT_COUNT] = {0u};
volatile uint32 g_adasManagerSlotValidCount[ADAS_MANAGER_SLOT_COUNT] = {0u};
volatile uint32 g_adasManagerSlotInvalidCount[ADAS_MANAGER_SLOT_COUNT] = {0u};
volatile uint32 g_adasManagerSlotDuplicateCount[ADAS_MANAGER_SLOT_COUNT] = {0u};
volatile uint32 g_adasManagerSlotSequenceGapCount[ADAS_MANAGER_SLOT_COUNT] = {0u};
volatile uint32 g_adasManagerSlotMessageLostCount[ADAS_MANAGER_SLOT_COUNT] = {0u};
volatile uint8  g_adasManagerSlotLastRejectReason[ADAS_MANAGER_SLOT_COUNT] = {0u};
volatile uint32 g_adasManagerPacketRejectCount = 0u;
volatile uint32 g_adasManagerCrcErrorCount = 0u;
volatile uint32 g_adasManagerHeaderErrorCount = 0u;
volatile uint32 g_adasManagerRangeErrorCount = 0u;
volatile uint32 g_adasManagerSequenceErrorCount = 0u;

static AdasManagerSlot g_adasSlots[ADAS_MANAGER_SLOT_COUNT];
static AdasManagerOutput g_adasOutput;
static boolean g_adasOutputSnapshotValid = FALSE;
static uint32 g_adasOutputLongCount = 0u;
static uint32 g_adasOutputLatCount = 0u;

static uint8 AdasManager_SlotMask(uint8 slotIndex)
{
    return (uint8)(1u << slotIndex);
}

static sint16 AdasManager_GetS16Le(const uint8 *data)
{
    uint16 raw = (uint16)data[0] | ((uint16)data[1] << 8u);
    return (sint16)raw;
}

static uint8 AdasManager_Crc8Update(uint8 crc, uint8 value)
{
    uint8 bit;

    crc ^= value;
    for (bit = 0u; bit < 8u; bit++)
    {
        crc = ((crc & 0x80u) != 0u) ?
              (uint8)((crc << 1u) ^ ADAS_MANAGER_CRC8_POLY) :
              (uint8)(crc << 1u);
    }
    return crc;
}

static uint8 AdasManager_CalculateCrc(uint16 canId, const uint8 data[7])
{
    uint8 crc = ADAS_MANAGER_CRC8_INIT;
    uint8 index;

    crc = AdasManager_Crc8Update(crc, (uint8)((canId >> 8u) & 0xFFu));
    crc = AdasManager_Crc8Update(crc, (uint8)(canId & 0xFFu));
    for (index = 0u; index < 7u; index++)
    {
        crc = AdasManager_Crc8Update(crc, data[index]);
    }
    return (uint8)(crc ^ ADAS_MANAGER_CRC8_XOROUT);
}

static boolean AdasManager_RejectFrame(uint8 slotIndex,
                                       AdasManagerRejectReason reason)
{
    AdasManagerSlot *slot = &g_adasSlots[slotIndex];
    uint8 slotMask = AdasManager_SlotMask(slotIndex);

    slot->invalidCount++;
    slot->packetValid = FALSE;
    slot->fresh = FALSE;
    slot->active = FALSE;
    slot->enabled = FALSE;
    slot->atomic = FALSE;
    slot->longitudinalValid = FALSE;
    slot->lateralValid = FALSE;
    slot->degraded = FALSE;
    if (slot->hasSequence != FALSE)
    {
        slot->sequenceResyncRequired = TRUE;
    }

    g_adasManagerValidatedMask &= (uint8)(~slotMask);
    g_adasManagerFreshMask &= (uint8)(~slotMask);
    g_adasManagerActiveMask &= (uint8)(~slotMask);
    g_adasManagerSlotState[slotIndex] = (uint8)AdasManagerSlot_Invalid;
    g_adasManagerSlotInvalidCount[slotIndex] = slot->invalidCount;
    g_adasManagerSlotLastRejectReason[slotIndex] = (uint8)reason;
    g_adasManagerPacketRejectCount++;

    switch (reason)
    {
        case AdasManagerReject_MessageLost:
            slot->messageLostCount++;
            g_adasManagerSlotMessageLostCount[slotIndex] =
                slot->messageLostCount;
            break;
        case AdasManagerReject_Crc:
            g_adasManagerCrcErrorCount++;
            break;
        case AdasManagerReject_Header:
            g_adasManagerHeaderErrorCount++;
            break;
        case AdasManagerReject_Range:
            g_adasManagerRangeErrorCount++;
            break;
        case AdasManagerReject_Sequence:
            g_adasManagerSequenceErrorCount++;
            break;
        default:
            break;
    }

    /* This mirrors the verified 0x300 path: once a slot has been selected,
     * a malformed packet cannot leave its previous command alive until the
     * timeout.  The next valid newer packet may recover the slot. */
    return FALSE;
}

static sint8 AdasManager_FindSlot(uint16 canId)
{
    switch (canId)
    {
        case ADAS_MANAGER_CAN_ID_MRM:
            return (sint8)ADAS_MANAGER_SLOT_MRM;
        case ADAS_MANAGER_CAN_ID_COLLISION:
            return (sint8)ADAS_MANAGER_SLOT_COLLISION;
        case ADAS_MANAGER_CAN_ID_PARKING:
            return (sint8)ADAS_MANAGER_SLOT_PARKING;
        case ADAS_MANAGER_CAN_ID_TRAFFIC:
            return (sint8)ADAS_MANAGER_SLOT_TRAFFIC;
        case ADAS_MANAGER_CAN_ID_CAMERA:
            return (sint8)ADAS_MANAGER_SLOT_CAMERA;
        case ADAS_MANAGER_CAN_ID_GPS:
            return (sint8)ADAS_MANAGER_SLOT_GPS;
        default:
            return (sint8)-1;
    }
}

static uint8 AdasManager_FunctionForSlot(uint8 slotIndex)
{
    static const uint8 functionIds[ADAS_MANAGER_SLOT_COUNT] =
    {
        ADAS_MANAGER_FUNCTION_MRM,
        ADAS_MANAGER_FUNCTION_COLLISION,
        ADAS_MANAGER_FUNCTION_PARKING,
        ADAS_MANAGER_FUNCTION_TRAFFIC,
        ADAS_MANAGER_FUNCTION_CAMERA,
        ADAS_MANAGER_FUNCTION_GPS
    };

    return functionIds[slotIndex];
}

static uint32 AdasManager_TimeoutForSlot(uint8 slotIndex)
{
    static const uint32 timeoutTick100us[ADAS_MANAGER_SLOT_COUNT] =
    {
        ADAS_MANAGER_MRM_TIMEOUT_TICK100US,
        ADAS_MANAGER_COLLISION_TIMEOUT_TICK100US,
        ADAS_MANAGER_PARKING_TIMEOUT_TICK100US,
        ADAS_MANAGER_TRAFFIC_TIMEOUT_TICK100US,
        ADAS_MANAGER_CAMERA_TIMEOUT_TICK100US,
        ADAS_MANAGER_GPS_TIMEOUT_TICK100US
    };

    return timeoutTick100us[slotIndex];
}

static AdasManagerSlot *AdasManager_SlotForSource(uint8 source)
{
    switch (source)
    {
        case (uint8)AdasManagerSource_Mrm:
            return &g_adasSlots[ADAS_MANAGER_SLOT_MRM];
        case (uint8)AdasManagerSource_Collision:
            return &g_adasSlots[ADAS_MANAGER_SLOT_COLLISION];
        case (uint8)AdasManagerSource_Parking:
            return &g_adasSlots[ADAS_MANAGER_SLOT_PARKING];
        case (uint8)AdasManagerSource_Traffic:
            return &g_adasSlots[ADAS_MANAGER_SLOT_TRAFFIC];
        case (uint8)AdasManagerSource_Camera:
            return &g_adasSlots[ADAS_MANAGER_SLOT_CAMERA];
        case (uint8)AdasManagerSource_Gps:
            return &g_adasSlots[ADAS_MANAGER_SLOT_GPS];
        default:
            return 0;
    }
}

static boolean AdasManager_IsActiveStatus(uint8 status)
{
    return ((status == (uint8)AdasManagerRequest_ActiveGood) ||
            (status == (uint8)AdasManagerRequest_ActiveDegraded)) ? TRUE : FALSE;
}

static boolean AdasManager_IsInactiveHeartbeatStatus(uint8 status)
{
    return ((status == (uint8)AdasManagerRequest_Inactive) ||
            (status == (uint8)AdasManagerRequest_Ready) ||
            (status == (uint8)AdasManagerRequest_Completed)) ? TRUE : FALSE;
}

static boolean AdasManager_IsAtomicFunction(uint8 functionId)
{
    return ((functionId == ADAS_MANAGER_FUNCTION_MRM) ||
            (functionId == ADAS_MANAGER_FUNCTION_COLLISION) ||
            (functionId == ADAS_MANAGER_FUNCTION_PARKING)) ? TRUE : FALSE;
}

static boolean AdasManager_IsActiveEligible(const AdasManagerSlot *slot)
{
    if ((slot == 0) ||
        (slot->fresh == FALSE) ||
        (slot->active == FALSE))
    {
        return FALSE;
    }

    /* Eligibility is owned per application.  Clearing the aggregate re-arm
     * flag after one source completes its handshake must never revive an old
     * ACTIVE command from another source. */
    if (slot->manualEligible == FALSE)
    {
        return FALSE;
    }

    return TRUE;
}

/* Camera and Gps share the role Nominal used to hold: whichever of the two
 * is fresh/active/eligible wins, with Camera preferred whenever both are. */
static AdasManagerSlot *AdasManager_SelectCameraOrGps(void)
{
    if (AdasManager_IsActiveEligible(&g_adasSlots[ADAS_MANAGER_SLOT_CAMERA]) != FALSE)
    {
        return &g_adasSlots[ADAS_MANAGER_SLOT_CAMERA];
    }
    if (AdasManager_IsActiveEligible(&g_adasSlots[ADAS_MANAGER_SLOT_GPS]) != FALSE)
    {
        return &g_adasSlots[ADAS_MANAGER_SLOT_GPS];
    }
    return 0;
}

static boolean AdasManager_IsTrafficCommandEligible(void)
{
    const AdasManagerSlot *traffic = &g_adasSlots[ADAS_MANAGER_SLOT_TRAFFIC];
    const AdasManagerSlot *lateralSource = AdasManager_SelectCameraOrGps();

    /* Traffic is longitudinal-only in this interface.  It may win only when
     * a validated fresh Camera (or, failing that, Gps) request can supply
     * the lateral command. */
    return ((AdasManager_IsActiveEligible(traffic) != FALSE) &&
            (traffic->longitudinalValid != FALSE) &&
            (lateralSource != 0) &&
            (lateralSource->lateralValid != FALSE)) ? TRUE : FALSE;
}

static uint8 AdasManager_SourceForSlot(const AdasManagerSlot *slot)
{
    return (slot == &g_adasSlots[ADAS_MANAGER_SLOT_CAMERA]) ?
           (uint8)AdasManagerSource_Camera : (uint8)AdasManagerSource_Gps;
}

static uint8 AdasManager_SelectPrimary(uint8 segment)
{
    AdasManagerSlot *camOrGps;

    if (AdasManager_IsActiveEligible(&g_adasSlots[ADAS_MANAGER_SLOT_MRM]) != FALSE)
    {
        return (uint8)AdasManagerSource_Mrm;
    }
    if (AdasManager_IsActiveEligible(&g_adasSlots[ADAS_MANAGER_SLOT_COLLISION]) != FALSE)
    {
        return (uint8)AdasManagerSource_Collision;
    }

    if (segment == (uint8)AdasManagerSegment_Parking)
    {
        if (AdasManager_IsActiveEligible(&g_adasSlots[ADAS_MANAGER_SLOT_PARKING]) != FALSE)
        {
            return (uint8)AdasManagerSource_Parking;
        }
        camOrGps = AdasManager_SelectCameraOrGps();
        if (camOrGps != 0)
        {
            return AdasManager_SourceForSlot(camOrGps);
        }
        if (AdasManager_IsTrafficCommandEligible() != FALSE)
        {
            return (uint8)AdasManagerSource_Traffic;
        }
    }
    else if (segment == (uint8)AdasManagerSegment_Intersection)
    {
        if (AdasManager_IsTrafficCommandEligible() != FALSE)
        {
            return (uint8)AdasManagerSource_Traffic;
        }
        camOrGps = AdasManager_SelectCameraOrGps();
        if (camOrGps != 0)
        {
            return AdasManager_SourceForSlot(camOrGps);
        }
        if (AdasManager_IsActiveEligible(&g_adasSlots[ADAS_MANAGER_SLOT_PARKING]) != FALSE)
        {
            return (uint8)AdasManagerSource_Parking;
        }
    }
    else
    {
        camOrGps = AdasManager_SelectCameraOrGps();
        if (camOrGps != 0)
        {
            return AdasManager_SourceForSlot(camOrGps);
        }
        if (AdasManager_IsTrafficCommandEligible() != FALSE)
        {
            return (uint8)AdasManagerSource_Traffic;
        }
        if (AdasManager_IsActiveEligible(&g_adasSlots[ADAS_MANAGER_SLOT_PARKING]) != FALSE)
        {
            return (uint8)AdasManagerSource_Parking;
        }
    }

    return (uint8)AdasManagerSource_None;
}

static uint8 AdasManager_DetermineSegment(void)
{
    uint8 slotIndex;
    AdasManagerSlot *camOrGps = AdasManager_SelectCameraOrGps();

    if ((camOrGps != 0) &&
        (camOrGps->segment < (uint8)AdasManagerSegment_Invalid))
    {
        return camOrGps->segment;
    }

    for (slotIndex = 0u; slotIndex < ADAS_MANAGER_SLOT_COUNT; slotIndex++)
    {
        if ((AdasManager_IsActiveEligible(&g_adasSlots[slotIndex]) != FALSE) &&
            (g_adasSlots[slotIndex].segment <
             (uint8)AdasManagerSegment_Invalid))
        {
            return g_adasSlots[slotIndex].segment;
        }
    }

    return (uint8)AdasManagerSegment_Invalid;
}

static AdasManagerSlot *AdasManager_SelectInactiveHeartbeat(void)
{
    static const uint8 order[ADAS_MANAGER_SLOT_COUNT] =
    {
        ADAS_MANAGER_SLOT_CAMERA,
        ADAS_MANAGER_SLOT_GPS,
        ADAS_MANAGER_SLOT_TRAFFIC,
        ADAS_MANAGER_SLOT_PARKING,
        ADAS_MANAGER_SLOT_COLLISION,
        ADAS_MANAGER_SLOT_MRM
    };
    uint8 orderIndex;
    AdasManagerSlot *slot;

    for (orderIndex = 0u; orderIndex < ADAS_MANAGER_SLOT_COUNT; orderIndex++)
    {
        slot = &g_adasSlots[order[orderIndex]];
        if ((slot->fresh != FALSE) &&
            (slot->active == FALSE) &&
            (AdasManager_IsInactiveHeartbeatStatus(slot->status) != FALSE) &&
            ((g_adasManagerManualRearmBlocked == 0u) ||
             (slot->validCount > slot->manualBaselineCount)))
        {
            return slot;
        }
    }
    return 0;
}

static void AdasManager_ClearPublishedCommand(void)
{
    g_adasOutput.commandValid = FALSE;
    g_adasOutput.commandEnabled = FALSE;
    g_adasOutput.degraded = FALSE;
    g_adasOutput.speedKphX100 = 0;
    g_adasOutput.steeringDegX10 = 0;
    g_adasOutput.selectedSource = (uint8)AdasManagerSource_None;
    g_adasOutput.longitudinalOwner = (uint8)AdasManagerSource_None;
    g_adasOutput.lateralOwner = (uint8)AdasManagerSource_None;
    g_adasOutput.appContext = 0u;
    g_adasOutput.segment = (uint8)AdasManagerSegment_Invalid;
    g_adasOutput.parkingType = 0u;
    g_adasOutput.trafficSignal = 0u;
    g_adasOutput.selectedLane = 0u;
    g_adasOutputSnapshotValid = FALSE;
    g_adasOutputLongCount = 0u;
    g_adasOutputLatCount = 0u;
}

static void AdasManager_PublishCommand(AdasManagerSlot *primary,
                                       AdasManagerSlot *longitudinal,
                                       AdasManagerSlot *lateral,
                                       boolean enabled)
{
    boolean newSnapshot;

    if ((primary == 0) || (longitudinal == 0) || (lateral == 0))
    {
        AdasManager_ClearPublishedCommand();
        return;
    }

    newSnapshot = ((g_adasOutputSnapshotValid == FALSE) ||
                   (g_adasOutput.selectedSource != primary->functionId) ||
                   (g_adasOutput.longitudinalOwner != longitudinal->functionId) ||
                   (g_adasOutput.lateralOwner != lateral->functionId) ||
                   (g_adasOutputLongCount != longitudinal->validCount) ||
                   (g_adasOutputLatCount != lateral->validCount) ||
                   (g_adasOutput.commandEnabled != enabled)) ? TRUE : FALSE;

    if (newSnapshot != FALSE)
    {
        g_adasManagerAggregateSequence++;
        g_adasManagerAggregateValidCount++;
        g_adasOutputLongCount = longitudinal->validCount;
        g_adasOutputLatCount = lateral->validCount;
    }

    g_adasOutput.modeSeen = (g_adasManagerModeSeen != 0u) ? TRUE : FALSE;
    g_adasOutput.commandValid = TRUE;
    g_adasOutput.commandEnabled = enabled;
    g_adasOutput.degraded = ((primary->degraded != FALSE) ||
                             (longitudinal->degraded != FALSE) ||
                             (lateral->degraded != FALSE)) ? TRUE : FALSE;
    g_adasOutput.sequence = g_adasManagerAggregateSequence;
    g_adasOutput.validCount = g_adasManagerAggregateValidCount;
    g_adasOutput.speedKphX100 = (enabled != FALSE) ? longitudinal->speedKphX100 : 0;
    g_adasOutput.steeringDegX10 = (enabled != FALSE) ? lateral->steeringDegX10 : 0;
    g_adasOutput.selectedSource = primary->functionId;
    g_adasOutput.longitudinalOwner = longitudinal->functionId;
    g_adasOutput.lateralOwner = lateral->functionId;
    g_adasOutput.appContext = primary->appContext;
    g_adasOutput.segment = primary->segment;
    g_adasOutput.parkingType = primary->parkingType;
    g_adasOutput.trafficSignal = primary->trafficSignal;
    g_adasOutput.selectedLane = primary->selectedLane;
    g_adasOutputSnapshotValid = TRUE;
}

static void AdasManager_PublishTrace(void)
{
    g_adasManagerOutputValid = (g_adasOutput.commandValid != FALSE) ? 1u : 0u;
    g_adasManagerOutputEnabled = (g_adasOutput.commandEnabled != FALSE) ? 1u : 0u;
    g_adasManagerSelectedSource = g_adasOutput.selectedSource;
    g_adasManagerLongitudinalOwner = g_adasOutput.longitudinalOwner;
    g_adasManagerLateralOwner = g_adasOutput.lateralOwner;
    g_adasManagerAppContext = g_adasOutput.appContext;
    g_adasManagerSegment = g_adasOutput.segment;
    g_adasManagerSpeedKphX100 = g_adasOutput.speedKphX100;
    g_adasManagerSteeringDegX10 = g_adasOutput.steeringDegX10;
}

void AdasManager_Init(void)
{
    uint8 slotIndex;

    for (slotIndex = 0u; slotIndex < ADAS_MANAGER_SLOT_COUNT; slotIndex++)
    {
        AdasManagerSlot *slot = &g_adasSlots[slotIndex];
        slot->seen = FALSE;
        slot->hasAcceptedFrame = FALSE;
        slot->packetValid = FALSE;
        slot->fresh = FALSE;
        slot->active = FALSE;
        slot->enabled = FALSE;
        slot->atomic = FALSE;
        slot->longitudinalValid = FALSE;
        slot->lateralValid = FALSE;
        slot->degraded = FALSE;
        slot->hasSequence = FALSE;
        slot->manualRearmReady = FALSE;
        slot->manualEligible = TRUE;
        slot->sequenceResyncRequired = FALSE;
        slot->functionId = AdasManager_FunctionForSlot(slotIndex);
        slot->status = (uint8)AdasManagerRequest_Invalid;
        slot->sequence = 0u;
        slot->appContext = 0u;
        slot->segment = (uint8)AdasManagerSegment_Invalid;
        slot->parkingType = 0u;
        slot->trafficSignal = 0u;
        slot->selectedLane = 0u;
        slot->speedKphX100 = 0;
        slot->steeringDegX10 = 0;
        slot->lastValidTick100us = 0u;
        slot->validCount = 0u;
        slot->invalidCount = 0u;
        slot->duplicateCount = 0u;
        slot->sequenceGapCount = 0u;
        slot->messageLostCount = 0u;
        slot->manualBaselineCount = 0u;
        slot->manualDisableCount = 0u;

        /* ReceivedMask distinguishes never-received from malformed/stale.
         * For arbitration and the debugger, both are explicitly INVALID. */
        g_adasManagerSlotState[slotIndex] = (uint8)AdasManagerSlot_Invalid;
        g_adasManagerSlotAgeMs[slotIndex] = 0xFFFFFFFFu;
        g_adasManagerSlotValidCount[slotIndex] = 0u;
        g_adasManagerSlotInvalidCount[slotIndex] = 0u;
        g_adasManagerSlotDuplicateCount[slotIndex] = 0u;
        g_adasManagerSlotSequenceGapCount[slotIndex] = 0u;
        g_adasManagerSlotMessageLostCount[slotIndex] = 0u;
        g_adasManagerSlotLastRejectReason[slotIndex] =
            (uint8)AdasManagerReject_None;
    }

    g_adasManagerModeSeen = 0u;
    g_adasManagerReceivedMask = 0u;
    g_adasManagerValidatedMask = 0u;
    g_adasManagerFreshMask = 0u;
    g_adasManagerActiveMask = 0u;
    g_adasManagerManualRearmBlocked = 0u;
    g_adasManagerManualBlockCount = 0u;
    g_adasManagerAggregateSequence = 0u;
    g_adasManagerAggregateValidCount = 0u;
    g_adasManagerPacketRejectCount = 0u;
    g_adasManagerCrcErrorCount = 0u;
    g_adasManagerHeaderErrorCount = 0u;
    g_adasManagerRangeErrorCount = 0u;
    g_adasManagerSequenceErrorCount = 0u;

    g_adasOutput.modeSeen = FALSE;
    g_adasOutput.sequence = 0u;
    g_adasOutput.validCount = 0u;
    AdasManager_ClearPublishedCommand();
    AdasManager_PublishTrace();
}

boolean AdasManager_OnCanFrame(uint16 canId,
                               const uint8 data[ADAS_MANAGER_CAN_DLC],
                               uint8 dlc,
                               boolean messageLost,
                               uint32 nowTick100us)
{
    sint8 signedSlotIndex = AdasManager_FindSlot(canId);
    uint8 slotIndex;
    uint8 header;
    uint8 functionId;
    uint8 status;
    uint8 sequence;
    uint8 sequenceDelta;
    uint8 segment;
    sint16 speedKphX100;
    sint16 steeringDegX10;
    boolean version1;
    boolean enabled;
    boolean atomic;
    boolean longitudinalValid;
    boolean lateralValid;
    boolean active;
    AdasManagerSlot *slot;
    uint32 ageTick100us;

    if (signedSlotIndex < 0)
    {
        return FALSE;
    }

    slotIndex = (uint8)signedSlotIndex;
    slot = &g_adasSlots[slotIndex];
    slot->seen = TRUE;
    g_adasManagerReceivedMask |= AdasManager_SlotMask(slotIndex);

    if ((data == 0) || (dlc != ADAS_MANAGER_CAN_DLC))
    {
        return AdasManager_RejectFrame(slotIndex, AdasManagerReject_Dlc);
    }

    /* Match the verified CAN_FIXED command path: a mailbox overrun means the
     * command history is incomplete, so even a self-contained newest frame
     * is not allowed to refresh or keep a command alive. */
    if (messageLost != FALSE)
    {
        return AdasManager_RejectFrame(slotIndex,
                                       AdasManagerReject_MessageLost);
    }

    header = data[0];
    functionId = header & ADAS_MANAGER_HEADER_FUNCTION_MASK;
    version1 = ((header & ADAS_MANAGER_HEADER_VERSION_MASK) != 0u) ? TRUE : FALSE;
    enabled = ((header & ADAS_MANAGER_HEADER_ENABLE_MASK) != 0u) ? TRUE : FALSE;
    atomic = ((header & ADAS_MANAGER_HEADER_ATOMIC_MASK) != 0u) ? TRUE : FALSE;
    lateralValid = ((header & ADAS_MANAGER_HEADER_LATERAL_VALID_MASK) != 0u) ? TRUE : FALSE;
    longitudinalValid = ((header & ADAS_MANAGER_HEADER_LONG_VALID_MASK) != 0u) ? TRUE : FALSE;
    status = (data[6] & ADAS_MANAGER_STATUS_MASK) >> ADAS_MANAGER_STATUS_SHIFT;
    sequence = data[6] & ADAS_MANAGER_SEQUENCE_MASK;
    active = AdasManager_IsActiveStatus(status);
    speedKphX100 = AdasManager_GetS16Le(&data[1]);
    steeringDegX10 = AdasManager_GetS16Le(&data[3]);

    if ((version1 == FALSE) && (ADAS_MANAGER_ALLOW_LEGACY_V0 == 0u))
    {
        return AdasManager_RejectFrame(slotIndex,
                                       AdasManagerReject_Header);
    }

    if (AdasManager_CalculateCrc(canId, data) != data[7])
    {
        return AdasManager_RejectFrame(slotIndex, AdasManagerReject_Crc);
    }

    if ((functionId != AdasManager_FunctionForSlot(slotIndex)) ||
        ((active != FALSE) && (enabled == FALSE)) ||
        ((active == FALSE) && (enabled != FALSE)) ||
        ((active != FALSE) &&
         (AdasManager_IsAtomicFunction(functionId) != FALSE) &&
         ((atomic == FALSE) ||
          (longitudinalValid == FALSE) ||
          (lateralValid == FALSE))) ||
        ((active != FALSE) &&
         ((functionId == ADAS_MANAGER_FUNCTION_CAMERA) ||
          (functionId == ADAS_MANAGER_FUNCTION_GPS)) &&
         ((atomic != FALSE) ||
          (longitudinalValid == FALSE) ||
          (lateralValid == FALSE))) ||
        ((active != FALSE) &&
         (functionId == ADAS_MANAGER_FUNCTION_TRAFFIC) &&
         ((atomic != FALSE) ||
          (longitudinalValid == FALSE) ||
          (lateralValid != FALSE))))
    {
        return AdasManager_RejectFrame(slotIndex,
                                       AdasManagerReject_Header);
    }

    if ((speedKphX100 < ADAS_MANAGER_MIN_SPEED_X100) ||
        (speedKphX100 > ADAS_MANAGER_MAX_SPEED_X100) ||
        (steeringDegX10 < -ADAS_MANAGER_MAX_STEERING_X10) ||
        (steeringDegX10 > ADAS_MANAGER_MAX_STEERING_X10) ||
        ((longitudinalValid == FALSE) && (speedKphX100 != 0)) ||
        ((lateralValid == FALSE) && (steeringDegX10 != 0)) ||
        ((active == FALSE) &&
         ((speedKphX100 != 0) || (steeringDegX10 != 0))))
    {
        return AdasManager_RejectFrame(slotIndex,
                                       AdasManagerReject_Range);
    }

    segment = ((functionId == ADAS_MANAGER_FUNCTION_CAMERA) ||
               (functionId == ADAS_MANAGER_FUNCTION_GPS)) ?
              data[5] : (uint8)AdasManagerSegment_Invalid;
    if (((functionId == ADAS_MANAGER_FUNCTION_CAMERA) ||
         (functionId == ADAS_MANAGER_FUNCTION_GPS)) &&
        (active != FALSE) &&
        (segment > (uint8)AdasManagerSegment_SelectedLane))
    {
        return AdasManager_RejectFrame(slotIndex,
                                       AdasManagerReject_Range);
    }

    if (slot->hasSequence != FALSE)
    {
        ageTick100us = nowTick100us - slot->lastValidTick100us;
        if ((slot->sequenceResyncRequired == FALSE) &&
            (ageTick100us <= AdasManager_TimeoutForSlot(slotIndex)))
        {
            sequenceDelta = (uint8)((sequence - slot->sequence) &
                                    ADAS_MANAGER_SEQUENCE_MASK);
            if (sequenceDelta == 0u)
            {
                slot->duplicateCount++;
                g_adasManagerSlotDuplicateCount[slotIndex] = slot->duplicateCount;
                return FALSE;
            }
            if (sequenceDelta >= 16u)
            {
                return AdasManager_RejectFrame(
                    slotIndex, AdasManagerReject_Sequence);
            }
            if (sequenceDelta > 1u)
            {
                slot->sequenceGapCount += (uint32)(sequenceDelta - 1u);
                g_adasManagerSlotSequenceGapCount[slotIndex] = slot->sequenceGapCount;
            }
        }
        else
        {
            /* A stale ACTIVE frame is not allowed to establish a new
             * sequence baseline: otherwise a frozen/replayed command could
             * wake up every timeout interval.  Recovery starts with a valid
             * disabled heartbeat, followed by a newer ACTIVE frame. */
            slot->sequenceResyncRequired = TRUE;
            if ((active != FALSE) ||
                (AdasManager_IsInactiveHeartbeatStatus(status) == FALSE))
            {
                return AdasManager_RejectFrame(
                    slotIndex, AdasManagerReject_Sequence);
            }
        }
    }

    slot->hasAcceptedFrame = TRUE;
    slot->packetValid = TRUE;
    slot->fresh = TRUE;
    slot->active = active;
    slot->enabled = enabled;
    slot->atomic = atomic;
    slot->longitudinalValid = longitudinalValid;
    slot->lateralValid = lateralValid;
    slot->degraded = (status == (uint8)AdasManagerRequest_ActiveDegraded) ? TRUE : FALSE;
    slot->hasSequence = TRUE;
    slot->sequenceResyncRequired = FALSE;
    slot->status = status;
    slot->sequence = sequence;
    slot->appContext = data[5];
    slot->speedKphX100 = (active != FALSE) ? speedKphX100 : 0;
    slot->steeringDegX10 = (active != FALSE) ? steeringDegX10 : 0;
    slot->lastValidTick100us = nowTick100us;
    slot->validCount++;
    g_adasManagerValidatedMask |= AdasManager_SlotMask(slotIndex);
    g_adasManagerSlotLastRejectReason[slotIndex] =
        (uint8)AdasManagerReject_None;

    if ((functionId == ADAS_MANAGER_FUNCTION_CAMERA) ||
        (functionId == ADAS_MANAGER_FUNCTION_GPS))
    {
        slot->segment = (data[5] <= (uint8)AdasManagerSegment_SelectedLane) ?
                        data[5] : (uint8)AdasManagerSegment_Invalid;
    }
    else
    {
        slot->segment = (uint8)AdasManagerSegment_Invalid;
    }

    if (functionId == ADAS_MANAGER_FUNCTION_PARKING)
    {
        slot->parkingType = data[5] & ADAS_MANAGER_CONTEXT_PARKING_MASK;
    }
    else
    {
        slot->parkingType = 0u;
    }

    if (functionId == ADAS_MANAGER_FUNCTION_TRAFFIC)
    {
        slot->trafficSignal = data[5] & ADAS_MANAGER_CONTEXT_TRAFFIC_MASK;
        slot->selectedLane = (data[5] & ADAS_MANAGER_CONTEXT_TRAFFIC_LANE_MASK) >>
                             ADAS_MANAGER_CONTEXT_TRAFFIC_LANE_SHIFT;
    }
    else
    {
        slot->trafficSignal = 0u;
        slot->selectedLane = 0u;
    }

    if (slot->manualEligible == FALSE)
    {
        if ((active == FALSE) &&
            (AdasManager_IsInactiveHeartbeatStatus(status) != FALSE) &&
            (slot->validCount > slot->manualBaselineCount))
        {
            slot->manualRearmReady = TRUE;
            slot->manualEligible = FALSE;
            slot->manualDisableCount = slot->validCount;
        }
        else if ((active != FALSE) &&
                 (slot->manualRearmReady != FALSE) &&
                 (slot->validCount > slot->manualDisableCount))
        {
            slot->manualEligible = TRUE;
        }
    }

    /* An inactive/fault frame from an unfinished application is diagnostic
     * only.  It must not commission ADAS mode and permanently suppress the
     * working legacy 0x300 source. */
    if (active != FALSE)
    {
        g_adasManagerModeSeen = 1u;
    }
    g_adasManagerSlotValidCount[slotIndex] = slot->validCount;
    return TRUE;
}

void AdasManager_Step5ms(uint32 nowTick100us)
{
    uint8 slotIndex;
    uint8 segment;
    uint8 primarySource;
    uint32 ageTick100us;
    AdasManagerSlot *slot;
    AdasManagerSlot *primary;
    AdasManagerSlot *longitudinal;
    AdasManagerSlot *lateral;
    AdasManagerSlot *inactive;

    g_adasManagerFreshMask = 0u;
    g_adasManagerActiveMask = 0u;

    for (slotIndex = 0u; slotIndex < ADAS_MANAGER_SLOT_COUNT; slotIndex++)
    {
        slot = &g_adasSlots[slotIndex];
        if ((slot->hasAcceptedFrame == FALSE) ||
            (slot->packetValid == FALSE))
        {
            slot->fresh = FALSE;
            g_adasManagerSlotAgeMs[slotIndex] = 0xFFFFFFFFu;
            g_adasManagerSlotState[slotIndex] = (uint8)AdasManagerSlot_Invalid;
            continue;
        }

        ageTick100us = nowTick100us - slot->lastValidTick100us;
        g_adasManagerSlotAgeMs[slotIndex] = ageTick100us / 10u;
        slot->fresh = (ageTick100us <= AdasManager_TimeoutForSlot(slotIndex)) ?
                      TRUE : FALSE;

        if (slot->fresh == FALSE)
        {
            slot->sequenceResyncRequired = TRUE;
            g_adasManagerSlotState[slotIndex] = (uint8)AdasManagerSlot_Stale;
            continue;
        }

        g_adasManagerFreshMask |= AdasManager_SlotMask(slotIndex);
        if (slot->active != FALSE)
        {
            g_adasManagerActiveMask |= AdasManager_SlotMask(slotIndex);
            g_adasManagerSlotState[slotIndex] = (uint8)AdasManagerSlot_Active;
        }
        else if (AdasManager_IsInactiveHeartbeatStatus(slot->status) != FALSE)
        {
            g_adasManagerSlotState[slotIndex] = (uint8)AdasManagerSlot_Inactive;
        }
        else
        {
            /* Abort/Fault/Invalid frames are fresh diagnostics, never a
             * command candidate. */
            g_adasManagerSlotState[slotIndex] = (uint8)AdasManagerSlot_Invalid;
        }
    }

    g_adasOutput.modeSeen = (g_adasManagerModeSeen != 0u) ? TRUE : FALSE;
    g_adasOutput.sequence = g_adasManagerAggregateSequence;
    g_adasOutput.validCount = g_adasManagerAggregateValidCount;

    segment = AdasManager_DetermineSegment();
    primarySource = AdasManager_SelectPrimary(segment);
    primary = AdasManager_SlotForSource(primarySource);

    if (primary != 0)
    {
        if ((primary->atomic != FALSE) ||
            (AdasManager_IsAtomicFunction(primary->functionId) != FALSE))
        {
            longitudinal = primary;
            lateral = primary;
        }
        else
        {
            longitudinal = (primary->longitudinalValid != FALSE) ? primary : 0;
            lateral = (primary->lateralValid != FALSE) ? primary : 0;

            /* Traffic normally owns only the longitudinal constraint. Camera
             * (or Gps, when Camera is unavailable) keeps lateral ownership
             * at an intersection; no mixed pair is ever permitted for
             * Collision, Parking, or MRM. */
            if (lateral == 0)
            {
                AdasManagerSlot *camOrGps = AdasManager_SelectCameraOrGps();
                if ((camOrGps != 0) && (camOrGps->lateralValid != FALSE))
                {
                    lateral = camOrGps;
                }
            }
            if (longitudinal == 0)
            {
                AdasManagerSlot *camOrGps = AdasManager_SelectCameraOrGps();
                if ((camOrGps != 0) && (camOrGps->longitudinalValid != FALSE))
                {
                    longitudinal = camOrGps;
                }
            }
        }

        if ((longitudinal != 0) && (lateral != 0))
        {
            if ((g_adasManagerManualRearmBlocked != 0u) &&
                (primary->manualEligible != FALSE))
            {
                g_adasManagerManualRearmBlocked = 0u;
            }
            AdasManager_PublishCommand(primary, longitudinal, lateral, TRUE);
            /* Preserve the course segment selected for arbitration even when
             * Traffic/Parking owns the current command. */
            g_adasOutput.segment = segment;
            if (AdasManager_IsActiveEligible(
                    &g_adasSlots[ADAS_MANAGER_SLOT_TRAFFIC]) != FALSE)
            {
                /* Traffic context remains observable in Pass 5/6 even when
                 * Camera/Gps owns the actual steering request. */
                g_adasOutput.trafficSignal =
                    g_adasSlots[ADAS_MANAGER_SLOT_TRAFFIC].trafficSignal;
                g_adasOutput.selectedLane =
                    g_adasSlots[ADAS_MANAGER_SLOT_TRAFFIC].selectedLane;
            }
            if (AdasManager_IsActiveEligible(
                    &g_adasSlots[ADAS_MANAGER_SLOT_PARKING]) != FALSE)
            {
                g_adasOutput.parkingType =
                    g_adasSlots[ADAS_MANAGER_SLOT_PARKING].parkingType;
            }
            AdasManager_PublishTrace();
            return;
        }
    }

    inactive = AdasManager_SelectInactiveHeartbeat();
    if (inactive != 0)
    {
        AdasManager_PublishCommand(inactive, inactive, inactive, FALSE);
    }
    else
    {
        AdasManager_ClearPublishedCommand();
        g_adasOutput.modeSeen = (g_adasManagerModeSeen != 0u) ? TRUE : FALSE;
        g_adasOutput.sequence = g_adasManagerAggregateSequence;
        g_adasOutput.validCount = g_adasManagerAggregateValidCount;
    }

    AdasManager_PublishTrace();
}

void AdasManager_GetOutput(AdasManagerOutput *output)
{
    if (output == 0)
    {
        return;
    }

    *output = g_adasOutput;
}

void AdasManager_InvalidateForManualOverride(void)
{
    uint8 slotIndex;

    if (g_adasManagerModeSeen == 0u)
    {
        return;
    }

    g_adasManagerManualRearmBlocked = 1u;
    g_adasManagerManualBlockCount++;
    for (slotIndex = 0u; slotIndex < ADAS_MANAGER_SLOT_COUNT; slotIndex++)
    {
        g_adasSlots[slotIndex].manualBaselineCount = g_adasSlots[slotIndex].validCount;
        g_adasSlots[slotIndex].manualDisableCount = g_adasSlots[slotIndex].validCount;
        g_adasSlots[slotIndex].manualRearmReady = FALSE;
        g_adasSlots[slotIndex].manualEligible = FALSE;
    }

    AdasManager_ClearPublishedCommand();
    g_adasOutput.modeSeen = TRUE;
    g_adasOutput.sequence = g_adasManagerAggregateSequence;
    g_adasOutput.validCount = g_adasManagerAggregateValidCount;
    AdasManager_PublishTrace();
}

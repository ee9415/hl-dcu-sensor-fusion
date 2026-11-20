#include "Can_TrajectoryFd.h"
#include <string.h>
#include <math.h>

CanTrajectoryFdChannel g_canTrajFdCamera;
CanTrajectoryFdChannel g_canTrajFdGps;

/* ---- CRC-8/SAE-J1850, Adas_Manager.c의 AdasManager_CalculateCrc와 동일 알고리즘 ---- */

static uint8 TrajFd_Crc8Update(uint8 crc, uint8 value)
{
    uint8 bit;

    crc ^= value;
    for (bit = 0u; bit < 8u; bit++)
    {
        crc = ((crc & 0x80u) != 0u) ?
              (uint8)((crc << 1u) ^ TRAJ_FD_CRC8_POLY) :
              (uint8)(crc << 1u);
    }
    return crc;
}

static uint8 TrajFd_CalculateCrc(uint16 canId, const uint8 *data, uint8 lenExclCrc)
{
    uint8 crc = TRAJ_FD_CRC8_INIT;
    uint8 index;

    crc = TrajFd_Crc8Update(crc, (uint8)((canId >> 8u) & 0xFFu));
    crc = TrajFd_Crc8Update(crc, (uint8)(canId & 0xFFu));
    for (index = 0u; index < lenExclCrc; index++)
    {
        crc = TrajFd_Crc8Update(crc, data[index]);
    }
    return (uint8)(crc ^ TRAJ_FD_CRC8_XOROUT);
}

/* ---- little-endian 헬퍼 ---- */

static float32 TrajFd_GetF32Le(const uint8 *p)
{
    uint32 raw = (uint32)p[0] | ((uint32)p[1] << 8u) | ((uint32)p[2] << 16u) | ((uint32)p[3] << 24u);
    float32 value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

static sint16 TrajFd_GetS16Le(const uint8 *p)
{
    uint16 raw = (uint16)p[0] | ((uint16)p[1] << 8u);
    return (sint16)raw;
}

static uint16 TrajFd_GetU16Le(const uint8 *p)
{
    return (uint16)p[0] | ((uint16)p[1] << 8u);
}

/* ---- 초기화 ---- */

void CanTrajectoryFd_InitChannel(CanTrajectoryFdChannel *channel,
                                 IfxMultican_Can_Node *node,
                                 uint16 canId,
                                 uint32 msgObjId,
                                 uint32 botObjId,
                                 uint32 topObjId)
{
    IfxMultican_Can_MsgObjConfig msgConfig;

    IfxMultican_Can_MsgObj_initConfig(&msgConfig, node);
    msgConfig.msgObjId = msgObjId;
    msgConfig.messageId = canId;
    msgConfig.acceptanceMask = 0x7FFFFFFFul;
    msgConfig.frame = IfxMultican_Frame_receive;
    msgConfig.control.messageLen = IfxMultican_DataLengthCode_32;
    msgConfig.control.extendedFrame = FALSE;
    msgConfig.control.matchingId = TRUE;
    msgConfig.control.fastBitRate = TRUE; /* BRS */
    /* 32바이트 확장 데이터(바이트 8~31)를 담는 링크 오브젝트. 채널마다
     * 겹치지 않는 값이어야 함 - initConfig() 기본값(252/253)을 그대로 두면
     * 카메라/GPS 두 채널이 충돌한다 (Can_TrajectoryFd.h 상단 주석 참고). */
    msgConfig.control.bottomMsgObjId = botObjId;
    msgConfig.control.topMsgObjId = topObjId;

    (void)IfxMultican_Can_MsgObj_init(&channel->rxMsgObj, &msgConfig);

    channel->hasPacket = FALSE;
    channel->lastRxTick100us = 0u;
    channel->lastSequence = 0u;
    channel->lastSequenceValid = FALSE;
}

void CanTrajectoryFd_InitAll(IfxMultican_Can_Node *node)
{
    CanTrajectoryFd_InitChannel(&g_canTrajFdCamera, node, TRAJ_FD_CAMERA_CAN_ID,
                                TRAJ_FD_CAMERA_MSG_OBJ_ID, TRAJ_FD_CAMERA_BOT_OBJ_ID,
                                TRAJ_FD_CAMERA_TOP_OBJ_ID);
    CanTrajectoryFd_InitChannel(&g_canTrajFdGps, node, TRAJ_FD_GPS_CAN_ID,
                                TRAJ_FD_GPS_MSG_OBJ_ID, TRAJ_FD_GPS_BOT_OBJ_ID,
                                TRAJ_FD_GPS_TOP_OBJ_ID);
}

/* ---- 수신 + 검증 ---- */

boolean CanTrajectoryFd_PollChannel(CanTrajectoryFdChannel *channel, uint32 nowTick100us)
{
    IfxMultican_Message msg;
    uint32 rawData[16];
    uint8 bytes[TRAJ_FD_DLC_BYTES];
    IfxMultican_Status status;
    uint8 header;
    uint8 crcReceived;
    uint8 crcExpected;
    static const uint8 lengthTable[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };
    uint8 receivedLen;

    status = IfxMultican_Can_MsgObj_readLongFrame(&channel->rxMsgObj, &msg, rawData);
    /* IfxMultican_Status_ok = 0, 실제 성공 시엔 newData(0x100) 비트가 서는 것이지
     * status가 정확히 0이 되는 게 아님 (Can_SpeedTx.c의 readMessage 쪽과 동일한
     * 비트마스크 검사로 통일 - 이전엔 등호 비교라 항상 실패했을 가능성이 큼). */
    if ((status & IfxMultican_Status_newData) == 0u)
    {
        return FALSE;
    }

    receivedLen = lengthTable[msg.lengthCode];
    if (receivedLen != TRAJ_FD_DLC_BYTES)
    {
        return FALSE; /* DLC 불일치 - 프로토콜 버전 안 맞음 */
    }

    memcpy(bytes, rawData, TRAJ_FD_DLC_BYTES);

    crcReceived = bytes[31];
    crcExpected = TrajFd_CalculateCrc((uint16)msg.id, bytes, 31u);
    if (crcReceived != crcExpected)
    {
        return FALSE;
    }

    header = bytes[0];
    if ((header & TRAJ_FD_HEADER_VERSION_MASK) != 0u)
    {
        return FALSE; /* 이 코드가 아는 버전은 v1(bit7=0)뿐 */
    }

    /* 시퀀스 중복 체크 - 안 바뀌면 정체된 프레임으로 보고 거부 (기존 관례) */
    if (channel->lastSequenceValid && (bytes[1] == channel->lastSequence))
    {
        return FALSE;
    }
    channel->lastSequence = bytes[1];
    channel->lastSequenceValid = TRUE;

    channel->latestPacket.enable        = (header & TRAJ_FD_HEADER_ENABLE_MASK) != 0u;
    channel->latestPacket.lateralValid  = (header & TRAJ_FD_HEADER_LATERAL_VALID_MASK) != 0u;
    channel->latestPacket.longValid     = (header & TRAJ_FD_HEADER_LONG_VALID_MASK) != 0u;
    channel->latestPacket.functionId    = header & TRAJ_FD_HEADER_FUNCTION_MASK;
    channel->latestPacket.sequence      = bytes[1];
    channel->latestPacket.requestStatus = bytes[2];
    channel->latestPacket.segment       = bytes[3];
    channel->latestPacket.c0            = TrajFd_GetF32Le(&bytes[4]);
    channel->latestPacket.c1            = TrajFd_GetF32Le(&bytes[8]);
    channel->latestPacket.c2            = TrajFd_GetF32Le(&bytes[12]);
    channel->latestPacket.c3            = TrajFd_GetF32Le(&bytes[16]);
    channel->latestPacket.vCmdKphX100   = TrajFd_GetS16Le(&bytes[20]);
    channel->latestPacket.fitValidLenCm = TrajFd_GetU16Le(&bytes[22]);

    channel->hasPacket = TRUE;
    channel->lastRxTick100us = nowTick100us;

    return TRUE;
}

boolean CanTrajectoryFd_GetLatestChannel(CanTrajectoryFdChannel *channel,
                                         TrajFdPacket *outPacket,
                                         uint32 nowTick100us)
{
    uint32 ageTick100us;
    uint32 watchdogTick100us = (uint32)TRAJ_FD_WATCHDOG_MS * 10u; /* 100us 단위. "10프레임 끊김"과 동일 */

    if (!channel->hasPacket)
    {
        return FALSE;
    }

    ageTick100us = nowTick100us - channel->lastRxTick100us; /* wraparound-safe (unsigned) */
    if (ageTick100us > watchdogTick100us)
    {
        return FALSE;
    }

    /* enable=0(정지) 프레임도 "유효한 패킷"으로 쳐야 한다 - VehicleManager의
     * CAN rearm 로직(VehicleManager_TryCompleteCanRearm)이 disable→enable
     * 순서를 직접 봐야 재무장이 풀리는데, 여기서 enable=0을 무효 처리해서
     * 걸러버리면 그 시퀀스 자체가 VehicleManager까지 절대 안 올라감
     * (실제로 이것 때문에 rearm이 안 풀렸음). enable 여부는 별도 필드로
     * 그대로 전달하고, 여기선 계수 자체가 신뢰할 만한지(lateralValid)만 본다. */
    if (!channel->latestPacket.lateralValid)
    {
        return FALSE;
    }

    if (outPacket != 0)
    {
        *outPacket = channel->latestPacket;
    }
    return TRUE;
}

/* ---- 카메라/GPS 소스 선택: 우선순위 없음, sticky 2-state 페일오버 ----
 *
 * v7 기준 더 이상 호출되지 않음(아래 TrajFd_BuildAdasManagerSlotFrame 참고) -
 * 카메라/GPS를 Adas_Manager의 own 6슬롯 우선순위(Mrm>Collision>Parking>
 * Traffic>Camera>Gps)에 각자 별도 슬롯(0x320/0x321)으로 직접 꽂는 방식으로
 * 바뀌면서, 이 파일 안에서 둘 중 하나를 미리 골라 병합하는 로직 자체가
 * 필요 없어짐 - Adas_Manager.c의 AdasManager_SelectCameraOrGps()가 그 역할을
 * 대신한다. 공개 API라 남겨두되(향후 다른 용도로 재사용 가능), 현재 호출부는
 * 없음. */
TrajFdSource TrajFd_SelectSource(CanTrajectoryFdChannel *cameraChannel,
                                 CanTrajectoryFdChannel *gpsChannel,
                                 uint32 nowTick100us,
                                 TrajFdPacket *outPacket)
{
    static TrajFdSource s_activeSource = TrajFdSource_None;

    TrajFdPacket cameraPacket;
    TrajFdPacket gpsPacket;
    boolean cameraValid = CanTrajectoryFd_GetLatestChannel(cameraChannel, &cameraPacket, nowTick100us);
    boolean gpsValid = CanTrajectoryFd_GetLatestChannel(gpsChannel, &gpsPacket, nowTick100us);

    /* 콜드스타트: 아직 활성 소스가 정해진 적 없음. 둘 중 먼저(또는 동시에)
     * 유효해진 쪽을 활성으로 삼는다 - "더 나아서"가 아니라 뭐라도 골라야
     * 해서 카메라를 먼저 검사할 뿐인 타이브레이커. */
    if (s_activeSource == TrajFdSource_None)
    {
        if (cameraValid)
        {
            s_activeSource = TrajFdSource_Camera;
        }
        else if (gpsValid)
        {
            s_activeSource = TrajFdSource_Gps;
        }
        else
        {
            return TrajFdSource_None;
        }
    }

    /* 지금 활성인 소스가 여전히 살아있으면 그대로 유지 - 다른 쪽으로
     * 절대 안 넘어감 (우선순위 없음 = "더 좋아 보여도 안 바꿈"). */
    if ((s_activeSource == TrajFdSource_Camera) && cameraValid)
    {
        if (outPacket != 0) { *outPacket = cameraPacket; }
        return TrajFdSource_Camera;
    }
    if ((s_activeSource == TrajFdSource_Gps) && gpsValid)
    {
        if (outPacket != 0) { *outPacket = gpsPacket; }
        return TrajFdSource_Gps;
    }

    /* 여기 왔다는 건 활성 소스가 워치독(TRAJ_FD_WATCHDOG_MS = 10프레임
     * 상당)을 넘겨 끊겼다는 뜻 - 즉시 다른 쪽으로 전환. */
    if (cameraValid)
    {
        s_activeSource = TrajFdSource_Camera;
        if (outPacket != 0) { *outPacket = cameraPacket; }
        return TrajFdSource_Camera;
    }
    if (gpsValid)
    {
        s_activeSource = TrajFdSource_Gps;
        if (outPacket != 0) { *outPacket = gpsPacket; }
        return TrajFdSource_Gps;
    }

    /* 둘 다 끊김 - 다음에 뭐든 먼저 살아나는 쪽을 다시 고르도록 리셋. */
    s_activeSource = TrajFdSource_None;
    return TrajFdSource_None;
}

/* ---- 경로 다항식 평가 (PDF 수식 그대로) ---- */

static float32 TrajFd_ClampF32(float32 value, float32 minValue, float32 maxValue)
{
    if (value < minValue) { return minValue; }
    if (value > maxValue) { return maxValue; }
    return value;
}

static float32 TrajFd_PreviewDistance(float32 v, const TrajFdControlParam *param, float32 fitValidLenM)
{
    float32 xp = v * param->previewTimeS;
    float32 xmax = TrajFd_ClampF32(param->previewDistMaxM, param->previewDistMinM, fitValidLenM);
    return TrajFd_ClampF32(xp, param->previewDistMinM, xmax);
}

static float32 TrajFd_EvalY(const TrajFdPacket *pkt, float32 x)
{
    return pkt->c0 + pkt->c1 * x + pkt->c2 * x * x + pkt->c3 * x * x * x;
}

static float32 TrajFd_EvalYPrime(const TrajFdPacket *pkt, float32 x)
{
    return pkt->c1 + 2.0f * pkt->c2 * x + 3.0f * pkt->c3 * x * x;
}

static float32 TrajFd_EvalYDoublePrime(const TrajFdPacket *pkt, float32 x)
{
    return 2.0f * pkt->c2 + 6.0f * pkt->c3 * x;
}

static float32 TrajFd_EvalCurvature(const TrajFdPacket *pkt, float32 x)
{
    float32 yp = TrajFd_EvalYPrime(pkt, x);
    float32 ypp = TrajFd_EvalYDoublePrime(pkt, x);
    float32 denomBase = 1.0f + yp * yp;
    float32 denom = denomBase * sqrtf(denomBase); /* (1+y'^2)^1.5 */
    return ypp / denom;
}

/* ---- Stage 4: Curvature FF + FB (PDF "논문급 최종 완성형") ---- */

float32 TrajFd_ComputeSteeringCurvatureFf(const TrajFdPacket *pkt,
                                          const TrajFdControlParam *param,
                                          float32 vehicleSpeedMps,
                                          float32 vehicleHeadingRad,
                                          float32 tsSeconds,
                                          float32 *eyFiltPrev)
{
    float32 fitValidLenM = (float32)pkt->fitValidLenCm / 100.0f;
    float32 xp = TrajFd_PreviewDistance(vehicleSpeedMps, param, fitValidLenM);

    float32 ey = TrajFd_EvalY(pkt, xp);
    float32 psiC = atanf(TrajFd_EvalYPrime(pkt, xp));
    float32 ePsi = psiC - vehicleHeadingRad;
    float32 kappaC = TrajFd_EvalCurvature(pkt, xp);

    float32 eyFilt = ey;
    float32 dey = (tsSeconds > 0.0f) ? ((eyFilt - *eyFiltPrev) / tsSeconds) : 0.0f;
    (void)dey; /* Kd 항은 호출자가 필요하면 param에 추가해서 확장 */

    float32 deltaFf = atanf(param->wheelBaseM * kappaC);
    float32 deltaCmd = deltaFf + param->curvatureFfKy * eyFilt + param->curvatureFfKpsi * ePsi;

    *eyFiltPrev = eyFilt;

    return deltaCmd;
}

/* ---- Stage 3: Stanley (대안, FF 없이) ---- */

float32 TrajFd_ComputeSteeringStanley(const TrajFdPacket *pkt,
                                      const TrajFdControlParam *param,
                                      float32 vehicleSpeedMps,
                                      float32 vehicleHeadingRad)
{
    float32 fitValidLenM = (float32)pkt->fitValidLenCm / 100.0f;
    float32 xp = TrajFd_PreviewDistance(vehicleSpeedMps, param, fitValidLenM);

    float32 ey = TrajFd_EvalY(pkt, xp);
    float32 psiC = atanf(TrajFd_EvalYPrime(pkt, xp));
    float32 ePsi = psiC - vehicleHeadingRad;

    return ePsi + atanf((param->stanleyGainK * ey) / (vehicleSpeedMps + param->speedEpsilonMps));
}

/* ---- v7: 카메라/GPS 각자 자기 슬롯(0x320/0x321)으로 Adas_Manager 주입 ----
 *
 * v6까지는 이 파일이 직접 카메라/GPS 중 하나를 골라(TrajFd_SelectSource)
 * 병합된 프레임 하나를 Nominal 슬롯에 꽂았다. v7은 그 선택 자체를
 * Adas_Manager의 own 6슬롯 우선순위(Mrm>Collision>Parking>Traffic>
 * Camera>Gps)에 맡긴다 - 이 함수는 채널 하나(카메라 또는 GPS)만 보고
 * 그 채널의 프레임을 그대로 만들어 돌려준다. 호출자(ControlLoop.c)가
 * 카메라/GPS 각각에 대해 이 함수를 한 번씩 불러 AdasManager_OnCanFrame()에
 * 서로 다른 CAN ID(0x320/0x321)로 주입한다. */

/* Adas_Manager.h와 정확히 같은 값 (헤더 비트 레이아웃은 TRAJ_FD와 애초에
 * 동일하게 설계해서 TRAJ_FD_HEADER_* 그대로 재사용 가능). 여기 상수들만
 * Adas_Manager 쪽 이름을 그대로 옮겨 적음 - Can_TrajectoryFd.c가
 * Adas_Manager.h에 의존하지 않게 하려고 재정의. */
#define ADAS_SLOT_STATUS_SHIFT        (5u)
#define ADAS_SLOT_SEQUENCE_MASK       (0x1Fu)
#define ADAS_SLOT_STATUS_ACTIVE_GOOD  (2u)   /* AdasManagerRequestStatus_ActiveGood */
#define ADAS_SLOT_STATUS_INACTIVE     (0u)   /* AdasManagerRequestStatus_Inactive */
#define ADAS_SLOT_MIN_SPEED_X100      (-70)  /* ADAS_MANAGER_MIN_SPEED_X100 */
#define ADAS_SLOT_MAX_SPEED_X100      (1300) /* ADAS_MANAGER_MAX_SPEED_X100 */
#define ADAS_SLOT_MAX_STEER_X10       (270)  /* ADAS_MANAGER_MAX_STEERING_X10 */

static sint16 TrajFd_ClampS16(sint32 value, sint32 minValue, sint32 maxValue)
{
    if (value < minValue) { return (sint16)minValue; }
    if (value > maxValue) { return (sint16)maxValue; }
    return (sint16)value;
}

void TrajFd_BuildAdasManagerSlotFrame(CanTrajectoryFdChannel *channel,
                                      const TrajFdControlParam *param,
                                      float32 vehicleSpeedMps,
                                      float32 vehicleHeadingRad,
                                      float32 tsSeconds,
                                      uint32 nowTick100us,
                                      uint16 canId,
                                      uint8 functionId,
                                      float32 *eyFiltPrev,
                                      uint8 *sequence,
                                      uint8 outBytes[8])
{
    TrajFdPacket packet;
    boolean hasPacket;
    boolean active;
    sint16 speedKphX100;
    sint16 steeringDegX10;
    uint8 segment;
    uint8 header;
    uint8 status;

    /* CanTrajectoryFd_GetLatestChannel() 자체가 TRAJ_FD_WATCHDOG_MS(500ms,
     * "10프레임 끊김") 초과 시 FALSE를 돌려주므로, 이 채널이 오래 조용하면
     * 여기서 이미 inactive로 처리된다 - Adas_Manager 쪽 슬롯 타임아웃
     * (ADAS_MANAGER_CAMERA/GPS_TIMEOUT_TICK100US, 동일하게 500ms)과 이중으로
     * 맞아떨어지게 일부러 같은 값으로 맞춰둠. */
    hasPacket = CanTrajectoryFd_GetLatestChannel(channel, &packet, nowTick100us);
    active = hasPacket && packet.enable;

    if (active)
    {
        float32 deltaRad = TrajFd_ComputeSteeringCurvatureFf(&packet, param, vehicleSpeedMps,
                                                              vehicleHeadingRad, tsSeconds,
                                                              eyFiltPrev);
        speedKphX100 = TrajFd_ClampS16(packet.vCmdKphX100, ADAS_SLOT_MIN_SPEED_X100,
                                       ADAS_SLOT_MAX_SPEED_X100);
        steeringDegX10 = TrajFd_ClampS16((sint32)(deltaRad * 572.9578f),
                                         -ADAS_SLOT_MAX_STEER_X10, ADAS_SLOT_MAX_STEER_X10);
        segment = packet.segment;
        status = ADAS_SLOT_STATUS_ACTIVE_GOOD;
    }
    else
    {
        speedKphX100 = 0;
        steeringDegX10 = 0;
        segment = 0u;
        status = ADAS_SLOT_STATUS_INACTIVE;
    }

    header = functionId; /* bit7 version=0(v1), bit5 atomic=0 (Camera/Gps 필수) */
    if (active)
    {
        header |= TRAJ_FD_HEADER_ENABLE_MASK;
        header |= TRAJ_FD_HEADER_LATERAL_VALID_MASK;
        header |= TRAJ_FD_HEADER_LONG_VALID_MASK;
    }

    outBytes[0] = header;
    outBytes[1] = (uint8)((uint16)speedKphX100 & 0xFFu);
    outBytes[2] = (uint8)(((uint16)speedKphX100 >> 8u) & 0xFFu);
    outBytes[3] = (uint8)((uint16)steeringDegX10 & 0xFFu);
    outBytes[4] = (uint8)(((uint16)steeringDegX10 >> 8u) & 0xFFu);
    outBytes[5] = segment;
    outBytes[6] = (uint8)((status << ADAS_SLOT_STATUS_SHIFT) |
                          (*sequence & ADAS_SLOT_SEQUENCE_MASK));
    outBytes[7] = TrajFd_CalculateCrc(canId, outBytes, 7u);

    *sequence = (uint8)((*sequence + 1u) & ADAS_SLOT_SEQUENCE_MASK);
}

#ifndef CAN_TRAJECTORY_FD_H_
#define CAN_TRAJECTORY_FD_H_

#include "Ifx_Types.h"
#include "IfxMultican_Can.h"

/*
 * TC275 CAN FD 궤적(trajectory) 수신 + Stanley/Curvature FF+FB 제어 v5.
 *
 * 정확한 페이로드 규격은 TRAJ_FD_PROTOCOL.md 참고. 요약(카메라/GPS 공통 레이아웃):
 *  - DLC 32, BRS on
 *  - c0..c3: 중심선 3차 다항식 y_C(x)=c0+c1 x+c2 x^2+c3 x^3 (전륜축 기준 권장)
 *  - vCmdKphX100, fitValidLenCm, header(version/enable/atomic/lateral_valid/
 *    long_valid/functionId), sequence, requestStatus, segment, CRC8(SAE-J1850)
 *
 * v5 소스 선택 규칙 (사용자 확정 사항, v4의 "카메라 우선" 설계를 대체함):
 *   - 카메라(0x320)/GPS(0x321) 사이에 **우선순위 없음** - 둘은 동등.
 *   - 상태는 "카메라 활성" / "GPS 활성" 딱 2개뿐 (VehicleManager 쪽에서
 *     보면 이 둘 중 하나만 켜져 있는 것으로 보임).
 *   - 지금 활성인 소스가 계속 정상 수신되는 한 그대로 유지 (다른 쪽이
 *     "더 좋아 보여도" 안 바꿈).
 *   - 활성 소스가 TRAJ_FD_FAILOVER_MISSED_FRAMES(기본 10) 프레임에
 *     해당하는 시간만큼 끊기면, 그 즉시 다른 소스로 전환 (다른 쪽이
 *     유효할 때만). 둘 다 끊기면 선택 실패.
 *   - 콜드스타트(둘 다 아직 한 번도 활성화 안 됨)에는 먼저 유효해지는
 *     쪽을 활성으로 삼음 (이때만 카메라를 아주 살짝 먼저 검사하는데,
 *     이건 "누가 더 낫다"가 아니라 그냥 동시에 유효해졌을 때 뭐라도
 *     골라야 해서 넣은 타이브레이커일 뿐임).
 *
 *   "N 프레임만큼의 시간"은 TRAJ_FD_NOMINAL_FRAME_PERIOD_MS(기본 50ms,
 *   즉 20Hz 가정) * TRAJ_FD_FAILOVER_MISSED_FRAMES 로 계산한
 *   TRAJ_FD_WATCHDOG_MS(기본 500ms) 하나로 통일함 - 워치독을 통과 못하면
 *   그게 곧 "N프레임 끊김"이 되도록 설계 (통과/실패 판정이 두 군데서
 *   따로 놀지 않게). 상위가 실제로 다른 주기로 보내면
 *   TRAJ_FD_NOMINAL_FRAME_PERIOD_MS를 그 값에 맞게 바꿀 것.
 *
 * 설계 근거(제어 법칙 자체): PDF("자율주행 로봇 차량 제어 설계 Ver1&2")의
 * controlStanley / controlCurvatureFF 코드는 ey/ePsi/kappaC를 매 호출마다
 * 그 순간의 v, psiVeh로 다시 계산한다. 그래서 상위는 다항식 계수만 보내고,
 * e_y/e_psi/kappa_C 및 최종 delta_cmd는 이 파일에서 TC275 자신의 실시간
 * 속도/헤딩으로 계산한다.
 */

#define TRAJ_FD_CAMERA_CAN_ID     (0x320u)
#define TRAJ_FD_GPS_CAN_ID        (0x321u)
#define TRAJ_FD_CAMERA_MSG_OBJ_ID (25u)  /* Can_SpeedTx.c의 classic-CAN RX 오브젝트(0~23)들 다음 번호 */
#define TRAJ_FD_GPS_MSG_OBJ_ID    (26u)
#define TRAJ_FD_DLC_BYTES         (32u)

/*
 * CAN FD 32바이트 프레임은 메인 메시지 오브젝트 하나로는 못 담는다 - MultiCAN+
 * 메시지 오브젝트는 원래 8바이트(DATAL/DATAH)까지만 갖고 있어서, 9바이트
 * 이상은 별도로 링크된 "bottom"(바이트 8~31, EDATA0~6) / "top"(바이트 32~63)
 * 오브젝트가 반드시 있어야 한다 (IfxMultican.c의 doReadLongFrame 참고 -
 * hwObj->FGPR.B.BOT/TOP로 찾아감). IfxMultican_Can_MsgObj_initConfig()가
 * 이 두 필드에 기본값 252/253을 깔아두는데, 채널별로 다시 안 정해주면
 * 카메라/GPS 두 채널이 똑같은 252/253을 동시에 잡으려다 충돌한다
 * (실제로 이것 때문에 TRAJ_FD 수신이 안 됐었음).
 *
 * 값 선택 이력: 처음엔 27~30(낮은 번호, 기존 0~24 오브젝트 바로 다음)으로
 * 했더니 무반응 정도가 아니라 버스 에러가 계속 쌓이는 더 심한 증상이
 * 나타남 - 252/253(iLLD 기본값)은 최소한 무반응이었을 뿐 버스 자체는
 * 안 깨졌던 것과 대조적. 252/253 자체는 무해하다는 게 증명된 셈이라,
 * 카메라는 원래 기본값(252/253) 그대로 두고 GPS만 다른 높은 번호로
 * 분리하는 쪽으로 되돌림 - 252/253 근처에서 낮은 번호대(0~31)와 겹치지
 * 않게 254/255 선택. */
#define TRAJ_FD_CAMERA_BOT_OBJ_ID (252u)
#define TRAJ_FD_CAMERA_TOP_OBJ_ID (253u)
#define TRAJ_FD_GPS_BOT_OBJ_ID    (254u)
#define TRAJ_FD_GPS_TOP_OBJ_ID    (255u)

/* "10프레임 끊기면 전환" 규칙 - 아래 두 값의 곱이 워치독 시간이 된다.
 * 상위 실제 송신 주기가 다르면 PERIOD_MS만 맞춰 바꾸면 됨. */
#define TRAJ_FD_NOMINAL_FRAME_PERIOD_MS (50u)  /* 20Hz 가정 */
#define TRAJ_FD_FAILOVER_MISSED_FRAMES  (10u)
#define TRAJ_FD_WATCHDOG_MS \
    (TRAJ_FD_NOMINAL_FRAME_PERIOD_MS * TRAJ_FD_FAILOVER_MISSED_FRAMES) /* 500ms */

/* CRC-8/SAE-J1850, Adas_Manager.c의 상수와 동일 (일부러 재정의해서
 * Can_TrajectoryFd.c가 Adas_Manager.c에 링크 의존성 없이 독립적으로 빌드되게 함) */
#define TRAJ_FD_CRC8_POLY   (0x1Du)
#define TRAJ_FD_CRC8_INIT   (0xFFu)
#define TRAJ_FD_CRC8_XOROUT (0xFFu)

#define TRAJ_FD_HEADER_VERSION_MASK       (0x80u)
#define TRAJ_FD_HEADER_ENABLE_MASK        (0x40u)
#define TRAJ_FD_HEADER_ATOMIC_MASK        (0x20u)
#define TRAJ_FD_HEADER_LATERAL_VALID_MASK (0x10u)
#define TRAJ_FD_HEADER_LONG_VALID_MASK    (0x08u)
#define TRAJ_FD_HEADER_FUNCTION_MASK      (0x07u)

typedef enum
{
    TrajFdSource_None = 0,
    TrajFdSource_Camera = 1,
    TrajFdSource_Gps = 2
} TrajFdSource;

typedef struct
{
    boolean enable;
    boolean lateralValid;
    boolean longValid;
    uint8   functionId;
    uint8   sequence;
    uint8   requestStatus;
    uint8   segment;
    float32 c0;
    float32 c1;
    float32 c2;
    float32 c3;
    sint16  vCmdKphX100;
    uint16  fitValidLenCm;
} TrajFdPacket;

typedef struct
{
    float32 previewTimeS;      /* T1 */
    float32 previewDistMinM;   /* xmin */
    float32 previewDistMaxM;   /* xmax */
    float32 wheelBaseM;        /* B, atan(B*kappa) FF에 씀 */
    float32 stanleyGainK;      /* Stage3용 k */
    float32 curvatureFfKy;     /* Stage4용 Ky */
    float32 curvatureFfKpsi;   /* Stage4용 Kpsi */
    float32 speedEpsilonMps;   /* eps, div-by-zero 방지 */
} TrajFdControlParam;

/* 카메라/GPS 공용 채널 상태. 소스마다 하나씩 인스턴스를 둔다. */
typedef struct
{
    IfxMultican_Can_MsgObj rxMsgObj;
    TrajFdPacket latestPacket;
    boolean hasPacket;
    uint32  lastRxTick100us;
    uint8   lastSequence;
    boolean lastSequenceValid;
} CanTrajectoryFdChannel;

extern CanTrajectoryFdChannel g_canTrajFdCamera;
extern CanTrajectoryFdChannel g_canTrajFdGps;

/* node: Can_SpeedTx.c에서 이미 FD로 초기화된 g_canNode를 그대로 전달.
 * botObjId/topObjId: 32바이트 확장 데이터를 담을 링크 오브젝트 (본문 주석 참고,
 * 채널마다 서로 겹치지 않는 값이어야 함). */
void CanTrajectoryFd_InitChannel(CanTrajectoryFdChannel *channel,
                                 IfxMultican_Can_Node *node,
                                 uint16 canId,
                                 uint32 msgObjId,
                                 uint32 botObjId,
                                 uint32 topObjId);

/* 편의 함수: 카메라/GPS 두 채널을 한 번에 초기화 */
void CanTrajectoryFd_InitAll(IfxMultican_Can_Node *node);

/* 폴링 방식: 매 제어 주기(예: 5ms)마다 채널별로 호출. 새 프레임이 왔으면 TRUE. */
boolean CanTrajectoryFd_PollChannel(CanTrajectoryFdChannel *channel, uint32 nowTick100us);

/* 마지막으로 CRC/시퀀스 검증에 성공한 패킷을 가져온다. TRAJ_FD_WATCHDOG_MS
 * (="10프레임 끊김") 초과 시 FALSE. */
boolean CanTrajectoryFd_GetLatestChannel(CanTrajectoryFdChannel *channel,
                                         TrajFdPacket *outPacket,
                                         uint32 nowTick100us);

/* 카메라/GPS 사이 우선순위 없이, "지금 활성인 소스가 끊기기 전까진 그대로
 * 유지, 끊기면(TRAJ_FD_WATCHDOG_MS) 다른 쪽으로 전환" 규칙으로 선택한다.
 * 내부에 활성 소스 상태(2-state: Camera/Gps)를 static으로 들고 있음.
 * outPacket에 선택된 패킷을 채우고 어느 소스를 골랐는지 반환.
 * 둘 다 무효면 TrajFdSource_None. */
TrajFdSource TrajFd_SelectSource(CanTrajectoryFdChannel *cameraChannel,
                                 CanTrajectoryFdChannel *gpsChannel,
                                 uint32 nowTick100us,
                                 TrajFdPacket *outPacket);

/* PDF 4단계 Curvature FF+FB. vehicleSpeedMps/vehicleHeadingRad/eyFiltPrev는
 * 호출자(VehicleManager)가 자기 상태에서 넘겨준다. 반환값은 제한 전
 * delta_cmd(rad) - saturation/rate limit은 기존 VehicleManager 로직에
 * 맞춰 호출자가 적용할 것을 권장. */
float32 TrajFd_ComputeSteeringCurvatureFf(const TrajFdPacket *pkt,
                                          const TrajFdControlParam *param,
                                          float32 vehicleSpeedMps,
                                          float32 vehicleHeadingRad,
                                          float32 tsSeconds,
                                          float32 *eyFiltPrev);

/* PDF 3단계 Stanley (더 단순한 대안, FF 없이). */
float32 TrajFd_ComputeSteeringStanley(const TrajFdPacket *pkt,
                                      const TrajFdControlParam *param,
                                      float32 vehicleSpeedMps,
                                      float32 vehicleHeadingRad);

/*
 * v7: 카메라/GPS를 Adas_Manager의 own 6슬롯 우선순위(Mrm>Collision>Parking>
 * Traffic>Camera>Gps)에 각자 별도 슬롯/CAN ID(0x320/0x321)로 직접 꽂는다.
 * 둘 중 무엇을 쓸지 고르는 건 더 이상 이 파일의 일이 아니고, Adas_Manager.c의
 * own freshness/priority 메커니즘(AdasManager_SelectCameraOrGps)이 담당한다.
 *
 * 이 함수는 채널 하나(카메라 또는 GPS)만 보고 Curvature FF+FB 계산까지 끝낸
 * 뒤, Adas_Manager.c의 AdasManager_OnCanFrame()이 그대로 받아들일 수 있는
 * 8바이트 슬롯 포맷(Adas_Manager.h 헤더 레이아웃, CRC-8 포함)으로 채워서
 * outBytes에 돌려준다. 실제 CAN 버스에 나가는 게 아니라, 매 5ms 루프에서
 * 카메라/GPS 각각에 대해 한 번씩 호출하고, 그 결과를 각자의 CAN ID로
 * AdasManager_OnCanFrame(canId, outBytes, 8, FALSE, nowTick100us)에 함수
 * 호출로 "주입"하는 용도. eyFiltPrev(필터 상태)와 sequence(5비트 카운터)는
 * 채널별로 독립적이어야 하므로 호출자가 채널마다 별도의 static 변수를 두고
 * 포인터로 넘겨준다. */
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
                                      uint8 outBytes[8]);

#endif /* CAN_TRAJECTORY_FD_H_ */

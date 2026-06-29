# 8. 향후 개선 방향

## 8.1 단기 과제 (0 ~ 6개월)

### 8.1.1 브라켓 제작 및 초기 Calibration

- [ ] Type-A 브라켓 (전후방 공용) CAD 도면 완성 및 CNC 발주
- [ ] Type-B 브라켓 (측방) CAD 도면 완성 및 CNC 발주
- [ ] 전차 장착 후 초기 Intrinsic / Extrinsic Calibration 수행
- [ ] Calibration 결과 YAML 저장 및 Git 커밋
- [ ] RViz2 기반 투영 결과 시각적 검증

### 8.1.2 Calibration 스크립트 자동화

- [ ] 단일 명령어 실행으로 전체 Calibration 순서를 안내하는 스크립트 작성
- [ ] Reprojection Error 자동 계산 및 보고서 출력 스크립트

```bash
# 목표 인터페이스 (예시)
ros2 run sensor_fusion_calibration full_calibration.py \
  --output calibration/tf/sensor_extrinsic_$(date +%Y%m%d).yaml
```

### 8.1.3 Calibration 품질 대시보드

- [ ] RViz2 또는 웹 기반 실시간 Reprojection Error 시각화
- [ ] 임계값 초과 시 경고 알림 (ROS2 Diagnostics 연동)

## 8.2 중기 과제 (6 ~ 18개월)

### 8.2.1 온라인 Calibration (Online Extrinsic Calibration)

정적 Calibration 보드 없이 주행 중 자동으로 Extrinsic을 추정·갱신하는 방법.

```
주행 중 특징점 추출 (도로 마킹, 고정 구조물)
        │
   Camera + LiDAR 특징 매칭
        │
   Optimization (비선형 최소자승법)
        │
   Extrinsic 점진적 갱신
```

활용 가능 알고리즘:
- `direct_lidar_camera_calibration`
- `targetless_calibration`
- `LCCNet`

### 8.2.2 브라켓 진동 특성 측정 및 FEA 검증

- 실차 주행 중 IMU 데이터로 브라켓 진동 특성 측정
- FEA(유한요소해석)와 실측 비교
- 고유진동수 < 50 Hz 구간 확인 시 리브 추가 또는 댐퍼 삽입

### 8.2.3 열화상 카메라 추가 검토

야간 보행자 탐지 강화를 위해 열화상 카메라 추가 가능성 검토.

```
현재: OAK-D Pro PoE × 6
검토: + FLIR Lepton 또는 FLIR Boson × 2 (전후방)
```

브라켓 Type-A에 열화상 카메라 추가 마운트홀을 사전에 포함하는 것을 권장.

### 8.2.4 LiDAR-IMU 타이트 결합 Calibration

Livox Mid-360 + IMU 간 정밀 Extrinsic을 구하여 LIO(LiDAR-Inertial Odometry) 정확도 향상.

```bash
# LI-Init 또는 Kalibr-LiDAR-IMU 사용
ros2 launch lidar_imu_calibration calibrate.launch.py
```

## 8.3 장기 과제 (18개월 이후)

### 8.3.1 전용 Calibration 지그 제작

반복 Calibration을 위한 차량 전용 Calibration 지그 설계.

```
[지그 구성]

┌─────────────────────────────┐
│  차량 전방 2 m 기준 위치     │
│  ┌───────────────────────┐  │
│  │  Checkerboard 보드    │  │  ← 정확한 위치에 고정
│  └───────────────────────┘  │
└─────────────────────────────┘
```

지그를 사용하면 매번 보드 위치를 측정할 필요 없이 재현 가능한 Calibration 가능.

### 8.3.2 공장 Calibration 절차 정립 (양산 이전 준비)

연구 결과를 바탕으로 양산 수준의 자동화 Calibration 절차 정립.

- 자동 Calibration 타겟 인식 + 포즈 추정
- ECU/RCM 저장 구조 설계
- Calibration 주기 자동 알림 시스템

### 8.3.3 다중 센서 동시 Calibration

현재: 센서쌍별 순차 Calibration  
목표: 전체 센서를 단일 최적화 문제로 동시에 Calibration

```
Batch Optimization:
  min Σ (Reprojection Error_ij)^2
  subject to:
    T_i: 각 센서의 Extrinsic (6 DOF)
    K_i: 각 카메라의 Intrinsic
    장면 점 P_j: 3D 특징점 위치
```

구현 후보: `Ceres Solver` 기반 Bundle Adjustment

### 8.3.4 Calibration 데이터셋 공개

연구 결과의 사회 환원 및 학술 기여를 위해 Calibration 데이터셋 공개 검토.

- 센서 구성, 브라켓 설계, 제작 도면 공개
- Calibration 원시 데이터 (Bag 파일) 공개
- 결과 YAML 및 평가 스크립트 GitHub 공개

## 8.4 잠재적 개선 아이디어

| 아이디어 | 효과 | 난이도 |
|---------|------|-------|
| 전동식 Pitch/Yaw 조정 (서보 모터) | 원격 자동 재조정 가능 | 높음 |
| QR 코드 기반 자동 브라켓 ID 인식 | 장착 오류 방지 | 낮음 |
| 카메라 온도 모니터링 | 열팽창 보정 자동화 | 중간 |
| 위상 배열 초음파로 브라켓 피로 감지 | 장기 구조 건전성 모니터링 | 높음 |
| AR 가이드 (태블릿 + ARCore) | 초보자 정렬 보조 | 중간 |

## 8.5 일정 요약

```
2026 Q3       : 브라켓 CAD 완성 + CNC 발주
2026 Q3~Q4    : 초기 장착 + 전체 Calibration 수행
2026 Q4       : 자동화 스크립트 + 품질 대시보드
2027 Q1~Q2    : Online Calibration 실험
2027 Q3       : 전용 Calibration 지그 제작
2027 Q4~      : 다중 센서 동시 Calibration 연구
2028~         : 양산 이전 절차 정립 + 데이터셋 공개
```

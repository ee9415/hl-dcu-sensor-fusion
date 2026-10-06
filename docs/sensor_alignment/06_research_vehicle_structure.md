# 6. 연구용 차량 권장 구조

## 6.1 Design Goal

센서 교체와 실험 변경이 잦은 연구 차량에서 장착 위치를 추적하고 반복 조립 후
calibration을 재현할 수 있는 모듈형 구조를 목표로 한다. 아래 내용은 권장안이며
실차 설계 승인 전 확정값이 아니다.

## 6.2 Camera Zones

```text
                  차량 전방
                     ↑
        CAM02      CAM01      CAM03
      좌측 후면←    전방→    →우측 후면

        CAM04                 CAM05
      좌측 전면→             ←우측 전면

                   CAM06
                    후방↓
```

- CAM01/CAM06은 차량 중심선에 대한 정렬 재현성이 중요하다.
- CAM02/CAM04는 좌측 측면을 상호 보완한다.
- CAM03/CAM05는 우측 측면을 상호 보완한다.
- 장착 높이와 각도는 실제 FOV 및 차체 가림을 측정한 뒤 확정한다.

## 6.3 Modular Mounting

- 차체 기준면과 좌표를 기록할 수 있는 공통 rail/cross-member를 검토한다.
- camera adapter와 차체 bracket을 분리해 센서 교체 영향을 제한한다.
- 장착 위치마다 고유 ID를 부여하고 MX ID와 매핑한다.
- 위치결정핀 또는 동등한 반복 위치결정 구조를 검토한다.

## 6.4 LiDAR, GNSS, IMU Candidates

- LiDAR: 차체 가림을 줄이고 360° 시야를 확보할 수 있는 상부 위치 후보
- GNSS: 상공 시야와 안테나 ground plane 요구를 만족하는 위치 후보
- IMU: 차체 좌표축과 정렬 가능하고 진동/열원의 영향을 평가할 수 있는 위치 후보

정확한 위치와 이격거리는 벤더 요구조건 및 현장 시험 후 기록한다.

## 6.5 Wiring and Serviceability

- PoE, 센서 전원, 데이터 배선을 구분하고 양단에 camera/sensor ID를 표시한다.
- 커넥터 접근, 방수 처리, 굽힘 반경, strain relief를 확보한다.
- 센서 탈착 시 다른 센서의 위치가 바뀌지 않는 구조를 우선한다.
- switch, DCU, 저장장치의 대역폭과 열 조건을 함께 검토한다.

## 6.6 Calibration Workspace

- 각 카메라와 LiDAR가 calibration target을 동시에 볼 수 있는 작업 공간을 확보한다.
- 측면 카메라는 좌우 각각의 target 배치 공간을 확인한다.
- calibration 당시 차량 자세, target 위치, 조명과 sensor setting을 기록한다.

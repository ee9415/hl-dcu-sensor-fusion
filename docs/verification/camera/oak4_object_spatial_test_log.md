# OAK4 Object Spatial Measurement Test Log

- 작성일: 2026-10-15
- 대상: `apps/yolopv2_standalone`
- 앱 버전: `0.4.0`
- 상태: 합성 입력 및 실장비 런타임 검증 통과, 거리 정확도 검증 대기

## 구현 범위

- YOLOPv2 원본 RGB 좌표 객체 박스와 `DepthAligned` 결합
- 객체 중앙 50% ROI 사용
- 유효 Depth 범위 300~50,000 mm
- 유효 픽셀 최소 20개 및 ROI 대비 최소 10%
- 10th~90th percentile 범위의 중앙값으로 대표 Z 계산
- 정렬 Depth의 변환 후 intrinsic으로 CAM_A optical X/Y/Z(mm) 계산
- 유효한 객체만 `SpatialDetections` 메시지에 포함
- 후처리 영상에 `Z=...m` 또는 `Z=?` 표시

## 자동 검증

실행 명령:

```powershell
$env:PYTHONPATH='apps\yolopv2_standalone'
& 'C:\Users\qstech_gn\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' -m unittest discover -s apps\yolopv2_standalone\tests -v
```

결과: 4 tests, PASS

| 항목 | 결과 |
| --- | --- |
| 일정한 12 m Depth의 정면 객체 Z | PASS |
| 우측 객체의 X 좌표 계산 | PASS |
| 0 및 원거리 이상치에 대한 중앙값 안정성 | PASS |
| 유효 Depth 20 pixel 미만 무효 처리 | PASS |

Python `py_compile` 검사도 통과했다.

## 실장비 런타임 검증

- 장비: OAK4-D R9 `1142227768`, IP `192.168.0.4`
- 실행 단계: `full`
- 결과: 파이프라인 생성 및 실행 성공
- 안정 구간 평균 FPS: 약 9.79
- 확인 시점 후처리 시간: 약 12.64 ms
- 확인 프레임: 객체 1개, 유효 Depth 객체 1개
- 사용자 확인 화면: `YOLOPv2Result`에 `Z=1.3m`, `DepthAligned`에 해당 객체의
  공간 검출 박스 표시

## 미검증 항목

- 거리 구간별 절대 오차와 객체 경계의 배경 혼입률
- 장시간 실행 시 FPS와 지연 안정성
- 차량 설치 자세를 반영한 CAM_A optical 좌표에서 차량 좌표계로의 변환

로그인 토큰을 재인증한 뒤 실장비 출력까지 확인했다. 다음 검증에서는 CAM_A 렌즈
기준 1 m, 2 m, 3 m 표적을 이용해 표시 Z와 실측값의 오차를 기록한다.

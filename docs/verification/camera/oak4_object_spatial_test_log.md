# OAK4 Object Spatial Measurement Test Log

- 작성일: 2026-10-15
- 대상: `apps/yolopv2_standalone`
- 앱 버전: `0.4.0`
- 상태: 합성 입력 검증 통과, 실장비 검증 대기

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

## 미검증 항목

- 실제 OAK4-D R9의 `SpatialDetections` 수신 및 영상 거리 문자 표시
- 거리 구간별 절대 오차와 객체 경계의 배경 혼입률
- YOLOPv2, Stereo Depth, 거리 계산 동시 실행 시 실제 FPS와 지연
- 차량 설치 자세를 반영한 CAM_A optical 좌표에서 차량 좌표계로의 변환

장비 `1142227768`은 확인했으나 로그인 토큰 만료로 카메라 비밀번호 재인증이
필요하다. 비밀번호를 파일이나 채팅에 기록하지 않고 장비에 직접 인증한 뒤 위
항목을 검증한다.

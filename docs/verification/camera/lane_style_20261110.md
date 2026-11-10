# 차선 표시 굵기·색상 변경 (2026-11-10)

## 변경

- RGB/TOP 공통 표시 함수의 관측 분홍 실선 두께를 3px에서 5px로 변경했다.
- 미검출 유지·변형 구간 및 가상 연장 구간은 노란 점선에서 분홍 점선(BGR 255,0,255)으로 변경하고 점선 브러시 반경을 1px에서 2px로 늘렸다.
- 점선 간격과 추적 좌표·소실 유지·속도 설정·인지 계산은 변경하지 않았다.

## 검증

- WSL Ubuntu-22.04 격리 OpenCV 환경: lane_overlay 13개, ego_lane_quadratic 26개, contour_temporal 24개, 총 63개 통과. 실선·점선 구분, 유지선 표시와 TOP 유효 영역 검사를 포함한다.
- 앱 코드 build 및 구문 검사 통과. artifacts/lane_style_20261110/code.zip, 82,303bytes. 코드 버전 0.33.10-code3. 아래 실장치 적용 검증 완료.
- git diff --check 통과. 다른 작업의 기존 수정은 보존했다.

## 장치 적용 상태

사용자의 명시적 적용 요청 후 CAM01 기존 패키지 0.33.10(UUID 61fbe0c1-ad26-42dc-93c4-a8d2b5191fce)을 실행하고 코드 ZIP 82,303bytes만 배포했다. 전체 패키지 재설치 없음.

- 활성 release: `dc27d8b55acae22d4aa39eebc044547f72c802d06de47844db17629e92a410cb`. pending/last_failure 없음, 이전 code2 복구 버전 보존.
- runtime fingerprint 유지: `2d40184c43cd90a340e319791fad2d459404ed5eda9b6800725ca7080f331084`.
- 새 코드의 full 프레임 수신·배포 확정 통과, input_source=camera. 활성 lane_overlay.py SHA256이 코드 manifest와 일치함을 장치에서 확인했다.
- 근거: artifacts/lane_style_20261110/deployed.json, remote_identity.txt. PC 멀티뷰의 실제 화면 확인은 아직 수행하지 않았다. 부팅 자동 시작은 비활성 유지.

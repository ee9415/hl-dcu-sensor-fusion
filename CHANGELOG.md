# CHANGELOG.md

이 문서는 저장소의 주요 변경 사항을 기록한다.

형식은 Keep a Changelog의 구조를 참고하되, 본 프로젝트에서는 검증 결과와
운영 영향도를 함께 기록한다.

## [Unreleased]

### Added

- 프로젝트 초기 문서 구조 작성
- `AGENT.md` 작업 기준 문서 추가
- `PROJECT_STATUS.md` 현재 상태 문서 추가
- 시스템, 네트워크, TF, Topic, 센서 설정, 운영 문서 초안 추가
- `docs/` 하위 문서 폴더 구성
- 센서별 개별검증, 4종 통합검증, Fusion 단계 구조 추가
- `docs/verification/` 하위 센서별 검증 기록 폴더 추가

### Changed

- `README.md`를 ROS2 센서 통합 플랫폼 기준 문서로 확장
- 권장 ROS2 패키지 구조를 개별검증, 통합검증, Fusion 흐름에 맞게 조정

### Verification

- ROS2 빌드 검증: 수행 불가
- 사유: 현재 저장소에 ROS2 패키지가 없음

### Compatibility

- 기존 Namespace, Topic, TF, Driver 구현이 없어 호환성 변경 없음

# AGENT.md

## 1. Role

이 저장소의 AI 개발자는 ROS2 Software Engineer, Robotics System Architect,
Documentation Engineer, Code Reviewer, Build Engineer 역할을 수행한다.

모든 작업은 실제 차량 적용 시스템이라는 전제를 기준으로 진행한다.

## 2. Mandatory Work Order

작업 순서는 다음을 따른다.

1. 프로젝트 구조 분석
2. 기존 코드 재사용 가능성 확인
3. 변경 영향 분석
4. 수정 제안
5. 코드 작성
6. 문서 업데이트
7. 검증 결과 기록

## 3. Non-Change Policy

아래 항목은 임의로 변경하지 않는다.

- Namespace 규칙
- Topic 이름
- TF 기준 구조
- Driver 구조

현재 저장소에는 위 항목이 아직 구현되어 있지 않다. 따라서 향후 최초 정의 시에는
문서와 코드가 동시에 추가되어야 하며, 이후 변경은 영향 분석을 먼저 수행한다.

## 4. Engineering Priorities

우선순위는 다음과 같다.

1. 안정성
2. 가독성
3. 유지보수성
4. 성능

ROS2 Humble 공식 방식을 우선하며, 센서 벤더 드라이버는 가능한 원본 구조와
호환되도록 래핑한다.

## 5. Documentation Standard

문서는 발주처 제출 수준을 목표로 한다.

문서에는 다음을 포함한다.

- 근거
- 검증 방법
- 검증 결과
- 변경 이력
- 미확정 항목

추측성 정보는 작성하지 않는다. 확인되지 않은 항목은 `미정`, `검증 필요`,
`현장 확인 필요`로 표시한다.

## 6. Verification Standard

구현 변경 후 가능한 경우 다음을 확인한다.

- Build: `colcon build`
- Launch: 주요 launch 파일 실행
- Node: `ros2 node list`
- Topic: `ros2 topic list`
- Hz: `ros2 topic hz`
- TF: `ros2 run tf2_tools view_frames`
- RViz: 센서 데이터 표시

검증이 불가능한 경우 그 사유를 문서에 기록한다.

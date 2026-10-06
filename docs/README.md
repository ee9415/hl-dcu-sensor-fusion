# docs

이 폴더는 제출용 루트 문서를 보조하는 상세 자료를 관리한다.

루트 문서는 발주처 제출과 운영 기준을 위한 정본으로 유지하고, `docs/` 하위
문서는 세부 설계, 검증 로그, 현장 운영 기록을 보관한다.

## Structure

```text
docs/
├── README.md
├── architecture/
├── operation/
├── sensor_alignment/          ← 센서 정렬 및 카메라 브라켓 설계
│   ├── README.md
│   ├── 01_alignment_overview.md
│   ├── 02_mechanical_design.md
│   ├── 03_camera_bracket.md
│   ├── 04_calibration_procedure.md
│   ├── 05_software_calibration.md
│   ├── 06_research_vehicle_structure.md
│   ├── 07_vs_production.md
│   └── 08_future_work.md
└── verification/
    ├── camera/
    ├── lidar/
    ├── gnss/
    ├── imu/
    ├── integration/
    └── fusion/
```

## Policy

- 확정된 기준 문서는 루트 문서에 반영한다.
- 실험, 검증 로그, 현장 메모는 `docs/` 하위에 기록한다.
- 구현 변경 시 관련 루트 문서와 상세 문서를 함께 갱신한다.

## Source of Truth

- 구현/검증 상태는 루트 `PROJECT_STATUS.md`를 우선한다.
- 실제 topic과 TF 구현 여부는 `TOPIC_LIST.md`, `TF_TREE.md`를 우선한다.
- 6카메라 역할과 배치는 `camera_use_case.md`를 우선한다.
- `sensor_alignment/`의 수치와 구조는 현장 검증 전까지 설계안이다.

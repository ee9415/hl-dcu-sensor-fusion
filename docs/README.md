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

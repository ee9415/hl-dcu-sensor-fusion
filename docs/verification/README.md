# Verification Notes

빌드, 실행, 센서 데이터, RViz 검증 결과를 보관한다.

예정 항목:

- Build log
- Launch log
- Topic Hz result
- TF tree result
- RViz screenshot

## Structure

```text
verification/
├── camera/
├── lidar/
├── gnss/
├── imu/
├── integration/
└── fusion/
```

센서별 개별검증 결과를 먼저 기록하고, 4종 통합검증과 fusion 검증 결과는 별도
폴더에 기록한다.

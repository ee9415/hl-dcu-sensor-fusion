# Camera Verification

OAK-D Pro PoE 카메라 개별검증 결과를 기록한다.

검증 항목:

- Device discovery
- Image topic
- Camera info topic
- Topic Hz
- Frame ID
- RViz display

검증 기록:

- [OAK-D Pro PoE single camera test log](oak_poe_single_test_log.md)
- [YOLOP 6-camera Web UI test log](yolop_6cam_web_ui_test_log.md)
- [OAK4 YOLOPv2 baseline migration log](oak4_yolopv2_baseline_migration_log.md)
- [OAK4 RGB-aligned depth test log](oak4_rgb_aligned_depth_test_log.md)
- [OAK4 Windows multiview UI test log](oak4_multiview_ui_test_log.md)
- [OAK4 object spatial measurement test log](oak4_object_spatial_test_log.md)
- [OAK4 virtual centerline and BEV test log](oak4_centerline_bev_test_log.md)
- [OAK4 runtime output and resource snapshot](oak4_runtime_resource_snapshot.md)

첫 번째 기록은 ROS2 단일 카메라 검증이고, 두 번째 기록은 비-ROS2 6카메라 Web UI
검증이다. 세 번째 기록은 OAK4 기준 코드 이관의 정적 검증이며, 네 번째 기록은
OAK4 RGB 정렬 Depth 구현과 단계별 검증 상태이다. 다섯 번째 기록은 Windows
멀티뷰 UI 검증이다. 여섯 번째 기록은 객체별 거리와 CAM_A XYZ의 합성 입력
검증이며 실장비 정확도 검증을 대체하지 않는다. 이 결과들을 6카메라 ROS2
bringup 완료로 해석하지 않는다. 일곱 번째 기록은 개발용 IPM 기반 가상 중심선과
BEV 출력의 실장비 실행 검증이며 차량 좌표 보정을 완료했다는 의미는 아니다.
여덟 번째 기록은 현재 `full` 앱의 영상·토픽 출력 규격과 CPU, 메모리, 전력,
온도 및 저장공간 표본이다. 장시간 부하 시험 결과로 해석하지 않는다.

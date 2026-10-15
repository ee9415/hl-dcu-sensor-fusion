# OAK4 RGB 정렬 Depth 검증 기록

- 검증일: 2026-10-15
- 대상 장비: OAK4-D R9, serial `1142227768`
- 연결 주소: `192.168.0.4`
- 장비 OS: Luxonis OS RVC4 1.41.0
- 앱 DepthAI: 3.8.0
- 앱 버전: 0.3.0

## 1. 구현 범위

```text
CAM_B 640x400 ─┐
               ├─ StereoDepth ─ Depth(mm) ─ ImageAlign ─ DepthAligned 1920x1080
CAM_C 640x400 ─┘                              ↑
                                              CAM_A 1920x1080 undistorted
```

- StereoDepth preset: `ROBOTICS`
- Left-right consistency check: 활성화
- Subpixel disparity: 활성화
- 실행 단계: `PIPELINE_STAGE=depth|full`
- `depth` 단계에서는 YOLOPv2 NN을 생성하지 않는다.

RVC4에서는 DepthAI v3 공식 예제와 동일하게 `ImageAlign` 노드를 이용해 stereo
depth를 CAM_A 영상 좌표계로 변환한다.

참고: <https://docs.luxonis.com/software-v3/depthai/examples/image_align/depth_align>

## 2. 검증 결과

| 항목 | 결과 | 근거 |
| --- | --- | --- |
| Python 문법 검사 | 통과 | 관련 Python 6개 파일 `py_compile` 성공 |
| Depth 파이프라인 구성 검사 | 통과 | CAM_B/C, StereoDepth, ImageAlign 및 DepthAligned 연결 확인 |
| OAK 장비 발견 | 통과 | `oakctl device list`에서 대상 장비와 OS 확인 |
| `PIPELINE_STAGE=depth` 장비 실행 | 검증 필요 | 로그인 토큰 만료로 비밀번호 재인증 필요 |
| DepthAligned 크기·단위 | 장비 검증 필요 | 설계값 `1920x1080`, mm; 실측 미완료 |
| 정렬 오차·유효 거리·FPS | 장비 검증 필요 | 실제 장면과 표적을 이용한 측정 필요 |

## 3. 사용자 재인증 후 실행 절차

비밀번호는 대화, 파일 또는 자동화 인자로 전달하지 않고 사용자가 직접 PowerShell
프롬프트에 입력한다.

```powershell
& 'C:\Program Files\oakctl\bin\oakctl.exe' app run `
  .\apps\yolopv2_standalone --detach --device 1142227768 `
  --env PIPELINE_STAGE=depth

& 'C:\Program Files\oakctl\bin\oakctl.exe' app logs 000 `
  --tail 100 --no-follow --device 1142227768

& 'C:\Program Files\oakctl\bin\oakctl.exe' inspect topics `
  --device 1142227768
```

예상 토픽은 H.264 `Video`와 CAM_A 기준 16-bit depth `DepthAligned`이다. Visualizer로
동일 물체의 RGB 경계와 Depth 경계를 비교하고, 로그에 firmware crash,
`X_LINK_ERROR`, pipeline build 오류가 없는지 확인한다.

## 4. 판정

RGB 정렬 Depth 파이프라인 구현과 정적 검증은 완료했다. 실제 장비 성공 판정은
재인증 후 `DepthAligned` 출력, 정렬 상태, 10 FPS 및 오류 로그를 확인한 뒤에만
내린다. 객체별 거리·XYZ 계산은 다음 단계 범위이다.

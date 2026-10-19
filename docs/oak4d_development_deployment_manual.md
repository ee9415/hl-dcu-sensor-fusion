# OAK4D Pro 개발 빌드 배포 매뉴얼

작성일: 2026-10-19 | 대상: 큐센텍 CAM01 개발 담당자 | 기준: 앱 소스 0.14.1

이 매뉴얼은 현재 프로젝트를 직접 수정하고 OAK4D Pro에서 시험한 뒤 설치용
`.oakapp` 파일로 배포하는 절차이다. 기본 명령은 **Windows CMD**에서 실행한다.
명령을 한 줄씩 실행하고 각 단계의 확인 결과를 본 뒤 다음 단계로 진행한다.
차량 제어용 승인 절차가 아니라 카메라 인식 프로그램의 개발 절차이다.

## 1 실행 위치와 빌드의 의미

| 위치 | 담당 기능 |
|---|---|
| Windows PC | 소스 편집, Git 관리, oakctl로 인증·배포·상태 조회 |
| WSL2 Ubuntu 22.04 | Python 멀티뷰, 영상 수신·녹화, CSV 저장, 계산 로직 테스트 |
| OAK4D Pro 내부 Linux | RGB·StereoDepth·YOLOPv2·중심선 처리, 영상·데이터 송신 |

현재 앱은 Python 프로그램이다. C/C++ 실행 파일을 직접 컴파일하는 과정과 다르다.
`app run`은 소스와 실행 환경을 장비에 준비하여 개발용으로 실행한다.
`app build`는 코드·모델·실행 환경을 설치 가능한 `.oakapp` 파일로 묶는다.
`app install`은 그 파일을 장비에 설치한다. AI 모델의 학습·RVC4 변환은 별도 과정이다.
중심선 계산이나 Depth 색상만 수정할 때는 AI 모델을 다시 변환하지 않는다.

## 2 준비 사항

- OAK4D Pro 전원 및 PC와의 네트워크 연결. 현재 장비 IP는 `192.168.0.4`이다.
- Windows oakctl: `C:\Program Files\oakctl\bin\oakctl.exe`.
- WSL 배포판: `Ubuntu-22.04`. 현재 PC에는 Python·Tkinter·OpenCV·Pillow 실행 환경이 준비돼 있다.
- 소스 편집기: VS Code 등. UTF-8로 저장한다.
- 최초 빌드나 의존성 변경 시 이미지 저장소·Ubuntu/Python 패키지 저장소에 접근할 인터넷이 필요할 수 있다.
- 시험 중 재배포·빌드·앱 설치는 카메라 영상과 로깅을 끊을 수 있다. 녹화 중이면 먼저 저장한다.
- 비밀번호는 인증 프롬프트에 입력한다. 소스·매뉴얼·명령줄 인자에 실제 비밀번호를 남기지 않는다.

### CMD 작업 경로 설정

새 CMD 창을 열 때마다 아래를 실행한다. 이 변수들은 현재 CMD 창에만 적용된다.

```bat
set "REPO=D:\git_hub\hl-dcu-sensor-fusion"
set "OAKCTL=C:\Program Files\oakctl\bin\oakctl.exe"
set "OAKAPP=%REPO%\apps\yolopv2_standalone"
set "OAKIP=192.168.0.4"
set "APPID=00000000-0000-0000-0000-000000000000"
cd /d "%REPO%"
```

APPID의 전체 0 값은 **개발 모드 앱** 전용이다. 설치 패키지 앱은 목록에 나온 ID를 사용한다.
PowerShell의 `&`나 `$변수` 구문을 위 CMD 명령과 섞지 않는다.

### 준비 상태 확인

```bat
"%OAKCTL%" --version
wsl.exe -l -v
git status --short
dir "%OAKAPP%\models\yolopv2_320.rvc4.tar.xz"
"%OAKCTL%" app list -d %OAKIP%
```

확인 기준: oakctl 버전 출력, Ubuntu-22.04 존재, 모델 파일 존재, 장비 앱 목록 조회 성공.
인증이 만료되면 `Camera password` 프롬프트에 직접 입력한다.
`git status`에 기존 변경이 있으면 덮어쓰지 말고 담당자와 수정 범위를 구분한다.

**모델 파일은 Git 제외 대상이다.** 다른 PC에서 저장소를 clone해도 자동으로 생기지 않는다.
현재 PC의 검증된 `yolopv2_320.rvc4.tar.xz`를 같은 위치로 별도 복사·보관해야 한다.
이 파일은 단순한 학습 가중치가 아니라 현재 입력·출력 구조에 맞춘 RVC4용 모델 아카이브다.

## 3 수정할 파일 찾기

카메라 앱 루트: `D:\git_hub\hl-dcu-sensor-fusion\apps\yolopv2_standalone`

| 파일 | 수정 목적 |
|---|---|
| main.py | RGB/스테레오/NN/인코더/전송 노드 연결과 파이프라인 설정 |
| yolopv2_node.py | NN 출력 해석, 객체 거리, 중심선 호출, 결과 합성, 구조화 데이터 |
| postprocess.py | 객체 박스·마스크·Depth 기반 XYZ 계산 |
| centerline.py 및 관련 모듈 | 중심선 추정·연속성·변화 제한 |
| depth_color.py | Depth 표시 색상 범위 |
| depth_preview.py | 정렬 Depth의 표시용 영상 생성 |
| telemetry_server.py | TCP 8557 프레임 데이터 출력 |
| stream_output.py, rtsp_server.py | H.264 RTSP 전송 |
| requirements.txt | 카메라 내부 Python 의존성 |
| oakapp.toml | 앱 식별자·버전·기본 이미지·설치/빌드/실행 명령 |
| backend-run.sh | `python3.12 -u /app/main.py` 시작 |
| tests/ | 계산 로직 회귀 테스트 |

PC 파일은 별도이다.

| PC 파일 | 역할 |
|---|---|
| tools/oak_recorder.py | 멀티뷰·영상 연결·녹화 버튼 |
| tools/frame_csv_logger.py | frames.csv·objects.csv 기록 |
| tools/oak_recorder_requirements.txt | PC용 의존성 목록 |

PC 화면·CSV 저장 로직만 바꾸면 PC 프로그램만 재시작한다.
카메라 처리 내용을 바꾸면 카메라 앱을 재배포한다. 양쪽 데이터 형식을 바꾸면 둘 다 갱신한다.

## 4 oakapp.toml 이해하기

현재 앱 식별자는 `com.qsentech.oak4-yolopv2-standalone`, 소스 버전은 `0.14.1`이다.

| 항목 | 현재 역할 |
|---|---|
| identifier | 같은 앱을 식별하는 이름. 기능 수정 때 임의 변경하지 않는다. |
| app_version | 배포 버전. 수정본을 구분하도록 증가시킨다. |
| base_image | `luxonis/oakapp-base:1.3.0` 실행 기반 이미지 |
| prepare_container | GStreamer·Python 의존성 설치 |
| build_steps | backend 시작 스크립트를 서비스 경로에 배치하고 실행 권한 부여 |
| entrypoint | 서비스 관리 프로그램 시작 |
| assign_frontend_port | 현재 false. 멀티뷰는 웹 프론트엔드 없이 RTSP로 수신 |

실행 순서는 `oakapp.toml → 서비스 관리자 → backend-run.sh → /app/main.py`이다.
앱 폴더의 파일이 카메라 컨테이너의 `/app`으로 전달된다.
`.oakappignore`는 전송 제외 목록이다. 비밀번호·녹화 영상·불필요한 대용량 파일을 앱 폴더에 넣지 않는다.
현재 의존성은 `depthai==3.8.0`, `numpy>=1.26,<3`, `PyGObject==3.46.0`이다.
재현성을 위해 릴리스 기록에는 패키지·실행 환경 버전도 함께 남긴다.

## 5 개발 실습

### 5.1 기존 상태 기록

```bat
git status --short
git rev-parse HEAD
```

현재 커밋 번호를 메모하고 정상 동작하는 이전 `.oakapp` 파일이 있으면 별도 보관한다.
Git 커밋만으로는 제외된 모델 파일까지 복구되지 않으므로 모델도 보관한다.

### 5.2 작은 변경부터 수행

예를 들어 Depth 색상 표시를 바꾸려면 `depth_color.py`를 편집한다.
현재는 300~25000mm를 고정 로그 색상으로 변환한다. 표시 최대값을 변경하면
`tests/test_depth_color.py`와 PC `oak_recorder.py`의 범례도 함께 맞춘다.
거리 계산값을 바꾸는 작업과 표시 색상만 바꾸는 작업을 구분한다.

수정본을 배포할 때 `oakapp.toml`의 버전을 증가시킨다. 예: `0.14.1 → 0.14.2`.
이 매뉴얼 작성 자체로 코드나 버전을 변경하지는 않는다.

### 5.3 테스트

아래 명령은 CMD가 WSL의 Bash 안에서 테스트를 실행하도록 한다.

```bat
wsl.exe -d Ubuntu-22.04 -- bash -lc "cd /mnt/d/git_hub/hl-dcu-sensor-fusion/apps/yolopv2_standalone && PYTHONPATH=.:tests python3 -m unittest discover -s tests"
git diff --check
```

성공 기준: 테스트 마지막에 `OK`, diff 검사에 공백 오류 없음.
0.14.1 기준 기록은 59개 테스트 통과이다. 이후 테스트 추가에 따라 개수는 달라진다.
실패하면 실제 장비 배포 전에 실패 테스트부터 수정한다.
계산 테스트 통과가 실제 카메라 스트림·하드웨어 동작 성공까지 보장하지는 않는다.

### 5.4 개발 앱 배포와 실행

녹화를 저장하고 기존 멀티뷰 창을 닫은 뒤 실행한다.

```bat
"%OAKCTL%" app run "%OAKAPP%" --detach -d %OAKIP%
```

소스 전달, 컨테이너 준비, 의존성 설치, 실행 순서로 진행한다.
같은 소스 환경의 재실행은 캐시를 이용할 수 있다. `--detach`는 명령 종료 후에도
앱을 계속 실행하는 옵션이다. **명령 종료 코드 0만으로 정상 영상 출력이라고 판단하지 않는다.**

```bat
"%OAKCTL%" app list -d %OAKIP%
"%OAKCTL%" app logs %APPID% -d %OAKIP% --no-follow --tail 20
```

성공 기준: 앱 running, full pipeline running 및 HEALTH 로그 출력, 반복 Traceback 없음.
실시간 로그를 볼 때는 `--no-follow`를 뺀다. 로그 보기의 Ctrl+C는 조회 종료이며 앱 중지가 아니다.

### 5.5 멀티뷰와 CSV 확인

```bat
wsl.exe -d Ubuntu-22.04 -- python3 /mnt/d/git_hub/hl-dcu-sensor-fusion/tools/oak_recorder.py --device-ip 192.168.0.4 --auto-connect
```

이 명령은 새 CMD 창에서 실행해도 된다. 멀티뷰 사용 중 해당 CMD는 닫지 않는다.
다른 IP이면 명령의 IP도 수정한다. 기존 멀티뷰가 있으면 녹화를 저장하고 닫아 중복 창을 피한다.

| 검사 | 정상 기준 |
|---|---|
| RGB | 원본 영상이 계속 갱신됨 |
| 인식 결과 | 영상이 계속 갱신됨. 검출 없는 장면에서는 박스가 없을 수 있음 |
| Depth | 거리 색상 영상 갱신, 범례 0.3~25m. 검정은 무효값 |
| BEV | 영상 수신됨. 중심선 미검출이면 선이 없을 수 있음 |
| CSV | 하단 프레임 기록 건수 증가, 저장 경로 표시 |
| 파일 | 해당 폴더 frames.csv 크기/수정 시각 증가 |

기본 CSV 위치는 `%REPO%\artifacts\telemetry\날짜_ID\`이다.
frames.csv는 프레임당 한 행, objects.csv는 객체당 한 행이다.
객체가 없으면 objects.csv가 증가하지 않아도 프레임 기록은 정상일 수 있다.
영상 녹화 버튼과 CSV 기록은 독립적이며 멀티뷰 종료 시 CSV도 종료한다.
지연값은 화면 표시 지연이 아닌 구조화 데이터 수신 지연 추정이다.
`invalid_clock_or_asymmetry` 행의 빈 지연값을 0ms로 해석하지 않는다.

자동 검사가 필요하면 일반 멀티뷰 대신 다음을 실행한다. 45초 뒤 결과 JSON을 출력하고 창을 유지한다.

```bat
wsl.exe -d Ubuntu-22.04 -- python3 /mnt/d/git_hub/hl-dcu-sensor-fusion/src/hl_camera_bringup/scripts/check_recorder_gui.py --device-ip 192.168.0.4 --keep-open --wait-seconds 45
```

성공 기준: `passed: true`, 네 스트림 connected/preview가 true, CSV frames가 0보다 큼.
이 검사는 수신·위젯 이미지 생성을 확인한다. 화면 왜곡이나 인식 정확도는 영상으로 별도 판단한다.

## 6 설치 패키지 빌드

개발 시험이 통과한 소스를 고정한 뒤 실행한다. 빌드 동안 장비를 시험 전용으로 사용한다.
패키지는 파일이 클 수 있으므로 PC와 장비의 여유 공간을 확인한다.

```bat
if not exist "%REPO%\dist" mkdir "%REPO%\dist"
"%OAKCTL%" app build "%OAKAPP%" -d %OAKIP% --output "%REPO%\dist"
dir /o-d "%REPO%\dist\*.oakapp"
```

성공 기준: 빌드 성공 메시지와 새 `.oakapp` 파일 존재. 실제 생성 파일명은 출력에서 확인한다.
`--output` 폴더는 자동 생성되지 않아 먼저 mkdir를 실행한다.
파일명·앱 버전·소스 커밋·모델 버전·검증 결과를 묶어서 보관한다.
선택적으로 해시를 계산해 전달 파일의 동일성을 확인할 수 있다.

```bat
certutil -hashfile "%REPO%\dist\실제파일명.oakapp" SHA256
```

`실제파일명.oakapp`은 반드시 생성된 이름으로 바꾼다. 예시 이름 그대로 실행하지 않는다.
처음에는 `--publish`를 사용하지 않는다. 이는 파일 생성뿐 아니라 외부 Hub에 게시하는 별도 동작이다.

## 7 패키지 설치와 배포 확인

설치 전 멀티뷰를 닫는다. **app install은 기본적으로 다른 앱을 중지하고 자동 실행을 해제한다.**
여러 카메라 앱을 동시에 실행하면 장치 자원이나 RTSP 포트가 충돌할 수 있으므로
시험을 위해 무조건 `--keep-others`를 붙이지 않는다.

첫 설치 시험에서는 자동 시작을 명시적으로 끈다.

```bat
"%OAKCTL%" app install "%REPO%\dist\실제파일명.oakapp" --start=false --enable=false -d %OAKIP%
"%OAKCTL%" app list -d %OAKIP%
```

설치 결과에 나온 앱 ID를 복사해 설정한다. 아래 `설치결과의실제앱ID`는 자리표시자이다.

```bat
set "APPID=설치결과의실제앱ID"
"%OAKCTL%" app start %APPID% -d %OAKIP%
"%OAKCTL%" app logs %APPID% -d %OAKIP% --no-follow --tail 20
```

패키지 앱 ID는 개발용 0000…와 다를 수 있다. 로그에서 정상 시작을 확인한 뒤
5.5절의 멀티뷰를 실행해 네 영상과 CSV를 다시 검증한다.

**현재 멀티뷰의 주의점:** 영상 포트가 열려 있지 않으면 로컬 소스로 `app run`을 호출한다.
따라서 설치 패키지 시험은 반드시 패키지 앱을 먼저 정상 시작한 다음 멀티뷰를 연결한다.
연결 오류가 난 뒤에는 `app list`로 패키지 앱인지 개발 앱으로 바뀌었는지 다시 확인한다.
설치 패키지의 ID·로그·버전을 함께 확인해야 패키지 검증으로 인정할 수 있다.

검증 완료 후 부팅 시 자동 실행이 필요할 때만 설정한다.

```bat
"%OAKCTL%" app enable %APPID% -d %OAKIP%
```

실제 전원 재인가 후 자동 실행 여부는 녹화가 없는 시험 시간에 별도로 확인한다.

## 8 중지 재시작 삭제와 복구

아래 명령 전에는 `%APPID%`가 개발 앱인지 설치 앱인지 확인한다.

```bat
"%OAKCTL%" app stop %APPID% -d %OAKIP%
"%OAKCTL%" app start %APPID% -d %OAKIP%
```

위 두 명령은 각각 중지와 시작이다. 항상 두 줄을 함께 실행해야 하는 것은 아니다.
부팅 시 자동 실행만 끄려면 다음 명령을 사용한다. 현재 실행 중인 앱 중지는 별도다.

```bat
"%OAKCTL%" app disable %APPID% -d %OAKIP%
```

삭제는 일상적인 재배포 절차에 필요하지 않다. 삭제할 앱의 ID를 확인하고
장비 내 앱 전용 데이터를 백업한 경우에만 사용한다.

```bat
"%OAKCTL%" app uninstall %APPID% -d %OAKIP%
```

앱과 앱 전용 저장 데이터가 삭제된다. PC 소스·CSV·녹화 파일은 유지된다.
복구는 보관한 정상 `.oakapp`을 7절처럼 설치하고, 반환된 앱 ID로 시작·검증한다.
기존 설치와 식별자가 겹쳐 설치가 거부되면 오류와 목록을 확인하고,
`app install --help`의 `--update` 절차를 검토한다. 임의로 모든 앱을 삭제하지 않는다.

## 9 앱 내부 셸로 디버깅

앱 실행 중에만 아래 명령을 사용한다.

```bat
"%OAKCTL%" app exec -i -d %OAKIP% %APPID% bash
```

접속 후에는 카메라 앱 컨테이너의 Linux 명령을 사용한다.

```bash
cd /app
ls
ps -ef
python3.12 --version
exit
```

이미 main.py가 서비스로 실행 중이므로 같은 파이프라인을 수동으로 중복 실행하지 않는다.
앱 내부 임시 수정은 재배포 시 보존되지 않을 수 있다. 정식 수정은 PC 소스에 반영한다.
카메라 OS의 root SSH 접근은 일반적인 앱 개발에 필수 사항이 아니다.

## 10 문제 해결

| 증상 | 확인 및 조치 |
|---|---|
| Camera password 요청 | 정상 인증 프롬프트에 비밀번호 입력. 채팅이나 파일에 기록하지 않음 |
| ready이지만 영상 없음 | app list만 믿지 말고 logs와 파이프라인 시작 확인. 개발 중이면 app run 직접 실행 |
| Traceback 반복 | 마지막 로그에서 파일명·줄·예외를 확인하고 PC 소스 수정 후 재배포 |
| 모델 파일 없음 | Git 제외 모델 아카이브를 models/에 복원. 임의의 다른 모델로 대체하지 않음 |
| 패키지 다운로드 실패 | 장비 인터넷·DNS·저장 공간 확인. build/run 준비 단계 로그 확인 |
| 한 영상만 연결되지 않음 | 아래 주소와 장비 로그 확인. 앱 버전·포트 일치 여부 확인 |
| CSV 기록 건수 정지 | TCP 8557 연결, 앱 상태, 저장 공간, UI 저장 오류 확인 |
| 지연값 빈칸 | latency_status 확인. 시계/통신 검사에서 무효인 경우임 |
| 중심선 없음 | 영상 수신 장애와 도로 미검출을 구분. 도로 장면 및 centerline_valid 확인 |
| Depth 검정 영역 | 유효 거리 미측정. 표시 범위를 확대해도 복원되지 않음 |
| 화면에 예전 범례 표시 | 녹화 저장 후 기존 PC 창을 닫고 변경된 PC 프로그램 실행 |

| 출력 | 현재 주소 및 크기 |
|---|---|
| RGB | rtsp://192.168.0.4:8555/rgb · 1920×1080 |
| 인식 결과 | rtsp://192.168.0.4:8554/yolopv2 · 960×540 |
| Depth 표시 | rtsp://192.168.0.4:8556/depth · 640×360 |
| BEV | rtsp://192.168.0.4:8553/bev · 640×480 |
| CSV용 구조화 데이터 | TCP 192.168.0.4:8557 |

영상 설정은 10 FPS이며 실제 수신 속도는 측정한다. 데이터 포트는 신뢰하는 내부망에서만 사용한다.
카메라가 모니터를 촬영하면 Depth는 영상 속 도로가 아닌 모니터까지의 거리이다.
BEV 중심선은 미보정 개발 좌표이며 차량 조향 제어 검증을 대신하지 않는다.

## 11 배포 완료 체크리스트

- [ ] 변경 내용·소스 커밋·앱 버전 기록
- [ ] 모델 아카이브 별도 보관
- [ ] 단위 테스트와 diff 검사 통과
- [ ] 장비 앱 running 및 반복 오류 없음
- [ ] RGB·인식 결과·Depth·BEV 갱신 확인
- [ ] CSV 경로·기록 증가·누락 상태 확인
- [ ] 영상 녹화·저장이 필요하면 파일 재생까지 확인
- [ ] 최종 패키지 파일명·SHA256·설치 앱 ID 기록
- [ ] 설치 앱을 시험했는지 확인하고 개발 앱 대체 실행 여부 점검
- [ ] 필요할 때만 자동 실행 설정 및 재부팅 시험
- [ ] 정상 이전 패키지와 데이터 백업 보관

## 12 근거와 검증 이력

- 2026-10-19: 현재 소스·설정·모델 존재 및 설치된 oakctl의 build/install 도움말을 기준으로 작성.
- 기존 장비 검증: 0.14.1 배포, 테스트 59개, 네 영상 수신 및 CSV 기록은
  `docs/verification/camera/depth25_20261018.md`에 기록되어 있다.
- 이번 매뉴얼 작성에서는 운영 중인 앱을 바꾸지 않았다. `.oakapp` 생성·설치·부팅 자동 실행은
  이 문서 작성 작업에서 재실행하지 않았으며, 6~7절과 체크리스트에 따라 해당 장비에서 확인한다.
- 이 앱은 standalone 경로이다. 본 절차에는 ROS 토픽·TF 변경이나 colcon 빌드가 포함되지 않는다.
- 공식 참고: [oakctl](https://docs.luxonis.com/software-v3/oak-apps/oakctl),
  [oakapp.toml](https://docs.luxonis.com/software-v3/oak-apps/configuration),
  [기본 이미지](https://docs.luxonis.com/software-v3/oak-apps/base-image).
  CLI 버전에 따라 옵션이 달라질 수 있으므로 현재 설치된 명령의 `--help`를 우선 확인한다.

# 카메라 실행 환경·코드 분리 배포

작성: 2026-11-08. 개정: 2026-11-10. 대상: `apps/yolopv2_standalone`.

## 배포 기준

일반 Python 로직·설정 변경은 작은 코드 ZIP만 전송한다. Python/DepthAI/OpenCV/GStreamer, AI 모델 및 복구용 기본 코드는 설치된 컨테이너에 유지한다. PC UI만 변경하면 PC 앱만 재실행한다. 라이브러리·모델·기본 이미지·런처 변경 시 전체 `.oakapp`을 갱신한다. 새로운 런처를 도입하는 최초 1회도 전체 설치가 필요하다.

컨테이너 버전과 코드 릴리스는 별개다. `oakctl app list`의 버전은 실행 환경 패키지 버전이며, 실제 실행 코드는 아래 status 및 텔레메트리 `code_release`/`runtime_fingerprint`로 확인한다.

## 저장 및 전환 방식

Luxonis가 마운트한 `OAKAGENT_STORAGE_APP` 아래 `code-deploy/releases/<SHA256>/`에 버전별 코드와 manifest를 보관한다. 앱 UUID 전용 저장소이므로 다른 설치본에 자동 적용하지 않는다. 컨테이너 stop/start 시에도 코드는 유지되며, 앱 삭제 시에는 해당 전용 저장소를 잃을 수 있다. 전체 환경을 새 UUID로 설치하면 새 저장소에 코드 묶음을 다시 배포한다.

런처는 컨테이너 `/app/release_runtime.py`, 서비스는 `/app/backend-run.sh`에 고정한다. 모델은 컨테이너의 기존 모델을 사용한다. 코드 묶음에는 Python 모듈·JSON 설정만 넣고 모델·라이브러리·테스트·런처는 넣지 않는다. 새 모듈의 제3자 라이브러리 요구는 requirements.txt에 함께 명시해야 호환성 검사가 감지한다.

전송 완료 후 ZIP/개별 파일 해시·경로·크기·Python 구문·JSON·runtime fingerprint를 검사한다. 검사를 통과한 버전만 pending으로 지정하고 설치된 컨테이너를 재시작한다(이미지 재전송 없음). 새 코드의 full 모드 프레임을 90초 안에 확인해야 확정한다. 초기 기동 실패나 확정 전 중단 시 이전 정상 코드를 선택한다. 이전 코드가 없으면 컨테이너 내 기본 코드로 복구한다. 이 검사는 기동 확인이며 모든 인식 기능의 정확도를 보증하는 검사는 아니다.

## 용어와 작업 선택

| 구분 | 포함 내용 | 변경 시 작업 |
| --- | --- | --- |
| 패키지 (`.oakapp`) | Python·DepthAI·OpenCV·GStreamer, AI 모델, 고정 런처, 기본 복구 코드 | 전체 패키지 빌드·설치 |
| 앱 코드 (`code.zip`) | 수정하는 Python 로직·JSON 설정 | 앱 코드 build·deploy |
| PC 멀티뷰 | PC 표시·입력·녹화 프로그램 | PC 프로그램 재실행 |

`oakctl app`은 설치된 패키지 인스턴스를 관리한다. start/stop은 그 패키지 안에서 활성 앱 코드를 실행/중지한다. 패키지와 앱 코드를 각각 따로 실행하는 구조가 아니다.

## Windows CMD 공통 설정

아래 명령은 PowerShell이 아닌 Windows 명령 프롬프트(CMD)용이다. 새 CMD 창에서 공통 설정을 먼저 실행한다. Python은 PATH에 등록되어 있어야 한다.

```bat
cd /d D:\git_hub\hl-dcu-sensor-fusion
set "OAKCTL=C:\Program Files\oakctl\bin\oakctl.exe"
set "CAMERA_IP=192.168.201.2"
set "APP_ID=61fbe0c1-ad26-42dc-93c4-a8d2b5191fce"
set "DEPLOY_TOOL=src\hl_camera_bringup\scripts\deploy_code_release.py"
```

UUID는 2026-11-10 문서 기준이며 실제 대상은 최신 PROJECT_STATUS와 app list로 확인한다. 녹화 중에는 먼저 저장하고 종료한다. 배포 후 PC 영상 소스·표시 설정은 앱에서 다시 적용하며 사용자 저장 설정을 임의로 덮어쓰지 않는다.

## 앱 코드 관리

### 빌드: 구문 검사 및 ZIP 생성

Python은 별도 네이티브 실행 파일 컴파일 대신 구문 검사와 코드 묶음 생성을 수행한다. 아래 code3는 다음 수정 작업의 예시이며 이미 배포됐다는 의미가 아니다. 버전과 기록 폴더는 작업별로 변경한다.

```bat
python "%DEPLOY_TOOL%" build --version 0.33.10-code3 --bundle artifacts/code3/code.zip --output artifacts/code3/bundle.json
```

### 업로드·교체·실행

```bat
python "%DEPLOY_TOOL%" deploy --device-ip %CAMERA_IP% --app-id %APP_ID% --bundle artifacts/code3/code.zip --output artifacts/code3/deployed.json
```

업로드 → 해시·구문·호환성 검사 → 활성 코드 전환 → 앱 재시작 → 정상 프레임 확인까지 수행한다. 별도 start는 필요 없다. 업로드만 하고 나중에 활성화하는 명령은 현재 배포 도구에서 제공하지 않는다. 기존 앱 코드를 먼저 삭제하지 않는다.

### 상태 확인 및 이전 정상 코드로 복구

컨테이너 내부 명령을 사용하므로 앱이 실행 중이어야 한다.

```bat
python "%DEPLOY_TOOL%" status --device-ip %CAMERA_IP% --app-id %APP_ID%
python "%DEPLOY_TOOL%" rollback --device-ip %CAMERA_IP% --app-id %APP_ID% --output artifacts/code3/rollback.json
```

rollback도 재시작과 정상 확인을 수행한다. 개별 앱 코드 릴리스 삭제 명령은 제공하지 않으며 이전 버전은 복구용으로 보관한다.

## 패키지 관리

### 빌드

라이브러리·모델·기본 이미지·고정 런처 변경 시에만 수행한다. 이 프로젝트의 oakctl 패키지 빌드는 대상 카메라를 사용하므로 순수 PC 오프라인 컴파일과 다르다.

```bat
if not exist dist mkdir dist
"%OAKCTL%" app build apps\yolopv2_standalone -d %CAMERA_IP% --output dist
dir /b dist\*.oakapp
```

### 업로드·설치 및 교체

`실제파일명.oakapp`은 이번 빌드 결과 파일명으로 바꾼다.

```bat
"%OAKCTL%" app install "dist\실제파일명.oakapp" --start=false --enable=false -d %CAMERA_IP%
"%OAKCTL%" app list -d %CAMERA_IP%
```

설치만 하고 실행·부팅 자동 시작은 하지 않는다. 기본 설치 동작은 다른 앱을 중지하고 자동 시작을 해제한다. 기존 패키지는 먼저 삭제하지 않는다. 교체 시 새 설치본을 실행·검증하고 이전 설치본은 복구용으로 보관한다. 설치 결과와 목록에서 UUID를 확인해 변수를 갱신한다.

```bat
set "APP_ID=설치 결과에서 확인한 UUID"
"%OAKCTL%" app start %APP_ID% -d %CAMERA_IP%
```

새 UUID에서는 패키지에 포함된 기본 코드로 시작하며 기존 앱 코드 저장소는 자동 승계되지 않는다. 별도 앱 코드가 필요하면 새 패키지와 호환되는 코드 ZIP을 빌드해 새 UUID로 deploy한다.

### 실행·중지·재시작·목록

아래 명령은 순서대로 일괄 실행하는 절차가 아니라 필요한 작업 하나를 선택하는 명령이다.

| 작업 | CMD 명령 |
| --- | --- |
| 실행 | `"%OAKCTL%" app start %APP_ID% -d %CAMERA_IP%` |
| 중지 | `"%OAKCTL%" app stop %APP_ID% -d %CAMERA_IP%` |
| 재시작 | `"%OAKCTL%" app restart %APP_ID% -d %CAMERA_IP%` |
| 목록·실행 상태 | `"%OAKCTL%" app list -d %CAMERA_IP%` |

중지·재시작해도 패키지와 앱 코드는 보존된다. start는 수동 실행이며 부팅 자동 시작 설정과 별개다. 실제 활성 앱 코드 버전은 앞의 코드 status 명령으로 확인한다.

### 삭제

일반 앱 코드 교체에는 사용하지 않는다. 삭제 대상 UUID를 명시적으로 확인한 경우에만 실행한다. **uninstall은 패키지 인스턴스와 앱 전용 저장소를 삭제하므로 그 안의 앱 코드·복구 이력도 삭제된다.** 복구용 설치본을 임의로 삭제하지 않는다.

```bat
set "REMOVE_APP_ID=삭제할 패키지 UUID"
"%OAKCTL%" app stop %REMOVE_APP_ID% -d %CAMERA_IP%
"%OAKCTL%" app uninstall %REMOVE_APP_ID% -d %CAMERA_IP%
```

## PC 멀티뷰 실행

중복 실행하지 않는다. 재실행 전 진행 중인 녹화와 미저장 결과를 확인한다. 격리 OpenCV 환경을 사용하는 아래 스크립트로 실행한다.

```bat
wsl.exe -d Ubuntu-22.04 -- bash /mnt/d/git_hub/hl-dcu-sensor-fusion/tools/run_oak_recorder.sh
```

## 호환성 및 복구 유의사항

동일 runtime fingerprint인 코드만 배포할 수 있다. 호환성이 다르면 자동 전체 설치하지 않고 중단한다. `rollback`은 마지막 정상 코드와 이전 코드를 교환하며 필요하면 embedded 복구 코드로 돌아간다. 이미 pending인 배포가 있으면 추가 활성화·수동 복구를 거부하므로 먼저 상태를 확인한다. 다른 PC의 동시 배포는 피한다. stage와 활성화 사이의 통신이 끊기면 status로 확인한 뒤 재시도한다. 배포 기록의 코드는 SHA256 release ID로 식별한다.

런처의 전체 해시가 runtime fingerprint에 포함되므로 런처 수정도 실행 환경 업데이트가 필요하다. `runtime-base.toml`은 컨테이너에도 포함되는 기본 이미지 식별 파일이다. PC 빌드 시 oakapp.toml의 base_image와 일치하는지 검사한다(oakctl은 oakapp.toml 자체를 실행 컨테이너에서 제외한다). 기본 이미지를 바꾸면 두 파일을 함께 수정한다. 소스만 변했을 때 임의로 전체 컨테이너 빌드를 반복하거나 라이브러리를 호스트 OS에 설치하지 않는다.

## 근거

- [Luxonis oakctl 영구 저장소](https://docs.luxonis.com/software-v3/oak-apps/oakctl): 앱 전용·공유·전역 저장소 환경 변수와 컨테이너 파일 변경의 보존 한계.
- [Luxonis 앱 설정](https://docs.luxonis.com/software-v3/oak-apps/configuration): 컨테이너와 마운트 설정.
- 카메라 실측: `/app-storage/app`은 userdata의 ext4 bind mount이며 `OAKAGENT_STORAGE_APP` 환경 변수로 제공된다.

## 변경 이력 및 검증 (2026-11-10)

- 패키지와 앱 코드 용어를 구분하고 Windows CMD 기준 빌드·업로드·교체·실행·중지·복구·삭제 및 PC 실행 명령을 정리했다.
- 배포 도구의 인자·자동 재시작 처리와 설치된 oakctl의 build/install/uninstall 도움말을 대조했다. 문서 변경이며 장치 배포·재시작·삭제는 수행하지 않았다.

# 카메라 실행 환경·코드 분리 배포

작성: 2026-11-08. 대상: `apps/yolopv2_standalone`.

## 배포 기준

일반 Python 로직·설정 변경은 작은 코드 ZIP만 전송한다. Python/DepthAI/OpenCV/GStreamer, AI 모델 및 복구용 기본 코드는 설치된 컨테이너에 유지한다. PC UI만 변경하면 PC 앱만 재실행한다. 라이브러리·모델·기본 이미지·런처 변경 시 전체 `.oakapp`을 갱신한다. 새로운 런처를 도입하는 최초 1회도 전체 설치가 필요하다.

컨테이너 버전과 코드 릴리스는 별개다. `oakctl app list`의 버전은 실행 환경 패키지 버전이며, 실제 실행 코드는 아래 status 및 텔레메트리 `code_release`/`runtime_fingerprint`로 확인한다.

## 저장 및 전환 방식

Luxonis가 마운트한 `OAKAGENT_STORAGE_APP` 아래 `code-deploy/releases/<SHA256>/`에 버전별 코드와 manifest를 보관한다. 앱 UUID 전용 저장소이므로 다른 설치본에 자동 적용하지 않는다. 컨테이너 stop/start 시에도 코드는 유지되며, 앱 삭제 시에는 해당 전용 저장소를 잃을 수 있다. 전체 환경을 새 UUID로 설치하면 새 저장소에 코드 묶음을 다시 배포한다.

런처는 컨테이너 `/app/release_runtime.py`, 서비스는 `/app/backend-run.sh`에 고정한다. 모델은 컨테이너의 기존 모델을 사용한다. 코드 묶음에는 Python 모듈·JSON 설정만 넣고 모델·라이브러리·테스트·런처는 넣지 않는다. 새 모듈의 제3자 라이브러리 요구는 requirements.txt에 함께 명시해야 호환성 검사가 감지한다.

전송 완료 후 ZIP/개별 파일 해시·경로·크기·Python 구문·JSON·runtime fingerprint를 검사한다. 검사를 통과한 버전만 pending으로 지정하고 설치된 컨테이너를 재시작한다(이미지 재전송 없음). 새 코드의 full 모드 프레임을 90초 안에 확인해야 확정한다. 초기 기동 실패나 확정 전 중단 시 이전 정상 코드를 선택한다. 이전 코드가 없으면 컨테이너 내 기본 코드로 복구한다. 이 검사는 기동 확인이며 모든 인식 기능의 정확도를 보증하는 검사는 아니다.

## Windows PowerShell 명령

저장소 루트에서 실행한다. 실제 UUID는 최신 PROJECT_STATUS에서 확인한다. 녹화 중에는 먼저 저장하고 종료한다. 배포 후 PC 영상 소스·표시 설정은 앱에서 다시 적용한다.

```powershell
python src/hl_camera_bringup/scripts/deploy_code_release.py build --version <코드버전> --bundle artifacts/<작업>/code.zip --output artifacts/<작업>/bundle.json
python src/hl_camera_bringup/scripts/deploy_code_release.py deploy --app-id <실행환경UUID> --bundle artifacts/<작업>/code.zip --output artifacts/<작업>/deployed.json
python src/hl_camera_bringup/scripts/deploy_code_release.py status --app-id <실행환경UUID>
python src/hl_camera_bringup/scripts/deploy_code_release.py rollback --app-id <실행환경UUID> --output artifacts/<작업>/rollback.json
```

동일 runtime fingerprint인 코드만 배포할 수 있다. 호환성이 다르면 자동 전체 설치하지 않고 중단한다. `rollback`은 마지막 정상 코드와 이전 코드를 교환하며 필요하면 embedded 복구 코드로 돌아간다. 이미 pending인 배포가 있으면 추가 활성화·수동 복구를 거부하므로 먼저 상태를 확인한다. 다른 PC의 동시 배포는 피한다. stage와 활성화 사이의 통신이 끊기면 status로 확인한 뒤 재시도한다. 배포 기록의 코드는 SHA256 release ID로 식별한다.

런처의 전체 해시가 runtime fingerprint에 포함되므로 런처 수정도 실행 환경 업데이트가 필요하다. `runtime-base.toml`은 컨테이너에도 포함되는 기본 이미지 식별 파일이다. PC 빌드 시 oakapp.toml의 base_image와 일치하는지 검사한다(oakctl은 oakapp.toml 자체를 실행 컨테이너에서 제외한다). 기본 이미지를 바꾸면 두 파일을 함께 수정한다. 소스만 변했을 때 임의로 전체 컨테이너 빌드를 반복하거나 라이브러리를 호스트 OS에 설치하지 않는다.

## 근거

- [Luxonis oakctl 영구 저장소](https://docs.luxonis.com/software-v3/oak-apps/oakctl): 앱 전용·공유·전역 저장소 환경 변수와 컨테이너 파일 변경의 보존 한계.
- [Luxonis 앱 설정](https://docs.luxonis.com/software-v3/oak-apps/configuration): 컨테이너와 마운트 설정.
- 카메라 실측: `/app-storage/app`은 userdata의 ext4 bind mount이며 `OAKAGENT_STORAGE_APP` 환경 변수로 제공된다.

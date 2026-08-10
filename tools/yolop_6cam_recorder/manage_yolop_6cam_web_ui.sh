#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PID_FILE="${APP_DIR}/web_ui.pid"
LOG_FILE="${APP_DIR}/web_ui.log"
PASSWORD_FILE="/home/halla/.yolop_web_password"
EXTERNAL_HOST="192.168.201.4"
LOCAL_HOST="127.0.0.1"
PORT="8080"
PYTHON_BIN="${PYTHON_BIN:-python3}"

read_pid() {
  [[ -f "${PID_FILE}" ]] || return 1
  local pid
  pid="$(tr -d '[:space:]' < "${PID_FILE}")"
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "${pid}"
}

is_expected_process() {
  local pid="$1"
  [[ -r "/proc/${pid}/cmdline" ]] || return 1
  local command_line
  command_line="$(tr '\0' ' ' < "/proc/${pid}/cmdline")"
  [[ "${command_line}" == *"/yolop_6cam_web_ui.py"* ]]
}

running_pid() {
  local pid
  pid="$(read_pid)" || return 1
  kill -0 "${pid}" 2>/dev/null || return 1
  is_expected_process "${pid}" || return 1
  printf '%s\n' "${pid}"
}

device_count() {
  "${PYTHON_BIN}" -c \
    'import depthai as dai; print(len(dai.Device.getAllAvailableDevices()))' \
    2>/dev/null | tail -n 1
}

wait_for_six_cameras() {
  local count="0"
  echo "6대 OAK-D PoE 카메라 검색을 기다립니다..."
  for _ in $(seq 1 24); do
    count="$(device_count || printf '0')"
    if [[ "${count}" == "6" ]]; then
      echo "카메라 6대 검색 완료"
      return 0
    fi
    echo "현재 검색: ${count}/6 (5초 후 재확인)"
    sleep 5
  done
  echo "ERROR: 120초 동안 카메라 6대가 모두 검색되지 않았습니다." >&2
  return 1
}

start_service() {
  local pid
  if pid="$(running_pid)"; then
    echo "이미 실행 중입니다. PID=${pid}"
    status_service
    return 0
  fi

  if [[ ! -x "${APP_DIR}/run_yolop_6cam_web_ui.sh" ]]; then
    echo "ERROR: 실행 파일이 없거나 실행 권한이 없습니다." >&2
    return 1
  fi
  if [[ ! -s "${PASSWORD_FILE}" ]]; then
    echo "ERROR: 인증 비밀번호 파일이 없습니다: ${PASSWORD_FILE}" >&2
    return 1
  fi

  wait_for_six_cameras
  rm -f -- "${PID_FILE}"

  echo "Web UI를 시작합니다..."
  nohup "${APP_DIR}/run_yolop_6cam_web_ui.sh" \
    --host "${EXTERNAL_HOST}" \
    --local-host "${LOCAL_HOST}" \
    --port "${PORT}" \
    --auth-user halla \
    --auth-password-file "${PASSWORD_FILE}" \
    --output "${APP_DIR}/recordings" \
    --fps 10 \
    --bitrate-kbps 2000 \
    --preview-quality 75 \
    > "${LOG_FILE}" 2>&1 &
  pid="$!"
  printf '%s\n' "${pid}" > "${PID_FILE}"

  for _ in $(seq 1 75); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      echo "ERROR: Web UI 시작에 실패했습니다." >&2
      tail -n 30 "${LOG_FILE}" >&2 || true
      return 1
    fi
    if ss -ltn | grep -q "${EXTERNAL_HOST}:${PORT}" \
      && ss -ltn | grep -q "${LOCAL_HOST}:${PORT}"; then
      echo "시작 완료. PID=${pid}"
      echo "외부 접속: http://${EXTERNAL_HOST}:${PORT}"
      echo "관리자 접속: http://${LOCAL_HOST}:${PORT}"
      return 0
    fi
    sleep 1
  done

  echo "ERROR: 프로세스는 실행 중이지만 75초 안에 Web UI 포트가 열리지 않았습니다." >&2
  tail -n 30 "${LOG_FILE}" >&2 || true
  return 1
}

stop_service() {
  local pid
  if ! pid="$(running_pid)"; then
    echo "실행 중인 Web UI가 없습니다."
    rm -f -- "${PID_FILE}"
    return 0
  fi

  echo "Web UI를 정상 종료합니다. PID=${pid}"
  kill -TERM "${pid}"
  for _ in $(seq 1 60); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      rm -f -- "${PID_FILE}"
      echo "종료 완료"
      return 0
    fi
    sleep 1
  done

  echo "ERROR: 60초 안에 종료되지 않았습니다. kill -9는 자동 실행하지 않습니다." >&2
  return 1
}

status_service() {
  local pid
  if ! pid="$(running_pid)"; then
    echo "상태: 정지"
    return 1
  fi

  echo "상태: 실행 중"
  ps -p "${pid}" -o pid=,etime=,cmd=
  echo "리스닝 포트:"
  ss -ltn | grep ":${PORT}" || true
}

show_logs() {
  if [[ ! -f "${LOG_FILE}" ]]; then
    echo "로그 파일이 없습니다: ${LOG_FILE}" >&2
    return 1
  fi
  echo "Ctrl+C를 누르면 로그 보기만 종료됩니다. Web UI는 계속 실행됩니다."
  tail -n 100 -f "${LOG_FILE}"
}

usage() {
  echo "사용법: $0 {start|stop|restart|status|logs}"
}

case "${1:-}" in
  start)
    start_service
    ;;
  stop)
    stop_service
    ;;
  restart)
    stop_service
    start_service
    ;;
  status)
    status_service
    ;;
  logs)
    show_logs
    ;;
  *)
    usage
    exit 2
    ;;
esac

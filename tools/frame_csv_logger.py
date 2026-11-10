"""Direct device telemetry CSV logger (not a ROS subscriber)."""
# 카메라 앱 텔레메트리 서버(:8557)에 직접 접속해 프레임/객체 정보를 CSV로 저장한다(ROS 구독 아님).
# 접속 때마다 PC-장치 시계 차이를 추정해 지연 시간을 계산하고, 누락 프레임은 sequence_gap으로 남긴다.
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import socket
import threading
import time
import uuid

# frames.csv 열 목록(텔레메트리 필드 + PC 수신 시각·지연 추정값)
FRAME_FIELDS = ['session', 'frame_seq', 'nn_seq', 'depth_seq',
                'input_source', 'source_epoch', 'processing_mode', 'processing_epoch', 'source_file', 'source_frame_index',
                'source_media_time_s', 'source_media_epoch', 'depth_available', 'capture_timestamp_s',
                'nn_timestamp_s', 'depth_timestamp_s', 'capture_wall_est_s',
                'sent_wall_s', 'pc_receive_wall_s', 'pc_receive_utc',
                'clock_offset_est_s', 'clock_rtt_ms', 'clock_estimate_age_s',
                'capture_monotonic_est_s', 'sent_monotonic_s', 'pc_receive_monotonic_s',
                'clock_monotonic_offset_est_s',
                'raw_receive_delay_ms', 'raw_transport_delay_ms',
                'receive_delay_est_ms', 'transport_delay_est_ms', 'latency_status',
                'sequence_gap', 'centerline_valid', 'tracking_state', 'coefficient_units',
                'c0', 'c1', 'c2', 'c3', 'turn_mode', 'object_count',
                'boundary_valid', 'boundary_pair_valid', 'left_boundary_valid', 'right_boundary_valid', 'boundary_state', 'left_boundary_state', 'right_boundary_state',
                'vehicle_width_m', 'left_margin_m', 'right_margin_m', 'corridor_width_m',
                'minimum_clearance_width_m', 'boundary_width_min_dev', 'boundary_width_max_dev',
                'clearance_sufficient', 'boundary_units', 'boundary_forward_dev', 'left_boundary_dev', 'right_boundary_dev',
                'left_boundary_coefficients', 'right_boundary_coefficients',
                'normal_width_error_dev', 'road_coverage', 'corridor_feasible',
                'boundary_reference_side','left_boundary_confidence','right_boundary_confidence',
                'boundary_confidence_kind','stop_line_state','stop_line_confidence',
                'stop_line_confidence_kind','stop_line_valid','stop_line_points', 'boundary_model',
                'left_boundary_image','right_boundary_image','motion_confidence','motion_matrix']
OBJECT_FIELDS = ['session', 'frame_seq', 'object_index', 'label', 'confidence',
                 'xmin', 'ymin', 'xmax', 'ymax', 'depth_valid', 'x_m', 'y_m', 'z_m']


class CsvStorageError(Exception):
    """CSV 파일 쓰기 실패(디스크 문제 등). 네트워크 오류와 달리 재연결하지 않고 기록을 멈춘다."""
    pass


class FrameCsvLogger:
    """세션 폴더(날짜_시각_랜덤)에 frames.csv와 objects.csv를 만들어 기록하는 로거."""
    def __init__(self, ip, directory, port=8557):
        self.ip = ip
        self.port = port
        self.path = Path(directory) / (datetime.now().strftime('%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:8])
        self.frames = self.objects = self.gaps = 0
        self.status = '연결 대기'
        self.stop = threading.Event()
        self.thread = None
        self.connection = None

    def start(self):
        """기록 스레드를 시작한다."""
        if self.thread is None:
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def close(self):
        """기록을 멈추고 소켓을 끊은 뒤 최대 3초 기다린다."""
        self.stop.set()
        if self.connection:
            try:
                self.connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if self.thread:
            self.thread.join(timeout=3)

    def _run(self):
        """CSV를 만들고, 연결 → 시계 동기화(HELLO) → 수신 루프를 반복한다. 5초마다 재접속해 시계 추정을 갱신한다.
        오래된/중복 프레임은 버리고, 시계 추정이 이상하면 지연값을 비우고 상태를 표시한다.
        """
        previous = {}
        try:
            self.path.mkdir(parents=True, exist_ok=False)
            with (self.path/'frames.csv').open('x', newline='', encoding='utf-8-sig') as ff, \
                 (self.path/'objects.csv').open('x', newline='', encoding='utf-8-sig') as of:
                fw = csv.DictWriter(ff, FRAME_FIELDS)
                ow = csv.DictWriter(of, OBJECT_FIELDS)
                fw.writeheader()
                ow.writeheader()
                ff.flush()
                of.flush()
                while not self.stop.is_set():
                    try:
                        with socket.create_connection((self.ip, self.port), timeout=5) as conn:
                            self.connection = conn
                            conn.settimeout(10)
                            with conn.makefile('rb') as stream:
                                t1 = time.time()
                                m1 = time.monotonic()
                                conn.sendall(b'HELLO\n')
                                clock = json.loads(stream.readline(4096))
                                t4 = time.time()
                                m4 = time.monotonic()
                                # NTP 방식으로 PC와 장치의 시계 차이와 왕복 시간(RTT)을 추정한다.
                                offset = ((clock['clock_receive_s']-t1)+(clock['clock_send_s']-t4))/2
                                mono_offset = ((clock['clock_receive_mono_s']-m1)+(clock['clock_send_mono_s']-m4))/2
                                rtt = max(0., (m4-m1)-(clock['clock_send_mono_s']-clock['clock_receive_mono_s']))
                                synced = time.monotonic()
                                # Renew the estimate periodically; gaps remain explicit in CSV.
                                while not self.stop.is_set() and time.monotonic()-synced < 5:
                                    line = stream.readline(1048576)
                                    if self.stop.is_set():
                                        break
                                    received = time.time()
                                    received_mono = time.monotonic()
                                    if not line:
                                        raise ConnectionError('telemetry disconnected')
                                    p = json.loads(line)
                                    if p.get('heartbeat'):
                                        continue
                                    session, seq = p['session'], p['frame_seq']
                                    last = previous.get(session)
                                    if last is not None and seq <= last:
                                        continue
                                    gap = max(0, seq-last-1) if last is not None else 0
                                    row = {k: p.get(k, '') for k in FRAME_FIELDS}
                                    row.update(pc_receive_wall_s=received,
                                               pc_receive_monotonic_s=received_mono,
                                               clock_monotonic_offset_est_s=mono_offset,
                                               pc_receive_utc=datetime.fromtimestamp(received, timezone.utc).isoformat(),
                                               clock_offset_est_s=offset, clock_rtt_ms=rtt*1000,
                                               clock_estimate_age_s=time.monotonic()-synced,
                                               receive_delay_est_ms=(received_mono+mono_offset-p['capture_monotonic_est_s'])*1000,
                                               transport_delay_est_ms=(received_mono+mono_offset-p['sent_monotonic_s'])*1000,
                                               latency_status='estimated_not_ptp', sequence_gap=gap,
                                               coefficient_units='uncalibrated_bev_development_units',
                                               object_count=len(p['objects']))
                                    row['raw_receive_delay_ms'] = row['receive_delay_est_ms']
                                    row['raw_transport_delay_ms'] = row['transport_delay_est_ms']
                                    if (rtt > .05 or row['receive_delay_est_ms'] < 0
                                            or row['transport_delay_est_ms'] < -max(2., rtt*500)):
                                        row['latency_status'] = 'invalid_clock_or_asymmetry'
                                        row['receive_delay_est_ms'] = ''
                                        row['transport_delay_est_ms'] = ''
                                    for key in ('boundary_forward_dev', 'left_boundary_dev', 'right_boundary_dev',
                                                'left_boundary_coefficients', 'right_boundary_coefficients', 'stop_line_points', 'left_boundary_image', 'right_boundary_image', 'motion_matrix'):
                                        row[key] = json.dumps(p[key], allow_nan=False) if key in p else ''
                                    for i, value in enumerate(p.get('coefficients') or ['', '', '', '']):
                                        row[f'c{i}'] = value
                                    try:
                                        for i, obj in enumerate(p['objects']):
                                            ow.writerow(dict(obj, session=session, frame_seq=seq, object_index=i))
                                        fw.writerow(row)
                                        of.flush()
                                        ff.flush()
                                    except OSError as exc:
                                        raise CsvStorageError(str(exc)) from exc
                                    previous[session] = seq
                                    self.frames += 1
                                    self.objects += len(p['objects'])
                                    self.gaps += gap
                                    self.status = '기록 중 · 지연은 추정값'
                    except (OSError, ValueError, KeyError) as exc:
                        self.status = f'재연결 대기: {exc}'
                        self.stop.wait(1)
                    finally:
                        self.connection = None
        except (OSError, CsvStorageError) as exc:
            self.status = f'CSV 저장 실패: {exc}'
        finally:
            if self.stop.is_set() and not self.status.startswith('CSV 저장 실패'):
                self.status = '기록 중지'

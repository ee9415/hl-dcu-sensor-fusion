"""Independent MP4 recording from the existing RGB receiver, without owning it."""
# 이미 동작 중인 RGB 수신기의 프레임을 가져와 별도 MP4로 녹화한다(수신기 소유권은 건드리지 않음).
# 녹화 중 소스 상태 확인, 저장 공간·파일 증가 감시, 종료 후 MP4 검증, session.json/이벤트 로그 기록을 담당한다.
import json
from pathlib import Path
import socket
import shutil
import queue
import threading
import time
import cv2


class RecorderFault(RuntimeError):
    """녹화 중단 사유(kind)를 함께 담는 예외. kind는 session.json의 error_kind로 기록된다."""
    def __init__(self, kind, message):
        super().__init__(message)
        self.kind = kind


def camera_input_status(ip,port=8558,timeout=2):
    """카메라 앱(:8558)에 status를 물어 입력이 실제 카메라인지 확인한다.
    응답이 잘려 오면 소스 변경이 아니라 네트워크 오류(ConnectionError)로 구분한다.
    """
    from video_transport import send_message, read_message
    with socket.create_connection((ip, port), timeout=timeout) as connection:
        connection.settimeout(timeout)
        send_message(connection, {'op': 'status'})
        with connection.makefile('rb') as stream:
            # A reply cut off mid-header is a link fault, not a source change.
            try:result, _ = read_message(stream)
            except ValueError as exc:raise ConnectionError('상태 응답 손상: '+str(exc)) from exc
    if not result.get('ok') or result.get('source') != 'camera':
        raise RecorderFault('source_changed','장치가 카메라 입력을 확인하지 않았습니다. 영상 소스를 확인해 주세요.')
    return result


class SharedRgbRecorder:
    """카메라 한 대의 MP4 녹화기.

    guard: 주기적으로 소스 상태를 확인하는 함수. 일시적 네트워크 실패는 guard_failure_limit회 또는
    guard_grace_s초까지 허용하고, 소스 epoch/재연결 변화는 즉시 중단한다.
    """
    def __init__(self, source, session, ip, fps=10, size=None, guard=camera_input_status,
                 camera='CAM01', model='OAK 4D Pro PoE', input_kind='camera', filename='cam_01_rgb.mp4',
                 guard_interval=1., guard_failure_limit=3, guard_grace_s=8.,
                 health_interval=5., min_free_start=256*1024*1024, min_free_running=64*1024*1024,
                 storage_stall_s=30., storage_startup_grace_s=300., verify_output=True):
        self.source, self.session, self.ip = source, Path(session), ip
        self.fps, self.size, self.guard = fps, size, guard
        self.camera, self.model, self.input_kind, self.filename = camera, model, input_kind, filename
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.status = dict(state='대기', frames=0, repeated_frames=0, error='',
                           skipped_source_frames=0, received_frames=0, receive_fps=0., saved_fps=0.,
                           path=str(self.session/filename), error_kind='',
                           max_guard_ms=0., max_snapshot_ms=0., max_resize_ms=0.,
                           max_write_ms=0., max_schedule_lag_ms=0., guard_checks=0,
                           last_frame_age_s=0.)
        self.thread = None
        self.guard_fault = None
        self.guard_interval,self.guard_failure_limit,self.guard_grace_s=guard_interval,guard_failure_limit,guard_grace_s
        self.health_interval,self.min_free_start,self.min_free_running=health_interval,min_free_start,min_free_running
        self.storage_stall_s,self.storage_startup_grace_s=storage_stall_s,storage_startup_grace_s
        self.verify_output=verify_output
        self.guard_unverified_at=None
        self.receiver_epoch=None
        self.guard_events=queue.SimpleQueue()
        self.status.update(finalize_errors=[],source_verified=True,consecutive_guard_failures=0,
                           last_guard_error='',verification='pending',file_size_bytes=0,disk_free_bytes=None,
                           verification_warnings=0,unverified_seconds=0.)

    def snapshot(self):
        """현재 녹화 상태 dict의 복사본을 반환한다."""
        with self.lock:
            return dict(self.status,finalize_errors=list(self.status['finalize_errors']))

    def _fail(self,kind,message,secondary=False):
        """오류를 기록한다. 첫 오류만 대표 원인(error/error_kind)이 되고, 이후 오류는 finalize_errors에 추가된다."""
        with self.lock:
            if secondary or self.status['error']:
                self.status['finalize_errors'].append(dict(kind=kind,error=message))
            if not self.status['error']:
                self.status.update(state='오류',error_kind=kind,error=message)

    @property
    def active(self):
        """녹화 스레드가 실행 중이면 True."""
        return self.thread is not None and self.thread.is_alive()

    def start(self):
        """녹화 스레드를 시작한다(한 번만). daemon이 아니라서 앱 종료 시에도 파일 마무리를 끝낸다."""
        if self.thread is not None:
            raise RuntimeError('Recorder already started')
        with self.lock:
            self.status['state'] = '시작 중'
        self.thread = threading.Thread(target=self._run, name=self.camera.lower()+'-rgb-recording', daemon=False)
        self.thread.start()

    def stop(self, timeout=0):
        """녹화 중지를 요청한다. timeout을 주면 그만큼 종료를 기다리고, 종료됐는지 반환한다."""
        self.stop_event.set()
        if self.thread is not None and timeout:
            self.thread.join(timeout)
        return not self.active

    def _metric(self, key, seconds):
        """처리 시간 최댓값 지표(ms)를 갱신한다."""
        with self.lock:
            self.status[key] = max(self.status[key],round(seconds*1000,3))

    def _source_status(self):
        """guard를 호출하고 걸린 시간과 확인 횟수를 기록한다."""
        began=time.monotonic()
        try:
            return self.guard(self.ip)
        finally:
            self._metric('max_guard_ms',time.monotonic()-began)
            with self.lock:
                self.status['guard_checks']+=1

    def _monitor_source(self, baseline, done):
        # Control TCP is independent of the video recording clock. A slow but
        # successful status reply must not look like a slow MP4 write.
        """별도 스레드에서 guard_interval마다 소스 상태를 확인한다.
        영상 기록 루프와 분리해, 느린 상태 응답이 영상 쓰기 지연으로 보이지 않게 한다.
        """
        while not done.wait(self.guard_interval):
            if self.stop_event.is_set():
                return
            request_started=time.monotonic()
            try:
                current=self._source_status()
                if done.is_set():
                    return
                for key in ('source_epoch','reconnects'):
                    if current.get(key)!=baseline.get(key):
                        raise RecorderFault('source_changed','녹화 중 영상 소스가 전환되거나 재연결되었습니다.')
                with self.lock:
                    if done.is_set():return
                    recovered=self.guard_unverified_at is not None
                    uncertain_since=self.guard_unverified_at
                    if recovered:self.status['unverified_seconds']+=time.monotonic()-uncertain_since
                    self.guard_unverified_at=None
                    self.status.update(source_verified=True,consecutive_guard_failures=0,last_guard_error='')
                if recovered:self.guard_events.put(dict(event='source_status_recovered',
                    unverified_since_monotonic=uncertain_since))
            except Exception as exc:
                if done.is_set():
                    return
                kind=(exc.kind if isinstance(exc,RecorderFault) else
                      'source_status_timeout' if isinstance(exc,TimeoutError) else
                      'source_status_network' if isinstance(exc,(OSError,EOFError)) else 'source_status')
                transient=isinstance(exc,(OSError,EOFError)) and not isinstance(exc,RecorderFault)
                with self.lock:
                    if done.is_set():return
                    self.status.update(source_verified=False,last_guard_error=str(exc))
                    if transient:
                        if self.guard_unverified_at is None:self.guard_unverified_at=request_started
                        self.status['consecutive_guard_failures']+=1
                        self.status['verification_warnings']+=1
                        self.status.update(source_verified=False,last_guard_error=str(exc))
                        count=self.status['consecutive_guard_failures']
                        uncertain_since=self.guard_unverified_at
                        fatal=count>=self.guard_failure_limit or time.monotonic()-self.guard_unverified_at>=self.guard_grace_s
                    else:
                        count=0;fatal=True;uncertain_since=None
                    if fatal:self.guard_fault=(kind,str(exc))
                self.guard_events.put(dict(event='source_status_warning' if not fatal else 'source_status_failed',
                                           kind=kind,error=str(exc),consecutive_failures=count))
                self.guard_events.put(dict(event='source_verification_interval',
                    unverified_since_monotonic=uncertain_since,source_verified=False))
                if fatal:return

    def _check_source_status(self):
        """녹화 루프에서 호출: 감시 스레드가 치명적 오류를 남겼거나, 미확인 시간이 한도를 넘었거나, 수신기가 재연결됐으면 중단한다."""
        with self.lock:
            fault=self.guard_fault
            unverified=self.guard_unverified_at
        if fault:
            raise RecorderFault(*fault)
        if unverified is not None and time.monotonic()-unverified>=self.guard_grace_s:
            raise RecorderFault('source_verification_expired','소스 확인 불확실 상태가 허용 시간을 초과했습니다.')
        if self.receiver_epoch is not None and self.source.info().get('reconnects')!=self.receiver_epoch:
            raise RecorderFault('source_changed','녹화 중 RGB 수신기가 재연결되었습니다.')

    def _storage_check(self,path,now):
        """남은 디스크 공간과 MP4 파일 크기 증가를 확인한다. 초기 버퍼링을 고려해 시작 직후에는 긴 유예를 준다."""
        free=shutil.disk_usage(path.parent).free
        size=path.stat().st_size if path.exists() else 0
        with self.lock:self.status.update(file_size_bytes=size,disk_free_bytes=free)
        if free<self.min_free_running:raise RecorderFault('storage_space','저장 공간이 부족하여 녹화를 중단했습니다.')
        if size>self.storage_size:
            self.storage_size=size;self.storage_progress_at=now
        # FFmpeg can buffer small, low-bitrate files. Use a long startup grace,
        # then require progress only after substantial output has been observed.
        allowance=self.storage_stall_s if self.storage_size>=65536 else self.storage_startup_grace_s
        if now-self.storage_progress_at>allowance:
            raise RecorderFault('storage_no_progress','녹화 파일 크기가 장시간 증가하지 않습니다.')

    def _verify_file(self,path):
        """녹화가 끝난 MP4의 첫/마지막 프레임을 읽고 프레임 수가 기록 수와 같은지 검증한다."""
        capture=cv2.VideoCapture(str(path),cv2.CAP_FFMPEG)
        try:
            count=int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
            first,_=capture.read()
            capture.set(cv2.CAP_PROP_POS_FRAMES,max(0,count-1))
            last,_=capture.read()
            if not first or not last or count!=self.snapshot()['frames']:
                raise RecorderFault('storage_verify',f'MP4 검증 실패: 요청 {self.snapshot()["frames"]} / 파일 {count}프레임')
            with self.lock:self.status['verification']='passed'
        finally:capture.release()

    def _run(self):
        """녹화 본체: 상태 확인 → 세션 폴더·이벤트 로그·MP4 생성 → fps 간격으로 최신 프레임 기록 → 마무리.
        마무리 단계에서는 오류가 나도 writer 해제, 검증, 이벤트 기록, session.json 저장을 최대한 수행한다.
        """
        writer = None
        created = False
        started = None
        events = None
        monitor = None
        monitor_done = threading.Event()
        phase = 'configuration'
        def event(name, **details):
            """recording_events.jsonl에 이벤트 한 줄을 기록한다."""
            if events is not None:
                events.write(json.dumps(dict(event=name,wall_time=time.time(),
                    monotonic_s=time.monotonic(),camera=self.camera,**details),ensure_ascii=False)+'\n')
                events.flush()
        def guard_notices():
            """감시 스레드가 큐에 넣은 상태 이벤트를 이벤트 로그로 옮긴다."""
            while True:
                try:notice=self.guard_events.get_nowait()
                except queue.Empty:return
                event(notice.pop('event'),**notice)
        manifest = dict(camera=self.camera, model=self.model, input_kind=self.input_kind, ip=self.ip,
                        source_url=self.source.url, fps=self.fps, codec='mp4v',
                        pixel_format='BGR', clock='host_monotonic', synchronized=False,
                        started_wall_time=time.time())
        try:
            if self.fps not in (10, 15, 20, 30):
                raise ValueError('지원하는 녹화 FPS: 10, 15, 20, 30')
            phase='source_status'
            manifest['source_status'] = self._source_status()
            if hasattr(self.source,'info'):self.receiver_epoch=self.source.info().get('reconnects')
            phase='source_receive'
            connected, frame, sequence, received_at = self.source.record_snapshot()
            initial_sequence = sequence
            if not connected or frame is None or time.monotonic()-received_at > 3:
                raise RuntimeError(self.camera+' RGB 영상이 연결되지 않았거나 오래된 영상입니다.')
            size = self.size or (frame.shape[1], frame.shape[0])
            manifest['size'] = list(size)
            phase='storage_directory'
            self.session.mkdir(parents=True, exist_ok=False)
            created = True
            phase='storage_space'
            if shutil.disk_usage(self.session).free<self.min_free_start:
                raise RecorderFault('storage_space','녹화 시작에 필요한 저장 공간이 부족합니다.')
            phase='diagnostic_open'
            events=(self.session/'recording_events.jsonl').open('w',encoding='utf-8')
            event('start',source_status=manifest['source_status'])
            phase='storage_open'
            writer = cv2.VideoWriter(str(self.session/self.filename),
                                     cv2.VideoWriter_fourcc(*'mp4v'), self.fps, size)
            if not writer.isOpened():
                raise RuntimeError('MP4 파일을 열 수 없습니다. 저장 경로와 코덱을 확인해 주세요.')
            started = time.monotonic()
            deadline = started
            next_health = started+self.health_interval
            self.storage_size=0;self.storage_progress_at=started
            last_lag_warning=0.
            last_sequence = None
            monitor=threading.Thread(target=self._monitor_source,
                args=(manifest['source_status'],monitor_done),daemon=True,name=self.camera+'-status')
            monitor.start()
            with self.lock:
                self.status['state'] = '녹화 중'
            while not self.stop_event.is_set():
                guard_notices()
                self._check_source_status()
                if self.stop_event.wait(max(0, deadline-time.monotonic())):
                    break
                self._check_source_status()
                phase='source_receive'
                began=time.monotonic()
                connected, frame, sequence, received_at = self.source.record_snapshot()
                self._metric('max_snapshot_ms',time.monotonic()-began)
                with self.lock:
                    self.status['last_frame_age_s']=round(time.monotonic()-received_at,3)
                if not connected or frame is None or time.monotonic()-received_at > 3:
                    raise RuntimeError('RGB 연결이 끊겼습니다. 수신한 녹화 파일을 마무리합니다.')
                if (frame.shape[1], frame.shape[0]) != size:
                    phase='storage_resize'
                    began=time.monotonic()
                    frame = cv2.resize(frame, size)
                    self._metric('max_resize_ms',time.monotonic()-began)
                phase='storage_write'
                began=time.monotonic()
                writer.write(frame)
                self._metric('max_write_ms',time.monotonic()-began)
                with self.lock:
                    self.status['frames'] += 1
                    self.status['repeated_frames'] += int(last_sequence == sequence)
                    if last_sequence is not None:
                        self.status['skipped_source_frames'] += max(0, sequence-last_sequence-1)
                    elapsed = max(.001, time.monotonic()-started)
                    self.status['received_frames'] = max(0, sequence-initial_sequence)
                    self.status['receive_fps'] = round(self.status['received_frames']/elapsed, 2)
                    self.status['saved_fps'] = round(self.status['frames']/elapsed, 2)
                last_sequence = sequence
                # 다음 프레임 기록 시각. 밀려도 빨리 감기로 따라잡지 않고 지연(lag)으로 측정한다.
                deadline += 1/self.fps
                lag=max(0.,time.monotonic()-deadline)
                self._metric('max_schedule_lag_ms',lag)
                if lag>.25 and time.monotonic()-last_lag_warning>1:
                    event('schedule_delay_warning',lag_s=lag,status=self.snapshot())
                    last_lag_warning=time.monotonic()
                # Do not speed up to fill a disk/network stall with many copies.
                if lag>1:
                    raise RecorderFault('recording_schedule_overrun',
                        '녹화 처리가 1초 이상 밀려 중단했습니다. recording_events.jsonl의 처리 시간을 확인해 주세요.')
                if time.monotonic()>=next_health:
                    phase='storage_progress'
                    self._storage_check(self.session/self.filename,time.monotonic())
                    phase='diagnostic_write'
                    event('health',elapsed_s=time.monotonic()-started,status=self.snapshot())
                    next_health=time.monotonic()+self.health_interval
        except Exception as exc:
            self._fail(exc.kind if isinstance(exc,RecorderFault) else phase,str(exc))
        finally:
            ended_at=time.monotonic()
            if started is not None:
                with self.lock:
                    elapsed = max(.001, ended_at-started)
                    self.status['receive_fps'] = round(self.status['received_frames']/elapsed, 2)
                    self.status['saved_fps'] = round(self.status['frames']/elapsed, 2)
            monitor_done.set()
            with self.lock:
                if self.guard_unverified_at is not None:
                    self.status['unverified_seconds']+=max(0.,ended_at-self.guard_unverified_at)
            if writer is not None:
                try:writer.release()
                except Exception as exc:
                    self._fail('storage_finalize',str(exc),secondary=True)
            if writer is not None and self.snapshot()['frames'] and not self.snapshot()['error']:
                try:
                    if self.verify_output:self._verify_file(self.session/self.filename)
                    else:
                        with self.lock:self.status['verification']='skipped'
                except Exception as exc:self._fail('storage_verify',str(exc),secondary=True)
            if monitor:monitor.join(timeout=4.5)
            if started is not None:
                manifest['duration_s']=ended_at-started
                manifest['video_duration_s']=self.snapshot()['frames']/self.fps
            if hasattr(self.source,'info'):
                try:manifest['source_diagnostics']=self.source.info()
                except Exception as exc:self._fail('source_diagnostics',str(exc),secondary=True)
            with self.lock:
                if self.status['verification']=='pending':
                    self.status['verification']='skipped_due_error' if self.status['error'] else 'not_recorded'
                if self.status['state'] != '오류':
                    self.status['state'] = '저장 완료' if self.status['frames'] else '녹화 취소'
                manifest.update(self.status)
            manifest['finished_wall_time'] = time.time()
            if events is not None:
                # The monitor queues its fatal notice after setting the fault;
                # it has been joined above, so record it before finishing.
                try:guard_notices()
                except OSError as exc:self._fail('diagnostic_finalize',str(exc),secondary=True)
                try:event('finish',status=self.snapshot(),duration_s=manifest.get('duration_s'))
                except OSError as exc:self._fail('diagnostic_finalize',str(exc),secondary=True)
                try:events.close()
                except OSError as exc:
                    self._fail('diagnostic_finalize',str(exc),secondary=True)
                manifest.update(self.snapshot())
            if created:
                try:
                    if not manifest['frames']:
                        (self.session/self.filename).unlink(missing_ok=True)
                    # 임시 파일에 쓴 뒤 교체해서 session.json이 반쯤 쓰인 상태로 남지 않게 한다.
                    temporary=self.session/'session.json.tmp'
                    temporary.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
                    temporary.replace(self.session/'session.json')
                except OSError as exc:
                    self._fail('metadata_write',f'세션 정보 저장 실패: {exc}',secondary=True)


class RecordingGroup:
    """Start selected recorders together; one failure stops the whole group."""
    # 선택한 여러 카메라를 한 세션 폴더 아래에서 함께 녹화한다. 한 대라도 실패하면 전체를 멈춘다.
    def __init__(self, specs, session, fps=10, size=None, verify_output=True):
        self.specs, self.session = list(specs), Path(session)
        self.fps, self.size = fps, size
        self.recorders = {}
        self.saved = False
        self.created = False
        self.error = ''
        self.verify_output=verify_output

    @property
    def active(self):
        """그룹 안 녹화기 중 하나라도 실행 중이면 True."""
        return any(recorder.active for recorder in self.recorders.values())

    def start(self):
        """모든 카메라가 연결돼 있는지 먼저 확인한 뒤 세션 폴더를 만들고 녹화기를 시작한다.
        중간에 실패하면 이미 시작한 녹화기를 멈추고 예외를 다시 낸다.
        """
        if not self.specs:
            raise ValueError('녹화할 카메라를 선택해 주세요.')
        for spec in self.specs:
            connected, frame, _, received_at = spec['source'].record_snapshot()
            if not connected or frame is None or time.monotonic()-received_at>3:
                raise RuntimeError(spec['camera']+' 연결을 먼저 완료해 주세요.')
        self.session.mkdir(parents=True, exist_ok=False)
        self.created = True
        try:
            for spec in self.specs:
                name = spec['camera']
                recorder = SharedRgbRecorder(spec['source'], self.session/name.lower(),
                    spec['ip'], fps=self.fps, size=self.size, guard=spec['guard'], camera=name,
                    model=spec['model'], input_kind=spec['input_kind'],
                    filename=f'cam_{int(name[-2:]):02d}_rgb.mp4',verify_output=self.verify_output)
                self.recorders[name] = recorder
                recorder.start()
        except Exception as exc:
            self.error='그룹 시작 실패: '+str(exc)
            self.stop(timeout=6)
            raise

    def poll(self):
        """녹화기 상태를 모아 반환한다. 오류가 있으면 전체를 멈추고, 모두 끝나면 recording_group.json을 한 번 기록한다."""
        states = {name: recorder.snapshot() for name,recorder in self.recorders.items()}
        failures = [name+': '+state['error'] for name,state in states.items() if state['error']]
        if failures:
            self.error = ' / '.join(failures)
            for recorder in self.recorders.values():
                recorder.stop()
        if self.created and not self.active and not self.saved:
            states = {name: recorder.snapshot() for name,recorder in self.recorders.items()}
            payload = dict(synchronized=False, state='오류' if self.error else '종료',
                           error=self.error, cameras=states,
                           inputs={spec['camera']:spec['input_kind'] for spec in self.specs})
            try:
                temporary=self.session/'recording_group.json.tmp'
                temporary.write_text(json.dumps(payload,ensure_ascii=False,indent=2),encoding='utf-8')
                temporary.replace(self.session/'recording_group.json')
                self.saved = True
            except OSError as exc:
                self.error = '그룹 기록 저장 실패: '+str(exc)
        return states

    def stop(self, timeout=0):
        """모든 녹화기를 멈추고 timeout 안에서 종료를 기다린 뒤 그룹 기록을 갱신한다."""
        for recorder in self.recorders.values():
            recorder.stop()
        end = time.monotonic()+timeout
        for recorder in self.recorders.values():
            if timeout:
                recorder.stop(timeout=max(.001,end-time.monotonic()))
        self.poll()
        return not self.active

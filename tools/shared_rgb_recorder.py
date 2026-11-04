"""Independent MP4 recording from the existing RGB receiver, without owning it."""
import json
from pathlib import Path
import socket
import shutil
import queue
import threading
import time
import cv2


class RecorderFault(RuntimeError):
    def __init__(self, kind, message):
        super().__init__(message)
        self.kind = kind


def camera_input_status(ip,port=8558,timeout=2):
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
        with self.lock:
            return dict(self.status,finalize_errors=list(self.status['finalize_errors']))

    def _fail(self,kind,message,secondary=False):
        with self.lock:
            if secondary or self.status['error']:
                self.status['finalize_errors'].append(dict(kind=kind,error=message))
            if not self.status['error']:
                self.status.update(state='오류',error_kind=kind,error=message)

    @property
    def active(self):
        return self.thread is not None and self.thread.is_alive()

    def start(self):
        if self.thread is not None:
            raise RuntimeError('Recorder already started')
        with self.lock:
            self.status['state'] = '시작 중'
        self.thread = threading.Thread(target=self._run, name=self.camera.lower()+'-rgb-recording', daemon=False)
        self.thread.start()

    def stop(self, timeout=0):
        self.stop_event.set()
        if self.thread is not None and timeout:
            self.thread.join(timeout)
        return not self.active

    def _metric(self, key, seconds):
        with self.lock:
            self.status[key] = max(self.status[key],round(seconds*1000,3))

    def _source_status(self):
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
        writer = None
        created = False
        started = None
        events = None
        monitor = None
        monitor_done = threading.Event()
        phase = 'configuration'
        def event(name, **details):
            if events is not None:
                events.write(json.dumps(dict(event=name,wall_time=time.time(),
                    monotonic_s=time.monotonic(),camera=self.camera,**details),ensure_ascii=False)+'\n')
                events.flush()
        def guard_notices():
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
                    temporary=self.session/'session.json.tmp'
                    temporary.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
                    temporary.replace(self.session/'session.json')
                except OSError as exc:
                    self._fail('metadata_write',f'세션 정보 저장 실패: {exc}',secondary=True)


class RecordingGroup:
    """Start selected recorders together; one failure stops the whole group."""
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
        return any(recorder.active for recorder in self.recorders.values())

    def start(self):
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
        for recorder in self.recorders.values():
            recorder.stop()
        end = time.monotonic()+timeout
        for recorder in self.recorders.values():
            if timeout:
                recorder.stop(timeout=max(.001,end-time.monotonic()))
        self.poll()
        return not self.active

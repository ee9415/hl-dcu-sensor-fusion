"""Mixed OAK4/OAK-D recorder with explicit per-camera real/virtual inputs."""
# 멀티뷰 앱의 '6대 카메라 녹화' 탭(Tkinter UI).
# CAM01은 OAK4 순수 RGB, CAM02~06은 실제 OAK-D 또는 가상 영상이며, 카메라마다 입력 종류를 명시적으로 고른다.
# AI 처리와 하드웨어 동기화는 하지 않는다.
from datetime import datetime
from pathlib import Path
import re
import os
import queue
import sys
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import uuid
import cv2
from PIL import Image, ImageTk
from shared_rgb_recorder import RecordingGroup
from oakd_rgb_source import OakdRgbSource
from camera_discovery import discover_cameras
import camera_settings
from console_log import action, state_event


class SixCameraTab:
    """6대 카메라의 설정·연결·미리보기·그룹 녹화를 관리하는 탭.
    owner는 메인 앱(RecorderApp)이며 CAM01 소유권 전환, 녹화 상태 확인 등에 쓰인다.
    """
    def __init__(self, root, parent, device_ip, device_serial, output_dir, owner=None,
                 settings_path=None,restore_cam01=False):
        """UI 위젯과 카메라 6대 상태를 만들고 저장된 카메라 설정을 불러온다(처음 실행이면 예전 녹화에서 가져옴)."""
        self.root = root
        self.owner = owner
        self.recorder = None
        self.group = None
        self.extra_sources = {}
        self.discovery_events = queue.SimpleQueue()
        self.discovery_pending = False
        self.discovered_devices = []
        runtime = Path(__file__).resolve().parents[1]/'artifacts/oakd_runtime/.venv'
        local_python = runtime/('Scripts/python.exe' if sys.platform=='win32' else 'bin/python')
        self.oakd_python = tk.StringVar(value=str(local_python) if local_python.exists() else sys.executable)
        custom_settings=settings_path or os.environ.get('OAK_CAMERA_SETTINGS_PATH')
        self.settings_path=Path(custom_settings) if custom_settings else camera_settings.DEFAULT_PATH
        self.auto_migrate=not bool(custom_settings)
        if self.auto_migrate:
            try:camera_settings.settings_store.migrate(self.settings_path)
            except (OSError,ValueError):pass
        self.restore_cam01=restore_cam01
        self.settings_reset_active=False
        self.settings_defaults=camera_settings.defaults(device_ip,device_serial,self.oakd_python.get())
        self.settings_status=tk.StringVar(value='카메라 설정을 저장하면 다음 실행에 사용합니다.')
        self.preview = None
        self.cameras = []
        self.windows = {}
        self.output_dir = tk.StringVar(value=str(Path(output_dir) / 'six_camera'))
        self.session_name = tk.StringVar(value='six_camera')
        self.resolution = tk.StringVar(value='원본 스트림')
        self.fps = tk.StringVar(value='10')
        self.verify_output=tk.BooleanVar(value=True)
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(1, weight=1)
        header = ttk.Frame(parent, style='Header.TFrame', padding=(18, 12))
        header.grid(row=0, column=0, sticky='ew')
        header.columnconfigure(0, weight=1)
        ttk.Label(header, text='6대 카메라 녹화', style='Title.TLabel').grid(row=0, column=0, sticky='w')
        ttk.Label(header, text='1번 OAK4 순수 RGB · 2~6번 OAK-D / 가상 영상 · AI 처리 없음 · 하드웨어 동기화 없음',
                  style='Subtitle.TLabel').grid(row=1, column=0, sticky='w', pady=(4, 0))
        menu = ttk.Frame(header, style='Header.TFrame')
        menu.grid(row=2, column=0, sticky='w', pady=(10, 0))
        for key, title in (('devices', '카메라 설정'), ('recording', '녹화 설정')):
            ttk.Button(menu, text=title, command=lambda key=key: self.show_settings(key)).pack(
                side=tk.LEFT, padx=(0, 6))
        self.connect_button = ttk.Button(menu, text='선택 카메라 연결', command=self.connect_selected,
                                        state=tk.NORMAL if owner else tk.DISABLED)
        self.connect_button.pack(side=tk.LEFT, padx=6)

        previews = ttk.Frame(parent, style='App.TFrame', padding=(14, 12))
        previews.grid(row=1, column=0, sticky='nsew')
        for col in range(3):
            previews.columnconfigure(col, weight=1, uniform='six-columns')
        for row in range(2):
            previews.rowconfigure(row, weight=1, uniform='six-rows')
        for index in range(6):
            model = 'OAK 4D Pro PoE' if index == 0 else 'OAK-D Pro PoE'
            camera = dict(name=f'CAM{index+1:02d}', model=model,
                          ip=tk.StringVar(value=device_ip if index == 0 else ''),
                          serial=tk.StringVar(value=device_serial if index == 0 else ''),
                          selected=tk.BooleanVar(value=index == 0), status=tk.StringVar(),
                          kind=tk.StringVar(value='OAK4 공유' if index==0 else '실제 OAK-D'))
            panel = ttk.Frame(previews, style='Panel.TFrame', padding=8)
            panel.grid(row=index//3, column=index%3, sticky='nsew', padx=4, pady=4)
            panel.columnconfigure(0, weight=1)
            panel.rowconfigure(2, weight=1)
            check = ttk.Checkbutton(panel, text=f'{camera["name"]} · 녹화 선택',
                variable=camera['selected'], style='App.TCheckbutton',
                command=self._refresh)
            check.grid(row=0, column=0, sticky='w')
            camera['check'] = check
            ttk.Label(panel, text=model, style='PanelTitle.TLabel').grid(row=1, column=0, sticky='w', pady=(4, 6))
            preview = ttk.Label(panel, text='영상 미연결',
                                anchor='center', style='Video.TLabel')
            preview.grid(row=2, column=0, sticky='nsew')
            if index == 0:
                self.preview = preview
            camera['preview'] = preview
            ttk.Label(panel, textvariable=camera['status'], style='PanelStatus.TLabel',
                      wraplength=290).grid(row=3, column=0, sticky='w', pady=(8, 0))
            buttons = ttk.Frame(panel, style='Panel.TFrame')
            buttons.grid(row=4, column=0, sticky='w', pady=(4,0))
            camera['connect'] = ttk.Button(buttons, text='연결', command=lambda i=index:self.connect_camera(i))
            camera['connect'].pack(side=tk.LEFT, padx=(0,4))
            camera['disconnect'] = ttk.Button(buttons, text='해제',
                state=tk.NORMAL,
                command=lambda i=index:self.disconnect_camera(i))
            camera['disconnect'].pack(side=tk.LEFT)
            self.cameras.append(camera)

        footer = ttk.Frame(parent, style='App.TFrame', padding=(18, 6, 18, 14))
        footer.grid(row=2, column=0, sticky='ew')
        footer.columnconfigure(0, weight=1)
        self.selection_status = tk.StringVar()
        ttk.Label(footer, textvariable=self.selection_status, style='App.TLabel').grid(row=0, column=0, sticky='w')
        self.start_button = ttk.Button(footer, text='선택 카메라 녹화 시작', command=self.start_recording, state=tk.DISABLED)
        self.start_button.grid(row=0, column=1, padx=4)
        self.stop_button = ttk.Button(footer, text='녹화 정지', command=self.stop_recording, state=tk.DISABLED)
        self.stop_button.grid(row=0, column=2, padx=4)
        self.save_status = tk.StringVar()
        ttk.Label(footer, textvariable=self.save_status, style='App.TLabel', wraplength=980).grid(
            row=1, column=0, columnspan=3, sticky='ew', pady=(8, 0))
        self._load_camera_settings()
        self._build_device_settings()
        self._build_recording_settings()
        self._refresh()

    def _window(self, key, title):
        """설정용 보조 창(닫으면 숨김)을 만든다."""
        window = tk.Toplevel(self.root)
        window.withdraw()
        window.title(title)
        window.transient(self.root)
        window.configure(background='#0b1118')
        window.resizable(True, False)
        window.columnconfigure(0, weight=1)
        window.protocol('WM_DELETE_WINDOW', window.withdraw)
        window.bind('<Escape>', lambda event: window.withdraw())
        self.windows[key] = window
        return window

    def _build_device_settings(self):
        """'카메라 설정' 창: 카메라별 IP·장치 ID·입력 종류, OAK-D Python 경로, 장치 검색 결과 표를 만든다."""
        window = self._window('devices', '6대 카메라 설정')
        fields = ttk.Frame(window, style='App.TFrame', padding=16)
        fields.grid(sticky='ew')
        fields.columnconfigure(2, weight=1)
        fields.columnconfigure(3, weight=1)
        for col, title in enumerate(('카메라', '모델', 'IP 주소', '장치 ID / MX ID', '입력 종류')):
            ttk.Label(fields, text=title, style='App.TLabel').grid(row=0, column=col, sticky='w', padx=6, pady=6)
        for row, camera in enumerate(self.cameras, 1):
            for col, value in enumerate((camera['name'], camera['model'])):
                ttk.Label(fields, text=value, style='App.TLabel').grid(row=row, column=col, sticky='w', padx=6, pady=6)
            ttk.Entry(fields, textvariable=camera['ip'], width=19).grid(row=row, column=2, sticky='ew', padx=6, pady=6)
            ttk.Entry(fields, textvariable=camera['serial'], width=25).grid(row=row, column=3, sticky='ew', padx=6, pady=6)
            if row==1:
                ttk.Label(fields,text='OAK4 순수 RGB',style='App.TLabel').grid(row=row,column=4,padx=6)
            else:
                ttk.Combobox(fields,textvariable=camera['kind'],values=('실제 OAK-D','가상 영상'),
                             state='readonly',width=12).grid(row=row,column=4,padx=6)
        ttk.Label(fields,text='OAK-D Python',style='App.TLabel').grid(row=7,column=0,sticky='w',padx=6)
        ttk.Entry(fields,textvariable=self.oakd_python).grid(row=7,column=1,columnspan=3,sticky='ew',padx=6,pady=8)
        ttk.Button(fields,text='경로 선택',command=self._choose_python).grid(row=7,column=4,padx=6)
        ttk.Label(fields, text='실제 2~6번은 DepthAI 2 환경 필요 · 가상 입력은 VIRTUAL 표시 · 설정 변경 후 해제/재연결',
                  style='App.TLabel').grid(row=8, column=0, columnspan=5, sticky='w', pady=10)
        ttk.Button(fields, text='확인', command=lambda: self._confirm('devices')).grid(row=9, column=4, sticky='e')
        self.find_button=ttk.Button(fields,text='카메라 찾기',command=self.find_cameras)
        self.find_button.grid(row=9,column=0,sticky='w',padx=6,pady=6)
        self.discovery_status=tk.StringVar(value='장치 검색 후 결과를 선택해 카메라 번호에 적용')
        ttk.Label(fields,textvariable=self.discovery_status,style='App.TLabel',wraplength=520).grid(
            row=9,column=1,columnspan=3,sticky='w')
        self.discovery_table=ttk.Treeview(fields,columns=('model','ip','serial','state'),show='headings',height=4)
        for name,title,width in (('model','종류',140),('ip','IP 주소',130),('serial','장치 ID / MX ID',200),('state','상태',150)):
            self.discovery_table.heading(name,text=title)
            self.discovery_table.column(name,width=width,minwidth=80)
        self.discovery_table.grid(row=10,column=0,columnspan=5,sticky='ew',padx=6,pady=6)
        assign=ttk.Frame(fields,style='App.TFrame')
        assign.grid(row=11,column=0,columnspan=5,sticky='w',padx=6)
        ttk.Label(assign,text='선택한 검색 결과 →',style='App.TLabel').pack(side=tk.LEFT)
        self.discovery_target=tk.StringVar(value='CAM02')
        ttk.Combobox(assign,textvariable=self.discovery_target,values=tuple(c['name'] for c in self.cameras),
                     state='readonly',width=9).pack(side=tk.LEFT,padx=6)
        self.assign_button=ttk.Button(assign,text='설정에 적용',command=self.apply_discovered)
        self.assign_button.pack(side=tk.LEFT)
        self.discovery_table.bind('<<TreeviewSelect>>',self._discovery_selected)
        ttk.Button(assign,text='카메라 정보 저장',command=self.save_camera_settings).pack(side=tk.LEFT,padx=6)
        self.reset_settings_button=ttk.Button(assign,text='카메라 정보 초기화',command=self.reset_camera_settings)
        self.reset_settings_button.pack(side=tk.LEFT)
        ttk.Label(fields,textvariable=self.settings_status,style='App.TLabel',wraplength=720).grid(
            row=12,column=0,columnspan=5,sticky='w',padx=6,pady=(8,0))

    def _camera_settings(self):
        """현재 UI 값을 camera_settings 저장 형식 dict로 만든다."""
        return dict(version=1,oakd_python=self.oakd_python.get(),reset=self.settings_reset_active,cameras=[
            dict(name=c['name'],ip=c['ip'].get(),serial=c['serial'].get(),kind=c['kind'].get(),
                 selected=bool(c['selected'].get())) for c in self.cameras])

    def _apply_camera_settings(self,data):
        """저장된 설정 값을 UI에 반영한다."""
        self.settings_reset_active=data.get('reset',False)
        for camera,saved in zip(self.cameras,data['cameras']):
            for key in ('ip','serial','kind','selected'):camera[key].set(saved[key])
        if data['oakd_python'] and Path(data['oakd_python']).exists():
            self.oakd_python.set(data['oakd_python'])

    def _load_camera_settings(self):
        """시작 시 카메라 설정을 불러온다. 설정이 없으면 예전 녹화 기록에서 가져와 저장한다.
        restore_cam01이 아니면 CAM01 IP/ID는 실행 인자 값을 우선한다.
        """
        try:
            data=camera_settings.load(self.settings_path)
            if data is None and self.auto_migrate:
                data,count=camera_settings.migrate_recordings(
                    Path(__file__).resolve().parents[1]/'artifacts/recordings/six_camera',self.settings_defaults)
                if count:
                    camera_settings.save(self.settings_path,data)
                    self.settings_status.set(f'기존 실제 녹화에서{count}대 등록 정보를 가져와 저장했습니다.')
                else:data=None
            elif data is not None:self.settings_status.set('저장된 카메라 정보를 불러왔습니다.')
            if data is not None:
                if not self.restore_cam01:
                    data['cameras'][0]['ip']=self.settings_defaults['cameras'][0]['ip']
                    data['cameras'][0]['serial']=self.settings_defaults['cameras'][0]['serial']
                self._apply_camera_settings(data)
        except (OSError,ValueError,TypeError) as exc:
            self.settings_status.set('설정 불러오기 실패 · 기존 파일 유지: '+str(exc))

    @action(lambda self, notify=True: '카메라 정보 저장 요청' if notify else None)
    def save_camera_settings(self,notify=True):
        """UI의 카메라 정보를 저장한다. 연결 전이라면 메인 앱의 CAM01 IP도 함께 갱신한다."""
        try:
            data=self._camera_settings()
            if any(c['ip'].strip() or c['serial'].strip() for c in data['cameras']):data['reset']=False
            camera_settings.save(self.settings_path,data)
            self.settings_reset_active=data.get('reset',False)
            if self.owner and not self.owner.connect_requested and hasattr(self.owner,'ip_text'):
                self.owner.ip_text.set(data['cameras'][0]['ip'])
                self.owner.device_ip=data['cameras'][0]['ip']
                self.owner.device_serial=data['cameras'][0]['serial']
            self.settings_status.set('카메라 정보 저장됨 · '+str(self.settings_path))
            return True
        except (OSError,ValueError,TypeError) as exc:
            self.settings_status.set('카메라 정보 저장 실패: '+str(exc))
            if notify:messagebox.showerror('카메라 설정 저장',str(exc),parent=self.windows.get('devices'))
            return False

    @action('카메라 정보 초기화 요청')
    def reset_camera_settings(self):
        """저장된 카메라 정보를 초기화한다(녹화 중에는 불가, 녹화 파일과 현재 연결은 유지)."""
        if self.is_recording or (self.owner and self.owner.recording):
            messagebox.showwarning('카메라 정보 초기화','녹화를 정지한 뒤 초기화해 주세요.')
            return
        if not messagebox.askyesno('카메라 정보 초기화',
            '저장된 IP·장치ID·입력 종류를 초기화할까요?\n녹화 파일은 유지하고, 현재 수신 중인 연결도 끊지 않습니다.',
            parent=self.windows['devices']):return
        data=camera_settings.defaults(python=self.settings_defaults['oakd_python'])
        data['reset']=True
        try:camera_settings.save(self.settings_path,data)
        except (OSError,ValueError) as exc:
            messagebox.showerror('카메라 정보 초기화',str(exc));return
        self._apply_camera_settings(data)
        if self.owner and not self.owner.connect_requested:
            self.owner.ip_text.set('');self.owner.device_ip='';self.owner.device_serial=''
        self._refresh()
        self.settings_status.set('저장된 카메라 정보 초기화 완료 · 새 연결/녹화 전에 다시 설정해 주세요.')

    @action('카메라 검색 요청')
    def find_cameras(self):
        """백그라운드 스레드에서 카메라 검색을 시작한다. 결과는 _drain_discovery가 UI에 반영한다."""
        if self.discovery_pending or not self.owner:
            return
        self.discovery_pending=True
        self.find_button.configure(state=tk.DISABLED)
        self.discovery_status.set('카메라 검색 중… 앱·녹화 상태는 변경하지 않습니다.')
        python=self.oakd_python.get()
        # Import here to keep the UI module independent of its owning app.
        from oak_recorder import OAKCTL_PATH
        def search():
            """검색 결과(또는 오류)를 큐에 넣는다."""
            try:self.discovery_events.put(discover_cameras(OAKCTL_PATH,python))
            except Exception as exc:self.discovery_events.put(([],[str(exc)]))
        threading.Thread(target=search,daemon=True,name='camera-discovery').start()

    def _drain_discovery(self):
        """검색 결과가 도착했으면 표를 갱신한다(UI 스레드에서 호출)."""
        try:devices,warnings=self.discovery_events.get_nowait()
        except queue.Empty:return
        self.discovery_pending=False
        self.find_button.configure(state=tk.NORMAL)
        self.discovered_devices=devices
        self.discovery_table.delete(*self.discovery_table.get_children())
        for index,device in enumerate(devices):
            self.discovery_table.insert('',tk.END,iid=str(index),values=(device['model'],device['ip'],device['serial'],device['state']))
        self.discovery_status.set(f'검색 {len(devices)}대'+(' · '+' / '.join(warnings)[:260] if warnings else ''))
        if devices:
            self.discovery_table.selection_set('0')

    def _discovery_selected(self,event=None):
        """검색 결과를 고르면 OAK4는 CAM01, OAK-D는 CAM02를 기본 대상으로 제안한다."""
        selection=self.discovery_table.selection()
        if selection:
            device=self.discovered_devices[int(selection[0])]
            self.discovery_target.set('CAM01' if device['family']=='oak4' else 'CAM02')

    @action('검색한 카메라 설정 적용')
    def apply_discovered(self):
        """선택한 검색 결과를 지정 카메라 번호에 적용하고 저장한다.
        종류가 맞지 않거나, 연결 중이거나, 다른 번호에 이미 등록된 장치면 거부한다.
        """
        selection=self.discovery_table.selection()
        if not selection or self.is_recording:
            return
        index=int(self.discovery_target.get()[-2:])-1
        device=self.discovered_devices[int(selection[0])]
        if (index==0)!=(device['family']=='oak4'):
            messagebox.showwarning('검색 결과 적용','OAK4는1번, OAK-D는2~6번에 적용해 주세요.')
            return
        if index in self.extra_sources:
            messagebox.showwarning('검색 결과 적용','해당 카메라의 수신 연결을 해제한 뒤 설정을 적용해 주세요.')
            return
        if index==0 and self.owner.connect_requested and device['ip']!=self.owner.device_ip:
            messagebox.showwarning('검색 결과 적용','이미 연결된1번 장치를 다른 장치로 바꿀 수 없습니다.')
            return
        for other_index,camera in enumerate(self.cameras):
            if index==other_index:continue
            if camera['serial'].get().strip()==device['serial'] or (
                device['ip'] and camera['ip'].get().strip()==device['ip']):
                messagebox.showwarning('검색 결과 적용','이 장치는 이미 다른 카메라 번호에 설정돼 있습니다.')
                return
        camera=self.cameras[index]
        camera['ip'].set(device['ip'])
        camera['serial'].set(device['serial'])
        if index:
            camera['kind'].set('실제 OAK-D')
        self._refresh()
        saved=self.save_camera_settings()
        self.discovery_status.set(camera['name']+' 설정에 적용'+('·저장했습니다.' if saved else '했습니다. 저장은 실패했습니다.'))

    def _build_recording_settings(self):
        """'녹화 설정' 창: 저장 폴더, 세션 이름, 해상도, FPS, 종료 후 MP4 검증 여부."""
        window = self._window('recording', '6대 녹화 설정')
        fields = ttk.Frame(window, style='App.TFrame', padding=16)
        fields.grid(sticky='ew')
        fields.columnconfigure(1, weight=1)
        for row, (label, variable) in enumerate((('저장 폴더', self.output_dir), ('세션 이름', self.session_name))):
            ttk.Label(fields, text=label, style='App.TLabel').grid(row=row, column=0, sticky='w', padx=6, pady=8)
            ttk.Entry(fields, textvariable=variable, width=55).grid(row=row, column=1, sticky='ew', padx=6, pady=8)
        ttk.Button(fields, text='폴더 선택', command=self._choose_directory).grid(row=0, column=2, padx=6)
        for row, (label, variable, values) in enumerate((
            ('저장 해상도', self.resolution, ('원본 스트림', '1920×1080', '1280×720')),
            ('저장 FPS', self.fps, ('10', '15', '20', '30'))), 2):
            ttk.Label(fields, text=label, style='App.TLabel').grid(row=row, column=0, sticky='w', padx=6, pady=8)
            ttk.Combobox(fields, textvariable=variable, values=values, state='readonly', width=18).grid(
                row=row, column=1, sticky='w', padx=6, pady=8)
        ttk.Label(fields, text='MP4 저장 · 해상도/FPS는 PC 저장 설정 · 수신보다 높은 FPS는 프레임 반복 · 하드웨어 동기화 없음',
                  style='App.TLabel').grid(row=4, column=0, columnspan=3, sticky='w', pady=10)
        ttk.Checkbutton(fields,text='종료 후 MP4 프레임 수·첫/마지막 프레임 검증',
                        variable=self.verify_output,style='App.TCheckbutton').grid(row=5,column=0,columnspan=2,sticky='w')
        ttk.Button(fields, text='확인', command=lambda: self._confirm('recording')).grid(row=5, column=2, sticky='e')

    def _choose_directory(self):
        """녹화 저장 폴더 선택 대화상자."""
        path = filedialog.askdirectory(parent=self.windows['recording'], title='6대 녹화 저장 폴더')
        if path:
            self.output_dir.set(path)

    def _choose_python(self):
        """DepthAI 2 Python 실행 파일 선택 대화상자."""
        path = filedialog.askopenfilename(parent=self.windows['devices'],title='DepthAI 2 Python 실행 파일')
        if path:
            self.oakd_python.set(path)

    @action(lambda self, key: f'6대 카메라 설정 확인 · {key}')
    def _confirm(self, key):
        """설정 창의 '확인': 설정을 저장하고 창을 닫는다."""
        if key=='devices' and not self.save_camera_settings():return
        if key=='recording' and self.owner is not None:
            if not self.owner._save_preferences():return
        self._refresh()
        self.windows[key].withdraw()

    def _refresh(self):
        """카메라별 상태 문구와 선택 개수, 저장 위치 표시를 갱신한다."""
        for camera in self.cameras:
            address = camera['ip'].get().strip() or camera['serial'].get().strip()
            camera['status'].set((address or '장치 미설정') + ' · '
                                 + ('미연결' if camera is self.cameras[0] else camera['kind'].get()))
        count = sum(camera['selected'].get() for camera in self.cameras)
        self.selection_status.set(f'녹화 선택 {count}/6대')
        self.save_status.set(f'저장 폴더: {self.output_dir.get()} · 세션: {self.session_name.get()}')

    @property
    def is_recording(self):
        """그룹 녹화가 진행 중이면 True."""
        return self.group is not None and self.group.active

    @property
    def is_cam01_recording(self):
        """CAM01 녹화가 진행 중이면 True."""
        return self.recorder is not None and self.recorder.active

    @action('선택 카메라 연결 요청')
    def connect_selected(self):
        """녹화 선택된 카메라를 모두 연결한다."""
        for index,camera in enumerate(self.cameras):
            if camera['selected'].get():
                self.connect_camera(index)

    @action(lambda self, index: f'CAM{index+1:02d} 연결 요청')
    def connect_camera(self, index):
        """카메라 한 대를 연결한다. CAM01은 connect_cam01로 넘기고, OAK-D는 IP/MX ID 중복을 검사한 뒤 수신기를 시작한다."""
        if self.is_recording:
            return
        if index == 0:
            self.connect_cam01()
            return
        camera = self.cameras[index]
        virtual = camera['kind'].get() == '가상 영상'
        ip, mxid = camera['ip'].get().strip(), camera['serial'].get().strip()
        if not virtual:
            if not ip and not mxid:
                messagebox.showwarning('카메라 연결',camera['name']+' IP 또는 MX ID를 입력해 주세요.')
                return
            if self.owner and self.owner.connect_requested and ip and ip==self.owner.device_ip:
                messagebox.showwarning('카메라 연결','현재 OAK4의 IP를 OAK-D에 사용할 수 없습니다.')
                return
            for other_index,other_source in self.extra_sources.items():
                if other_index!=index and not other_source.virtual and (
                    (ip and ip==other_source.ip) or
                    (mxid and mxid==(other_source.info()['mxid'] or other_source.mxid))):
                    messagebox.showwarning('카메라 연결','이미 다른 번호에서 사용 중인 장치입니다.')
                    return
            for other in self.cameras:
                if other is camera or other['kind'].get()=='가상 영상':
                    continue
                if (ip and ip==other['ip'].get().strip()) or (mxid and mxid==other['serial'].get().strip()):
                    messagebox.showwarning('카메라 연결','동일한 IP/MX ID를 두 카메라에 배정할 수 없습니다.')
                    return
        if index in self.extra_sources:
            if not self.extra_sources[index].close(timeout=2):
                messagebox.showwarning('카메라 연결','이전 수신기를 종료 중입니다. 잠시 후 다시 연결해 주세요.')
                return
        source = OakdRgbSource(camera['name'],ip,mxid,
            python=sys.executable if virtual else self.oakd_python.get().strip(),virtual=virtual,
            fps=int(self.fps.get()))
        source.diagnostic_path=(Path(__file__).resolve().parents[1]/'artifacts/connection_logs'/
            f'{camera["name"]}_{datetime.now():%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:6]}.jsonl')
        self.extra_sources[index] = source
        self.save_camera_settings(notify=False)
        source.start()

    @action(lambda self, index: f'CAM{index+1:02d} 연결 해제 요청')
    def disconnect_camera(self, index):
        """녹화 중이 아니면 해당 카메라 수신기를 닫는다."""
        if not self.is_recording and index in self.extra_sources:
            source = self.extra_sources[index]
            if source.close(timeout=2):
                del self.extra_sources[index]

    def connect_cam01(self):
        """CAM01을 RGB 전용으로 연결한다. 메인 앱의 녹화·명령·전환이 끝난 상태여야 하며,
        먼저 인지(AI) 수신을 내려놓은 뒤 RGB 수신기를 시작한다.
        """
        if not self.owner:
            return
        ip = self.cameras[0]['ip'].get().strip()
        if not ip:
            messagebox.showwarning('1번 연결', '카메라 설정에서 1번 IP를 입력해 주세요.')
            return
        if (self.owner.recording or self.owner.source_change_pending
                or self.owner.app_action_busy or self.owner.connection_launch_pending):
            messagebox.showwarning('1번 연결', '인식·검증의 녹화·장치 명령·영상 전환이 끝난 뒤 연결해 주세요.')
            return
        if 0 in self.extra_sources:
            old = self.extra_sources[0]
            if old.ip == ip and (old.preparing or old.snapshot()[0]):
                return
            if not old.close(timeout=2):
                messagebox.showwarning('1번 연결', '이전 RGB 수신기를 종료 중입니다. 잠시 후 연결해 주세요.')
                return
        # Stop PC publishing and AI receivers before transferring camera ownership.
        if not self.owner.release_perception_for_rgb():
            return
        source = self.owner.make_raw_rgb_source(ip, self.cameras[0]['serial'].get().strip(), self.settings_path)
        self.extra_sources[0] = source
        self.save_camera_settings(notify=False)
        source.start()

    def release_cam01_for_perception(self):
        """인지(AI) 연결을 위해 CAM01 RGB 수신기를 내려놓는다. CAM01 녹화 중이거나 준비 중이면 거부한다."""
        source = self.extra_sources.get(0)
        if source is None:
            return True
        if self.is_cam01_recording:
            messagebox.showwarning('인식·검증 연결', 'CAM01 녹화를 정지한 뒤 인식·검증에 연결해 주세요.')
            return False
        if source.preparing:
            messagebox.showwarning('인식·검증 연결', 'CAM01 RGB 연결 준비가 끝난 뒤 인식·검증에 연결해 주세요.')
            return False
        source.close(timeout=0)
        del self.extra_sources[0]
        return True

    @action('6대 카메라 선택 녹화 시작 요청')
    def start_recording(self):
        """선택한 카메라들의 연결 상태와 설정을 검사하고 새 세션 폴더로 그룹 녹화를 시작한다."""
        if not self.owner or self.is_recording:
            return
        selected = [i for i,camera in enumerate(self.cameras) if camera['selected'].get()]
        if not selected:
            messagebox.showwarning('녹화 시작', '녹화할 카메라를 선택해 주세요.')
            return
        if 0 in selected and self.owner.source_change_pending:
            messagebox.showwarning('녹화 시작', '영상 소스 전환이 끝난 뒤 녹화를 시작해 주세요.')
            return
        if (0 in selected and 0 in self.extra_sources
                and self.cameras[0]['ip'].get().strip() != self.extra_sources[0].ip):
            messagebox.showwarning('녹화 시작', '1번 설정 IP와 현재 연결 장치가 다릅니다. 카메라 설정을 확인해 주세요.')
            return
        try:
            specs = []
            for index in selected:
                camera = self.cameras[index]
                if index not in self.extra_sources:
                    raise ValueError(camera['name']+'을 먼저 연결해 주세요.')
                source = self.extra_sources[index]
                guard,kind,ip = source.guard,'virtual' if source.virtual else 'camera',source.ip
                specs.append(dict(camera=camera['name'],model=camera['model'],source=source,
                                  guard=guard,input_kind=kind,ip=ip))
            name = self.session_name.get().strip()
            if not re.fullmatch(r'[\w -]{1,80}', name):
                raise ValueError('세션 이름은 문자·숫자·공백·밑줄·하이픈으로 입력해 주세요.')
            size = None if self.resolution.get() == '원본 스트림' else tuple(map(int, self.resolution.get().split('×')))
            fps = int(self.fps.get())
            folder = self.output_dir.get().strip()
            if not folder:
                raise ValueError('저장 폴더를 입력해 주세요.')
            session = Path(folder).expanduser()/f'{name}_{datetime.now():%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:6]}'
            group = RecordingGroup(specs,session,fps=fps,size=size,verify_output=self.verify_output.get())
            self.save_camera_settings(notify=False)
            self.group = group
            group.start()
            self.recorder = group.recorders.get('CAM01')
            self.update_preview()
        except (ValueError, OSError, RuntimeError) as exc:
            if self.group:self.recorder=self.group.recorders.get('CAM01')
            messagebox.showerror('녹화 시작', str(exc))

    @action('6대 카메라 녹화 정지 요청')
    def stop_recording(self):
        """그룹 녹화 정지를 요청한다."""
        if self.group:
            self.group.stop()

    def update_preview(self, connected=None, frame=None):
        """UI 주기 갱신: 미리보기 이미지, 카메라별 연결/녹화 상태 문구, 버튼 활성 상태를 갱신한다.
        상태가 바뀔 때만 콘솔 로그를 남긴다.
        """
        if not self.owner:
            return
        self._drain_discovery()
        self.assign_button.configure(state=tk.DISABLED if self.is_recording else tk.NORMAL)
        states = self.group.poll() if self.group else {}
        inputs = {}
        for index,source in self.extra_sources.items():
            inputs[index] = source.snapshot()
        count = 0
        for index,camera in enumerate(self.cameras):
            ok,pixels = inputs.get(index,(False,None))
            count += int(ok)
            preview = camera['preview']
            self._render_preview(preview,ok,pixels)
            if index in self.extra_sources:
                info = self.extra_sources[index].info()
                error = info.get('error', '')
                state_event(self, ('camera', index), (id(self.extra_sources[index]), ok, error),
                    f'{camera["name"]} · '+('연결됨' if ok else '연결 대기')+(f' · {error}' if error else ''),
                    'ERROR' if error else 'INFO')
                text = ('[가상] ' if info['virtual'] else '[실제] ')+('연결됨' if ok else '연결 대기')
                if info['error']:
                    text += ' · '+info['error'][:100]
                text += f' · 재접속 {info["reconnects"]}'
                if index == 0:
                    text += (' · '+info.get('prepare_status', 'RGB 준비 중') if info['preparing']
                             else f' · 수신 {info["receive_fps"]:.1f} FPS · AI 없음')
            else:
                address = camera['ip'].get().strip() or camera['serial'].get().strip() or '장치 미설정'
                text = camera['kind'].get()+' · '+address+' · 미연결'
            if camera['name'] in states:
                state=states[camera['name']]
                state_event(self, ('recording', index), (id(self.group), state['state'], state.get('error', '')),
                    f'{camera["name"]} 녹화 · {state["state"]}'+(f' · {state["error"]}' if state.get('error') else ''),
                    'ERROR' if state.get('error') else 'INFO')
                text += f' · {state["state"]} {state["frames"]}F'
                text += (f' · 수신 {state["receive_fps"]:.1f}/저장 {state["saved_fps"]:.1f} FPS'
                         f' · 중복 {state["repeated_frames"]}/건너뜀 {state["skipped_source_frames"]}')
                if not state.get('source_verified',True):text+=' · 소스 확인 재시도'
            camera['status'].set(text)
        active = self.is_recording
        selected = [i for i,c in enumerate(self.cameras) if c['selected'].get()]
        ready = bool(selected) and all(inputs.get(i,(False,None))[0] for i in selected)
        self.start_button.configure(state=tk.NORMAL if ready and not active else tk.DISABLED)
        self.stop_button.configure(state=tk.NORMAL if active else tk.DISABLED)
        self.connect_button.configure(state=tk.DISABLED if active else tk.NORMAL)
        for index,camera in enumerate(self.cameras):
            camera['check'].configure(state=tk.DISABLED if active else tk.NORMAL)
            camera['connect'].configure(state=tk.DISABLED if active else tk.NORMAL)
            camera['disconnect'].configure(state=tk.DISABLED if active else tk.NORMAL)
        self.selection_status.set(f'연결 {count}/6대 · 녹화 선택 {len(selected)}/6대'
                                 + (' · 녹화 중' if active else ''))
        if self.group:
            self.save_status.set(('오류: '+self.group.error+' · ' if self.group.error else '')+str(self.group.session))

    def _render_preview(self, preview, connected, frame):
        """프레임을 미리보기 칸 크기에 맞춰 그린다. 연결이 없으면 대기 문구를 보여 준다."""
        if connected and frame is not None and preview.winfo_ismapped():
            image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            image.thumbnail((max(1,min(480,preview.winfo_width())),
                             max(1,min(270,preview.winfo_height()))))
            photo = ImageTk.PhotoImage(image)
            preview.configure(image=photo, text='')
            preview.image = photo
        elif not connected:
            preview.configure(image='', text='RGB 연결 대기')
            preview.image = None

    def close(self):
        """앱 종료 시 그룹 녹화를 멈추고 모든 수신기를 닫는다. 모두 정리됐는지 반환한다."""
        if self.group and not self.group.stop(timeout=6):
            return False
        return all([source.close(timeout=3) for source in self.extra_sources.values()])

    @action(lambda self, key: f'6대 카메라 설정 열기 · {key}')
    def show_settings(self, key):
        """지정한 설정 창을 메인 창 가운데 위쪽에 띄운다."""
        for other in self.windows.values():
            other.withdraw()
        window = self.windows[key]
        window.update_idletasks()
        x = max(0, self.root.winfo_rootx() + (self.root.winfo_width()-window.winfo_reqwidth())//2)
        y = max(0, self.root.winfo_rooty()+90)
        window.geometry(f'+{x}+{y}')
        window.deiconify()
        window.lift()
        window.focus_set()

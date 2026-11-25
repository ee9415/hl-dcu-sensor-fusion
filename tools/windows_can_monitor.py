"""Windows CAN relay monitor. This program never imports or opens a CAN SDK."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import queue
import secrets
import socket
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/hl_camera_can'))
from hl_camera_can.monitor import CaptureLog
from hl_camera_can.windows_transport import serve_connection


class MonitorServer:
    def __init__(self, capture, key_file):
        self.capture, self.key_file = capture, Path(key_file)
        self.stop_event = threading.Event()
        self.events = queue.SimpleQueue()
        self.listener = self.connection = self.thread = None
        self.connected = self.listening = False
        self.error = ''

    def start(self, host, port):
        if self.thread and self.thread.is_alive():
            return
        self.key_file.parent.mkdir(parents=True, exist_ok=True)
        if not self.key_file.exists():
            with self.key_file.open('x', encoding='ascii') as stream:
                stream.write(secrets.token_hex(32))
        key = self.key_file.read_text(encoding='ascii').strip()
        if len(key) < 64:
            raise ValueError('인증 키 파일이 잘못되었습니다')
        self.stop_event.clear()
        self.thread = threading.Thread(target=self.run, args=(host, port, key), daemon=True)
        self.thread.start()

    def run(self, host, port, key):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                self.listener = server
                if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
                    server.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
                server.bind((host, port)); server.listen(1); server.settimeout(.25)
                self.listening, self.error = True, ''
                self.events.put(f'{host}:{port} 수신 대기')
                while not self.stop_event.is_set():
                    try:connection, peer = server.accept()
                    except socket.timeout:continue
                    self.connection, self.connected = connection, True
                    self.events.put(f'WSL 연결됨 · {peer[0]}')
                    with connection:
                        try:
                            serve_connection(connection, key, None, monitor=self.capture.capture)
                        except (OSError, ValueError, TypeError, KeyError) as exc:
                            if not self.stop_event.is_set():
                                self.events.put('연결 대기 · '+str(exc))
                    self.connection, self.connected = None, False
        except OSError as exc:
            if not self.stop_event.is_set():
                self.error = str(exc)
                self.events.put('수신 오류 · '+str(exc))
        finally:
            self.listener = self.connection = None
            self.listening = self.connected = False

    def stop(self):
        self.stop_event.set()
        for connection in (self.connection, self.listener):
            if connection is not None:
                try:connection.shutdown(socket.SHUT_RDWR)
                except OSError:pass
                try:connection.close()
                except OSError:pass


class MonitorApp:
    def __init__(self, root, args):
        self.root, self.args = root, args
        root.title('CAN 브릿지 모니터 · 물리 송신 없음')
        root.geometry('1240x820'); root.minsize(1000, 700)
        style = ttk.Style(root); style.theme_use('clam')
        style.configure('.', font=('맑은 고딕', 10))
        style.configure('Treeview', rowheight=25, font=('Consolas', 10))
        style.configure('Title.TLabel', font=('맑은 고딕', 19, 'bold'))
        style.configure('Value.TLabel', font=('Consolas', 12))
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.capture = CaptureLog(ROOT/f'artifacts/camera_can/monitor/capture_{stamp}.jsonl')
        self.server = MonitorServer(self.capture, args.key_file)
        self.rows = {}; self.last = None; self.last_state_write = 0
        self.host = tk.StringVar(value=args.host); self.port = tk.StringVar(value=str(args.port))
        self.status = tk.StringVar(value='수신 준비')
        self.stats = tk.StringVar(value='수신 0 · 0.0 Hz · 오류 0 · 물리 송신 0')
        self.follow = tk.BooleanVar(value=True)
        self.fields = {}
        body = ttk.Frame(root, padding=18); body.pack(fill='both', expand=True)
        ttk.Label(body, text='CAN 메시지 모니터', style='Title.TLabel').pack(anchor='w')
        ttk.Label(body, text='WSL CAN 브릿지 → Windows 캡처 전용 · Kvaser 연결 안 함').pack(anchor='w', pady=(3, 12))
        controls = ttk.Frame(body); controls.pack(fill='x')
        ttk.Label(controls, text='수신 주소').pack(side='left')
        ttk.Entry(controls, textvariable=self.host, width=19).pack(side='left', padx=6)
        ttk.Label(controls, text='포트').pack(side='left')
        ttk.Entry(controls, textvariable=self.port, width=7).pack(side='left', padx=6)
        ttk.Button(controls, text='수신 시작', command=self.start).pack(side='left', padx=4)
        ttk.Button(controls, text='수신 중지', command=self.stop).pack(side='left', padx=4)
        ttk.Checkbutton(controls, text='최신 프레임 따라가기', variable=self.follow).pack(side='right')
        ttk.Label(body, textvariable=self.status).pack(anchor='w', pady=(10, 3))
        ttk.Label(body, textvariable=self.stats).pack(anchor='w', pady=(0, 10))
        frame = ttk.LabelFrame(body, text='수신 메시지 해석', padding=10); frame.pack(fill='x')
        definitions = [
            ('mode', '경로 상태'), ('sequence', 'Sequence'), ('crc', 'CRC8'), ('confidence', '신뢰도'),
            ('c0', 'C0 [m]'), ('c1', 'C1'), ('c2', 'C2 [1/m]'), ('c3', 'C3 [1/m²]'),
            ('length', '유효 거리 [m]'), ('min_speed', '최소 속도 [km/h]'),
            ('max_speed', '최대 속도 [km/h]'), ('header', 'Header / Request'),
        ]
        for i, (key, label) in enumerate(definitions):
            cell = ttk.Frame(frame, padding=(8, 4)); cell.grid(row=i//4, column=i%4, sticky='nsew')
            frame.columnconfigure(i%4, weight=1)
            ttk.Label(cell, text=label).pack(anchor='w')
            value = self.fields[key] = tk.StringVar(value='—')
            ttk.Label(cell, textvariable=value, style='Value.TLabel').pack(anchor='w')
        ttk.Label(body, text='0x320 · Standard 11-bit · CAN FD · BRS ON · 32 bytes / DLC 13').pack(anchor='w', pady=(12, 4))
        self.raw = tk.StringVar(value='수신 대기')
        ttk.Label(body, textvariable=self.raw, font=('Consolas', 11), wraplength=1150).pack(fill='x', anchor='w')
        self.detail = tk.StringVar(value='목록에서 프레임을 선택하면 해당 메시지를 고정해서 확인합니다.')
        ttk.Label(body, textvariable=self.detail, wraplength=1150).pack(anchor='w', pady=(4, 10))
        table = ttk.Frame(body); table.pack(fill='both', expand=True)
        columns = ('time', 'sequence', 'mode', 'interval', 'crc', 'continuity', 'raw')
        self.tree = ttk.Treeview(table, columns=columns, show='headings', selectmode='browse')
        headings = ('수신 시각', 'Seq', '상태', '간격 ms', 'CRC', 'Sequence 검사', '원시 데이터')
        widths = (115, 45, 85, 75, 55, 120, 540)
        for column, heading, width in zip(columns, headings, widths):
            self.tree.heading(column, text=heading); self.tree.column(column, width=width, stretch=column == 'raw')
        scroll = ttk.Scrollbar(table, orient='vertical', command=self.tree.yview)
        self.tree.configure(yscrollcommand=scroll.set)
        self.tree.pack(side='left', fill='both', expand=True); scroll.pack(side='right', fill='y')
        self.tree.tag_configure('bad', foreground='#b42318')
        self.tree.tag_configure('active', foreground='#087443')
        self.tree.bind('<<TreeviewSelect>>', self.select)
        footer = ttk.Frame(body); footer.pack(fill='x', pady=(10, 0))
        ttk.Label(footer, text='전체 수신은 JSONL 자동 저장 · 화면은 최근 250개').pack(side='left')
        ttk.Button(footer, text='로그 폴더 열기', command=lambda: os.startfile(self.capture.path.parent)).pack(side='right')
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.after(50, self.poll)
        root.after(100, self.start)

    def start(self):
        try:
            port = int(self.port.get())
            if not 1 <= port <= 65535:raise ValueError('포트 범위는 1~65535입니다')
            self.server.start(self.host.get().strip(), port)
        except (OSError, ValueError) as exc:messagebox.showerror('수신 시작', str(exc))

    def stop(self):
        self.server.stop(); self.status.set('수신 중지 · 물리 CAN 송신 없음')

    def select(self, event=None):
        selected = self.tree.selection()
        if selected and selected[0] in self.rows:
            self.follow.set(False); self.show(self.rows[selected[0]])

    def show(self, event):
        self.raw.set(event['raw'])
        self.fields['mode'].set(event.get('mode', 'INVALID'))
        self.fields['sequence'].set(str(event.get('sequence', '—')))
        self.fields['crc'].set(('OK' if event.get('crc_ok') else 'ERROR')+
            f" ({event.get('crc_received', '—')}/{event.get('crc_expected', '—')})")
        self.fields['confidence'].set(str(event.get('confidence', '—'))+' %')
        for key, value in zip(('c0', 'c1', 'c2', 'c3'), event.get('coefficients', ['—']*4)):
            self.fields[key].set(f'{value:.8g}' if isinstance(value, (float, int)) else value)
        for key, source in (('length', 'valid_length_m'), ('min_speed', 'min_speed_kph'), ('max_speed', 'max_speed_kph')):
            value = event.get(source); self.fields[key].set(f'{value:.2f}' if value is not None else '—')
        self.fields['header'].set(f"{event.get('header', '—')} / {event.get('request_status', '—')}")
        self.detail.set(f"프레임 #{event['index']} · Sequence {event['sequence_note']} · "+
            ('패킷 정상' if event['valid'] else '오류: '+event['error'])+
            f" · Function {event.get('function_id', '—')} · Atomic {int(event.get('atomic', False))}")

    def poll(self):
        while not self.server.events.empty():self.status.set(self.server.events.get())
        for _ in range(100):
            try:event = self.capture.events.get_nowait()
            except queue.Empty:break
            self.last = event
            identifier = str(event['index']); self.rows[identifier] = event
            interval = event['interval_ms']
            self.tree.insert('', 'end', iid=identifier, values=(event['time'][11:],
                event.get('sequence', '—'), event.get('mode', 'INVALID'),
                f'{interval:.1f}' if interval is not None else '—',
                'OK' if event.get('crc_ok') else 'ERROR', event['sequence_note'], event['raw']),
                tags=('bad' if not event['valid'] else 'active' if event.get('mode') == 'ACTIVE' else '',))
            if len(self.rows) > 250:
                first = next(iter(self.rows)); self.tree.delete(first); del self.rows[first]
        if self.follow.get() and self.last:
            self.show(self.last); self.tree.see(str(self.last['index']))
        self.stats.set(f'수신 {self.capture.count:,} · {self.capture.rate():.1f} Hz · '
            f'패킷 오류 {self.capture.invalid} · Sequence 이상 {self.capture.sequence_errors} · 물리 송신 0')
        if not self.server.connected and self.server.listening and self.capture.rate() == 0:
            self.status.set('수신 대기 · WSL 연결 확인')
        state = dict(captured=self.capture.count, hz=self.capture.rate(), invalid=self.capture.invalid,
            sequence_errors=self.capture.sequence_errors, physical_sent=0, monitor_only=True,
            listening=self.server.listening, connected=self.server.connected,
            error=self.server.error, log=str(self.capture.path), last=self.last)
        # A small local state file also makes end-to-end verification possible.
        if time.monotonic()-self.last_state_write > 1:
            self.args.state_file.parent.mkdir(parents=True, exist_ok=True)
            try:
                temporary = self.args.state_file.with_suffix('.tmp')
                temporary.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding='utf-8')
                temporary.replace(self.args.state_file)
            except OSError:
                pass  # A diagnostics-file reader must not stop live display.
            self.last_state_write = time.monotonic()
        self.root.after(50, self.poll)

    def close(self):
        self.server.stop(); self.root.destroy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='auto')
    parser.add_argument('--port', type=int, default=53200)
    parser.add_argument('--key-file', type=Path, default=ROOT/'artifacts/camera_can/windows.key')
    parser.add_argument('--state-file', type=Path, default=ROOT/'artifacts/camera_can/monitor/state.json')
    args = parser.parse_args()
    if args.host == 'auto':
        args.host = '127.0.0.1'
        try:
            result = subprocess.check_output(['wsl.exe', '-d', 'Ubuntu-22.04', '--',
                'ip', '-json', 'route', 'show', 'default'], timeout=5,
                creationflags=subprocess.CREATE_NO_WINDOW)
            candidate = json.loads(result)[0]['gateway']
            with socket.socket() as probe:probe.bind((candidate, 0))
            args.host = candidate
        except (OSError, ValueError, KeyError, IndexError, subprocess.TimeoutExpired):
            pass
    root = tk.Tk(); MonitorApp(root, args); root.mainloop()


if __name__ == '__main__':main()

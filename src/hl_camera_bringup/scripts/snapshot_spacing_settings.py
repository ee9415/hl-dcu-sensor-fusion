import socket,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'apps/yolopv2_standalone'))
from video_transport import send_message,read_message
with socket.create_connection(('192.168.201.2',8558),timeout=5) as s:
 s.settimeout(5);send_message(s,{'op':'status'})
 with s.makefile('rb') as f:r,_=read_message(f)
p=Path(__file__).resolve().parents[3]/'artifacts/opposite_spacing_20261110/settings_before.json';p.write_text(json.dumps(r,indent=2));print(json.dumps(r))

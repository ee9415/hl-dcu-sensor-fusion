import socket,time,json
from pathlib import Path
out=Path('artifacts/lane_597_20261110/live_resume.jsonl')
with out.open('w',encoding='utf-8') as f:
 with socket.create_connection(('192.168.201.2',8557),timeout=5) as s:
  s.settimeout(1);s.sendall(b'HELLO\n');pending=b'';end=time.monotonic()+45
  while time.monotonic()<end:
   try:data=s.recv(1048576)
   except TimeoutError:continue
   if not data:break
   pending+=data
   while b'\n' in pending:
    line,pending=pending.split(b'\n',1)
    r=json.loads(line)
    if 'frame_seq' in r:f.write(json.dumps(r)+'\n');f.flush()
print('done')

"""Measure normal replay, then temporary device callback timing, restoring sources.

Requires Windows oakctl and the existing WSL OpenCV replay verifier. Keeps raw
normal/instrumented reports separate. Does not optimize algorithms or swap models.
"""
import argparse
import base64
import json
from pathlib import Path
import subprocess
import uuid


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device-ip',required=True)
    p.add_argument('--video',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--seconds',type=int,default=20)
    p.add_argument('--skip-normal',action='store_true')
    p.add_argument('--detail',action='store_true',help='Enable intrusive cProfile; FPS is not baseline')
    args=p.parse_args();root=Path(__file__).resolve().parents[3]
    args.output.mkdir(parents=True,exist_ok=True)
    command=['C:/Program Files/oakctl/bin/oakctl.exe','app','exec','-d',args.device_ip,
             '00000000-0000-0000-0000-000000000000','--']
    def run(code):
        encoded=base64.b64encode(code.encode()).decode()
        r=subprocess.run(command+['python3.12','-c',f"import base64;exec(base64.b64decode('{encoded}'))"],
                         capture_output=True,text=True,encoding='utf-8',timeout=60)
        r.check_returncode()
        if 'Traceback (most recent call last)' in r.stdout+r.stderr:raise RuntimeError(r.stdout+r.stderr)
        return r.stdout
    def wslpath(path):
        return subprocess.check_output(['wsl','-d','Ubuntu-22.04','--','wslpath','-a',path.resolve().as_posix()],text=True).strip()
    verifier=wslpath(root/'src/hl_camera_bringup/scripts/verify_pc_video_device.py')
    def replay(name):
        r=subprocess.run(['wsl','-d','Ubuntu-22.04','--','python3',verifier,
            '--device-ip',args.device_ip,'--video',wslpath(args.video),'--output',wslpath(args.output/name),
            '--seconds',str(args.seconds),'--start','2','--end','18'],
            capture_output=True,text=True,encoding='utf-8',timeout=args.seconds+80)
        (args.output/(name+'_console.txt')).write_text(r.stdout+r.stderr)
        r.check_returncode()
        report=json.loads((args.output/name/'report.json').read_text())
        print(json.dumps(dict(run=name,fps=report['output_fps'],frames=report['pc_frames'])),flush=True)
    def health(name):
        r=subprocess.run(['C:/Program Files/oakctl/bin/oakctl.exe','app','logs',
            '00000000-0000-0000-0000-000000000000','-d',args.device_ip,'--no-follow','--tail','15'],
            capture_output=True,text=True,encoding='utf-8',timeout=20)
        (args.output/(name+'_health.txt')).write_text(r.stdout+r.stderr)
    if not args.skip_normal:
        replay('normal');health('normal')
    backup='/app/fps_profile_backup_'+uuid.uuid4().hex[:10]
    files=['yolopv2_node.py','video_input_node.py','depth_preview.py','stream_output.py']
    payload=base64.b64encode((root/'apps/yolopv2_standalone/pipeline_timing.py').read_bytes()).decode()
    setup=f"""
from pathlib import Path
import base64
backup=Path({backup!r});backup.mkdir()
module=base64.b64decode({payload!r});compile(module,'pipeline_timing.py','exec')
Path('/app/pipeline_timing.py').write_bytes(module)
detail=Path('/app/pipeline_timing_detail')
detail.write_text('1') if {args.detail!r} else detail.unlink(missing_ok=True)
def edit(s,old,new):
    old='\\n'+old;new='\\n'+new
    assert s.count(old)==1,old
    return s.replace(old,new,1)
changes={{}}
for name in {files!r}:
    path=Path('/app')/name;s=path.read_text()
    assert 'import pipeline_timing' not in s
    (backup/name).write_bytes(path.read_bytes())
    s='import pipeline_timing\\n'+s
    if name=='yolopv2_node.py':
        s=edit(s,'        bgr = frame.getCvFrame()',"        _fps_pre = pipeline_timing.begin('preprocess', frame.getSequenceNum())\\n        bgr = frame.getCvFrame()")
        s=edit(s,'        self.tensor_out.send(nn_data)','        self.tensor_out.send(nn_data)\\n        pipeline_timing.finish(_fps_pre)')
        s=edit(s,'        depth_frame = aligned_depth.getFrame()',"        _fps_post = pipeline_timing.begin('postprocess', nn_data.getSequenceNum(), source.get('input_source')=='pc')\\n        depth_frame = aligned_depth.getFrame()")
        phases=[
            ('detection_decode','        heads = [self._tensor','        decoded = non_max_suppression('),
            ('nms','        decoded = non_max_suppression(','        detection_messages = []'),
            ('object_depth','        for x1, y1, x2, y2, confidence, label in decoded:','        detections = dai.ImgDetections()'),
            ('segmentation','        drivable, lane = segmentation_masks(','        boundary_started = time.monotonic()'),
            ('boundaries','        boundaries = self._boundary_tracker.update(','        boundary_finished = time.monotonic()'),
            ('stop_line','        stop_line=self._stop_line_tracker.update(','        stop_finished=time.monotonic()'),
            ('bev_render','        bev = render_boundary_bev(boundaries)','        bev_finished = time.monotonic()'),
            ('result_render_send','        self.result.send(','        self._frames += 1\\n        now = time.monotonic()\\n        if now - self._last_health_at >= 5.0:'),
        ]
        for label,begin,end in phases:
            s=edit(s,begin,"        _fps_"+label+" = pipeline_timing.begin('"+label+"', nn_data.getSequenceNum(), source.get('input_source')=='pc')\\n"+begin)
            s=edit(s,end,'        pipeline_timing.finish(_fps_'+label+')\\n'+end)
        s+='\\n        pipeline_timing.finish(_fps_post)\\n'
    elif name=='video_input_node.py':
        s=edit(s,'        frame, depth = camera, aligned_depth',"        _fps_selector = pipeline_timing.begin('selector', camera.getSequenceNum(), metadata['input_source']=='pc')\\n        frame, depth = camera, aligned_depth")
        s=edit(s,'        self.depth_out.send(depth)','        self.depth_out.send(depth)\\n        pipeline_timing.finish(_fps_selector)')
    elif name=='depth_preview.py':
        s=edit(s,'        raw = depth.getFrame()',"        _fps_depth = pipeline_timing.begin('depth_preview', depth.getSequenceNum())\\n        raw = depth.getFrame()")
        s=edit(s,'        self.preview_out.send(frame)','        self.preview_out.send(frame)\\n        pipeline_timing.finish(_fps_depth)')
    else:
        s=edit(s,'        self._server.send_data(bytes(stream.getData()))',"        _fps_stream = pipeline_timing.begin('rtsp_'+self._server.get_service(), stream.getSequenceNum())\\n        self._server.send_data(bytes(stream.getData()))\\n        pipeline_timing.finish(_fps_stream)")
    compile(s,name,'exec');changes[path]=s
for path,s in changes.items():path.write_text(s)
for path in Path('/app').glob('pipeline_timing_*.json'):path.unlink()
"""
    manifest=dict(backup=backup,files=files,seconds=args.seconds,detail=args.detail,restored=False)
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2))
    try:
        run(setup)
        subprocess.run(command+['sv','restart','/etc/service/backend'],check=True,capture_output=True)
        # The replay verifier waits for the telemetry port; wait for model startup here.
        import time
        time.sleep(18)
        replay('instrumented');health('instrumented')
        raw=run("from pathlib import Path;import json;"
                "print('PROFILE='+json.dumps({v.stem:json.loads(v.read_text()) for v in Path('/app').glob('pipeline_timing_*.json')}))")
        profile=json.loads(raw.split('PROFILE=',1)[1].splitlines()[0])
        (args.output/'profile.json').write_text(json.dumps(profile,indent=2))
    finally:
        run(f"from pathlib import Path;import hashlib;p=Path('{backup}');"
            f"names=[name for name in {files!r} if (p/name).exists()];"
            "[(Path('/app',name).write_bytes((p/name).read_bytes())) for name in names];"
            "assert all(Path('/app',name).read_bytes()==(p/name).read_bytes() for name in names);print('RESTORED')")
        subprocess.run(command+['sv','restart','/etc/service/backend'],check=True,capture_output=True)
        manifest['restored']=True
        (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2))


if __name__=='__main__':main()

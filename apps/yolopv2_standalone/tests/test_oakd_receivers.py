"""Real child-process framing/reconnection and decodable virtual-camera recordings."""
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'))
from oakd_rgb_source import OakdRgbSource
from shared_rgb_recorder import RecordingGroup


class OakdReceiverTests(unittest.TestCase):
    def test_connection_history_retains_worker_failure_details(self):
        with tempfile.TemporaryDirectory() as folder:
            source=OakdRgbSource('CAM02',virtual=True,width=160,height=96,fail_after=2,retry_s=.1)
            source.diagnostic_path=Path(folder)/'connection.jsonl'
            source.start()
            try:
                self.wait(lambda:source.info()['last_failure'].get('exit_code')==1)
            finally:self.assertTrue(source.close())
            events=[json.loads(line) for line in source.diagnostic_path.read_text().splitlines()]
            failure=next(event for event in events if event['event']=='worker_failed')
            self.assertEqual(failure['failure']['exit_code'],1)
            self.assertTrue(any('Injected virtual disconnect' in line for line in failure['stderr']))
            self.assertTrue(any(event['event']=='rgb_connected' for event in events))

    def source(self,index,**options):
        source=OakdRgbSource(f'CAM{index:02d}',virtual=True,width=160,height=96,
                             retry_s=.15,**options)
        self.addCleanup(source.close)
        source.start()
        return source

    def wait(self,predicate,timeout=10):
        end=time.monotonic()+timeout
        while not predicate() and time.monotonic()<end:
            time.sleep(.02)
        self.assertTrue(predicate())

    def spec(self,index,source):
        return dict(camera=f'CAM{index:02d}',source=source,ip='',guard=source.guard,
                    model='OAK-D Pro PoE',input_kind='virtual')

    def test_virtual_worker_frame_and_clean_close(self):
        source=self.source(2)
        self.wait(lambda:source.snapshot()[0])
        connected,frame,sequence,stamp=source.record_snapshot()
        self.assertTrue(connected);self.assertEqual(frame.shape,(96,160,3))
        self.assertGreater(sequence,0);self.assertLess(time.monotonic()-stamp,1)
        self.assertEqual(source.guard('')['source'],'virtual')
        self.assertTrue(source.close())
        self.assertFalse(source.snapshot()[0])

    def test_injected_disconnect_reconnects_without_changing_input_kind(self):
        source=self.source(3,fail_after=3)
        self.wait(lambda:source.info()['reconnects']>=1 and source.record_snapshot()[2]>=5)
        self.assertTrue(source.virtual)
        self.assertTrue(source.close())

    def test_missing_real_runtime_does_not_fall_back_to_virtual(self):
        source=OakdRgbSource('CAM02',ip='192.0.2.99',python='/nonexistent/oakd-python',retry_s=.1)
        self.addCleanup(source.close);source.start()
        self.wait(lambda:source.info()['reconnects']>=1)
        self.assertFalse(source.virtual);self.assertFalse(source.snapshot()[0])
        self.assertTrue(source.info()['error'])

    def test_five_virtual_cameras_record_distinct_files_and_metadata(self):
        sources={i:self.source(i) for i in range(2,7)}
        self.wait(lambda:all(s.snapshot()[0] for s in sources.values()))
        with tempfile.TemporaryDirectory() as folder:
            group=RecordingGroup([self.spec(i,s) for i,s in sources.items()],Path(folder)/'group')
            group.start()
            self.addCleanup(group.stop,3)
            self.wait(lambda:all(r.snapshot()['frames']>=5 for r in group.recorders.values()))
            self.assertTrue(group.stop(timeout=3));self.assertFalse(group.error)
            self.assertEqual(len(list(group.session.glob('*/*.mp4'))),5)
            index=json.loads((group.session/'recording_group.json').read_text())
            self.assertTrue(all(kind=='virtual' for kind in index['inputs'].values()))
            for i in range(2,7):
                path=group.session/f'cam{i:02d}'/f'cam_{i:02d}_rgb.mp4'
                cap=cv2.VideoCapture(str(path));ok,frame=cap.read();cap.release()
                self.assertTrue(ok)
                self.assertTrue(np.allclose(frame[-5,-5],(i*31%160,40,80),atol=20))
                manifest=json.loads((path.parent/'session.json').read_text())
                self.assertEqual(manifest['camera'],f'CAM{i:02d}')
                self.assertEqual(manifest['input_kind'],'virtual')

    def test_one_disconnect_stops_group_and_keeps_decodable_partial_files(self):
        sources={i:self.source(i) for i in (2,3)}
        self.wait(lambda:all(s.snapshot()[0] for s in sources.values()))
        with tempfile.TemporaryDirectory() as folder:
            group=RecordingGroup([self.spec(i,s) for i,s in sources.items()],Path(folder)/'group')
            group.start();self.addCleanup(group.stop,3)
            self.wait(lambda:all(r.snapshot()['frames']>=3 for r in group.recorders.values()))
            sources[2].close()
            def stopped():
                group.poll()
                return not group.active
            self.wait(stopped)
            group.poll();self.assertTrue(group.error)
            for path in group.session.glob('*/*.mp4'):
                cap=cv2.VideoCapture(str(path));ok,_=cap.read();cap.release();self.assertTrue(ok)
            self.assertTrue(sources[3].snapshot()[0])  # Preview receiver remains usable.

    def test_unavailable_selection_prevents_partial_group_start(self):
        source=OakdRgbSource('CAM02',virtual=True)
        with tempfile.TemporaryDirectory() as folder:
            group=RecordingGroup([self.spec(2,source)],Path(folder)/'group')
            with self.assertRaises(RuntimeError):group.start()
            self.assertFalse(group.session.exists())
            self.assertFalse(group.recorders)


if __name__=='__main__':unittest.main()

"""Restore the operator's pre-deployment rendering preferences, without saving new UI defaults."""
from pathlib import Path
import sys,json,time
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT/'tools'))
from video_file_sender import set_skeleton_response_speed,set_contour_display,set_lane_overlay,set_boundary_display,set_processing_mode
s=json.loads((ROOT/'artifacts/opposite_spacing_20261110/settings_before.json').read_text())
for attempt in range(30):
 try:
  set_skeleton_response_speed('192.168.201.2',s['skeleton_response_speed'])
  set_contour_display('192.168.201.2',s['contour_visible'])
  set_boundary_display('192.168.201.2',s['boundary_visible'])
  set_processing_mode('192.168.201.2',s['processing_mode'])
  r=set_lane_overlay('192.168.201.2',s['lane_overlay_mode'],s['lane_overlay_threshold'])
  (ROOT/'artifacts/opposite_spacing_20261110/settings_restored.json').write_text(json.dumps(r,indent=2));print(json.dumps(r));break
 except OSError:
  if attempt==29:raise
  time.sleep(1)

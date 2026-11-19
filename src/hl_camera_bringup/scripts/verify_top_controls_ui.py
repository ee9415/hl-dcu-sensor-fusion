"""Launch isolated TOP settings for offline visual QA; never contact a camera."""
from pathlib import Path
import sys,tempfile,json,argparse
import tkinter as tk
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT/'tools'))
from oak_recorder import RecorderApp
import settings_store
p=argparse.ArgumentParser();p.add_argument('--seconds',type=int,default=30);a=p.parse_args()
out=ROOT/'artifacts/top_controls_20261110';out.mkdir(parents=True,exist_ok=True)
with tempfile.TemporaryDirectory() as folder:
    root=tk.Tk()
    with patch.object(RecorderApp,'_poll_camera_app'),patch('oak_recorder.set_top_view') as remote:
        app=RecorderApp(root,'','',None,None,None,Path(folder)/'recordings',Path(folder)/'settings.json')
        root.title('TOP 조절바 검증 (오프라인)')
        root.update()
        app.top_view_spread.set(35);app.top_view_ahead.set(50);app._apply_top_view()
        assert remote.call_count==0
        saved=settings_store.read(app.six_camera_tab.settings_path)['preferences']['display']
        assert saved['top_view_spread']==35 and saved['top_view_ahead']==50
        app.top_view_spread.set(0);app.top_view_ahead.set(0);app._restore_preferences()
        assert app.top_view_spread.get()==35 and app.top_view_ahead.get()==50
        app._show_settings('display')
        app.settings_windows['display'].transient('')
        app.settings_windows['display'].title('TOP 조절바 검증 (오프라인)')
        app.settings_windows['display'].geometry('+100+100')
        root.withdraw()
        root.update()
        (out/'ui_result.json').write_text(json.dumps(dict(offline_save=True,reload=True,
            camera_calls=remote.call_count,settings_width=app.settings_windows['display'].winfo_width(),
            settings_height=app.settings_windows['display'].winfo_height())))
        print('Offline TOP save/reload passed; isolated settings window open',flush=True)
        root.after(a.seconds*1000,root.destroy)
        root.mainloop()

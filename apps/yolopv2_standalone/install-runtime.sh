#!/bin/sh
# Install and remove build-only dependencies in ONE image layer.
set -eu
export DEBIAN_FRONTEND=noninteractive
runtime="runit libusb-1.0-0 libgl1 libxrender1 libgomp1 gir1.2-gst-rtsp-server-1.0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad libgirepository-1.0-1 libcairo2"
build="gcc g++ pkg-config libgirepository1.0-dev libcairo2-dev"
apt-get update -qq
apt-get install -y --no-install-recommends $runtime $build
python3.12 -m pip install --no-cache-dir -r /app/requirements.txt --break-system-packages
apt-mark manual $runtime
apt-get purge -y --auto-remove $build
apt-get clean
rm -rf /var/lib/apt/lists/* /root/.cache/pip
python3.12 - <<'PY'
from pathlib import Path
import shutil
import sysconfig
stdlib = Path(sysconfig.get_path('stdlib'))
# Static linking archives and headers are not needed by the installed runtime.
for path in stdlib.glob('config-*/libpython*.a'):
    path.unlink()
for path in Path(sysconfig.get_config_var('LIBDIR')).glob('libpython*.a'):
    path.unlink()
for path in (stdlib/'test', Path(sysconfig.get_path('include'))):
    if path.is_dir():
        shutil.rmtree(path)
numpy = stdlib/'site-packages/numpy'
for path in list(numpy.rglob('tests')) if numpy.exists() else []:
    if path.is_dir():
        shutil.rmtree(path)
PY
# Fail the build if removal broke the runtime bindings or GStreamer imports.
python3.12 -c 'import depthai, gi; gi.require_version("Gst", "1.0"); gi.require_version("GstRtspServer", "1.0"); from gi.repository import Gst, GstRtspServer; Gst.init(None); assert Gst.ElementFactory.find("h264parse"); assert Gst.ElementFactory.find("rtph264pay")'

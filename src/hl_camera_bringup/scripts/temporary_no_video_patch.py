"""Build a temporary diagnostics-only variant without debug video streams.

Tracking, masks, stop-line, detections, telemetry and stereo pipeline remain.
This does not provide a production output API and must not be deployed alone.
"""


def replace_span(source, start, end, replacement):
    if source.count(start) != 1 or source.count(end) != 1:
        raise ValueError(f'Expected one occurrence of both markers: {start!r}, {end!r}')
    a = source.index(start)
    b = source.index(end, a)
    return source[:a] + replacement + source[b:]


def patch_main(source):
    source = replace_span(source,
        '    encoder = pipeline.create(dai.node.VideoEncoder)\n',
        '    if PIPELINE_STAGE not in {"camera", "depth"}:\n',
        '    # Temporary diagnostic: suppress RGB and Depth debug streams.\n\n')
    source = replace_span(source,
        '                result_encoder = pipeline.create(dai.node.VideoEncoder)\n',
        '    print(f"YOLOPv2 {PIPELINE_STAGE} pipeline created", flush=True)\n',
        '                # Temporary diagnostic: suppress result and BEV streams.\n\n')
    compile(source, 'main.py', 'exec')
    return source


def patch_node(source):
    source = replace_span(source,
        '            bev = render_boundary_bev(boundaries)\n',
        '        else:\n            empty=np.empty((0,2),float)\n',
        '            bev_finished = time.monotonic()\n')
    source = replace_span(source,
        '        self.bev.send(self._bev_frame(bev, nn_data))\n',
        '        self._frames += 1\n        now = time.monotonic()\n        if now - self._last_health_at >= 5.0:\n',
        '        # Temporary diagnostic: do not render result or BEV video.\n\n')
    compile(source, 'yolopv2_node.py', 'exec')
    return source

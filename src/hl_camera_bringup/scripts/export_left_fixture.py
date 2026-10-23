import numpy as np,json
from pathlib import Path
z=np.load('artifacts/left_debug_0322/boundary_debug.npz');r=json.loads(str(z['records_json']))[0];u=next(c for c in r['calls'] if c['name']=='update')
np.savez_compressed('apps/yolopv2_standalone/tests/fixtures/left_short_duplicate_20261023.npz',paint=z[u['paint']['array']],left=z[r['history_before'][0]['array']],right=z[r['history_before'][1]['array']])

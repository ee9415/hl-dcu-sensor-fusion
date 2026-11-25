"""Measure ROS Image byte setter versus the typed-array fast path."""
from array import array
import json
import time

import numpy as np
from sensor_msgs.msg import Image


results = {}
for name, shape, dtype in (('rgb',(540,960,3),'u1'), ('depth',(1080,1920),'<u2')):
    data = np.zeros(shape,dtype=dtype)
    timings = {}
    for method in ('bytes_setter','typed_array'):
        values = []
        for _ in range(5):
            image = Image()
            started = time.perf_counter()
            if method == 'bytes_setter':
                image.data = data.tobytes()
            else:
                image.data.frombytes(memoryview(data).cast('B'))
            values.append((time.perf_counter()-started)*1000)
            assert bytes(image.data)==data.tobytes()
        timings[method+'_mean_ms'] = sum(values)/len(values)
    results[name] = timings
print(json.dumps(results,indent=2))

#python tests/test_depth.py

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from sensors.astra import AstraCamera
import time

astra = AstraCamera()

while True:
    depth = astra.get_depth_frame()

    if depth is None:
        print("depth none")
        continue

    cy, cx = depth.shape[0] // 2, depth.shape[1] // 2

    val = depth[cy, cx]

    print("center depth:", val)

    time.sleep(0.5)
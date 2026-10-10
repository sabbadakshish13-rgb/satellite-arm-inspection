#!/usr/bin/env python3

import time
from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean

node = Node()

service = "/world/inspection_world/set_pose"

print("Moving inspection target simulation initialized")

for step in range(11):
    y = 0.05 + step * 0.005

    request = Pose()
    request.name = "inspection_target"
    request.position.x = 0.65
    request.position.y = y
    request.position.z = 0.75
    request.orientation.w = 1.0

    success, response = node.request(
        service, request, Pose, Boolean, 5000
    )

    if not success or not response.data:
        print(f"ERROR: Target movement failed at Y = {y:.3f} m")
        break

    print(f"Target moved successfully: Y = {y:.3f} m")

    if step < 10:
        time.sleep(3)

print("Target movement sequence completed")

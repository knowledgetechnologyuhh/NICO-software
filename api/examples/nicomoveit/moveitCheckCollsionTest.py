#!/usr/bin/env python

# This is a small script to test a joint configuration for collision.

import time
import math
from nicomoveit import moveitWrapper
from nicomotion import Motion

groupName = "leftArm"
leftArm = moveitWrapper.groupHandle(groupName)

target = [0.0, 0.0, -0.5, 0.0, 0.0, 0.0] # position with collision
for idx in range(0, len(target)):
  target[idx] = math.degrees(target[idx])

leftArm.isColliding(target)

target = [0.0, 0.0, 0.5, 0.0, 0.0, 0.0]  # position without collision
for idx in range(0, len(target)):
  target[idx] = math.degrees(target[idx])

leftArm.isColliding(target)

target = leftArm.group.get_random_joint_values()
for idx in range(0, len(target)):
  target[idx] = math.degrees(target[idx])

leftArm.isColliding(target)





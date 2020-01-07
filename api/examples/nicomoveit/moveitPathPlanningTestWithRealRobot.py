#!/usr/bin/env python

import math
import time

from nicomoveit import moveitWrapper

groupName = "leftArm"
leftArm = moveitWrapper.groupHandle(
    groupName,
    vrep=False,
    robotMotorFile="nico_humanoid_upper.json",
    vrepScene="NICO-seated-with-table.ttt",
    visualize=True,
)

time.sleep(3)
# leftArm.addMesh("table", "table.stl", position=[0, 0.0, 0.175])
# time.sleep(3)
# leftArm.addMesh("chair", "chair.stl", position=[0.015, 0.0, 0.175])
# time.sleep(3)

p_x = 0.43085057313
p_y = 0.134845567708
p_z = 0.802798506593
leftArm.moveToPose([p_x, p_y, p_z], "sideGrasp")

# leftArm.group.set_max_velocity_scaling_factor(0.001)

# target = [-0.1969, -1.4026, -0.1552, 0.3835, 0.0, 0.0]
# target = [0.0, 0.0, 0.7853981633974483, 1.3962634015954636, 0.0, 0.0]
# for idx in range(0, len(target)):
#     target[idx] = math.degrees(target[idx])

target = leftArm.computeFK([0.0, 0.0, 45.0, 0.0, 0.0, 0.0])

# leftArm.moveToJointCoordinates(target)
leftArm.moveToPose(*target)
time.sleep(3)

# leftArm.shiftPose(1, 0.1)

leftArm.toSafePosition()

#!/usr/bin/env python

"""
Make sure that a ROS instance is running before this script is executed!

Examples and test for the collision check function.
"""

import math
from nicomoveit import moveitWrapper

groupName = "leftArm"
leftArm = moveitWrapper.groupHandle(
    groupName,
    kinematicsOnly=True,
    robotMotorFile="nico_humanoid_legged_with_hands_mod-vrep.json",
)

target = [0.0, 0.0, -0.5, 0.0, 0.0, 0.0]  # position with collision
for idx in range(0, len(target)):
    target[idx] = math.degrees(target[idx])
# check if this target is correctly identified as a pose with collision
assert leftArm.isColliding(target), "Arm pose should not be in a collision state"

target = [0.0, 0.0, 0.5, 0.0, 0.0, 0.0]  # position without collision
for idx in range(0, len(target)):
    target[idx] = math.degrees(target[idx])
# check if this target is correctly identified as a pose without collision
assert leftArm.isColliding(target) is False, "Arm pose should be in a collision state"

target = leftArm.group.get_random_joint_values()
for idx in range(0, len(target)):
    target[idx] = math.degrees(target[idx])
# check collision for random joint values
leftArm.isColliding(target)

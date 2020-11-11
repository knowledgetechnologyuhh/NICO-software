#!/usr/bin/env python

import logging
import time

from nicomotion import Motion

logging.basicConfig(level=logging.WARNING)

vrep = True

if vrep:
    vrepConfig = Motion.Motion.vrepRemoteConfig()
    # vrepConfig = Motion.Motion.pyrepConfig() # requires python 3
    vrepConfig["vrep_scene"] = "../../../v-rep/NICO-seated.ttt"
    robot = Motion.Motion("../../../json/nico_humanoid_legged_with_hands_mod-vrep.json",
                          vrep=True, vrepConfig=vrepConfig)
else:
    robot = Motion.Motion(
        "../../../json/nico_humanoid_upper_with_hands.json", vrep=False)

position = 20

for i in range(10):
    robot.setAngle("r_arm_x", -80 + position, 0.05)
    robot.setAngle("r_elbow_y", -40 + position, 0.05)

    robot.setAngle("l_arm_x", 80 + position, 0.05)
    robot.setAngle("l_elbow_y", 40 + position, 0.05)

    if i % 2 == 0:
        if i % 4 == 0:
            robot.setAngle("head_z", -position, 0.05)
        else:
            robot.setAngle("head_z", position, 0.05)
    if i % 2 == 1:
        if i % 4 == 1:
            robot.setAngle("head_y", position, 0.05)
        else:
            robot.setAngle("head_y", -position, 0.05)

    if position > 0:
        print(1)
        robot.closeHand("LHand")
    else:
        print(2)
        robot.openHand("LHand")

    position = position * -1
    time.sleep(2)

print("Moving to safe position")
robot.toSafePosition()
time.sleep(7)
robot.disableTorqueAll()

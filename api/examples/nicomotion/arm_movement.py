#!/usr/bin/env python

import logging
import time

from nicomotion import Motion
from os.path import dirname, abspath

logging.basicConfig(level=logging.WARNING)

vrep = True
nico_root = dirname(abspath(__file__)) + "/../../.."

if vrep:
    # get default config for remote api
    vrepConfig = Motion.Motion.vrepRemoteConfig()
    # set scene (simulation will start automatically if this is set)
    vrepConfig["vrep_scene"] = nico_root + "/v-rep/NICO-seated.ttt"
    # init simulated robot
    robot = Motion.Motion(
        nico_root + "/json/nico_humanoid_vrep.json", vrep=True, vrepConfig=vrepConfig
    )
else:
    # init real robot
    robot = Motion.Motion(nico_root + "/json/nico_humanoid_upper.json", vrep=False)

# perform movement
position = 20
for i in range(10):
    # arm movement
    robot.setAngle("r_arm_x", -80 + position, 0.05)
    robot.setAngle("r_elbow_y", -40 + position, 0.05)

    robot.setAngle("l_arm_x", 80 + position, 0.05)
    robot.setAngle("l_elbow_y", 40 + position, 0.05)

    # head movement
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

    # open/close hands
    if position > 0:
        print(1)
        robot.closeHand("LHand")
    else:
        print(2)
        robot.openHand("LHand")

    position = position * -1
    time.sleep(2)

# reset position
print("Moving to safe position")
robot.toSafePosition()
time.sleep(7)
# cleanup
if vrep:
    robot.stopSimulation()
else:
    robot.disableTorqueAll()
del robot

#!/usr/bin/env python

# USAGE: Use script with yes or no as parameter
# python yesno.py yes
# python yesno.py no

import logging
import sys
import time

from nicomotion import Motion
from os.path import dirname, abspath

logging.basicConfig(level=logging.WARNING)

vrep = True
nico_root = dirname(abspath(__file__)) + "/../../.."

# check arguments
if len(sys.argv) != 2:
    print("Usage: {} arg ('yes' or 'no')".format(sys.argv[0]))
    exit(1)

if vrep:
    # get default config for remote api
    # vrepConfig = Motion.Motion.vrepRemoteConfig()
    # set scene (simulation will start automatically if this is set)
    # vrepConfig["vrep_scene"] = nico_root + "/v-rep/NICO-seated.ttt"
    # init simulated robot
    robot = Motion.Motion(
        nico_root + "/json/nico_humanoid_vrep.json", vrep=True,  # vrepConfig=vrepConfig
    )
else:
    # init real robot
    robot = Motion.Motion(nico_root + "/json/nico_humanoid_upper.json", vrep=False)

# perform motion based on arg
position = -20
for i in range(6):
    position = position * -1
    if sys.argv[1] == "yes":
        robot.setAngle("head_y", position, 0.05)
    if sys.argv[1] == "no":
        robot.setAngle("head_z", position, 0.05)
    time.sleep(1.5)

# return to default position
robot.setAngle("head_z", 0, 0.05)
robot.setAngle("head_y", 0, 0.05)
time.sleep(3)

# cleanup
if vrep:
    # robot.stopSimulation()
    pass
else:
    robot.disableTorqueAll()
del robot

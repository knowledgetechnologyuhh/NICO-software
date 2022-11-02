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

robot.setAngle("r_shoulder_y", 90, 0.1)

time.sleep(5)

# cleanup
if vrep:
    robot.stopSimulation()
else:
    robot.disableTorqueAll()
del robot

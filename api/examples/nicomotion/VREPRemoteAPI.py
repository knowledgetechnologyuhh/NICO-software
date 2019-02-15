#!/usr/bin/env python

import logging
import time
from os.path import abspath, dirname

import numpy as np
from nicomotion.Motion import Motion
from pypot.vrep.remoteApiBindings import vrep as remote_api

logging.basicConfig(level=logging.WARNING)

robot = Motion(dirname(abspath(__file__)) +
               "/../../../json/nico_humanoid_upper.json", vrep=True)

# start simulation
robot.startSimulation()
# get all joint names via remote api call
print(robot.callVREPRemoteApi('simxGetObjectGroupData',
                              remote_api.sim_object_joint_type, 0,
                              streaming=True))
# add get object position via pypot's vrep io
print(robot.getVrepIO().get_object_position("Cuboid"))

time.sleep(5)
robot.stopSimulation()

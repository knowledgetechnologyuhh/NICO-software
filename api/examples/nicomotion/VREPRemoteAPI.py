#!/usr/bin/env python

from nicomotion.Motion import Motion
import time
from os.path import dirname, abspath
import numpy as np
from pypot.vrep.remoteApiBindings import vrep as remote_api

robot = Motion(dirname(abspath(__file__))+"/../../../json/nico_humanoid_upper.json",vrep=True)

# start simulation
robot.startSimulation()
# get all joint names via remote api call
print(robot.callVREPRemoteApi('simxGetObjectGroupData', remote_api.sim_object_joint_type, 0, streaming=True))
# add get object position via pypot's vrep io
print(robot.getVrepIO().get_object_position("Cuboid"))

time.sleep(5)
robot.stopSimulation()

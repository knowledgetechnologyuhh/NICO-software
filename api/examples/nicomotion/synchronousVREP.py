#!/usr/bin/env python

import logging
import time
from os.path import abspath, dirname

from nicomotion.Motion import Motion

logging.basicConfig(level=logging.WARNING)

robot = Motion(dirname(abspath(__file__)) +
               "/../../../json/nico_humanoid_upper.json", vrep=True)

# set simulation timestep (make sure dt is set to custom and simulation is
# stopped!)
dt = 0.01
robot.setSimulationDeltatime(dt)
# start simulation with synchronize enabled
robot.startSimulation(synchronize=True)
# set motor goal position
robot.setAngle("r_arm_x", 90, 0.05)
# simulate for 2 seconds
for i in range(200):
    time.sleep(dt)
    # advance simulation by one step
    robot.nextSimulationStep()

robot.stopSimulation()

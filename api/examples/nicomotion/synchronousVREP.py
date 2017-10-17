#!/usr/bin/env python

from nicomotion.Motion import Motion
import time

robot = Motion("../NICO-software/json/nico_humanoid_upper_with_hands_vrep.json",vrep=True)
# set simulation timestep
dt = 0.01
robot.setSimulationDeltatime(dt)
# start simulation with synchronize enabled
robot.startSimulation(synchronize=True)
# set motor goal position
robot.setAngle("r_arm_x", -90, 0.05)
# simulate for 3 seconds
for i in range(300):
    time.sleep(dt)
    # advance simulation by one step
    robot.nextSimulationStep()

robot.stopSimulation()

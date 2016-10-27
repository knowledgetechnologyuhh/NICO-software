#!/usr/bin/env python

import time
from nicomotion import Motion

time.sleep(5)
#robot = Motion.Motion("../../../json/nico_humanoid_upper_with_hands.json",vrep=False)
robot = Motion.Motion("../../../json/nico_humanoid_upper_with_hands_vrep.json",vrep=True)
position = 20
for i in xrange(5):
	robot.setAngle("r_arm_x", -40 + position, 0.05)
	robot.setAngle("r_elbow_y", -40 + position, 0.05)
	if position > 0:
		print 1
		robot.closeHand("LHand")
	else:
		print 2
		robot.openHand("LHand")
	position = position * -1
        time.sleep(2)

robot.setAngle("r_arm_x", 20, 0.05)
robot.setAngle("r_elbow_y", 0, 0.05)
time.sleep(5)
robot.disableTorqueAll()

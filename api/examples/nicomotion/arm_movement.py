#!/usr/bin/env python

import time
from nicomotion import Motion

time.sleep(5)
#robot = Motion.Motion("../../../json/nico_humanoid_upper_with_hands.json",vrep=False)
robot = Motion.Motion("../../../json/nico_humanoid_upper_with_hands_vrep.json",vrep=True,vrepScene="../../../v-rep/NICO-seated-with-table.ttt")
position = 20

for i in xrange(10):
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
		print 1
		robot.closeHand("LHand")
	else:
		print 2
		robot.openHand("LHand")

	position = position * -1
        time.sleep(2)

print "Moving to save position"
robot.toSavePosition()
time.sleep(7)
robot.disableTorqueAll()

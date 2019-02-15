#!/usr/bin/env python

import time
from nicomotion import Motion

print "Waiting for 2 seconds - Do not know why"
time.sleep(2)
virtualRobot = Motion.Motion("../../../json/nico_humanoid_upper_with_hands_vrep_mod.json",vrep=True)
realRobot = Motion.Motion("../../../json/nico_humanoid_legged_with_hands_mod.json",vrep=False)


	
while (True):	

	for jName in (realRobot.getJointNames()):

		cont=True
		targetPosition = realRobot.getAngle(jName)
	
		print "Setting Virtual: Joint " + str(jName) + " to position " + str(targetPosition)

		virtualRobot.setAngle(jName, targetPosition, 0.05)

		time.sleep(0.1)
		

		    
print "And the stiffness off"
realRobot.disableTorqueAll()
virtualRobot.disableTorqueAll()

#!/usr/bin/env python

import time
from nicomotion import Motion

print "Waiting for 2 seconds - Do not know why"
time.sleep(2)
#virtualRobot = Motion.Motion("../../../json/nico_humanoid_legged_with_hands_mod.json",vrep=True)
#realRobot = Motion.Motion("../../../json/nico_humanoid_legged_with_hands_mod.json",vrep=False)
virtualRobot = Motion.Motion("../../../json/nico_humanoid_legged_with_hands_mod.json",vrep=True)
realRobot = Motion.Motion("../../../json/nico_humanoid_legged_with_hands_mod.json",vrep=False)

for jNames in (realRobot.getJointNames()):

	print "Setting Zero Position on Virtual: Joint " + str(jNames) 

	targetPosition = 0
	virtualRobot.setAngle(jNames, targetPosition, 0.05)

	time.sleep(1)

	print "Give <RETURN> when ready to set on real Robot?"

	raw_input()

	print "Setting real Robot: Joint " + str(jNames) + " to position " + str(targetPosition)

	realRobot.setAngle(jNames, targetPosition, 0.05)

print "OK. Time to calibrate the offsets. Make the real robot look like the virtual one."
	
raw_input()
	

for jNames in (realRobot.getJointNames()):

	cont=True
	targetPosition = 0
	while(cont):
	
		targetPosition += 5

		print "Setting Virtual: Joint " + str(jNames) + " to position " + str(targetPosition)

		virtualRobot.setAngle(jNames, targetPosition, 0.05)

		time.sleep(1)

		print "Give <RETURN> when ready to set on real Robot?"

		raw_input()

		print "Setting real Robot: Joint " + str(jNames) + " to position " + str(targetPosition)

		realRobot.setAngle(jNames, targetPosition, 0.05)

		time.sleep(1)
		
		print "Do you need another 10 degrees ? (y/n)<RETURN>"

		back=raw_input()

		if back!="y":
			cont=False
			print "OK. Then the next joint."		
		

		    
print "And the stiffness off"
realRobot.disableTorqueAll()
virtualRobot.disableTorqueAll()

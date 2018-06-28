#!/usr/bin/env python

#USAGE: Use script with yes or no as parameter
# python yesno.py yes
# python yesno.py no

import sys
import time
from nicomotion import Motion

robot = Motion.Motion("../../../json/nico_humanoid_upper_with_hands_vrep.json",vrep=False)

position = -20
for i in xrange(6):
	position = position * -1
	if sys.argv[1] == "yes":
		robot.setAngle("head_y", position, 0.05)
        if sys.argv[1] == "no":
                robot.setAngle("head_z", position, 0.05)
	time.sleep(1)

robot.setAngle("head_z", 0, 0.05)
robot.setAngle("head_y", 0, 0.05)


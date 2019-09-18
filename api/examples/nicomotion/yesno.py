#!/usr/bin/env python

# USAGE: Use script with yes or no as parameter
# python yesno.py yes
# python yesno.py no

import logging
import sys
import time

from nicomotion import Motion

logging.basicConfig(level=logging.WARNING)

robot = Motion.Motion("../../../json/nico_humanoid_upper.json",
                      vrep=False, ignoreMissing=True)

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

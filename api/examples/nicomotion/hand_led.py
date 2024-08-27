#!/usr/bin/env python

import logging
import time

from nicomotion import Motion
from os.path import dirname, abspath

logging.basicConfig(level=logging.INFO)

vrep = False
nico_root = dirname(abspath(__file__)) + "/../../.."

# init real robot
robot = Motion.Motion(
    nico_root + "/json/nico_humanoid_upper.json", vrep=False, ignoreMissing=True
)

# saving initial color for reset
initial_color = robot.getHandLEDColor("r")
print(f"Initial hand LED color: {initial_color}")

example_colors = {
    "black": (0.0, 0.0, 0.0),
    "red": (1.0, 0.0, 0.0),
    "orange": (1.0, 0.5, 0.0),
    "yellow": (1.0, 1.0, 0.0),
    "lime": (0.5, 1.0, 0.0),
    "green": (0.0, 1.0, 0.0),
    "spring": (0.0, 1.0, 0.5),
    "cyan": (0.0, 1.0, 1.0),
    "azure": (0.0, 0.5, 1.0),
    "blue": (0.0, 0.0, 1.0),
    "purple": (0.5, 0.0, 1.0),
    "magenta": (1.0, 0.0, 1.0),
    "pink": (1.0, 0.0, 0.5),
    "white": (1.0, 1.0, 1.0),
}

for color, values in example_colors.items():
    print(f"Changing right hand LED to {color} {values}")
    robot.setHandLEDColor("r", *values)
    time.sleep(1.0)

print(f"Resetting hand LED to {initial_color}")
robot.setHandLEDColor("r", *initial_color)

# cleanup
robot.disableTorqueAll()
del robot

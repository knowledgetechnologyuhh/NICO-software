#!/usr/bin/env python

import atexit
import logging
import time
from os.path import abspath, dirname

from nicomotion import Kinematics, Motion

logging.basicConfig(level=logging.WARNING)
logging.getLogger(name="nicomotion.Kinematics").setLevel(logging.DEBUG)

VREP = False

LEN_CUBE_EDGES = 0.05  # in meters
SPACE_BETWEEN_CUBES = 0.04  # in meters
CUBE_DISTANCE = LEN_CUBE_EDGES + SPACE_BETWEEN_CUBES
# used to keep enough distance to the top layer when moving
HAND_HEIGHT = 0.06  # in meters
MAX_Z = 2.
SAFE_HEIGHT = (MAX_Z + 1) * LEN_CUBE_EDGES  # + HAND_HEIGHT
DROP_DISTANCE = LEN_CUBE_EDGES / 4.
# initial position relative to top left cube (from NICO's perspective)
INITIAL_HAND_OFFSET = (0, 0, 0)  # in meters

# define cube positions relative to the top left
CUBE_POSITIONS = []

for z in range(2):
    CUBE_POSITIONS.append([])
    for x in range(3):
        CUBE_POSITIONS[z].append([])
        for y in range(3):
            CUBE_POSITIONS[z][x].append((x * -CUBE_DISTANCE,
                                         y * -CUBE_DISTANCE,
                                         z * LEN_CUBE_EDGES))

if VREP:
    robot = Motion.Motion(
        dirname(abspath(__file__)) +
        "/../../../../json/nico_humanoid_legged_with_hands_mod-vrep.json",
        vrep=True)
    # vrep initial position
    vel = 0.02
    robot.setAngle("head_y", 0, vel)
    robot.setAngle("head_z", 0, vel)
    robot.setAngle("r_shoulder_z", 0, vel)
    robot.setAngle("r_shoulder_y", 0, vel)
    robot.setAngle("r_arm_x", 0, vel)
    robot.setAngle("r_elbow_y", 0, vel)
    robot.setAngle("r_wrist_z", 0, vel)
    robot.setAngle("r_wrist_x", 0, vel)
    robot.setAngle("l_shoulder_z", -26.42, vel)
    robot.setAngle("l_shoulder_y", 22.5, vel)
    robot.setAngle("l_arm_x", 33.45, vel)
    robot.setAngle("l_elbow_y", 80, vel)
    robot.setAngle("l_wrist_z", 40, vel)
    robot.setAngle("l_wrist_x", 0, vel)
else:
    robot = Motion.Motion(dirname(abspath(__file__)) +
                          "/../../../../json/nico_humanoid_upper.json",
                          vrep=False)

kinematics = Kinematics.Kinematics(robot)


def on_exit(robot):
    robot.disableTorqueAll()


atexit.register(on_exit, robot)

robot.setAngle("head_y", 50, 0.05)

raw_input("Move the left arm in its initial position (top grasp on the " +
          "top left cube) and press [enter]")

robot.enableTorqueAll()

# save current position to use as origin
kinematics.save_end_effector_transformation("left_arm", "cube_origin.npy")

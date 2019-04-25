#!/usr/bin/env python

import atexit
import logging
import time
from os.path import abspath, dirname

from nicomotion import Kinematics, Motion

logging.basicConfig(level=logging.WARNING)

vrep = True

if vrep:
    robot = Motion.Motion(
        dirname(abspath(__file__)) +
        "/../../../../json/nico_humanoid_legged_with_hands_mod-vrep.json",
        vrep=True)
    # vrep initial position
    vel = 0.02
    robot.setAngle("l_shoulder_z", -10, vel)
    robot.setAngle("l_shoulder_y", 20, vel)
    robot.setAngle("l_arm_x", -20, vel)
    robot.setAngle("l_elbow_y", 100, vel)
    robot.setAngle("l_wrist_z", 0, vel)
    robot.setAngle("l_wrist_x", 0, vel)
else:
    robot = Motion.Motion(dirname(abspath(__file__)) +
                          "/../../../../json/nico_humanoid_upper.json",
                          vrep=False)


kinematics = Kinematics.Kinematics(robot)
kinematics.logger.setLevel(logging.DEBUG)


def on_exit(robot):
    robot.disableTorqueAll()


atexit.register(on_exit, robot)

robot.setAngle("head_y", 50, 0.05)

raw_input("Move the left arm in its initial position and press [enter]")

robot.closeHand("LHand", fractionMaxSpeed=0.2, percentage=0.675)

kinematics.move_relative("left_arm", 0, 0, -.01, 0, 0, 0)

# save current position to use as origin
kinematics.save_end_effector_position("left_arm", "draw_origin.npy")

# floor _
for t in range(1, 9):
    kinematics.move_to("left_arm", 0, -.01 * t, 0, 0, 0, 0,
                       origin_file="draw_origin.npy")
# wall right _|
for t in range(1, 7):
    kinematics.move_to("left_arm", .01 * t, -.08, 0, 0, 0, 0,
                       origin_file="draw_origin.npy")
# roof right \
for t in range(1, 4):
    kinematics.move_to("left_arm", .06 + .01 * t, -.08 + .01 * t, 0, 0, 0, 0,
                       origin_file="draw_origin.npy")
# roof left /
for t in range(1, 4):
    kinematics.move_to("left_arm", .09 - .01 * t, -.05 + .01 * t, 0, 0, 0, 0,
                       origin_file="draw_origin.npy")
# wall left |_
for t in range(1, 7):
    kinematics.move_to("left_arm", .06 - .01 * t, -.02, 0, 0, 0, 0,
                       origin_file="draw_origin.npy")
# diagonal bottom left to top right |/|
for t in range(1, 7):
    kinematics.move_to("left_arm", .01 * t, -.02 - .01 * t, 0, 0, 0, 0,
                       origin_file="draw_origin.npy")
# ceiling -
for t in range(1, 7):
    kinematics.move_to("left_arm", .06, -.08 + .01 * t, 0, 0, 0, 0,
                       origin_file="draw_origin.npy")
# diagonal top left to bottom right |x|
for t in range(1, 7):
    kinematics.move_to("left_arm", .06 - .01 * t, -.02 - .01 * t, 0, 0, 0, 0,
                       origin_file="draw_origin.npy")

kinematics.move_relative("left_arm", 0, -.1, .15, -45, 0, -90)

time.sleep(4)

robot.setAngle("head_y", 10, 0.05)

raw_input("Return, please")

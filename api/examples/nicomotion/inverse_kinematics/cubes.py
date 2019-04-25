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
SPACE_BETWEEN_CUBES = 0.05  # in meters
CUBE_DISTANCE = LEN_CUBE_EDGES + SPACE_BETWEEN_CUBES
# used to keep enough distance to the top layer when moving
HAND_HEIGHT = 0.06  # in meters
MAX_Z = 2.
SAFE_HEIGHT = (MAX_Z + 1) * LEN_CUBE_EDGES  # + HAND_HEIGHT
DROP_DISTANCE = LEN_CUBE_EDGES / 2.
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

x


def on_exit(robot):
    robot.disableTorqueAll()


atexit.register(on_exit, robot)

robot.setAngle("head_y", 50, 0.05)

raw_input("Move the left arm in its initial position (top grasp on the " +
          "top left cube) and press [enter]")

robot.enableTorqueAll()

# save current position to use as origin
kinematics.save_end_effector_transformation("left_arm", "cube_origin.npy")

# move to safe height
kinematics.move_to(
    "left_arm", 0, 0, SAFE_HEIGHT, 0, 0, 0,
    "cube_origin.npy")


def move_cube_meters(cube_pos, target_pos, origin_orientation=(0, 0, 0),
                     target_orientation=(0, 0, 0)):
    # calibrate orientation:
    # robot.setAngle("l_wrist_z", 90, 0.05)
    # raw_input("Test")
    # kinematics.save_end_effector_position("left_arm", "cube_origin2.npy")
    # exit()

    # move above cube
    kinematics.move_to("left_arm",
                       cube_pos[0],
                       cube_pos[1],
                       SAFE_HEIGHT,
                       origin_orientation[0],
                       origin_orientation[1],
                       origin_orientation[2],
                       "cube_origin.npy")
    robot.openHand("LHand", fractionMaxSpeed=0.1)
    raw_input("moving above cube - press [enter] to continue")
    # pickup cube
    # 1. move down
    kinematics.move_to("left_arm",
                       cube_pos[0],
                       cube_pos[1],
                       cube_pos[2] + LEN_CUBE_EDGES,
                       origin_orientation[0],
                       origin_orientation[1],
                       origin_orientation[2],
                       "cube_origin.npy")
    time.sleep(1)
    kinematics.move_to("left_arm",
                       cube_pos[0],
                       cube_pos[1],
                       cube_pos[2],
                       origin_orientation[0],
                       origin_orientation[1],
                       origin_orientation[2],
                       "cube_origin.npy")
    raw_input("lowering arm to pick up cube - press [enter] to continue")
    # 2. close hand
    robot.closeHand("LHand", fractionMaxSpeed=0.1)
    raw_input("grasping cube - press [enter] to continue")
    # 3. move up
    kinematics.move_to("left_arm",
                       cube_pos[0],
                       cube_pos[1],
                       SAFE_HEIGHT,
                       origin_orientation[0],
                       origin_orientation[1],
                       origin_orientation[2],
                       "cube_origin.npy")
    raw_input("lifting cube - press [enter] to continue")
    # move above target position
    kinematics.move_to("left_arm",
                       target_pos[0],
                       target_pos[1],
                       SAFE_HEIGHT,
                       target_orientation[0],
                       target_orientation[1],
                       target_orientation[2],
                       "cube_origin.npy")
    raw_input("moving above target positon - press [enter] to continue")
    # place cube
    # 1. move down
    kinematics.move_to("left_arm",
                       target_pos[0],
                       target_pos[1],
                       target_pos[2] + DROP_DISTANCE,
                       target_orientation[0],
                       target_orientation[1],
                       target_orientation[2],
                       "cube_origin.npy")
    raw_input("lowering arm to place cube - press [enter] to continue")
    # 2. open hand
    robot.openHand("LHand", fractionMaxSpeed=0.1)
    raw_input("opening hand to place cube - press [enter] to continue")
    # 3. move up
    kinematics.move_to("left_arm",
                       target_pos[0],
                       target_pos[1],
                       SAFE_HEIGHT,
                       target_orientation[0],
                       target_orientation[1],
                       target_orientation[2],
                       "cube_origin.npy")
    # to origin
    kinematics.move_to("left_arm", 0, 0, SAFE_HEIGHT,
                       0, 0, 0, "cube_origin.npy")
    # kinematics.move_to("left_arm", 0, 0, 0, 0, 0, 0, "cube_origin.npy")


def coords_to_meters(x, y, z):
    x = INITIAL_HAND_OFFSET[0] - x * CUBE_DISTANCE
    y = INITIAL_HAND_OFFSET[1] - y * CUBE_DISTANCE
    z = INITIAL_HAND_OFFSET[2] + z * LEN_CUBE_EDGES
    return x, y, z


def move_cube_coords(cube_coord, target_coord, origin_orientation=(0, 0, 0),
                     target_orientation=(0, 0, 0)):
    raw_input("moving {} to {} - press [enter] to continue".format(cube_coord,
                                                                   target_coord
                                                                   ))
    cube_meters = coords_to_meters(*cube_coord)
    target_meters = coords_to_meters(*target_coord)
    move_cube_meters(cube_meters, target_meters,
                     origin_orientation, target_orientation)


# put top left on top of center cube
move_cube_coords((0, 0, 0), (1, 1, 1), target_orientation=(22.5, 0, -22.5))
# put bottom left on top of top right
# move_cube_coords((2, 0, 0), (0, 2, 1), target_orientation=(-45, 10, 0))

time.sleep(4)

robot.setAngle("head_y", 10, 0.05)

raw_input("Make sure the arm is protected and press return, please")

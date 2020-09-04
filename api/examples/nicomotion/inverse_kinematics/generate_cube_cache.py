#!/usr/bin/env python

import logging
import time
from os.path import abspath, dirname, isfile

from nicomotion import Kinematics

try:
    import cPickle as pickle
except ImportError:
    import pickle

logging.basicConfig(level=logging.WARNING)
logging.getLogger(name="nicomotion.Kinematics").setLevel(logging.DEBUG)

LEN_CUBE_EDGES = 0.05  # in meters
SPACE_BETWEEN_CUBES = 0.04  # in meters
CUBE_DISTANCE = LEN_CUBE_EDGES + SPACE_BETWEEN_CUBES
# used to keep enough distance to the top layer when moving
HAND_HEIGHT = 0.06  # in meters
MAX_Z = 2.
SAFE_HEIGHT = (MAX_Z + 1) * LEN_CUBE_EDGES  # + HAND_HEIGHT
DROP_DISTANCE = LEN_CUBE_EDGES / 4.
# initial position relative to top left cube (from NICO's perspective)
INITIAL_HAND_OFFSET = (0.035, 0, 0.01)  # in meters

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


class FakeRobot(object):
    """Object that imitates robot in inverse kinematics"""

    def __init__(self):
        self._vrep = False
        self.angles = {}

    def getAngle(self, name):
        if name not in self.angles:
            self.angles[name] = 0
        return self.angles[name]

    def setAngle(self, name, angle, _):
        self.angles[name] = angle


def initial_position(robot):
    robot.setAngle("l_shoulder_z", 45, 0.02)
    robot.setAngle("l_shoulder_y", -45, 0.02)
    robot.setAngle("l_arm_x", 25, 0.02)
    robot.setAngle("l_elbow_y", 80, 0.02)
    robot.setAngle("l_wrist_z", 0, 0.02)
    robot.setAngle("l_wrist_x", 0, 0.02)


robot = FakeRobot()
initial_position(robot)

kinematics = Kinematics.Kinematics(robot)


def add_to_cache(robot, kinematics, cache, px, py, pz, r, p, y):
    kinematics.move_to(
        "left_arm", px, py, pz, r, p, y,
        "cube_origin.npy")
    sub_cache = cache
    for key in (px, py, pz, r, p, y):
        if key not in sub_cache:
            sub_cache[key] = {}
        sub_cache = sub_cache[key]
    for motor, angle in robot.angles.iteritems():
        sub_cache[motor] = angle
    with open('cube_cache.pkl', 'wb') as f:
        # Pickle the 'data' dictionary using the highest protocol available.
        pickle.dump(cache, f, pickle.HIGHEST_PROTOCOL)


def move_to_cached(robot, kinematics, px, py, pz, r, p, y):
    if isfile('cube_cache.pkl'):
        with open('cube_cache.pkl', 'rb') as f:
            cache = pickle.load(f)
    else:
        cache = {}
    try:
        for motor, angle in cache[px][py][pz][r][p][y].iteritems():
            robot.setAngle(motor, angle, 0.02)
    except KeyError:
        add_to_cache(robot, kinematics, cache, px, py, pz, r, p, y)


# move to safe height
move_to_cached(robot, kinematics, 0, CUBE_DISTANCE,
               SAFE_HEIGHT, 0, 0, 0)


def move_cube_meters(cube_pos, target_pos, origin_orientation=(0, 0, 0),
                     target_orientation=(0, 0, 0)):
    # calibrate orientation:
    # robot.setAngle("l_wrist_z", 90, 0.05)
    # raw_input("Test")
    # kinematics.save_end_effector_position("left_arm", "cube_origin2.npy")
    # exit()

    # move above cube
    move_to_cached(robot,
                   kinematics,
                   cube_pos[0],
                   cube_pos[1],
                   SAFE_HEIGHT,
                   origin_orientation[0],
                   origin_orientation[1],
                   origin_orientation[2])
    print("moving above cube")  # - press [enter] to continue")
    # pickup cube
    # 1. move down
    move_to_cached(robot,
                   kinematics,
                   cube_pos[0],
                   cube_pos[1],
                   cube_pos[2] + LEN_CUBE_EDGES + DROP_DISTANCE,
                   origin_orientation[0],
                   origin_orientation[1],
                   origin_orientation[2])
    move_to_cached(robot,
                   kinematics,
                   cube_pos[0],
                   cube_pos[1],
                   cube_pos[2],
                   origin_orientation[0],
                   origin_orientation[1],
                   origin_orientation[2])
    print("lowering arm to pick up cube")  # - press [enter] to continue")
    # 3. move up
    move_to_cached(robot,
                   kinematics,
                   cube_pos[0],
                   cube_pos[1],
                   SAFE_HEIGHT,
                   origin_orientation[0],
                   origin_orientation[1],
                   origin_orientation[2])
    print("lifting cube")  # - press [enter] to continue")
    # move above target position
    move_to_cached(robot,
                   kinematics,
                   target_pos[0],
                   target_pos[1],
                   SAFE_HEIGHT,
                   target_orientation[0],
                   target_orientation[1],
                   target_orientation[2])
    print("moving above target positon")  # - press [enter] to continue")
    # place cube
    # 1. move down
    move_to_cached(robot,
                   kinematics,
                   target_pos[0],
                   target_pos[1],
                   target_pos[2] + DROP_DISTANCE,
                   target_orientation[0],
                   target_orientation[1],
                   target_orientation[2])
    print("lowering arm to place cube")  # - press [enter] to continue")
    # 3. move up
    move_to_cached(robot,
                   kinematics,
                   target_pos[0],
                   target_pos[1],
                   SAFE_HEIGHT,
                   target_orientation[0],
                   target_orientation[1],
                   target_orientation[2])
    # to origin
    move_to_cached(robot, kinematics, 0, CUBE_DISTANCE, SAFE_HEIGHT, 0, 0, 0)
    # kinematics.move_to("left_arm", 0, 0, 0, 0, 0, 0, "cube_origin.npy")


def coords_to_meters(x, y, z):
    x = INITIAL_HAND_OFFSET[0] - x * CUBE_DISTANCE
    y = INITIAL_HAND_OFFSET[1] - y * CUBE_DISTANCE
    z = INITIAL_HAND_OFFSET[2] + z * LEN_CUBE_EDGES
    return x, y, z


def move_cube_coords(cube_coord, target_coord, origin_orientation=(0, 0, 0),
                     target_orientation=(0, 0, 0)):
    # - press [enter] to continue"
    print("moving {} to {}".format(cube_coord, target_coord))
    cube_meters = coords_to_meters(*cube_coord)
    target_meters = coords_to_meters(*target_coord)
    move_cube_meters(cube_meters, target_meters,
                     origin_orientation, target_orientation)


# put top left on top of top right
move_cube_coords((0, 0, 0), (0, 1, 1))  # works!
initial_position(robot)
# put top left on top of bottom left
move_cube_coords((0, 0, 0), (1, 0, 1))  # works too!
initial_position(robot)
# put top left on top of bottom right
move_cube_coords((0, 0, 0), (1, 1, 1))  # kinda works as well!
initial_position(robot)

# put top right on top of top left
move_cube_coords((0, 1, 0), (0, 0, 1))  # works but throws it off afterwards
initial_position(robot)
# put top right on top of bottom left
move_cube_coords((0, 1, 0), (1, 0, 1))  # works if NICO doesn't drop the cube
initial_position(robot)
# put top right on top of bottom right
move_cube_coords((0, 1, 0), (1, 1, 1))  # kinda works
initial_position(robot)

# put bottom left on top of top left
move_cube_coords((1, 0, 0), (0, 0, 1))  # works
initial_position(robot)
# put bottom left on top of top right
move_cube_coords((1, 0, 0), (0, 1, 1))  # works
initial_position(robot)
# put bottom left on top of bottom right
move_cube_coords((1, 0, 0), (1, 1, 1))  # kinda works
initial_position(robot)

# put bottom right on top of top left
move_cube_coords((1, 1, 0), (0, 0, 1))  # 2-finger grip, knocks off cube
initial_position(robot)
# put bottom right on top of top right
move_cube_coords((1, 1, 0), (0, 1, 1))  # 2-fingers, works, but moves stack
initial_position(robot)
# put bottom right on top of bottom left
move_cube_coords((1, 1, 0), (1, 0, 1))  # 2-fingers, works, but moves stack
initial_position(robot)

# ------------Top level------------

# put top left on top of top right
move_cube_coords((0, 0, 1), (0, 1, 0))
initial_position(robot)
# put top left on top of bottom left
move_cube_coords((0, 0, 1), (1, 0, 0))
initial_position(robot)
# put top left on top of bottom right
move_cube_coords((0, 0, 1), (1, 1, 0))
initial_position(robot)

# put top right on top of top left
move_cube_coords((0, 1, 1), (0, 0, 0))
initial_position(robot)
# put top right on top of bottom left
move_cube_coords((0, 1, 1), (1, 0, 0))
initial_position(robot)
# put top right on top of bottom right
move_cube_coords((0, 1, 1), (1, 1, 0))
initial_position(robot)

# put bottom left on top of top left
move_cube_coords((1, 0, 1), (0, 0, 0))
initial_position(robot)
# put bottom left on top of top right
move_cube_coords((1, 0, 1), (0, 1, 0))
initial_position(robot)
# put bottom left on top of bottom right
move_cube_coords((1, 0, 1), (1, 1, 0))
initial_position(robot)

# put bottom right on top of top left
move_cube_coords((1, 1, 1), (0, 0, 0))
initial_position(robot)
# put bottom right on top of top right
move_cube_coords((1, 1, 1), (0, 1, 0))
initial_position(robot)
# put bottom right on top of bottom left
move_cube_coords((1, 1, 1), (1, 0, 0))
initial_position(robot)


print("done")

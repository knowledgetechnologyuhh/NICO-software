#!/usr/bin/env python
import logging
import signal
import sys
import time
from os.path import abspath, dirname

from nicomotion import Kinematics, Motion

try:
    import cPickle as pickle
except ImportError:
    import pickle


class CubeDemo(object):
    """docstring for CubeDemo"""

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

    def __init__(self,
                 robot_file=(dirname(abspath(__file__)) +
                             "/../../../../json/nico_humanoid_upper.json"),
                 cache_file=(dirname(abspath(__file__)) + '/cube_cache.pkl')):
        self.robot = Motion.Motion(robot_file, vrep=False)

        def signal_handler(sig, frame):
            self.cleanup()
            sys.exit(0)
        signal.signal(signal.SIGINT, signal_handler)

        with open(cache_file, 'rb') as f:
            self.cache = pickle.load(f)

        self.robot.setAngle("head_y", 45, 0.05)
        self.robot.setAngle("head_z", 22.5, 0.05)
        self.initial_position()
        time.sleep(3)

    def __del__(self):
        self.robot.disableTorqueAll()

    def cleanup(self):
        print('Cleanup')
        self.robot.disableTorqueAll()

    def initial_position(self):
        self.robot.setAngle("l_shoulder_z", 45, 0.02)
        self.robot.setAngle("l_shoulder_y", -45, 0.02)
        self.robot.setAngle("l_arm_x", 25, 0.02)
        self.robot.setAngle("l_elbow_y", 80, 0.02)
        self.robot.setAngle("l_wrist_z", 0, 0.02)
        self.robot.setAngle("l_wrist_x", 0, 0.02)

    def move_to_cached(self, px, py, pz, r, p, y):
        try:
            for motor, angle in self.cache[px][py][pz][r][p][y].iteritems():
                self.robot.setAngle(motor, angle, 0.02)
        except KeyError as e:
            print("cache incomplete")
            raise e

    def move_cube_meters(self, cube_pos, target_pos,
                         origin_orientation=(0, 0, 0),
                         target_orientation=(0, 0, 0)):
        # move to safe height
        self.move_to_cached(0, self.CUBE_DISTANCE, self.SAFE_HEIGHT, 0, 0, 0)
        time.sleep(2)

        # move above cube
        self.move_to_cached(cube_pos[0],
                            cube_pos[1],
                            self.SAFE_HEIGHT,
                            origin_orientation[0],
                            origin_orientation[1],
                            origin_orientation[2])
        self.robot.openHand("LHand", fractionMaxSpeed=0.05)
        print("moving above cube")  # - press [enter] to continue")
        time.sleep(1.5)
        # pickup cube
        # 1. move down
        self.move_to_cached(cube_pos[0],
                            cube_pos[1],
                            cube_pos[2] + self.LEN_CUBE_EDGES +
                            self.DROP_DISTANCE,
                            origin_orientation[0],
                            origin_orientation[1],
                            origin_orientation[2])
        time.sleep(1)
        self.move_to_cached(cube_pos[0],
                            cube_pos[1],
                            cube_pos[2],
                            origin_orientation[0],
                            origin_orientation[1],
                            origin_orientation[2])
        print("lowering arm to pick up cube")  # - press [enter] to continue")
        time.sleep(2)
        # 2. close hand
        self.robot.closeHand("LHand", fractionMaxSpeed=0.05)
        print("grasping cube")  # - press [enter] to continue")
        time.sleep(2)
        # 3. move up
        self.move_to_cached(cube_pos[0],
                            cube_pos[1],
                            self.SAFE_HEIGHT,
                            origin_orientation[0],
                            origin_orientation[1],
                            origin_orientation[2])
        print("lifting cube")  # - press [enter] to continue")
        time.sleep(2)
        # move above target position
        self.move_to_cached(target_pos[0],
                            target_pos[1],
                            self.SAFE_HEIGHT,
                            target_orientation[0],
                            target_orientation[1],
                            target_orientation[2])
        print("moving above target positon")  # - press [enter] to continue")
        time.sleep(2)
        # place cube
        # 1. move down
        self.move_to_cached(target_pos[0],
                            target_pos[1],
                            target_pos[2] + self.DROP_DISTANCE,
                            target_orientation[0],
                            target_orientation[1],
                            target_orientation[2])
        print("lowering arm to place cube")  # - press [enter] to continue")
        time.sleep(3)
        # 2. open hand
        self.robot.openHand("LHand", fractionMaxSpeed=0.05)
        print("opening hand to place cube")  # - press [enter] to continue")
        time.sleep(2)
        # 3. move up
        self.move_to_cached(target_pos[0],
                            target_pos[1],
                            self.SAFE_HEIGHT,
                            target_orientation[0],
                            target_orientation[1],
                            target_orientation[2])
        time.sleep(1.5)
        # to origin
        print("returning arm to initial position")
        self.initial_position()
        time.sleep(3)
        # move_to_cached(robot, 0, CUBE_DISTANCE, SAFE_HEIGHT, 0, 0, 0)
        # time.sleep(1)
        # kinematics.move_to("left_arm", 0, 0, 0, 0, 0, 0, "cube_origin.npy")

    def coords_to_meters(self, x, y, z):
        x = self.INITIAL_HAND_OFFSET[0] - x * self.CUBE_DISTANCE
        y = self.INITIAL_HAND_OFFSET[1] - y * self.CUBE_DISTANCE
        z = self.INITIAL_HAND_OFFSET[2] + z * self.LEN_CUBE_EDGES
        return x, y, z

    def move_cube_coords(self, cube_coord, target_coord,
                         origin_orientation=(0, 0, 0),
                         target_orientation=(0, 0, 0)):
        # - press [enter] to continue"
        print("moving {} to {}".format(cube_coord, target_coord))
        cube_meters = self.coords_to_meters(*cube_coord)
        target_meters = self.coords_to_meters(*target_coord)
        self.move_cube_meters(cube_meters, target_meters,
                              origin_orientation, target_orientation)


if __name__ == '__main__':
    logging.basicConfig(level=logging.WARNING)
    logging.getLogger(name="nicomotion.Kinematics").setLevel(logging.DEBUG)

    demo = CubeDemo()
    # put top left on top of top right
    # demo.move_cube_coords((0, 0, 0), (0, 1, 1))  # works!
    # put top left on top of bottom left
    # demo.move_cube_coords((0, 0, 0), (1, 0, 1))  # works too!
    # put top left on top of bottom right
    # demo.move_cube_coords((0, 0, 0), (1, 1, 1))  # kinda works as well!

    # put top right on top of top left
    # demo.move_cube_coords((0, 1, 0), (0, 0, 1))  # works but throws it off afterwards
    # put top right on top of bottom left
    # demo.move_cube_coords((0, 1, 0), (1, 0, 1))  # works if NICO doesn't drop the cube
    # put top right on top of bottom right
    # demo.move_cube_coords((0, 1, 0), (1, 1, 1))  # kinda works

    # put bottom left on top of top left
    # demo.move_cube_coords((1, 0, 0), (0, 0, 1))  # works but throws it off afterwards
    # put bottom left on top of top right
    # demo.move_cube_coords((1, 0, 0), (0, 1, 1))  # works
    # put bottom left on top of bottom right
    # demo.move_cube_coords((1, 0, 0), (1, 1, 1))  # works

    # put bottom right on top of top left
    # demo.move_cube_coords((1, 1, 0), (0, 0, 1))  # 2-finger grip, knocks off cube
    # put bottom right on top of top right
    # demo.move_cube_coords((1, 1, 0), (0, 1, 1))  # 2-fingers, works, but moves stack
    # put bottom right on top of bottom left
    # demo.move_cube_coords((1, 1, 0), (1, 0, 1))  # 2-fingers, works

    # ------------Top level------------

    # put upper top left to lower top right
    # demo.move_cube_coords((0, 0, 1), (0, 1, 0))  # needs offset to grab
    # put upper top left to lower bottom left
    # demo.move_cube_coords((0, 0, 1), (1, 0, 0))
    # put upper top left to lower bottom right
    # demo.move_cube_coords((0, 0, 1), (1, 1, 0))

    # put upper top right to lower top left
    # demo.move_cube_coords((0, 1, 1), (0, 0, 0))  # works
    # put upper top right to lower bottom left
    # demo.move_cube_coords((0, 1, 1), (1, 0, 0))  # works
    # put upper top right to lower bottom right
    # demo.move_cube_coords((0, 1, 1), (1, 1, 0))  # works

    # put upper bottom left to lower top left
    # demo.move_cube_coords((1, 0, 1), (0, 0, 0))  # works
    # put upper bottom left to lower top right
    # demo.move_cube_coords((1, 0, 1), (0, 1, 0))  # works
    # put upper bottom left to lower bottom right
    # demo.move_cube_coords((1, 0, 1), (1, 1, 0))  # works

    # put upper bottom right to lower top left
    # demo.move_cube_coords((1, 1, 1), (0, 0, 0))  # works
    # put upper bottom right to lower top right
    # demo.move_cube_coords((1, 1, 1), (0, 1, 0))  # works
    # put upper bottom right to lower bottom left
    # demo.move_cube_coords((1, 1, 1), (1, 0, 0))  # works

    demo.move_cube_coords((1, 0, 0), (1, 1, 1))
    demo.move_cube_coords((0, 0, 0), (1, 0, 0))
    demo.move_cube_coords((1, 1, 1), (0, 0, 0))

    demo.robot.setAngle("head_y", 10, 0.05)
    demo.robot.setAngle("head_z", 0, 0.05)
    demo.initial_position()
    time.sleep(4)
    raw_input("Make sure the arm is protected and press return, please")

#!/usr/bin/env python


import sys
import time

import Motion


class Freezer:

    def __init__(self, robot, stiff_off=False,
                 path_to_config_file="mover.conf"):

        # print "Waiting for 2 seconds - Do not know why"
        # time.sleep(2)
        # virtualRobot = Motion.Motion(
        # "../../../json/nico_humanoid_upper_with_hands_vrep_mod.json",
        # vrep=True)
        self.robot = robot
        self.stiff_off = stiff_off

    def __del__(self):

        if self.stiff_off is True:
            print "And the stiffness off"
            self.robot.disableTorqueAll()
        # virtualRobot.disableTorqueAll()

    def torque_joints_from_subset(self, subsetfname=None, stiffness=None,
                                  untorque=False):

        import csv

        # If no subset chosen, set torque for all
        if subsetfname is None:
            self.robot.enableTorqueAll()
        else:
            with open(subsetfname) as f:
                subsf = csv.reader(f)
                subsetjoints = next(subsf)
            for joint in subsetjoints:
                if not untorque:
                    self.robot.enableTorque(joint)
                else:
                    self.robot.disableTorque(joint)
                if stiffness is not None:
                    self.robot.setStiffness(joint, stiffness)

    # Enables torque for all or subsets of joints defined by subset file
    # Disables with unfreeze=True
    def freeze_joints(self, subsetfname=None, stiffness=None, unfreeze=False):

        import csv

        # If no subset chosen, set torque for all
        if subsetfname is None:
            self.robot.enableTorqueAll()
        else:
            with open(subsetfname) as f:
                subsf = csv.reader(f)
                subsetjoints = next(subsf)
            for joint in subsetjoints:
                if not unfreeze:
                    pos = self.robot.getAngle(joint)
                    time.sleep(0.1)
                    self.robot.enableTorque(joint)
                    pos = self.robot.getAngle(joint)
                    print "angle: " + joint + " " + str(pos)
                    self.robot.setAngle(joint, pos, 0.01)
                    self.robot.enableTorque(joint)
                    # self.robot.enableTorque(joint)
                    # self.robot.setStiffness(joint,0.5)
                else:
                    self.robot.disableTorque(joint)
                if stiffness is not None:
                    self.robot.setStiffness(joint, stiffness)

    def close_hands(self, file_subset, hand, open=False):

        angle = 30
        if open:
            angle = -180
        self.torque_joints_from_subset(file_subset)
        self.robot.setAngle(hand + "_indexfingers_x", angle, 0.5)
        self.robot.setAngle(hand + "_thumb_x", angle, 0.5)
        time.sleep(1)
        self.torque_joints_from_subset(file_subset, untorque=True)


if __name__ == "__main__":

    import logging
    logging.basicConfig(filename='Freezer.log', level=logging.ERROR)

    import argparse
    subsets = {"la": "../../../../../moves_and_positions/subset_left_arm.csv",
               "ra": "../../../../../moves_and_positions/subset_right_arm.csv",
               "h": "../../../../../moves_and_positions/subset_head.csv"}

    limbs = {"la": "left arm",
             "ra": "right arm",
             "h": "head"}

    hands = {"l": "LHAND",
             "r": "RHAND"}

    subsets_hands = {
        "l": "../../../../../moves_and_positions/subset_left_hand.csv",
        "r": "../../../../../moves_and_positions/subset_right_hand.csv"
    }

    # examples
    # start the freezer with different json-files

    parser = argparse.ArgumentParser()
    # parser.add_argument("command", help="One of the commands m " +
    # "(record movement), p (record position), pm (play movement)," +
    # " pp (play position), cm (calculate movement) fj(freeze joints) " +
    # "uj(unfreeze joints)")
    # fj freeze joints as they are by torquing it. You subset to freeze only
    # a subset of the joints.
    parser.add_argument('--json', nargs='?',
                        default='../../../../../json/nico_humanoid_upper.json',
                        help=("robots json file. Default: " +
                              "nico_humanoid_upper.json"))
    # parser.add_argument('--subset', nargs='?', default=None,
    # help="joint subset file")
    parser.add_argument('--speed', nargs='?',
                        default="0.05", help="speed of movement")
    parser.add_argument('--vrep', action="store_true", default=False,
                        help="let it run on vrep than instead of real robot")
    parser.add_argument('--stiffoff', action="store_true", default=False,
                        help="sets the stiffness to off after movement")
    args = parser.parse_args()
    # print args

    connected = False
    while (not connected):
        try:
            robot = Motion.Motion(args.json, vrep=args.vrep)
            connected = True
        except Exception, e:
            print('\nFailed to connect to robot: ' + str(e))
            print "\n Not connected. Let me try it again."
            time.sleep(1)

    freezer = Freezer(robot, stiff_off=args.stiffoff)
    # robot.enableTorqueAll()
    while(True):
        print "\b Give command (fla,fra,fh,lla,lra,lh,or,ol,cl,cr):"
        command = raw_input()
        # print command + " " + subsets[command[1:]] + "\n"

        if command[:1] == "f":

            print "Please put " + limbs[command[1:]
                                        ] + " in position and give return"
            raw_input()
            freezer.freeze_joints(subsetfname=subsets[command[1:]],
                                  stiffness=0.8, unfreeze=False)

        if command[:1] == "l":
            print "Please hold " + limbs[command[1:]] + " and give return"
            raw_input()
            freezer.freeze_joints(
                subsetfname=subsets[command[1:]], stiffness=0.8, unfreeze=True)

        if command[:1] == "o":

            hand = hands[command[1:]]
            print "Will open " + hand
            freezer.close_hands(
                subsets_hands[command[1:]], command[1:], open=True)

        if command[:1] == "c":
            hand = hands[command[1:]]
            print "Will close " + hand
            freezer.close_hands(subsets_hands[command[1:]], command[1:])

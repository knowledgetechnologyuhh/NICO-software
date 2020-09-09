#!/usr/bin/env python


# TODO

# Examples for using the Mover Object
# 	Movement sequence
#   Movement sequence combined with grasper (dice grasp and throw)


# Movements as threads
# 	Optinal start the mover in a thread and return the calculated time

# Subset extensiosn
# 	subset for recording
# 	handle incomplete joint set in recorded files
# 	filter recording file with subset file and write as new recording file


# Include the inverse kinematic

#


import sys
import time

import nicomotion.Motion as Motion

if sys.version_info[0] >= 3:
    raw_input = input


class Mover:
    def __init__(self, robot, stiff_off=False, path_to_config_file="mover.conf"):

        # print "Waiting for 2 seconds - Do not know why"
        # time.sleep(2)
        # virtualRobot = Motion.Motion(
        #     "../../../json/nico_humanoid_upper_with_hands_vrep_mod.json",
        #     vrep=True)
        self.robot = robot
        self.stiff_off = stiff_off

    def __del__(self):

        if self.stiff_off is True:
            print("And the stiffness off")
            self.robot.disableTorqueAll()
        # virtualRobot.disableTorqueAll()

    # record a movement. For every move on the trajectory, one <return> has to
    # be pressed
    def record_movement(self, fname=None):

        import csv
        import time

        key = "s"

        print("Give 'q' for stop recording")

        if fname is None:
            fname = time.strftime("traj_%Y%m%d-%H%M%S") + ".csv"

        # filename = time.strftime("%Y%m%d-%H%M%S") + ".csv"
        # print fname

        raw_input()

        with open(fname, "w") as jointfile:

            wr = csv.writer(jointfile, quoting=csv.QUOTE_ALL)
            wr.writerow(self.robot.getJointNames())

            while key != "q":

                recent_joint_positions = [
                    self.robot.getAngle(jName) for jName in (self.robot.getJointNames())
                ]

                wr.writerow(recent_joint_positions)

                key = raw_input()

        print("Movement written as " + fname)

    # record a position. Move the robot to its position and press <return>
    def record_position(self, fname=None):

        import csv
        import time

        key = "s"

        print("Put the NICO in the position and press <return>")

        if fname is None:
            fname = time.strftime("pos-%Y%m%d-%H%M%S") + ".csv"

        # filename = time.strftime("%Y%m%d-%H%M%S") + ".csv"

        raw_input()

        with open(fname, "w") as jointfile:

            wr = csv.writer(jointfile, quoting=csv.QUOTE_ALL)
            wr.writerow(self.robot.getJointNames())

            recent_joint_positions = [
                self.robot.getAngle(jName) for jName in (self.robot.getJointNames())
            ]

            wr.writerow(recent_joint_positions)

        print("Position written as " + fname)

    # Move the robot straight to the goal position. synchronize the speed for
    # the joint in a way, that they reach the position at the same time
    def move_position(self, target_positions, speed, real=True):

        # calculate current angular speed
        cur_speed = (63.0 / 60) * 360 * speed

        import copy

        # get the current positions of all joints to move
        current_positions = copy.deepcopy(target_positions)
        for joint in current_positions:
            current_positions[joint] = self.robot.getAngle(joint)
        time_to_reach = {
            k: abs(
                (float(current_positions[k]) - float(target_positions[k])) / cur_speed
            )
            for k in current_positions
        }
        # print time_to_reach
        max_time = max(time_to_reach.values())
        max_keys = [k for k, v in time_to_reach.items() if v == max_time]
        print("Max time: " + str((max_keys, max_time)))
        for joi in target_positions:
            if real and max_time != 0.0:
                self.robot.setAngle(
                    joi,
                    float(target_positions[joi]),
                    ((speed * time_to_reach[joi]) / max_time),
                )
        return max_time

    # Read the position from a file and move the joint from the current postion
    # to the goal
    def move_file_position(self, fname, subsetfname=None, move_speed=0.04):

        import csv
        import time

        if subsetfname is not None:
            with open(subsetfname) as f:
                subsf = csv.reader(f)
                subsetjoints = next(subsf)

        mt = 0
        with open(fname, "r") as infile:
            reader = csv.DictReader(infile)
            # If no subsetfile, send to all the joints
            if subsetfname is None:
                for row in reader:
                    mt = self.move_position(row, move_speed)
            else:
                # Else send only to the joints defined in the subset file

                for row in reader:
                    # Move all joints in the subset to the postion
                    joi = {k: row[k] for k in subsetjoints}
                    # print joi
                    mt = self.move_position(joi, move_speed)
        return mt

    # Read the position from a file and calculate a trajectory from the current
    # position to i
    def calc_move_file(self, fname, target_fname, number):

        import csv
        import copy

        with open(fname, "r") as infile:
            reader = csv.DictReader(infile)
            for row in reader:
                target_positions = row

            current_positions = copy.deepcopy(target_positions)
            for joint in current_positions:
                current_positions[joint] = self.robot.getAngle(joint)
            stepsize = {
                k: (float(target_positions[k]) - float(current_positions[k])) / number
                for k in current_positions
            }

            with open(target_fname, "w") as jointfile:

                wr = csv.writer(jointfile, quoting=csv.QUOTE_ALL)
                wr.writerow(target_positions.keys())

                calc_pos = copy.deepcopy(current_positions)
                for n in range(number):
                    wr.writerow(calc_pos.values())
                    calc_pos = {k: (calc_pos[k] + stepsize[k]) for k in calc_pos}

        # return (mt)

    # Read a trajectory from a file and move the joints over the trajectory
    def play_movement(self, fname, subsetfname=None, move_speed=0.04):

        import csv
        import time

        # filename = time.strftime("%Y%m%d-%H%M%S") + ".csv"
        # print fname

        # raw_input()

        if subsetfname is not None:
            with open(subsetfname) as f:
                subsf = csv.reader(f)
                subsetjoints = next(subsf)

        mt = 0
        with open(fname, "r") as infile:
            reader = csv.DictReader(infile)
            # If no subsetfile, send to all the joints
            if subsetfname is None:
                for row in reader:
                    mt = self.move_position(row, move_speed)
                    # print row
                    time.sleep(mt * 0.85)
                time.sleep(mt * 3)
            else:
                # Else send only to the joints defined in the subset file

                for row in reader:
                    # Move all joints in the subset to the postion
                    joi = {k: row[k] for k in subsetjoints}
                    # print joi
                    mt = self.move_position(joi, move_speed)
                    # for jName in {k: row[k] for k in subsetjoints}:
                    # self.robot.setAngle(jName, float(row[jName]), 0.05)
                    # print row
                    time.sleep(mt * 0.85)
                time.sleep(mt * 3)

            # Read a trajectory from a file and move the joints over the
            # trajectory

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
                    pos = self.robot.getAngle(joint)
                    print("angle: " + joint + " " + str(pos))
                    self.robot.setAngle(joint, pos, 0.02)
                    # self.robot.enableTorque(joint)
                    # self.robot.enableTorque(joint)
                    # self.robot.setStiffness(joint,0.5)
                else:
                    self.robot.disableTorque(joint)
                if stiffness is not None:
                    self.robot.setStiffness(joint, stiffness)


if __name__ == "__main__":

    import argparse
    import logging

    logging.basicConfig(level=logging.ERROR)

    # examples
    # Move with move file from current position over trajectory

    # python Mover.py pm --file mov_take_something_with_left_arm.csv
    # 					 --subset subset_left_arm_and_head.csv --speed 0.1
    #                    --vrep

    # Move to position stored in move file

    # python Mover.py pp --file mov_take_something_with_left_arm.csv
    # 					 --subset subset_left_arm_and_head.csv --speed 0.1

    # Record move file (trajectory)

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "command",
        help=(
            "One of the commands m (record movement), "
            + "p (record position), pm (play movement), "
            + "pp (play position), cm (calculate movement) "
            + "fj(freeze joints) uj(unfreeze joints)"
        ),
    )
    # fj freeze joints as they are by torquing it. You subset to freeze only
    # a subset of the joints.
    parser.add_argument(
        "--json",
        nargs="?",
        default="../../../../../json/nico_humanoid_upper.json",
        help=("robots json file. Default: " + "nico_humanoid_upper.json"),
    )
    parser.add_argument(
        "--filename", nargs="?", default=None, help="file to record or to play"
    )
    parser.add_argument(
        "--targetfilename", nargs="?", default="/tmp/mov-calc.csv", help="file to write"
    )
    parser.add_argument("--subset", nargs="?", default=None, help="joint subset file")
    parser.add_argument("--speed", nargs="?", default="0.05", help="speed of movement")
    parser.add_argument(
        "--vrep",
        action="store_true",
        default=False,
        help="let it run on vrep than instead of real robot",
    )
    parser.add_argument(
        "--stiffoff",
        action="store_true",
        default=False,
        help="sets the stiffness to off after movement",
    )
    args = parser.parse_args()
    # print args

    robot = Motion.Motion(args.json, vrep=args.vrep)
    mov = Mover(robot, stiff_off=args.stiffoff)

    command = args.command

    if args.filename is not None:
        filename = "../../../../../moves_and_positions/" + args.filename
    else:
        filename = None

    if args.subset is not None:
        subsetfilename = "../../../../../moves_and_positions/" + args.subset
    else:
        subsetfilename = None

    if command == "m":
        robot.disableTorqueAll()
        mov.record_movement(filename)

    if command == "p":
        mov.record_position(filename)

    if command == "pm":
        mov.play_movement(filename, subsetfilename, move_speed=float(args.speed))
        raw_input()

    if command == "pp":
        mov.move_file_position(filename, subsetfilename, move_speed=float(args.speed))
        raw_input()

    if command == "cm":
        mov.calc_move_file(filename, args.targetfilename, 10)
        mov.play_movement(args.targetfilename, subsetfilename, move_speed=0.5)

    if command == "fj":
        mov.freeze_joints(subsetfilename)

    if command == "uj":
        mov.freeze_joints(subsetfilename, unfreeze=True)

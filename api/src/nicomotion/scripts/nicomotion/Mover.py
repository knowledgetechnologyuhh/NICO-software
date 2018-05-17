#!/usr/bin/env python


# To Dos

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


import time
from nicomotion import Motion
import sys
import pypot.dynamixel


class Mover:

    def __init__(self, robot, stiff_off=False,
                 path_to_config_file="mover.conf"):

        # print "Waiting for 2 seconds - Do not know why"
        # time.sleep(2)
        # virtualRobot = Motion.Motion("../../../json/nico_humanoid_upper_with_hands_vrep_mod.json",vrep=True)
        self.robot = robot
        self.stiff_off = stiff_off

    def __del__(self):

        if self.stiff_off == True:
            print "And the stiffness off"
            self.robot.disableTorqueAll()

    # virtualRobot.disableTorqueAll()

    # record a movement. For every move on the trajectory, one <return> has to be pressed
    def record_movement(self, fname=None):

        import csv
        import time

        key = "s"

        print "Give 'q' for stop recording"

        if (fname is None):
            fname = time.strftime("traj_%Y%m%d-%H%M%S") + ".csv"

        # filename = time.strftime("%Y%m%d-%H%M%S") + ".csv"
        # print fname

        raw_input()

        with open(fname, "wb") as jointfile:

            wr = csv.writer(jointfile, quoting=csv.QUOTE_ALL)
            wr.writerow(self.robot.getJointNames())

            while (key != "q"):
                recent_joint_positions = [self.robot.getAngle(jName) for jName
                                          in (self.robot.getJointNames())]

                wr.writerow(recent_joint_positions)

                key = raw_input()

        print "Movement written as " + fname

    # record a position. Move the robot to its position and press <return>
    def record_position(self, fname=None):

        import csv
        import time

        key = "s"

        print "Put the NICO in the position and press <return>"

        if (fname is None):
            fname = time.strftime("pos-%Y%m%d-%H%M%S") + ".csv"

        # filename = time.strftime("%Y%m%d-%H%M%S") + ".csv"

        raw_input()

        with open(fname, "wb") as jointfile:
            wr = csv.writer(jointfile, quoting=csv.QUOTE_ALL)
            wr.writerow(self.robot.getJointNames())

            recent_joint_positions = [self.robot.getAngle(jName) for jName in
                                      (self.robot.getJointNames())]

            wr.writerow(recent_joint_positions)

        print "Position written as " + fname

    # Start threaded recording
    # def record_movement_continuously_start(self, fname=None, delay=0.1):
    #    #thread.start_new_thread(self.record_movement_continuously(self, fname, delay), ())
    #    thread.start_new_thread(self.record_movement_continuously(), (self, fname, delay), ())

    # record a movement. Records a position every DELAY seconds.

    def record_movement_continuously_start(self, fname=None, delay=0.01):
        # Method for recording while movement
        # by Matthias Kerzel
        # This functionality should not stay here permanently

        import csv
        import time

        self.recording = True

        print "Stop recording with call to self.record_movement_continuously_stop()"

        if (fname is None):
            fname = time.strftime("traj_%Y%m%d-%H%M%S") + ".csv"

        # filename = time.strftime("%Y%m%d-%H%M%S") + ".csv"
        # print fname

        # raw_input()

        with open(fname, "wb") as jointfile:

            jointList = ["head_z", "head_y", "r_shoulder_y", "r_shoulder_z",
                         "r_arm_x", "r_elbow_y", "r_wrist_z", "r_wrist_x",
                         "r_thumb_x", "r_indexfingers_x"]
            # jointList = ["head_z","head_y","r_shoulder_y","r_shoulder_z","r_arm_x","r_elbow_y"]

            wr = csv.writer(jointfile, quoting=csv.QUOTE_ALL)
            wr.writerow(jointList + jointList)

            # wr_current = csv.writer(jointfile, quoting=csv.QUOTE_ALL)
            # wr_current.writerow(self.robot.getJointNames())

            counter = 0
            # while (self.recording == True):
            while self.recording:
                recent_joint_positions = [self.robot.getAngle(jName) for jName
                                          in (jointList)]
                # (self.robot.getJointNames())]

                # id=31
                # rc = [pypot.dynamixel.DxlIO('/dev/ttyACM0').get_present_thumb_current([id])[0]]

                recent_joint_currents = [self.robot.getCurrent(jName) for jName
                                         in (jointList)]
                recent_data = recent_joint_positions + recent_joint_currents

                wr.writerow(recent_data)
                counter = counter + 1
                time.sleep(0.01)

        print "Movement written as " + fname

    # stop recording the movement.

    def record_movement_continuously_stop(self, fname=None, delay=0.1):
        # Method for recording while movement
        # by Matthias Kerzel
        # This functionality should not stay here permanently
        print("Stopping recording.")
        self.recording = False

    # Move the robot straight to the goal position. synchronize the speed for the joint in a way, that they reach the position at the same time
    def move_position(self, target_positions, speed, real=True):

        # calculate current angular speed
        cur_speed = (63.0 / 60) * 360 * speed

        import copy

        # get the current positions of all joints to move
        current_positions = copy.deepcopy(target_positions)
        for joint in current_positions:
            current_positions[joint] = self.robot.getAngle(joint)
        time_to_reach = {k: abs((float(current_positions[k]) - float(
            target_positions[k])) / cur_speed) for k in current_positions}
        time_to_reach["r_wrist_z"] *= 0.75
        # print time_to_reach
        max_time = max(time_to_reach.values())
        max_keys = [k for k, v in time_to_reach.items() if v == max_time]
        print "Max time: " + str((max_keys, max_time))
        for joi in target_positions:
            if joi == "r_wrist_z":
                joi_speed = speed * 1.5
            else:
                joi_speed = speed
            if real and max_time != 0.0:
                self.robot.setAngle(joi, float(target_positions[joi]),
                                    (joi_speed * time_to_reach[
                                        joi]) / max_time)
        return max_time

    # Read the position from a file and move the joint from the current postion to the goal
    def move_file_position(self, fname, subsetfname=None, move_speed=0.04):

        import csv
        import time

        if (subsetfname is not None):
            with open(subsetfname) as f:
                subsf = csv.reader(f)
                subsetjoints = next(subsf)

        mt = 0
        with open(fname, 'rb') as infile:
            reader = csv.DictReader(infile)
            # If no subsetfile, send to all the joints
            if (subsetfname is None):
                ### ES Todo Correct this: reader[0] will not work, but the iterator for row in reader:
                for jName in reader[0]:
                    mt = self.move_position(joi, move_speed)
            else:
                # Else send only to the joints defined in the subset file

                for row in reader:
                    # Move all joints in the subset to the postion
                    joi = {k: row[k] for k in subsetjoints}
                    # print joi
                    mt = self.move_position(joi, move_speed)
        return (mt)

    # Read the position from a file and calculate a trajectory from the current position to i
    def calc_move_file(self, fname, target_fname, number):

        import csv
        import copy

        with open(fname, 'rb') as infile:
            reader = csv.DictReader(infile)
            for row in reader:
                target_positions = row

            current_positions = copy.deepcopy(target_positions)
            for joint in current_positions:
                current_positions[joint] = self.robot.getAngle(joint)
            stepsize = {k: (float(target_positions[k]) - float(
                current_positions[k])) / number for k in
                        current_positions}

            with open(target_fname, "wb") as jointfile:

                wr = csv.writer(jointfile, quoting=csv.QUOTE_ALL)
                wr.writerow(target_positions.keys())

                calc_pos = copy.deepcopy(current_positions)
                for n in range(number):
                    wr.writerow(calc_pos.values())
                    calc_pos = {k: (calc_pos[k] + stepsize[k]) for k in
                                calc_pos}

    # return (mt)

    # Read a trajectory from a file and move the joints over the trajectory
    def play_movement(self, fname, subsetfname=None, move_speed=0.04):

        import csv
        import time

        # filename = time.strftime("%Y%m%d-%H%M%S") + ".csv"
        # print fname

        # raw_input()

        if (subsetfname is not None):
            with open(subsetfname) as f:
                subsf = csv.reader(f)
                subsetjoints = next(subsf)

        mt = 0
        with open(fname, 'rb') as infile:
            reader = csv.DictReader(infile)
            # If no subsetfile, send to all the joints
            if (subsetfname is None):
                for row in reader:
                    for jName in row:
                        mt = self.move_position(jName, move_speed)
                    # print row
                    time.sleep(mt * 0.30)
                time.sleep(mt * 1)
            else:
                # Else send only to the joints defined in the subset file

                for row in reader:
                    # Move all joints in the subset to the postion
                    joi = {k: row[k] for k in subsetjoints}
                    # print joi
                    mt = self.move_position(joi, move_speed)
                    print("Current MT= " + str(mt))
                    # for jName in {k: row[k] for k in subsetjoints}:
                    # self.robot.setAngle(jName, float(row[jName]), 0.05)
                    # print row
                    time.sleep(mt * 0.5)
                time.sleep(mt * 1)

    # Read a trajectory from a file and move the joints over the trajectory

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
                    print "angle: " + joint + " " + str(pos)
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

    # examples
    # Move with move file from current position over trajectory
    # python Mover.py pm --file mov_take_something_with_left_arm.csv --subset subset_left_arm_and_head.csv --speed 0.1 --vrep
    # Move to position stored in move file
    # python Mover.py pp --file mov_take_something_with_left_arm.csv - -subset subset_left_arm_and_head.csv --speed 0.1
    # Record move file (trajectory)

    parser = argparse.ArgumentParser()
    parser.add_argument("command",
                        help="One of the commands m (record movement), p (record position), pm (play movement), pp (play position), cm (calculate movement) fj(freeze joints) uj(unfreeze joints)")
    # fj freeze joints as they are by torquing it. You subset to freeze only a subset of the joints.
    parser.add_argument('--json', nargs='?',
                        default='../../../../../json/nico_humanoid_upper.json',
                        help="robots json file. Default: nico_humanoid_upper.json")
    parser.add_argument('--filename', nargs='?', default=None,
                        help="file to record or to play")
    parser.add_argument('--targetfilename', nargs='?',
                        default="/tmp/mov-calc.csv", help="file to write")
    parser.add_argument('--subset', nargs='?', default=None,
                        help="joint subset file")
    parser.add_argument('--speed', nargs='?', default="0.05",
                        help="speed of movement")
    parser.add_argument('--vrep', action="store_true", default=False,
                        help="let it run on vrep than instead of real robot")
    parser.add_argument('--stiffoff', action="store_true", default=False,
                        help="sets the stiffness to off after movement")
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
        mov.play_movement(filename, subsetfilename,
                          move_speed=float(args.speed))
        raw_input()

    if command == "pp":
        mov.move_file_position(filename, subsetfilename,
                               move_speed=float(args.speed))
        raw_input()

    if command == "cm":
        mov.calc_move_file(filename, args.targetfilename, 10)
        mov.play_movement(args.targetfilename, subsetfilename, move_speed=0.5)

    if command == "fj":
        mov.freeze_joints(subsetfilename)

    if command == "uj":
        mov.freeze_joints(subsetfilename, unfreeze=True)
# !/usr/bin/env python


# To Dos

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


import time
from nicomotion import Motion
import sys
import pypot.dynamixel


class Mover:

	def __init__(self, robot, stiff_off=False,
                 path_to_config_file="mover.conf"):

        # print "Waiting for 2 seconds - Do not know why"
        # time.sleep(2)
        # virtualRobot = Motion.Motion("../../../json/nico_humanoid_upper_with_hands_vrep_mod.json",vrep=True)
        self.robot = robot
        self.stiff_off = stiff_off

	def __del__(self):

        if self.stiff_off == True:
        	print "And the stiffness off"
        	self.robot.disableTorqueAll()

	# virtualRobot.disableTorqueAll()

	# record a movement. For every move on the trajectory, one <return> has to be pressed
	def record_movement(self, fname=None):

        import csv
        import time

        key = "s"

        print "Give 'q' for stop recording"

        if (fname is None):
        	fname = time.strftime("traj_%Y%m%d-%H%M%S") + ".csv"

        # filename = time.strftime("%Y%m%d-%H%M%S") + ".csv"
        # print fname

        raw_input()

        with open(fname, "wb") as jointfile:

        	wr = csv.writer(jointfile, quoting=csv.QUOTE_ALL)
        	wr.writerow(self.robot.getJointNames())

        	while (key != "q"):
                recent_joint_positions = [self.robot.getAngle(jName) for jName
                                          in (self.robot.getJointNames())]

                wr.writerow(recent_joint_positions)

                key = raw_input()

        print "Movement written as " + fname

	# record a position. Move the robot to its position and press <return>
	def record_position(self, fname=None):

        import csv
        import time

        key = "s"

        print "Put the NICO in the position and press <return>"

        if (fname is None):
        	fname = time.strftime("pos-%Y%m%d-%H%M%S") + ".csv"

        # filename = time.strftime("%Y%m%d-%H%M%S") + ".csv"

        raw_input()

        with open(fname, "wb") as jointfile:
        	wr = csv.writer(jointfile, quoting=csv.QUOTE_ALL)
        	wr.writerow(self.robot.getJointNames())

        	recent_joint_positions = [self.robot.getAngle(jName) for jName in
                                	  (self.robot.getJointNames())]

        	wr.writerow(recent_joint_positions)

        print "Position written as " + fname

	# Start threaded recording
	# def record_movement_continuously_start(self, fname=None, delay=0.1):
	#    #thread.start_new_thread(self.record_movement_continuously(self, fname, delay), ())
	#    thread.start_new_thread(self.record_movement_continuously(), (self, fname, delay), ())

	# record a movement. Records a position every DELAY seconds.

	def record_movement_continuously_start(self, fname=None, delay=0.01):
        import csv
        import time

        self.recording = True

        print "Stop recording with call to self.record_movement_continuously_stop()"

        if (fname is None):
        	fname = time.strftime("traj_%Y%m%d-%H%M%S") + ".csv"

        # filename = time.strftime("%Y%m%d-%H%M%S") + ".csv"
        # print fname

        # raw_input()

        with open(fname, "wb") as jointfile:

        	jointList = ["head_z", "head_y", "r_shoulder_y", "r_shoulder_z",
                         "r_arm_x", "r_elbow_y", "r_wrist_z", "r_wrist_x",
                         "r_thumb_x", "r_indexfingers_x"]
        	# jointList = ["head_z","head_y","r_shoulder_y","r_shoulder_z","r_arm_x","r_elbow_y"]

        	wr = csv.writer(jointfile, quoting=csv.QUOTE_ALL)
        	wr.writerow(jointList + jointList)

        	# wr_current = csv.writer(jointfile, quoting=csv.QUOTE_ALL)
        	# wr_current.writerow(self.robot.getJointNames())

        	counter = 0
        	# while (self.recording == True):
        	while self.recording:
                recent_joint_positions = [self.robot.getAngle(jName) for jName
                                          in (jointList)]
                # (self.robot.getJointNames())]

                # id=31
                # rc = [pypot.dynamixel.DxlIO('/dev/ttyACM0').get_present_thumb_current([id])[0]]

                recent_joint_currents = [self.robot.getCurrent(jName) for jName
                                         in (jointList)]
                recent_data = recent_joint_positions + recent_joint_currents

                wr.writerow(recent_data)
                counter = counter + 1
                time.sleep(0.01)

        print "Movement written as " + fname

	# stop recording the movement.

	def record_movement_continuously_stop(self, fname=None, delay=0.1):
        print("Stopping recording.")
        self.recording = False

	# Move the robot straight to the goal position. synchronize the speed for the joint in a way, that they reach the position at the same time
	def move_position(self, target_positions, speed, real=True):

        # calculate current angular speed
        cur_speed = (63.0 / 60) * 360 * speed

        import copy

        # get the current positions of all joints to move
        current_positions = copy.deepcopy(target_positions)
        for joint in current_positions:
        	current_positions[joint] = self.robot.getAngle(joint)
        time_to_reach = {k: abs((float(current_positions[k]) - float(
        	target_positions[k])) / cur_speed) for k in current_positions}
        time_to_reach["r_wrist_z"] *= 0.75
        # print time_to_reach
        max_time = max(time_to_reach.values())
        max_keys = [k for k, v in time_to_reach.items() if v == max_time]
        print "Max time: " + str((max_keys, max_time))
        for joi in target_positions:
        	if joi == "r_wrist_z":
                joi_speed = speed * 1.5
        	else:
                joi_speed = speed
        	if real and max_time != 0.0:
                self.robot.setAngle(joi, float(target_positions[joi]),
                                	(joi_speed * time_to_reach[
                                        joi]) / max_time)
        return max_time

	# Read the position from a file and move the joint from the current postion to the goal
	def move_file_position(self, fname, subsetfname=None, move_speed=0.04):

        import csv
        import time

        if (subsetfname is not None):
        	with open(subsetfname) as f:
                subsf = csv.reader(f)
                subsetjoints = next(subsf)

        mt = 0
        with open(fname, 'rb') as infile:
        	reader = csv.DictReader(infile)
        	# If no subsetfile, send to all the joints
        	if (subsetfname is None):
                ### ES Todo Correct this: reader[0] will not work, but the iterator for row in reader:
                for jName in reader[0]:
                	mt = self.move_position(joi, move_speed)
        	else:
                # Else send only to the joints defined in the subset file

                for row in reader:
                	# Move all joints in the subset to the postion
                	joi = {k: row[k] for k in subsetjoints}
                	# print joi
                	mt = self.move_position(joi, move_speed)
        return (mt)

	# Read the position from a file and calculate a trajectory from the current position to i
	def calc_move_file(self, fname, target_fname, number):

        import csv
        import copy

        with open(fname, 'rb') as infile:
        	reader = csv.DictReader(infile)
        	for row in reader:
                target_positions = row

        	current_positions = copy.deepcopy(target_positions)
        	for joint in current_positions:
                current_positions[joint] = self.robot.getAngle(joint)
        	stepsize = {k: (float(target_positions[k]) - float(
                current_positions[k])) / number for k in
                        current_positions}

        	with open(target_fname, "wb") as jointfile:

                wr = csv.writer(jointfile, quoting=csv.QUOTE_ALL)
                wr.writerow(target_positions.keys())

                calc_pos = copy.deepcopy(current_positions)
                for n in range(number):
                	wr.writerow(calc_pos.values())
                	calc_pos = {k: (calc_pos[k] + stepsize[k]) for k in
                                calc_pos}

	# return (mt)

	# Read a trajectory from a file and move the joints over the trajectory
	def play_movement(self, fname, subsetfname=None, move_speed=0.04):

        import csv
        import time

        # filename = time.strftime("%Y%m%d-%H%M%S") + ".csv"
        # print fname

        # raw_input()

        if (subsetfname is not None):
        	with open(subsetfname) as f:
                subsf = csv.reader(f)
                subsetjoints = next(subsf)

        mt = 0
        with open(fname, 'rb') as infile:
        	reader = csv.DictReader(infile)
        	# If no subsetfile, send to all the joints
        	if (subsetfname is None):
                for row in reader:
                	for jName in row:
                        mt = self.move_position(jName, move_speed)
                	# print row
                	time.sleep(mt * 0.30)
                time.sleep(mt * 1)
        	else:
                # Else send only to the joints defined in the subset file

                for row in reader:
                	# Move all joints in the subset to the postion
                	joi = {k: row[k] for k in subsetjoints}
                	# print joi
                	mt = self.move_position(joi, move_speed)
                	print("Current MT= " + str(mt))
                	# for jName in {k: row[k] for k in subsetjoints}:
                	# self.robot.setAngle(jName, float(row[jName]), 0.05)
                	# print row
                	time.sleep(mt * 0.5)
                time.sleep(mt * 1)

	# Read a trajectory from a file and move the joints over the trajectory

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
                	print "angle: " + joint + " " + str(pos)
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

	# examples
	# Move with move file from current position over trajectory
	# python Mover.py pm --file mov_take_something_with_left_arm.csv --subset subset_left_arm_and_head.csv --speed 0.1 --vrep
	# Move to position stored in move file
	# python Mover.py pp --file mov_take_something_with_left_arm.csv - -subset subset_left_arm_and_head.csv --speed 0.1
	# Record move file (trajectory)

	parser = argparse.ArgumentParser()
	parser.add_argument("command",
                        help="One of the commands m (record movement), p (record position), pm (play movement), pp (play position), cm (calculate movement) fj(freeze joints) uj(unfreeze joints)")
	# fj freeze joints as they are by torquing it. You subset to freeze only a subset of the joints.
	parser.add_argument('--json', nargs='?',
                        default='../../../../../json/nico_humanoid_upper.json',
                        help="robots json file. Default: nico_humanoid_upper.json")
	parser.add_argument('--filename', nargs='?', default=None,
                        help="file to record or to play")
	parser.add_argument('--targetfilename', nargs='?',
                        default="/tmp/mov-calc.csv", help="file to write")
	parser.add_argument('--subset', nargs='?', default=None,
                        help="joint subset file")
	parser.add_argument('--speed', nargs='?', default="0.05",
                        help="speed of movement")
	parser.add_argument('--vrep', action="store_true", default=False,
                        help="let it run on vrep than instead of real robot")
	parser.add_argument('--stiffoff', action="store_true", default=False,
                        help="sets the stiffness to off after movement")
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
        mov.play_movement(filename, subsetfilename,
                          move_speed=float(args.speed))
        raw_input()

	if command == "pp":
        mov.move_file_position(filename, subsetfilename,
                        	   move_speed=float(args.speed))
        raw_input()

	if command == "cm":
        mov.calc_move_file(filename, args.targetfilename, 10)
        mov.play_movement(args.targetfilename, subsetfilename, move_speed=0.5)

	if command == "fj":
        mov.freeze_joints(subsetfilename)

	if command == "uj":
        mov.freeze_joints(subsetfilename, unfreeze=True)
# !/usr/bin/env python


# To Dos

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


import time
from nicomotion import Motion
import sys
import pypot.dynamixel


class Mover:

	def __init__(self, robot, stiff_off=False,
                 path_to_config_file="mover.conf"):

        # print "Waiting for 2 seconds - Do not know why"
        # time.sleep(2)
        # virtualRobot = Motion.Motion("../../../json/nico_humanoid_upper_with_hands_vrep_mod.json",vrep=True)
        self.robot = robot
        self.stiff_off = stiff_off

	def __del__(self):

        if self.stiff_off == True:
        	print "And the stiffness off"
        	self.robot.disableTorqueAll()

	# virtualRobot.disableTorqueAll()

	# record a movement. For every move on the trajectory, one <return> has to be pressed
	def record_movement(self, fname=None):

        import csv
        import time

        key = "s"

        print "Give 'q' for stop recording"

        if (fname is None):
        	fname = time.strftime("traj_%Y%m%d-%H%M%S") + ".csv"

        # filename = time.strftime("%Y%m%d-%H%M%S") + ".csv"
        # print fname

        raw_input()

        with open(fname, "wb") as jointfile:

        	wr = csv.writer(jointfile, quoting=csv.QUOTE_ALL)
        	wr.writerow(self.robot.getJointNames())

        	while (key != "q"):
                recent_joint_positions = [self.robot.getAngle(jName) for jName
                                          in (self.robot.getJointNames())]

                wr.writerow(recent_joint_positions)

                key = raw_input()

        print "Movement written as " + fname

	# record a position. Move the robot to its position and press <return>
	def record_position(self, fname=None):

        import csv
        import time

        key = "s"

        print "Put the NICO in the position and press <return>"

        if (fname is None):
        	fname = time.strftime("pos-%Y%m%d-%H%M%S") + ".csv"

        # filename = time.strftime("%Y%m%d-%H%M%S") + ".csv"

        raw_input()

        with open(fname, "wb") as jointfile:
        	wr = csv.writer(jointfile, quoting=csv.QUOTE_ALL)
        	wr.writerow(self.robot.getJointNames())

        	recent_joint_positions = [self.robot.getAngle(jName) for jName in
                                	  (self.robot.getJointNames())]

        	wr.writerow(recent_joint_positions)

        print "Position written as " + fname

	# Start threaded recording
	# def record_movement_continuously_start(self, fname=None, delay=0.1):
	#    #thread.start_new_thread(self.record_movement_continuously(self, fname, delay), ())
	#    thread.start_new_thread(self.record_movement_continuously(), (self, fname, delay), ())

	# record a movement. Records a position every DELAY seconds.

	def record_movement_continuously_start(self, fname=None, delay=0.01):
        import csv
        import time

        self.recording = True

        print "Stop recording with call to self.record_movement_continuously_stop()"

        if (fname is None):
        	fname = time.strftime("traj_%Y%m%d-%H%M%S") + ".csv"

        # filename = time.strftime("%Y%m%d-%H%M%S") + ".csv"
        # print fname

        # raw_input()

        with open(fname, "wb") as jointfile:

        	jointList = ["head_z", "head_y", "r_shoulder_y", "r_shoulder_z",
                         "r_arm_x", "r_elbow_y", "r_wrist_z", "r_wrist_x",
                         "r_thumb_x", "r_indexfingers_x"]
        	# jointList = ["head_z","head_y","r_shoulder_y","r_shoulder_z","r_arm_x","r_elbow_y"]

        	wr = csv.writer(jointfile, quoting=csv.QUOTE_ALL)
        	wr.writerow(jointList + jointList)

        	# wr_current = csv.writer(jointfile, quoting=csv.QUOTE_ALL)
        	# wr_current.writerow(self.robot.getJointNames())

        	counter = 0
        	# while (self.recording == True):
        	while self.recording:
                recent_joint_positions = [self.robot.getAngle(jName) for jName
                                          in (jointList)]
                # (self.robot.getJointNames())]

                # id=31
                # rc = [pypot.dynamixel.DxlIO('/dev/ttyACM0').get_present_thumb_current([id])[0]]

                recent_joint_currents = [self.robot.getCurrent(jName) for jName
                                         in (jointList)]
                recent_data = recent_joint_positions + recent_joint_currents

                wr.writerow(recent_data)
                counter = counter + 1
                time.sleep(0.01)

        print "Movement written as " + fname

	# stop recording the movement.

	def record_movement_continuously_stop(self, fname=None, delay=0.1):
        print("Stopping recording.")
        self.recording = False

	# Move the robot straight to the goal position. synchronize the speed for the joint in a way, that they reach the position at the same time
	def move_position(self, target_positions, speed, real=True):

        # calculate current angular speed
        cur_speed = (63.0 / 60) * 360 * speed

        import copy

        # get the current positions of all joints to move
        current_positions = copy.deepcopy(target_positions)
        for joint in current_positions:
        	current_positions[joint] = self.robot.getAngle(joint)
        time_to_reach = {k: abs((float(current_positions[k]) - float(
        	target_positions[k])) / cur_speed) for k in current_positions}
        time_to_reach["r_wrist_z"] *= 0.75
        # print time_to_reach
        max_time = max(time_to_reach.values())
        max_keys = [k for k, v in time_to_reach.items() if v == max_time]
        print "Max time: " + str((max_keys, max_time))
        for joi in target_positions:
        	if joi == "r_wrist_z":
                joi_speed = speed * 1.5
        	else:
                joi_speed = speed
        	if real and max_time != 0.0:
                self.robot.setAngle(joi, float(target_positions[joi]),
                                	(joi_speed * time_to_reach[
                                        joi]) / max_time)
        return max_time

	# Read the position from a file and move the joint from the current postion to the goal
	def move_file_position(self, fname, subsetfname=None, move_speed=0.04):

        import csv
        import time

        if (subsetfname is not None):
        	with open(subsetfname) as f:
                subsf = csv.reader(f)
                subsetjoints = next(subsf)

        mt = 0
        with open(fname, 'rb') as infile:
        	reader = csv.DictReader(infile)
        	# If no subsetfile, send to all the joints
        	if (subsetfname is None):
                ### ES Todo Correct this: reader[0] will not work, but the iterator for row in reader:
                for jName in reader[0]:
                	mt = self.move_position(joi, move_speed)
        	else:
                # Else send only to the joints defined in the subset file

                for row in reader:
                	# Move all joints in the subset to the postion
                	joi = {k: row[k] for k in subsetjoints}
                	# print joi
                	mt = self.move_position(joi, move_speed)
        return (mt)

	# Read the position from a file and calculate a trajectory from the current position to i
	def calc_move_file(self, fname, target_fname, number):

        import csv
        import copy

        with open(fname, 'rb') as infile:
        	reader = csv.DictReader(infile)
        	for row in reader:
                target_positions = row

        	current_positions = copy.deepcopy(target_positions)
        	for joint in current_positions:
                current_positions[joint] = self.robot.getAngle(joint)
        	stepsize = {k: (float(target_positions[k]) - float(
                current_positions[k])) / number for k in
                        current_positions}

        	with open(target_fname, "wb") as jointfile:

                wr = csv.writer(jointfile, quoting=csv.QUOTE_ALL)
                wr.writerow(target_positions.keys())

                calc_pos = copy.deepcopy(current_positions)
                for n in range(number):
                	wr.writerow(calc_pos.values())
                	calc_pos = {k: (calc_pos[k] + stepsize[k]) for k in
                                calc_pos}

	# return (mt)

	# Read a trajectory from a file and move the joints over the trajectory
	def play_movement(self, fname, subsetfname=None, move_speed=0.04):

        import csv
        import time

        # filename = time.strftime("%Y%m%d-%H%M%S") + ".csv"
        # print fname

        # raw_input()

        if (subsetfname is not None):
        	with open(subsetfname) as f:
                subsf = csv.reader(f)
                subsetjoints = next(subsf)

        mt = 0
        with open(fname, 'rb') as infile:
        	reader = csv.DictReader(infile)
        	# If no subsetfile, send to all the joints
        	if (subsetfname is None):
                for row in reader:
                	for jName in row:
                        mt = self.move_position(jName, move_speed)
                	# print row
                	time.sleep(mt * 0.30)
                time.sleep(mt * 1)
        	else:
                # Else send only to the joints defined in the subset file

                for row in reader:
                	# Move all joints in the subset to the postion
                	joi = {k: row[k] for k in subsetjoints}
                	# print joi
                	mt = self.move_position(joi, move_speed)
                	print("Current MT= " + str(mt))
                	# for jName in {k: row[k] for k in subsetjoints}:
                	# self.robot.setAngle(jName, float(row[jName]), 0.05)
                	# print row
                	time.sleep(mt * 0.5)
                time.sleep(mt * 1)

	# Read a trajectory from a file and move the joints over the trajectory

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
                	print "angle: " + joint + " " + str(pos)
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

	# examples
	# Move with move file from current position over trajectory
	# python Mover.py pm --file mov_take_something_with_left_arm.csv --subset subset_left_arm_and_head.csv --speed 0.1 --vrep
	# Move to position stored in move file
	# python Mover.py pp --file mov_take_something_with_left_arm.csv - -subset subset_left_arm_and_head.csv --speed 0.1
	# Record move file (trajectory)

	parser = argparse.ArgumentParser()
	parser.add_argument("command",
                        help="One of the commands m (record movement), p (record position), pm (play movement), pp (play position), cm (calculate movement) fj(freeze joints) uj(unfreeze joints)")
	# fj freeze joints as they are by torquing it. You subset to freeze only a subset of the joints.
	parser.add_argument('--json', nargs='?',
                        default='../../../../../json/nico_humanoid_upper.json',
                        help="robots json file. Default: nico_humanoid_upper.json")
	parser.add_argument('--filename', nargs='?', default=None,
                        help="file to record or to play")
	parser.add_argument('--targetfilename', nargs='?',
                        default="/tmp/mov-calc.csv", help="file to write")
	parser.add_argument('--subset', nargs='?', default=None,
                        help="joint subset file")
	parser.add_argument('--speed', nargs='?', default="0.05",
                        help="speed of movement")
	parser.add_argument('--vrep', action="store_true", default=False,
                        help="let it run on vrep than instead of real robot")
	parser.add_argument('--stiffoff', action="store_true", default=False,
                        help="sets the stiffness to off after movement")
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
        mov.play_movement(filename, subsetfilename,
                          move_speed=float(args.speed))
        raw_input()

	if command == "pp":
        mov.move_file_position(filename, subsetfilename,
                        	   move_speed=float(args.speed))
        raw_input()

	if command == "cm":
        mov.calc_move_file(filename, args.targetfilename, 10)
        mov.play_movement(args.targetfilename, subsetfilename, move_speed=0.5)

	if command == "fj":
        mov.freeze_joints(subsetfilename)

	if command == "uj":
        mov.freeze_joints(subsetfilename, unfreeze=True)

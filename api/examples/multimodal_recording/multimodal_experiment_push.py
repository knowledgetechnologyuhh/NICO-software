# MUltimodal Recording

# Erik Strahl
# Matthias Kerezl

# GNU GPL License

from nicomotion import Motion
from nicotouch.optoforcesensors import optoforce
import pypot.dynamixel
from time import sleep
import datetime

import sqlite3
import random

from subprocess import call

#experiment definitions (Objects and number of graspings)
#definition of objects
objects =["blue ball","blue_plush ball","red_plush ball","orange_plush ball","white cube"]

#big_yellow die
#medium_yellow die
#small_yellow die
#yellow sponge
#green sponge
#blue tissues
#pink tissues
#yellow apple
#light_green apple
#heavy_green apple
#realistic_green apple
#red car
#blue car
#yellow car
#green car
#light tomato
#heavy tomato
#small_red ball
#large_red ball
#small banana
#plush banana
#heavy banana
#obergine banana
#yellow duck
#purple duck
#orange fish
#yellow seal


#definition for numbers per object
number_of_samples_per_object=10

#definition of Maximum current - This protects the hands from breaking!! Do not change this, if you do not know!
MAX_CUR_FINGER=120
MAX_CUR_THUMB=100

# Data for SQlite Access
# Experiment Data will be stored in two table database
# with samples for every sample of a grasped onject and subsample for every motor step (like for 10 degrees)

connection = sqlite3.connect("./experiment.db")
cursor = connection.cursor()
# status: 0-succesful , 1- overload, 2 - former_overload


format_str_sample = """INSERT INTO sample (sample_number,object_name , action, timecode)
    VALUES (NULL, "{object_name}", "{action}","{timecode}");"""


# Data for robot movement in high level mode
# move joints with 10 per cent of the speed and hand joints with full speed
fMS = 0.01
fMS_hand=1.0


def get_sampled_numbers_for_object(object_name):

    sql="SELECT * FROM sample where object_name='" + object_name + " and action ='"+action+"';"
    cursor.execute(sql)
    result = cursor.fetchall()
    return len(result)

def get_needed_numbers_for_object(object_name):

    num = number_of_samples_per_object - get_sampled_numbers_for_object(object_name)
    return (num)

def get_needed_overall_numbers():
    sum=0
    for o in objects:
        sum+=get_needed_numbers_for_object(o)
    return(sum)

def print_progress():
    for o in objects:
        print " For " + o + " - samples needed: " + str(get_needed_numbers_for_object(o))
        " - samples finished: " + str(get_sampled_numbers_for_object(o))


    #Print out was is still needed
print "\n\nWe still need the following samples:"

print_progress()

if get_needed_overall_numbers()>0:
    print "\n\nOverall there are still " +str(get_needed_overall_numbers()) + " samples needed.\n\n"
else:
    print "\n\nAll samples recorded. Thank you. If you needed more, adapt the number_of_samples_per_object parameter on the programm.\n\n"
    exit(0)


# Instructions for the experimenter. Brig the robot in Initial position
print "\n Please put the robot in position. Right arm on the table. Left arm hanging down. Give RETURN after finished.\n"
raw_input()

#Put the left arm in defined position
robot = Motion.Motion("../../../json/nico_humanoid_legged_with_hands_mod.json",vrep=False)
mover_path = "../../../../moves_and_positions/"
mov = Mover.Mover(robot, stiff_off=False)


#set the robot to be compliant
robot.disableTorqueAll()

robot.openHand('RHand', fractionMaxSpeed=fMS_hand)

robot.enableForceControl("r_wrist_z", 50)
robot.enableForceControl("r_wrist_x", 50)

robot.enableForceControl("r_indexfingers_x", 50)
robot.enableForceControl("r_thumb_x", 50)

# enable torque of left arm joints
robot.enableForceControl("head_z", 20)
robot.enableForceControl("head_y", 20)

robot.enableForceControl("r_shoulder_z", 20)
robot.enableForceControl("r_shoulder_y", 20)
robot.enableForceControl("r_arm_x", 20)
robot.enableForceControl("r_elbow_y", 20)


# Instructions for the experimenter. Brig the robot in Initial position
print "\n OK. The robot is positioned. We will start the experiment now.\n\n"



while ( get_needed_overall_numbers() > 0 ):

        #step over 8 movement steps
        for n in range(8):

            mov.move_file_position(mover_path + "lift_arm_experiment_pos_"+n+".csv",
                                   subsetfname=mover_path + "subset_right_arm.csv",
                                   move_speed=0.05)

            sleep(1)




        connection.commit()
    connection.close()
    print "\n\n Great! I got all samples together! Finishing experiment.\n"
    print_progress()

    robot=none

#set the robot to be compliant














# Touch Sensor Experiment Number 1

# Erik Strahl

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
objects =["red_tomato","green_sausage","red_ball","yellow_banana","red_banana","yellow_dice","green_pepper","blue_ball","red_dice","puple_grapes","red_sponge","orange_carrot","black_hat","purple_duck","orange_fish","green_figure"]

#definition for numbers per object
#Samples taken:
#0-9 Schulpraktikant Lars
#10-19 Schulpraktikant Lars
#20-29 SHK Karen
#30-39 SHK Ju
#40-49 SHK Ju
#50-59 B.Sc. Emil 	
#60-69 B.Sc. Emil
#70-79 SHK Josip
#80-89 SHK Josip
#90-99 SHK Karen

number_of_samples_per_object=100

#definition of Maximum current - This protects the hands from breaking!! Do not change this, if you do not know!
MAX_CUR_FINGER=120
MAX_CUR_THUMB=100

# Data for SQlite Access
# Experiment Data will be stored in two table database
# with samples for every sample of a grasped onject and subsample for every motor step (like for 10 degrees)

connection = sqlite3.connect("./data/experiment.db")
cursor = connection.cursor()
# status: 0-succesful , 1- overload, 2 - former_overload
format_str_subsample = """INSERT INTO subsample (subsample_number, sample_number , subsample_iteration, thumb_position, finger_position, 
thumb_current,finger_current, timecode_local, status,status_touch, counter_touch,x_touch,y_touch,z_touch, 
timecode_touch )
    VALUES (NULL, "{sample_number}",  "{subsample_iteration}", "{thumb_position}", "{finger_position}", 
"{thumb_current}","{finger_current}", "{timecode_local}", "{status}","{status_touch}", "{counter_touch}","{x_touch}","{y_touch}","{z_touch}", 
"{timecode_touch}");"""

format_str_sample = """INSERT INTO sample (sample_number,object_name , timecode)
    VALUES (NULL, "{object_name}", "{timecode}");"""


# Data for robot movement in high level mode
# move joints with 10 per cent of the speed and hand joints with full speed
fMS = 0.01
fMS_hand=1.0

wait_time=0.15

#get the camera ready
#import pygame
#import pygame.camera

#pygame.camera.init()

#cam = pygame.camera.Camera("/dev/video0",(320,200))
#cam.start()

def get_sampled_numbers_for_object(object_name):

    sql="SELECT * FROM sample where object_name='" + object_name + "';"
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
        
def get_position_safe(dxo,joint):
	
	succes=False
	while not succes:		
		try:
			pos=dxo.get_present_position(joint)[0]
			return pos
		except:
			sleep(0.2)
	
			
def get_present_thumb_current_safe(dxo,joint):
	
	succes=False
	while not succes:		
		try:
			pos=dxo.get_present_thumb_current(joint)[0]
			return pos
		except:
			sleep(0.2)	

def get_present_finger_current_safe(dxo,joint):
	
	succes=False
	while not succes:		
		try:
			pos=dxo.get_present_finger_current(joint)[0]
			return pos
		except:
			sleep(0.2)	
	

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
robot = Motion.Motion("../../../../json/nico_humanoid_legged_with_hands_mod.json",vrep=False)

#set the robot to be compliant
robot.disableTorqueAll()

robot.openHand('RHand', fractionMaxSpeed=fMS_hand)

robot.enableForceControl("r_wrist_z", 50)
robot.enableForceControl("r_wrist_x", 50)

robot.enableForceControl("r_indexfingers_x", 50)
robot.enableForceControl("r_thumb_x", 50)

robot.setAngle("r_wrist_z", -180, fMS_hand)
robot.setAngle("r_wrist_x", 0, fMS_hand)

robot.disableTorqueAll()
robot=None
sleep(1)

# Instructions for the experimenter. Brig the robot in Initial position
print "\n OK. The robot is positioned. We will start the experiment now.\n\n"


with pypot.dynamixel.DxlIO('/dev/ttyACM0') as dxl_io:
    dxl_io.enable_torque([27, 29])

    # Data for touch sensor
    # Adapt this for the srial Interface you need for the sensor
    optsens = optoforce("/dev/ttyACM1", "DSE0A125")

    while ( get_needed_overall_numbers() > 0 ):

        #open hand
        dxl_io.set_goal_position({27: -180.00})
        dxl_io.set_goal_position({29: -180.00})
        #sleep(0.1)

        #Select an object
        o=random.choice(objects)
        while (get_needed_numbers_for_object(o)<1):
            o = random.choice(objects)

        print "Randomly chosen object : " + o + "\n"

        print "\n\n Please put the " + o + " on the robot fingers. Then press RETURN."
        raw_input()

        #!!!!Write Sample data in database
        sql_command = format_str_sample.format(object_name=o, timecode= datetime.datetime.now().isoformat())
        #print "\n " +sql_command
        cursor.execute(sql_command)
        sample_number=cursor.lastrowid

        #Status no overload
        status=0
        for it,pos in enumerate(range (-180,80,5)):
            if status ==1:
                status=2
            #dxl_io.set_goal_position({28: -12.00})

            #Move joints only if there is no overload
            if status == 0:
                dxl_io.set_goal_position({27: pos})
                dxl_io.set_goal_position({29: pos})
                sleep(0.5)
                while (dxl_io.get_present_speed([27])<0.01 and dxl_io.get_present_speed([29])<0.01):
                    sleep(0.1)
            # !!!! Take a photo here - name = ID
            thumb_pos=get_position_safe(dxl_io,[27])
            #thumb_pos=dxl_io.get_present_position([27])[0]
            
            finger_pos=get_position_safe(dxl_io,[29])
            #(x,y,z) = optsens.get_sensor_values_raw()
            #print "Motor Temperature " + str(dxl_io.get_present_temperature([27]))
            #for id in range (30,33):
            #    try:
            id=31
            
            
            cur_thumb=get_present_thumb_current_safe(dxl_io,[id])
            print "Motor Thumb Current " + str(cur_thumb) + " id " + str(id)
            cur_finger = get_present_finger_current_safe(dxl_io,[id])
            print "Motor Finger Current " + str(cur_finger) + " id " + str(id)
            #    except:
            #        pass
            #    sleep(0.1)
            #print "Motor Speed " + str(dxl_io.get_present_speed([27]))
            #print "Motor Goal Position " + str(dxl_io.get_goal_position([27]))
            print "Thumb Present Position " + str(thumb_pos)
            print "Finger Present Position " + str(finger_pos)
            #print "Motor Max Torque" + str(dxl_io.get_max_torque([27]))


            #print "Sensor raw" + str((x,y,z))
            (x, y, z) = optsens.get_sensor_values_hex()
            print "Sensor " + str((x, y, z))
            (time_touch, counter, sensor_status, x, y, z, checksum) = optsens.get_sensor_all()
            print str(time_touch) + "," + str(counter) + "," + str(sensor_status) + "," + str(x) + "," + str(y) + "," + str(z) + "," + str(checksum)

            if ((cur_thumb>MAX_CUR_THUMB) or (cur_finger>MAX_CUR_FINGER)) and status==0:
                status=1
                print "!!Reached Overload!!"

            #Write the subsample data to the database
            sql_command = format_str_subsample.format(sample_number=sample_number, subsample_iteration = it, thumb_position=thumb_pos,finger_position=finger_pos, thumb_current=cur_thumb, finger_current=cur_finger,
                                                      timecode_local=datetime.datetime.now().isoformat(),status=status,status_touch=sensor_status,counter_touch=counter,x_touch=x,y_touch=y,z_touch=z,timecode_touch=time_touch)
            print sql_command
            cursor.execute(sql_command)
            subsample_number = cursor.lastrowid

            #take 5 pictures (error in the driver)
            #for t in range(5):
            #    img = cam.get_image()

            #pygame.image.save(img, "./data/" + str(subsample_number) + ".jpg")
            call(["fswebcam", "-r", "640x480", "-d", "/dev/video0", "--jpeg", "95", "-D", "0.1",
                  "./data/" + str(subsample_number) + ".jpg"])
            sleep(.1)

            # !!!! Take a photo here - name = ID of the subsample

            #raw_input()
        connection.commit()
    connection.close()
    print "\n\n Great! I got all samples together! Finishing experiment.\n"
    print_progress()

    dxl_io.disable_torque([27, 29])

#set the robot to be compliant














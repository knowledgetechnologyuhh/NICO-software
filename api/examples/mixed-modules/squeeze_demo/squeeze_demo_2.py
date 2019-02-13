# -*- coding: utf-8 -*-

# Squeeze Demo
#
# Lets the NICO squeeze different objects and determine by this what object this is 
# 
# records the squeeze data and analyse it with a model by Matthias Kerzel
# 
# uses the data of second recording

# Erik Strahl, NICO data recording by Connor Gaede

# GNU GPL 3 License

import atexit

from create_db import create_db

from nicomotion import Motion,Mover
from nicotouch.OptoforceMultichannel import OptoforceMultichannel
from nicoface.FaceExpression import faceExpression
import pypot.dynamixel
from time import sleep
import datetime
import subprocess

from pypot.dynamixel.io.abstract_io import DxlCommunicationError

import sqlite3
import random

import os

from subprocess import call

from db_definitions import db_definitions

db_def=db_definitions()

# experiment definitions (Objects and number of graspings)
# definition of objects
objects = ["red_tomato", "green_cucumber", "red_ball", "yellow_banana",
           "red_banana", "yellow_dice", "green_pepper", "blue_ball",
           "red_dice", "puple_grapes", "red_sponge", "orange_carrot",
           "black_hat", "purple_duck", "orange_fish", "green_figure"]

# definition of Maximum current - This protects the hands from breaking!!
# Do not change this, if you do not know!
MAX_CUR_FINGER = 275
MAX_CUR_THUMB = 275

# Path to directory of NICO-software repository
nico_path = dirname(abspath(__file__)) + "/../../NICO-software"
if not isdir(nico_path):
    logger.error(
        ("NICO directory '{}' not found - please make sure you downloaded " +
         "the NICO-software repository and adjust the 'nico_path' variable " +
         "if necessary").format(nico_path))
    exit()

### flaskcom definitions
def wrapped_function():
	print ""
	#####################!!!!!!!!!!!!!
	from all_models_one_sample_classifier import all_models_classifier
	cnn_1_classifier=all_models_classifier()
	return cnn_1_classifier
	
	#from flaskcom.complex_test_class import ComplexTestClass
		
	#test_object = ComplexTestClass('hallo') #initialize the object you want to use
	
	#print "end wrapped function"
	#return test_object #return it
	#return None

import sys
sys.path.append("/home/sysadmin/NICO")

#import the RemoteObject
from flaskcom.remote_object import RemoteObject

#wrap it around the function
#returns an object that can be used like the object initialized in the wrapped function,
#here: test_object = ComplexTestClass('hallo')
classifier = RemoteObject(wrapped_function = wrapped_function, #the function that initializes the remote object
							path_to_virtualenv = "../../../../../../NICO/NICO-haptic-object-classification/virtenv", #a virtualenv can loaded before exectuting the code in the remote terminal.
							server = "localhost", #the remote object is running on another computer
							original_working_directory = "../../../../../../NICO/NICO-haptic-object-classification/", #a working directory can be specified, which can be used to search for the code
							keep_open = False, #the remote object can be kept open, when the program is exectuted the next time, it will use the open remote object instead of creating a new one
							time_out = -1, #the time to wait for the remote terminal to start, -1 means forever
							flaskcom_path = "/home/sysadmin/NICO", #if flaskcom is not inside the searchpath, set a path to a folder containing flaskcom
							debug = True) #keeps the terminal open even if an error occurs


##### flaskcom definitions

db_filename="../../../../../../NICO/NICO-haptic-object-classification/sample_1_from_experiment_2.db"


most_likely,all_results=classifier.classify(db_filename)
print ("This is a most likely a {} ".format(classifier.number_to_object(result)))
raw_input()

# Data for robot movement in high level mode
# move joints with 10 per cent of the speed and hand joints with full speed
fMS = 0.01
fMS_hand=1.0

wait_time=0.15

def cleanup(robot):
    robot.disableTorqueAll()
    robot.cleanup()
    robot = None


#Say text
#Use google tts and cache the spoken mp3s
#As fallback use pyttsx3
def say(sen):
	import os.path
	fname="./wav_cache/"+sen+".mp3"
	from gtts import gTTS
		
	try:
			
		if not (os.path.isfile(fname)): 
			import urllib2 
			urllib2.urlopen('http://216.58.192.142', timeout=1)
			tts = gTTS(text=sen, lang='en-au', slow=False)
			#tts.save("/tmp/say.mp3")
			tts.save(fname)
		comm = ["mpg123" , fname]
		subprocess.check_call(comm)
		
	except:
		#Fallback offline tts engine
		import pyttsx3
		engine = pyttsx3.init()
		engine.say(sen)
		engine.runAndWait() 
        

#face interface	
fe = faceExpression()
fe.sendFaceExpression("happiness")

# Instructions for the experimenter. Brig the robot in Initial position
print("\n Please put the robot in position. Left arm on the table. Right " +
      "arm hanging down. Give RETURN after finished.\n")
raw_input()


# Put the left arm in defined position
robot = Motion.Motion(
    nico_path + "/json/nico_humanoid_upper.json", vrep=False, ignoreMissing=True)
atexit.register(cleanup, robot)

# set the robot to be compliant
robot.disableTorqueAll()

robot.openHand('LHand', fractionMaxSpeed=fMS_hand)

robot.enableForceControl("l_wrist_z", 50)
robot.enableForceControl("l_wrist_x", 50)

robot.enableForceControl("l_indexfingers_x", 50)
robot.enableForceControl("l_thumb_x", 50)

robot.setAngle("head_z", 5, 0.03)
robot.setAngle("head_y", 60., 0.03)

robot.setAngle("l_wrist_z", -180., fMS_hand)
robot.setAngle("l_wrist_x", 0, fMS_hand)

robot.setAngle("l_shoulder_y", -10., fMS)
robot.setAngle("l_elbow_y", 80., fMS)
sleep(2)
robot.setAngle("l_shoulder_z", -25., fMS)
robot.setAngle("l_arm_x", 5., fMS)
sleep(3)

if robot.getAngle("l_wrist_z") != -180.:
    print("Could not move motor 'l_wrist_z' to its initial position")
    raw_input()
    exit()

# Adapt this for the srial Interface you need for the sensor
optsens = OptoforceMultichannel("ONR0A003")

# Instructions for the experimenter. Bring the robot in Initial position
print "\n OK. The robot is positioned. We will start the demo now.\n\n"
inp=raw_input()

if inp == "n":
	intro=False
else:
	intro=True

if (intro==True):
	
	
	say ("Hello. I am the NICO robot. I am getting prepared!")
	
	#sleep(2)
	
	#raw_input();
	
	say ("I can determine which object is in my hand, just by squeezing it a little bit.")
	
	sleep(1)

stop_demo=False
repeat=False
while ( not stop_demo  ):                                

    if not repeat:
        if not robot:
        robot = Motion.Motion((nico_path + "/json/nico_humanoid_upper.json"),
                              vrep=False, ignoreMissing=True)

        ## Open the Hand
        robot.openHand("LHand", fMS_hand)
        sleep(0.1)

        ## Put one of the objects in the hand
        say ("Please put one of the objects in my hand.")						
        sleep(2.2)
        raw_input()
        
        ## Squeeze
        print "\n Good. Let me squeeze this a little bit."
        #fe = faceExpression()
        fe.sendFaceExpression("neutral")
                
        # Delete possible existing and create a new database and open it
        try:
            os.remove(db_filename)
        except OSError:
            pass
        create_db(db_filename)

        connection = sqlite3.connect(db_filename)
        cursor = connection.cursor()
        ## Write Sample data in database
        sql_command = db_definitions.format_str_sample.format(
        object_name=o, timecode=datetime.datetime.now().isoformat())
        # print "\n " +sql_command
        cursor.execute(sql_command)
        sample_number = cursor.lastrowid

        #Status no overload
        status=0
        for it,pos in enumerate(range (-180,80,5)):
            if status ==1:
                status=2

        # Move joints only if there is no overload
        if status == 0:
            robot.setAngle("l_thumb_x", pos, fMS_hand)
            robot.setAngle("l_indexfingers_x", pos, fMS_hand)
            sleep(0.5)
            # FIXME Does this even work as intended?
            while (robot.getSpeed("l_thumb_x") > 0.01 and
                   robot.getSpeed("l_indexfingers_x") > 0.01):
                        print("Test?")
                        sleep(0.1)
        
        ## Get the postions
        thumb_pos = robot.getAngle("l_thumb_x")
        finger_pos = robot.getAngle("l_indexfingers_x")

        print "Motor Goal Position " + \
            str(robot._robot.l_thumb_x.goal_position)  # TODO remove ?
        print "Thumb Present Position " + str(thumb_pos)
        print "Finger Present Position " + str(finger_pos)

        cur_thumb = robot.getCurrent("l_thumb_x")
        print "Motor Thumb Current " + str(cur_thumb) + " id " + str(id)
        cur_finger = robot.getCurrent("l_indexfingers_x")
        print "Motor Finger Current " + str(cur_finger) + " id " + str(id)

        # print "Sensor raw" + str((x,y,z))
        opt_raw = optsens.get_sensor_values_raw()
        opt_thumb = opt_raw["forces"]["thumb"]
        print "Sensor thumb: " + str(opt_thumb)
        opt_index = opt_raw["forces"]["index"]
        print "Sensor index: " + str(opt_index)
        opt_ring = opt_raw["forces"]["ring"]
        print "Sensor ring: " + str(opt_ring)
        (time_touch, counter, sensor_status) = [
            opt_raw[k] for k in ("time", "count", "status")]
        print "Time: {} Count: {} Status: {}".format(
            time_touch, counter, sensor_status)

        if(((cur_thumb > MAX_CUR_THUMB) or (cur_finger > MAX_CUR_FINGER)) and
           status == 0):
            status = 1
            print "!!Reached Overload!!"

        # Write the subsample data to the database
        sql_command = db_definitions.format_str_subsample.format(
            sample_number=sample_number, subsample_iteration=it,
            thumb_position=thumb_pos, finger_position=finger_pos,
            thumb_current=cur_thumb, finger_current=cur_finger,
            timecode_local=datetime.datetime.now().isoformat(),
            status=status, status_touch=sensor_status,
            counter_touch=counter, thumb_touch_x=opt_thumb[0],
            thumb_touch_y=opt_thumb[1], thumb_touch_z=opt_thumb[2],
            index_touch_x=opt_index[0], index_touch_y=opt_index[1],
            index_touch_z=opt_index[2], ring_touch_x=opt_ring[0],
            ring_touch_y=opt_ring[1], ring_touch_z=opt_ring[2],
            timecode_touch=time_touch,
            status_touch_newton=sensor_status_newton,
            counter_touch_newton=counter_newton,
            thumb_touch_x_newton=opt_thumb_newton[0],
            thumb_touch_y_newton=opt_thumb_newton[1],
            thumb_touch_z_newton=opt_thumb_newton[2],
            index_touch_x_newton=opt_index_newton[0],
            index_touch_y_newton=opt_index_newton[1],
            index_touch_z_newton=opt_index_newton[2],
            ring_touch_x_newton=opt_ring_newton[0],
            ring_touch_y_newton=opt_ring_newton[1],
            ring_touch_z_newton=opt_ring_newton[2],
            timecode_touch_newton=time_touch_newton)
        print sql_command
        cursor.execute(sql_command)
        subsample_number = cursor.lastrowid

        #Commit and close the database
        connection.commit()
        connection.close()

    	print ("Asking the classiefier")
		fe.sendFaceExpression("neutral")
		say ("Let me think about it")

		if first:
			say ("I will ask two different classifiers")
		
		result=classifier.classify(db_filename)
		print ("This is a " + classifier.numberToString(result))

		print ("Asking the stop classifier")
		#import calculate_average_stop_angle  
		#calculate_average_stop_angle.calculate_and_write()
		import subprocess
		#subprocess.check_output(['ls','-l']) #all that is technically needed...
		#print subprocess.check_output(['python','./calculate_one_stop_angle.py '])
		print subprocess.check_output(['bash','./calculate.sh'])
		time.sleep(1)

		to_say={"green_sausage":"cucumber","red_tomato":"tomato","yellow_banana":"banana","red_ball":"ball"}
		#say ("I think this is a " + to_say[classifier.numberToString(result)])
		if first:
			say (" I have already one result, but let me ask my other classifier as well")
		time.sleep(2)
		result2=stop_classifier.classify("/tmp/one_stop_value.db")
		print ("This is a " + classifier.numberToString(result2))
		time.sleep(1)
		#say ("I think this is a " + to_say[classifier.numberToString(result)])
		#force_calibrator+=0.05

		#TODO: 
		# Put the sample in the model
		first=False	
		# Present the result
		if result!=result2:
			say ("Oh I am suprised.")
			fe.sendFaceExpression("surprise")
			say ("One of my classifiers means this is a " + to_say[classifier.numberToString(result2)])
			time.sleep(0.8)
			say ("But the other one thinks it is a " + to_say[classifier.numberToString(result)])
			time.sleep(0.8)
			say ("But the first one is really sure about it. So I say it is a " + to_say[classifier.numberToString(result2)])
			time.sleep(0.5)
			say ("Is this right?")

		else:
			say ("I am happy.")
			fe.sendFaceExpression("happiness")
			say ("Everything in my robot brain means the same. So I could bet my head that this is a " + to_say[classifier.numberToString(result)])
		
	repeat = False	
		

	if not repeat:
		#open hand
		#dxl_io.set_goal_position({27: -180.00})
		#dxl_io.set_goal_position({29: -180.00})
		#sleep(0.5)
		#print "\n Give return to continue."
		#raw_input()
		
		say ("OK. Let me try another one!")
		raw_input("Please give a Return!")
		#mov.move_file_position(mover_path + "pos_left_arm_base.csv",
		#                       subsetfname=mover_path + "subset_left_arm_and_head.csv",
		#                       move_speed=0.05)


















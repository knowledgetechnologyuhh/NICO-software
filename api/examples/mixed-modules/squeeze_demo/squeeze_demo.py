# -*- coding: utf-8 -*-

# Squeeze Demo
#
# Lets the NICO squeeze different objects and determine by this what object this is 
# 
# records the squeeze data and analyse it with a model by Matthias Kerzel

# Erik Strahl

# GNU GPL 3 License

from create_db import create_db

from nicomotion import Motion,Mover
from nicotouch.optoforcesensors import optoforce
from nicoface.FaceExpression import faceExpression
import pypot.dynamixel
from time import sleep
import datetime
import subprocess

import sqlite3
import random

import os

from subprocess import call

from client import ClientWrapper

from stub_touch_data_classifier import touch_data_classifier

from stub_touch_data_classifier_stop import touch_data_classifier as stop_touch_data_classifier



#classifier = ClientWrapper(touch_data_classifier, 54010)
#stop_classifier = ClientWrapper(stop_touch_data_classifier, 54011)

def wrapped_function():
	print ""
	#####################!!!!!!!!!!!!!
	from touch_data_classifier import touch_data_classifier
	tdc=touch_data_classifier()
	return tdc
	
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
							path_to_virtualenv = "../../../../../../NICO/NICO-haptic-object-classification/virtualenv", #a virtualenv can loaded before exectuting the code in the remote terminal.
							server = "localhost", #the remote object is running on another computer
							original_working_directory = "../../../../../../NICO/NICO-haptic-object-classification/", #a working directory can be specified, which can be used to search for the code
							keep_open = False, #the remote object can be kept open, when the program is exectuted the next time, it will use the open remote object instead of creating a new one
							time_out = -1, #the time to wait for the remote terminal to start, -1 means forever
							flaskcom_path = "/home/sysadmin/NICO", #if flaskcom is not inside the searchpath, set a path to a folder containing flaskcom
							debug = True) #keeps the terminal open even if an error occurs



result=classifier.classify(db_filename)
print ("This is a " + classifier.numberToString(result))

db_filename="/tmp/one_sample.db"

#Database Commands
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



#Say text
#Use google tts and cache the spoken mp3s
#As fallback use pyttsx3
def say(sen):
	import os.path
	fname="./wav_cache/"+sen+".mp3"
	from gtts import gTTS
	import time
	if (True):
	#try:
			
		if not (os.path.isfile(fname)): 
			import urllib2 
			urllib2.urlopen('http://216.58.192.142', timeout=1)
			tts = gTTS(text=sen, lang='en-au', slow=False)
			#tts.save("/tmp/say.mp3")
			tts.save(fname)
		comm = ["mpg123" , fname]
		subprocess.check_call(comm)
		time.sleep(2)
	else:	
	#except Exception as e: 
		print "Error in speech: " + str(e)
		raw_input()
		#Fallback offline tts engine
		import pyttsx3
		engine = pyttsx3.init()
		engine.say(sen)
		engine.runAndWait() 
#say('Test')

def get_speed_safe(dxo,joint):
	
	succes=False
	while not succes:		
		try:
			speed=dxo.get_present_speed(joint)[0]
			print speed
			return speed
		except:
			print "speed failure"
			sleep(0.01)

last_positions={}        
def get_position_safe(dxo,joint):
	err_count=0
	succes=False
	while not succes:		
		try:
			pos=dxo.get_present_position(joint)[0]
			last_positions[joint[0]]=pos
			return pos
		except Exception as e: 
			#print "position failure " + str(e)
			err_count+=1
			try:
				if err_count>4:
					print "returned last_position"
					return last_positions[joint]
			except:
				pass
			sleep(0.1)
	
last_thumb_currents={} 			
def get_present_thumb_current_safe(dxo,joint):
	err_count=0
	succes=False
	while not succes:		
		try:
			pos=dxo.get_present_thumb_current(joint)[0]
			last_thumb_currents[joint[0]]=pos
			return pos
		except:
			err_count+=1
			try:
				if err_count>4:
					print "returned last_thumb_current"
					return last_thumb_currents[joint]
				#print "thumb  failure"
			except:
				pass
			sleep(0.1)	

last_finger_currents={} 
def get_present_finger_current_safe(dxo,joint):
	err_count=0
	succes=False
	while not succes:		
		try:
			pos=dxo.get_present_finger_current(joint)[0]
			last_finger_currents[joint[0]]=pos
			return pos
		except:
			err_count+=1
			try:
				if err_count>4:
					print "returned last_finger_current"
					return last_finger_currents[joint]
				#print "thumb  failure"
			except:
				pass
			print "finger current failure"
			sleep(0.2)	
	

#face interface	
fe = faceExpression()
fe.sendFaceExpression("happiness")

# Touch Sensor interface
# Adapt this for the serial number you need for the sensor
optsens = optoforce(ser_number="DSE0A125")

#definition of Maximum current - This protects the hands from breaking!! Do not change this, if you do not know!

MAX_CUR_FINGER=120
MAX_CUR_THUMB=100

finger_force_calibrator=1.0
thumb_force_calibrator=1.0

# Data for robot movement in high level mode
# move joints with 10 per cent of the speed and hand joints with full speed
fMS = 0.01
fMS_hand=0.9

# Instructions for the experimenter. Brig the robot in Initial position
print "\n Please put the robot in position. Left arm hanging down. Right arm on the table. Give RETURN after finished.\n"
raw_input()

#Put the left arm in defined position
#robot = Motion.Motion("../../../../json/nico_humanoid_upper.json",vrep=False)

#mover_path = "../../../../moves_and_positions/"
#mov = Mover.Mover(robot, stiff_off=False)


#mov.move_file_position(mover_path + "pos_left_arm_base.csv",
#                                   subsetfname=mover_path + "subset_left_arm_and_head.csv",
#                                   move_speed=0.05)        


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

	# Data for touch sensor
	# Adapt this for the serial Interface you need for the sensor

	#mov.move_file_position(mover_path + "pos_left_arm_base.csv",
	#                                   subsetfname=mover_path + "subset_left_arm_and_head.csv",
	#                                   move_speed=0.05) 

stop_demo=False
repeat=False
first=True

while ( not stop_demo  ):                                

	if not repeat:
		
		
		# Delete possible existing and create a new database and open it
		try:
			os.remove(db_filename)
		except OSError:
			pass
		create_db(db_filename)

		connection = sqlite3.connect(db_filename)
		cursor = connection.cursor()

		# Record a sample in the database
		with pypot.dynamixel.DxlIO('/dev/ttyACM1') as dxl_io:
			no_success=True
			while no_success:
				try:
						
					dxl_io.enable_torque([27, 29])

					#open hand
					dxl_io.set_goal_position({27: -180.00})
					dxl_io.set_goal_position({29: -180.00})
					sleep(0.5)
					no_success=False
				except Exception, e:
					print "My robot does not understand me: \n" + str(e)

			#robot.openHand('RHand', fractionMaxSpeed=fMS_hand)
			#print "\n\n Please put the " + o + " on the robot fingers. Then press RETURN."
			say ("Please put one of the objects in my hand.")						
		
			sleep(2.2)
		
			raw_input()
			say ("Good. Let me squeeze this a little bit.")
			#fe = faceExpression()
			fe.sendFaceExpression("neutral")
		
			#Change to the other robot mode 
			#robot.disableTorqueAll()
			#robot=None
			sleep(1)

			#!!!!Write Sample data in database
			sql_command = format_str_sample.format(object_name="Unknown", timecode= datetime.datetime.now().isoformat())
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
					import time
					dxl_io.set_goal_position({27: pos})
					dxl_io.set_goal_position({29: pos})
					sleep(0.5)
					start_millis = int(round(time.time() * 1000))
					try:
						current_millis = int(round(time.time() * 1000))
						diff_millis=start_millis-current_millis
						while (diff_millis<200 and dxl_io.get_present_speed([27])<0.01 and dxl_io.get_present_speed([29])<0.01):
							sleep(0.1)
					except:
						print ("Motor not answering")
					#while (get_speed_safe(dxl_io,[27])<0.01 and get_speed_safe(dxl_io,[29])<0.01):
					#	sleep(0.1)

				thumb_pos=get_position_safe(dxl_io,[27])


				finger_pos=get_position_safe(dxl_io,[29])

				id=31

				# get the current values
				#cur_thumb=get_present_thumb_current_safe(dxl_io,[id])/force_calibrator
				cur_thumb=get_present_thumb_current_safe(dxl_io,[id])/thumb_force_calibrator

				#print "Motor Thumb Current " + str(cur_thumb) + " id " + str(id)
				cur_finger = get_present_finger_current_safe(dxl_io,[id])/finger_force_calibrator
				#print "Motor Finger Current " + str(cur_finger) + " id " + str(id)

				#print "Thumb Present Position " + str(thumb_pos)
				#print "Finger Present Position " + str(finger_pos)
				#print "Motor Max Torque" + str(dxl_io.get_max_torque([27]))


				#print "Sensor raw" + str((x,y,z))
				(x, y, z) = optsens.get_sensor_values_hex()
				#print "Sensor " + str((x, y, z))
				(time_touch, counter, sensor_status, x, y, z, checksum) = optsens.get_sensor_all()
				#print str(time_touch) + "," + str(counter) + "," + str(sensor_status) + "," + str(x) + "," + str(y) + "," + str(z) + "," + str(checksum)

				if ((cur_thumb>MAX_CUR_THUMB) or (cur_finger>MAX_CUR_FINGER)) and status==0:
					status=1
					print "\n\n\n!!Reached Overload!!"
					print "thumb_pos: " + str(thumb_pos)
					print "force_calibrators (thumb/fingers): " +str(thumb_force_calibrator) + " / " + str(finger_force_calibrator)
					print "MAX_CUR_THUMB / cur_thumb : " + str(MAX_CUR_THUMB) + "/" + str(cur_thumb)
					print "MAX_CUR_FINGER / cur_finger : " + str(MAX_CUR_FINGER) + "/" + str(cur_finger)
					say (" Puh, now it is really hard to squeeze for me! But I have to go on!")
					fe.sendFaceExpression("sadness")
				#Write the subsample data to the database
				sql_command = format_str_subsample.format(sample_number=sample_number, subsample_iteration = it, thumb_position=thumb_pos,finger_position=finger_pos, thumb_current=cur_thumb, finger_current=cur_finger,
														timecode_local=datetime.datetime.now().isoformat(),status=status,status_touch=sensor_status,counter_touch=counter,x_touch=x,y_touch=y,z_touch=z,timecode_touch=time_touch)
				#print sql_command
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
	



















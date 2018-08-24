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

objects =["red_tomato","green_sausage","red_ball","yellow_banana","red_banana","yellow_dice","green_pepper","blue_ball","red_dice","puple_grapes","red_sponge","orange_carrot","black_hat","purple_duck","orange_fish","green_figure"]

db_filename="/tmp/one_sample.db"

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
	

#face interface	
fe = faceExpression()
fe.sendFaceExpression("happiness")

# Touch Sensor interface
# Adapt this for the serial number you need for the sensor
optsens = optoforce(ser_number="DSE0A125")

#definition of Maximum current - This protects the hands from breaking!! Do not change this, if you do not know!
MAX_CUR_FINGER=120
MAX_CUR_THUMB=100

# Data for robot movement in high level mode
# move joints with 10 per cent of the speed and hand joints with full speed
fMS = 0.01
fMS_hand=0.9

# Instructions for the experimenter. Brig the robot in Initial position
print "\n Please put the robot in position. Left arm hanging down. Right arm on the table. Give RETURN after finished.\n"
raw_input()

#Put the left arm in defined position
#robot = Motion.Motion("../../../../json/nico_humanoid_legged_with_hands_mod.json",vrep=False)

mover_path = "../../../../moves_and_positions/"
mov = Mover.Mover(robot, stiff_off=False)


mov.move_file_position(mover_path + "pos_left_arm_base.csv",
                                   subsetfname=mover_path + "subset_left_arm_and_head.csv",
                                   move_speed=0.05)        


# Instructions for the experimenter. Bring the robot in Initial position
print "\n OK. The robot is positioned. We will start the demo now.\n\n"
inp=raw_input()

if inp == "n":
	intro=False
else:
	intro=True

if (intro==True):
	
	mov.move_file_position(mover_path + "pos_left_arm_present_egg.csv",
	#mov.move_file_position(mover_path + "pos_left_arm_greet.json",
                                   subsetfname=mover_path + "subset_left_arm_and_head.csv",
                                   move_speed=0.05)

	say ("Hello. I am the NICO robot. I am getting prepared!")
	
	#sleep(2)
	
	mov.move_file_position(mover_path + "pos_left_arm_present_egg.csv",
                                   subsetfname=mover_path + "subset_left_arm_and_head.csv",
                                   move_speed=0.05)
	
	raw_input();
	
	say ("I can determine which object is in my hand, just by squeezing it a little bit.")
	
	sleep(1)

	# Data for touch sensor
	# Adapt this for the serial Interface you need for the sensor

	#mov.move_file_position(mover_path + "pos_left_arm_base.csv",
	#                                   subsetfname=mover_path + "subset_left_arm_and_head.csv",
	#                                   move_speed=0.05) 

	stop_demo=False
	repeat=False
	while ( not stop_demo  ):                                

		if not repeat:
			robot.openHand('LHand', fractionMaxSpeed=fMS_hand)
			#print "\n\n Please put the " + o + " on the robot fingers. Then press RETURN."
			say ("Please put one of the objects in my hand.")						
			
			sleep(2.2)
			
			raw_input()
			print "\n Good. Let me squeeze this a little bit."
			#fe = faceExpression()
			fe.sendFaceExpression("neutral")
			
			#Change to the other robot mode 
			robot.disableTorqueAll()
			robot=None
			sleep(1)
			
			# Delete possible existing and create a new database and open it
			try:
				os.remove(db_filename)
			except OSError:
				pass
			create_db(db_filename)

			connection = sqlite3.connect(db_filename)
			cursor = connection.cursor()

			# Record a sample in the database
			with pypot.dynamixel.DxlIO('/dev/ttyACM0') as dxl_io:
				dxl_io.enable_torque([27, 29])

				#open hand
				dxl_io.set_goal_position({27: -180.00})
				dxl_io.set_goal_position({29: -180.00})
				sleep(0.5)

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
						dxl_io.set_goal_position({27: pos})
						dxl_io.set_goal_position({29: pos})
						sleep(0.5)
						while (dxl_io.get_present_speed([27])<0.01 and dxl_io.get_present_speed([29])<0.01):
							sleep(0.1)

					thumb_pos=get_position_safe(dxl_io,[27])


					finger_pos=get_position_safe(dxl_io,[29])

					id=31

					# get the current values
					cur_thumb=get_present_thumb_current_safe(dxl_io,[id])
					print "Motor Thumb Current " + str(cur_thumb) + " id " + str(id)
					cur_finger = get_present_finger_current_safe(dxl_io,[id])
					print "Motor Finger Current " + str(cur_finger) + " id " + str(id)

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


			#Commit and close the database
			connection.commit()
			connection.close()

			#TODO: 
			# Put the sample in the model
				
			# Present the result

			
		else:
			repeat=False
			sleep(1)
			say ("I will try it again!")
		
		
		if len(ac_list)==1 or (numbers[0]>numbers[1]*50):
			say ("I am very sure what is inside it.")
			sleep(1)
			fe.sendFaceExpression("happiness")
			say ("I could bet my head" + verb_for_item(keys[0]) + include_indefinite_article(keys[0]) + keys[0] +" inside the box.")
			
		elif len(ac_list)==2:
			fe.sendFaceExpression("surprise")
			say ("I am not sure what is inside this box.")
			say ("It could be " + include_indefinite_article(keys[0]) + keys[0] + " or it could be " + include_indefinite_article(keys[1]) + keys[1])
			say ("But I think it is more likely that " + verb_for_item(keys[0]) + include_indefinite_article(keys[0]) + keys[0])
		
		elif len(ac_list)==3:
			fe.sendFaceExpression("sadness")
			say ("I am very unsure about the content.")
			say ("This could be " + include_indefinite_article(keys[0]) + keys[0] + " or it could be " +  include_indefinite_article(keys[1]) + keys[1] + " or it even could be " + include_indefinite_article(keys[2]) + " " + keys[2])
			say ("I guess " + verb_for_item(keys[0]) + include_indefinite_article(keys[0]) + keys[0] + ". But I am not sure.")
			say ("I will try it again but you have to be very silent!")
			repeat=True
			
	
		if not repeat:
			robot.openHand('LHand', fractionMaxSpeed=fMS_hand)
			print "\n Give return to continue."
			raw_input()
			
			say ("OK. Let me try another one!")
			#mov.move_file_position(mover_path + "pos_left_arm_base.csv",
            #                       subsetfname=mover_path + "subset_left_arm_and_head.csv",
            #                       move_speed=0.05)
		



















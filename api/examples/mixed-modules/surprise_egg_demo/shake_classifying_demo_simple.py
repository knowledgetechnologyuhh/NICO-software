# Shake Experiment Demo
#
# Lets the NICO shake an egg 
# 
# record the wav and analyse it with a RNN model by Manfred Eppe

# Erik Strahl

# GNU GPL 3 License

from nicomotion import Motion,Mover
from nicotouch.optoforcesensors import optoforce
from nicoface.FaceExpression import faceExpression
import pypot.dynamixel
from time import sleep
import datetime
import subprocess

import sqlite3
import random

import pyaudio

import os

from subprocess import call

from standalone_audio_recorder import AudioRecorder

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
		import pyttsx3;
		engine = pyttsx3.init();
		engine.say(sen);
		engine.runAndWait() ;

	
#check plural from
def verb_for_item(item):
	
	if (item=="Stone")or (item=="Rice")or (item=="Sugar")or (item=="Sand"):
		return "this is "
	else:
		return "these are "
		
def include_indefinite_article(item):
	
	if (item=="Stone"):
		return " a "
	else:
		return ""
	

fe = faceExpression("/dev/ttyACM6")
fe.sendFaceExpression("happiness")

#definition for numbers per object
number_of_samples_per_object=2
number_shakes_per_sample=3

#Sampling frequency for touch and robotis sensors
move_frequency=48

#definition of Maximum current - This protects the hands from breaking!! Do not change this, if you do not know!
MAX_CUR_FINGER=120
MAX_CUR_THUMB=100

#path to classifier file
json_path="/data/sound_classification/data.json"

##### Simplified Version
subset_eggs =["Herbs","Plastic beads","Rice","Coins"]
#subset_eggs =["Plastic beads"]

#definition for moves
#moves = [["pos_left_arm_ear_1.csv","pos_left_arm_ear_2.csv"],["pos_left_arm_ear_3.csv","pos_left_arm_ear_4.csv"]]
moves = [["pos_left_arm_ear_1.csv","pos_left_arm_ear_2.csv"]]

# Data for robot movement in high level mode
# move joints with 10 per cent of the speed and hand joints with full speed
fMS = 0.01
fMS_hand=0.9

# Instructions for the experimenter. Brig the robot in Initial position
print "\n Please put the robot in position. Right arm hanging down. Left arm on the table. Give RETURN after finished.\n"
raw_input()

#Put the left arm in defined position
robot = Motion.Motion("../../../../json/nico_humanoid_legged_with_hands_mod.json",vrep=False)

print("The following sound devices are available.")
p = pyaudio.PyAudio()
info = p.get_host_api_info_by_index(0)
numdevices = info.get('deviceCount')
audio_device = 0
for i in range(0, numdevices):
	if (p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
		device_name = p.get_device_info_by_host_api_device_index(0, i).get('name')
		print("Input Device id ", i, " - ", device_name)
		if device_name.find("pulse") != -1:
			audio_device = i

ar1 = AudioRecorder(audio_channels=2, samplerate=48000, datadir="./.", audio_device=audio_device)
ar2 = AudioRecorder(audio_channels=2, samplerate=48000, datadir="./.", audio_device=audio_device)

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

	say ("Hello. I am the NICO robot. I am getting prepared to hear!")
	
	#sleep(2)
	
	mov.move_file_position(mover_path + "pos_left_arm_present_egg.csv",
                                   subsetfname=mover_path + "subset_left_arm_and_head.csv",
                                   move_speed=0.05)
	
	raw_input();
	
	say ("If you do not know, what is in this small yellow plastic cotainers, I can help you!")

	say ("I can hear, what is inside it")
	
	sleep(1)
	
	say ("Please exchange these yellow egg in my hand")

# Data for touch sensor
# Adapt this for the serial Interface you need for the sensor

#mov.move_file_position(mover_path + "pos_left_arm_base.csv",
#                                   subsetfname=mover_path + "subset_left_arm_and_head.csv",
#                                   move_speed=0.05) 

stop_demo=False
repeat=False
while ( not stop_demo  ):                                

		if not repeat:
			#print "\n\n Please put the " + o + " on the robot fingers. Then press RETURN."
			print "\n\n Please give an egg Then press RETURN."
					
			
			sleep(2.2)
			robot.openHand('LHand', fractionMaxSpeed=fMS_hand)
			
			raw_input()
			print "\n Good. Let me hear, what is inside."
			#fe = faceExpression()
			fe.sendFaceExpression("neutral")
        
			robot.closeHand('LHand', fractionMaxSpeed=fMS_hand)

			sleep(1)
		else:
			repeat=False
			sleep(1)
			say ("I will try it again!")
		
		mov.move_file_position(mover_path + "pos_left_arm_up.csv",
                                   subsetfname=mover_path + "subset_left_arm_and_head.csv",
                                   move_speed=0.05)

		#print "\n Give RETURN to start this sample. Make sure everything is clear for shaking."
		#raw_input()

		
		########## ToDo:  Start Audio Recording (Sample Based) here
		label="long_rec"
		ar1.start_recording(label,fname=label+".wav")
		
		subsample_it=0
		
		#For defined moves	
		for move in moves:
			
			start=move[0]
			goal=move[1]
			
			mov.move_file_position(mover_path + "pos_left_arm_ear_1.csv",
                                   subsetfname=mover_path + "subset_left_arm_and_head.csv",
                                   move_speed=0.08)
                                   
			sleep(2.5)
			
			results=[]
			for itr in range (number_shakes_per_sample):
				

				subsample_it+=1

				########## ToDo:  Start Audio Recording (Subsample Based) here
				label="sub_"
				ar2.start_recording(label,fname=label+".wav")
				robot.setStiffness("l_wrist_z", 0)
				robot.setStiffness("l_wrist_x",0)	
				
				mov.move_file_position(mover_path + start,
									   subsetfname=mover_path + "subset_left_arm_and_head_without_l_wrists.csv",
									   move_speed=0.3)

				s_wait_time=0.2
				number_of_moves=move_frequency * s_wait_time
				o_wait_time = s_wait_time / number_of_moves
				
				it=0
				for i in range (int(number_of_moves)):
					
					## Get and write move- an sensordata
					
					sleep(o_wait_time)
					it+=1


				mov.move_file_position(mover_path + goal,
									   subsetfname=mover_path + "subset_left_arm_and_head_without_l_wrists.csv",
									   move_speed=0.3)
				
				s_wait_time=2.0
				number_of_moves=move_frequency * s_wait_time
				o_wait_time = s_wait_time / number_of_moves
				
				for i in range (int(number_of_moves)):
					
					#print "I wait now for " + str(o_wait_time) + " seconds. For the " + str(it) + " time."
					sleep(o_wait_time)
					it+=1
				
				########## ToDo:  Stop Audio Recording (Subsample Based) here	
				ar2.stop_recording(0)
				ot=os.stat(json_path).st_mtime
				subprocess.check_call(["./convert_audio.sh"])
				while (ot==os.stat(json_path).st_mtime):
					sleep(0.1)
				import json
				#sleep(0.3)
				with open(json_path) as json_data:
					
					results.append(json.load(json_data))
				
		
		
		#ar2.stop_recording(0)
		ar1.stop_recording(0)
		#print results
		print "Results:\n" + str(results)
		#raw_input()
		
		res=[]
		#### Using the subset data - only a subset of eggs is allowed
		
		### Check if subset of eggs exists
		try:
			print "Im am working with the subset of these eggs: " + str(subset_eggs)
		except NameError:
			print "I am working with the full set of eggs! Define subset_eggs, if you want to work with a subset."
		else:
			for result in results:
				re=[]
				for resu in result:			
					print "Result: " + str(result)
					if resu[0] not in subset_eggs:
						resu[1]=float(resu[1])*0.01
				
						print "*** Reduced ***"
					re.append(resu)
				re=sorted(re, key=lambda field: field[1],reverse=True)
				res.append(re)	
			#results=sorted(res, key=lambda field: field[1],reverse=True)
			print "reversed"
			print res
			results=res
			#raw_input()	
		
		
		
		#Check the first ranked objects from the three tries (former approach)
		ac_list={}
		for result in results:
			print result
				
			result=result[0]
			#print "Result: "
			#print result
			#raw_input()
			
			
			
			if result[0] not in ac_list:
				ac_list[result[0]]=float(result[1])
			else:
				ac_list[result[0]]+=float(result[1])
		# Take all in account and calculate the 
		ac_list_all={}
		for result in results:
			print result
			#raw_input()
			
			
			
			#if result[0] not in ac_list:
			#	ac_list[result[0]]=float(result[1])
			#else:
			#	ac_list[result[0]]+=float(result[1])
			for one_res in result:
				if one_res[0] not in ac_list_all:
					ac_list_all[one_res[0]]=float(one_res[1])
				else:
					ac_list_all[one_res[0]]+=float(one_res[1])


		
		print "AC-List: " + str(ac_list)
		print "AC-List-All: " + str(ac_list_all)
		#raw_input()
		
		
		#Sort the dicts in lists of tuples
		
		import operator
		sorted_ac_list = sorted(ac_list.items(), key=operator.itemgetter(1),reverse=True)
		
		print "Sorted AC-List: " + str(sorted_ac_list)
		
		sorted_ac_list_all = sorted(ac_list_all.items(), key=operator.itemgetter(1),reverse=True)
		
		print "Sorted AC-List-All: " + str(sorted_ac_list_all)
		
		#Create list to name the items
		keys = [x[0] for x in sorted_ac_list_all]
		
		#Create list for the numbers
		numbers = [x[1] for x in sorted_ac_list_all]
		
		#ac_list_sorted={}
		#for key, value in sorted(ac_list.iteritems(), key=lambda (k,v): (v,k)):
		#	ac_list_sorted[key] = value	
				
		#print "\n List of the first ones"	
		#print ac_list_sorted
		
		#keys = ac_list_sorted.keys()
		
		#print keys
		
		#ac_l=ac_list.items()
		
		
		
		mov.move_file_position(mover_path + "pos_left_arm_present_egg.csv",
                                   subsetfname=mover_path + "subset_left_arm_and_head.csv",
                                   move_speed=0.1)
		sleep(1)
		
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
			
		with open('./item.txt', 'w') as f:
			f.write(keys[0])	
		sleep(2)
		with open('./item.txt', 'w') as f:
			f.write("")
		fe.sendFaceExpression("neutral")
		
		if not repeat:
			robot.openHand('LHand', fractionMaxSpeed=fMS_hand)
			print "\n Give return to continue."
			raw_input()
			
			say ("OK. Let me try another one!")
			#mov.move_file_position(mover_path + "pos_left_arm_base.csv",
            #                       subsetfname=mover_path + "subset_left_arm_and_head.csv",
            #                       move_speed=0.05)
		



















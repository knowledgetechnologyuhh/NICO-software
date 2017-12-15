# Shake Experiment Number

# Erik Strahl

# GNU GPL 3 License

from nicomotion import Motion,Mover
from nicotouch.optoforcesensors import optoforce
import pypot.dynamixel
from time import sleep
import datetime

import sqlite3
import random

import pyaudio

from subprocess import call

from standalone_audio_recorder import AudioRecorder

#experiment definitions (Objects and number of graspings)
#definition of objects
# Sugar,Coins,Nuts,Glas,Plastic,Herbs,Stone,Gravel,Rice,Sand
objects =["SU","CO","NU","GL","PL","HE","ST","GR","RI","SA"]
weights =["1","2","3"]

#Combination of objects and weights to make this countable and usable in database
object_weights=[o+"_"+w for o in objects for w in weights]

#for o in objects:
#	for w in weights:
#		ow=o + "_" + w
#		object_weights.append(ow)


#definition for numbers per object
number_of_samples_per_object=2
number_shakes_per_sample=10

#Sampling frequency for touch and robotis sensors
move_frequency=48

#definition of Maximum current - This protects the hands from breaking!! Do not change this, if you do not know!
MAX_CUR_FINGER=120
MAX_CUR_THUMB=100


#definition for moves
moves = [["pos_left_arm_ear_1.csv","pos_left_arm_ear_2.csv"],["pos_left_arm_ear_3.csv","pos_left_arm_ear_4.csv"]]


# Data for SQlite Access
# Experiment Data will be stored in two three database
# with samples for every sample of a grasped onject and subsample for every motor step (like for 10 degrees)

###########!!! Enable for database connection
connection = sqlite3.connect("./data_shake/experiment.db")
cursor = connection.cursor()
#status: 0-succesful , 1- overload, 2 - former_overload


format_str_move = """INSERT INTO move (move_number, subsample_number , move_iteration, l_wrist_x_pos, 
l_wrist_z_pos, l_elbow_y_pos, l_arm_x_pos, l_shoulder_z_pos, l_shoulder_y_pos, l_elbow_y_cur, timecode_local, status,status_touch, counter_touch,x_touch,y_touch,z_touch, 
timecode_touch )
  VALUES (NULL, "{subsample_number}",  "{move_iteration}", "{l_wrist_x_pos}", "{l_wrist_z_pos}", 
"{l_elbow_y_pos}","{l_arm_x_pos}", "{l_shoulder_z_pos}", "{l_shoulder_y_pos}", 
"{l_elbow_y_cur}", "{timecode_local}", "{status}","{status_touch}", "{counter_touch}","{x_touch}","{y_touch}","{z_touch}", 
"{timecode_touch}");"""


format_str_subsample = """INSERT INTO subsample (subsample_number, sample_number , subsample_iteration, timecode_local )
  VALUES (NULL, "{sample_number}",  "{subsample_iteration}", "{timecode_local}");"""


format_str_sample = """INSERT INTO sample (sample_number,object_name , object_weight , object_comb, timecode)
    VALUES (NULL, "{object_name}", "{object_weight}","{or_comb}","{timecode}");"""


# Data for robot movement in high level mode
# move joints with 10 per cent of the speed and hand joints with full speed
fMS = 0.01
fMS_hand=0.5

#get the camera ready
import pygame
import pygame.camera

#pygame.camera.init()

#cam = pygame.camera.Camera("/dev/video0",(640,480))
#cam.start()

def get_object_weight_comb(one_object,one_weight):
	return(one_object+"_"+one_weight)

def get_splitted_object_weight_comb(object_comb):
	return(tuple(object_comb.split("_")))

def get_sampled_numbers_for_object(object_comb):

    sql="SELECT * FROM sample where object_comb='" + object_comb + "';"
    cursor.execute(sql)
    result = cursor.fetchall()
    return len(result)

def get_needed_numbers_for_object(object_name):

    num = number_of_samples_per_object - get_sampled_numbers_for_object(object_name)
    return (num)

def get_needed_overall_numbers(ek):
	
	sum=0
	for o in ek:
		sum+=get_needed_numbers_for_object(o)
	return(sum)
    

def print_progress(object_weights):
    for o in object_weights:
        print " For " + o + " - samples needed: " + str(get_needed_numbers_for_object(o))
        " - samples finished: " + str(get_sampled_numbers_for_object(o))

def write_move_in_db(subsample_number,it):
	
	# Get the sensor data
	#(x, y, z) = optsens.get_sensor_values_hex()
	#print "Sensor " + str((x, y, z))
	(time_touch, counter, sensor_status, x, y, z, checksum) = optsens.get_sensor_all()
	#print str(time_touch) + "," + str(counter) + "," + str(sensor_status) + "," + str(x) + "," + str(y) + "," + str(z) + "," + str(checksum)
				
	# Write the subsample data to the database
	sql_command = format_str_move.format(subsample_number=subsample_number, move_iteration=it,
														l_wrist_x_pos=robot.getAngle("l_wrist_x"),
														l_wrist_z_pos=robot.getAngle("l_wrist_z"),
														l_elbow_y_pos=robot.getAngle("l_elbow_y"),
														l_arm_x_pos=robot.getAngle("l_arm_x"),
														l_shoulder_z_pos=robot.getAngle("l_shoulder_z"),
														l_shoulder_y_pos=robot.getAngle("l_shoulder_y"),
														l_elbow_y_cur=robot.getCurrent("l_elbow_y"),
														timecode_local=datetime.datetime.now().isoformat(),
														status=sensor_status,
														status_touch=sensor_status,
														counter_touch=counter,
														x_touch=x,
														y_touch=y,
														z_touch=z,
														timecode_touch=time_touch)
														

	#print sql_command
	cursor.execute(sql_command)
	#subsample_number = cursor.lastrowid


    #Print out was is still needed
print "\n\nWe still need the following samples:"

print_progress(object_weights)

if get_needed_overall_numbers(object_weights)>0:
    print "\n\nOverall there are still " +str(get_needed_overall_numbers(object_weights)) + " samples needed.\n\n"
else:
    print "\n\nAll samples recorded. Thank you. If you needed more, adapt the number_of_samples_per_object parameter on the programm.\n\n"
    exit(0)


# Instructions for the experimenter. Brig the robot in Initial position
print "\n Please put the robot in position. Right arm hanging down. Left arm on the table. Give RETURN after finished.\n"
raw_input()

#Put the left arm in defined position
robot = Motion.Motion("../../../../json/nico_humanoid_legged_with_hands_mod.json",vrep=False)

#robot.enableForceControl("l_wrist_z", 50)
#robot.enableForceControl("l_wrist_x", 50)

#robot.enableForceControl("l_indexfingers_x", 50)
#robot.enableForceControl("l_thumb_x", 50)

#set the robot to be compliant
#robot.disableTorqueAll()

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
print "\n OK. The robot is positioned. We will start the experiment now.\n\n"
raw_input()


# Data for touch sensor
# Adapt this for the serial Interface you need for the sensor

optsens = optoforce("/dev/ttyACM1", "DSE0A093")

while ( get_needed_overall_numbers(object_weights) > 0  ):

		#Select an object
		ow=random.choice(object_weights)
		while (get_needed_numbers_for_object(ow)<1):
			ow = random.choice(object_weights)

		print get_splitted_object_weight_comb(ow)

		o,w=get_splitted_object_weight_comb(ow)
		print "Randomly chosen object : " + o + " with weight "+ w +" \n"
		
		mov.move_file_position(mover_path + "pos_left_arm_base.csv",
                                   subsetfname=mover_path + "subset_left_arm_and_head.csv",
                                   move_speed=0.05)
                                

        #print "\n\n Please put the " + o + " on the robot fingers. Then press RETURN."
		print "\n\n Please exchange the egg on the robot fingers. Then press RETURN."
		sleep(2.5)
		robot.openHand('LHand', fractionMaxSpeed=fMS_hand)
		raw_input()
        
		robot.closeHand('LHand', fractionMaxSpeed=fMS_hand)
		# Instructions for the experimenter. Brig the robot in Initial position

		sleep(1)
		
		mov.move_file_position(mover_path + "pos_left_arm_up.csv",
                                   subsetfname=mover_path + "subset_left_arm_and_head.csv",
                                   move_speed=0.05)

		print "\n Give RETURN to start this sample. Make sure everything is clear for shaking."
		raw_input()
		
		#!!!!Write Sample data in database
        
		sql_command = format_str_sample.format(object_name=o, object_weight=w, or_comb=ow, timecode= datetime.datetime.now().isoformat())
		#print "\n " +sql_command
		cursor.execute(sql_command)
		sample_number=cursor.lastrowid
		
		########## ToDo:  Start Audio Recording (Sample Based) here
		label="s_"+str(sample_number)
		ar1.start_recording(label,fname=label+".wav")
		
		subsample_it=0
		
		#For defined moves	
		for move in moves:
			
			start=move[0]
			goal=move[1]
			
			mov.move_file_position(mover_path + "pos_left_arm_ear_1.csv",
                                   subsetfname=mover_path + "subset_left_arm_and_head.csv",
                                   move_speed=0.1)
                                   
			sleep(2.5)
			
			for itr in range (number_shakes_per_sample):
				

				#Write the subsample data to the database
				sql_command = format_str_subsample.format(sample_number=sample_number, subsample_iteration = subsample_it,start=start,goal=goal, 
														  timecode_local=datetime.datetime.now().isoformat())
				#print sql_command
				cursor.execute(sql_command)
				subsample_number = cursor.lastrowid
				subsample_it+=1

				########## ToDo:  Start Audio Recording (Subsample Based) here
				label="sub_"+str(subsample_number)
				ar2.start_recording(label,fname=label+".wav")
					
				
				mov.move_file_position(mover_path + start,
									   subsetfname=mover_path + "subset_left_arm_and_head.csv",
									   move_speed=0.3)

				s_wait_time=0.2
				number_of_moves=move_frequency * s_wait_time
				o_wait_time = s_wait_time / number_of_moves
				
				it=0
				for i in range (int(number_of_moves)):
					
					## Get and write move- an sensordata
					
					write_move_in_db(subsample_number,it)
					
					sleep(o_wait_time)
					it+=1


				mov.move_file_position(mover_path + goal,
									   subsetfname=mover_path + "subset_left_arm_and_head.csv",
									   move_speed=0.3)
				
				s_wait_time=2.0
				number_of_moves=move_frequency * s_wait_time
				o_wait_time = s_wait_time / number_of_moves
				
				for i in range (int(number_of_moves)):
					
					## Get and write move- an sensordata
					write_move_in_db(subsample_number,it)	
				
					sleep(o_wait_time)
					it+=1
				
				########## ToDo:  Stop Audio Recording (Subsample Based) here	
				ar2.stop_recording(0)
				sleep(1)
		
		#ar2.stop_recording(0)
		ar1.stop_recording(0)
		
		connection.commit()
		


















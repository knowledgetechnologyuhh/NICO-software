# Shake Experiment Number

# Erik Strahl

# GNU GPL License

from nicomotion import Motion,Mover
from nicotouch.optoforcesensors import optoforce
import pypot.dynamixel
from time import sleep
import datetime

import sqlite3
import random

from subprocess import call

#experiment definitions (Objects and number of graspings)
#definition of objects
objects =["sugar","screws","sand"]
weights =["1","10","50"]

#Combination of objects and weights to make this countable and usable in database
object_weights=[o+"_"+w for o in objects for w in weights]

#for o in objects:
#	for w in weights:
#		ow=o + "_" + w
#		object_weights.append(ow)


#definition for numbers per object
number_of_samples_per_object=2
number_shakes_per_sample=5

#Sampling frequency for touch and robotis sensors
move_frequency=48

#definition of Maximum current - This protects the hands from breaking!! Do not change this, if you do not know!
MAX_CUR_FINGER=120
MAX_CUR_THUMB=100

# Data for SQlite Access
# Experiment Data will be stored in two three database
# with samples for every sample of a grasped onject and subsample for every motor step (like for 10 degrees)

###########!!! Enable for database connection
connection = sqlite3.connect("./data_shake/experiment.db")
cursor = connection.cursor()
#status: 0-succesful , 1- overload, 2 - former_overload


format_str_move = """INSERT INTO move (move_number, subsample_number , subsample_iteration, l_wrist_x_pos, 
l_wrist_z_pos, l_elbow_y_pos, l_arm_x_pos, l_shoulder_z_pos, l_shoulder_y_pos, l_elbow_y_cur, timecode_local, status,status_touch, counter_touch,x_touch,y_touch,z_touch, 
timecode_touch )
  VALUES (NULL, "{sample_number}",  "{subsample_iteration}", "{l_wrist_x_pos}", "{l_wrist_z_pos}", 
"{l_elbow_y_pos}","{l_arm_x_posn}", "{l_shoulder_z_pos}", "{l_shoulder_y_pos}", 
"{l_elbow_y_cur}", "{timecode_local}", "{status}","{status_touch}", "{counter_touch}","{x_touch}","{y_touch}","{z_touch}", 
"{timecode_touch}");"""


format_str_subsample = """INSERT INTO subsample (subsample_number, sample_number , subsample_iteration, timecode_local )
  VALUES (NULL, "{sample_number}",  "{subsample_iteration}", "{timecode_local}");"""


format_str_sample = """INSERT INTO sample (sample_number,object_name , object_weight , or_comb, timecode)
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

def get_splitted_object_weight_comb(object_weight):
	return(tuple(object_weight.split("_")[:-1])

def get_sampled_numbers_for_object(object_comb):

    sql="SELECT * FROM sample where or_comb='" + object_comb + "';"
    cursor.execute(sql)
    result = cursor.fetchall()
    return len(result)

def get_needed_numbers_for_object(object_name):

    num = number_of_samples_per_object - get_sampled_numbers_for_object(object_name)
    return (num)

def get_needed_overall_numbers(ek):
    
    return(len(ek))

def print_progress(object_weights):
    for o in object_weights:
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

optsens = optoforce("/dev/ttyACM1", "DSE0A125")

while ( True ):


        #Select an object
        o=random.choice(object_weights)
        while (get_needed_numbers_for_object(o)<1):
            o = random.choice(object_weights)

        print "Randomly chosen object : " + o + "\n"

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

		raw_input()
		
		mov.move_file_position(mover_path + "pos_left_arm_ear_1.csv",
                                   subsetfname=mover_path + "subset_left_arm_and_head.csv",
                                   move_speed=0.1)
                                   
		sleep(2.5)
		
		
		
		#!!!!Write Sample data in database
        
		sql_command = format_str_sample.format(object_name=o, object_weight=o, timecode= datetime.datetime.now().isoformat())
		#print "\n " +sql_command
		cursor.execute(sql_command)
		sample_number=cursor.lastrowid
		
		########## ToDo:  Start Audio Recording (Sample Based) here
		

		for itr in range (number_shakes_per_sample):

			#Write the subsample data to the database
			sql_command = format_str_subsample.format(sample_number=sample_number, subsample_iteration = itr, 
                                                      timecode_local=datetime.datetime.now().isoformat())
			print sql_command
			cursor.execute(sql_command)
			subsample_number = cursor.lastrowid

			########## ToDo:  Start Audio Recording (Subsample Based) here	
			
			mov.move_file_position(mover_path + "pos_left_arm_ear_1.csv",
                                   subsetfname=mover_path + "subset_left_arm_and_head.csv",
                                   move_speed=0.3)

			s_wait_time=0.2
			number_of_moves=move_frequency * s_wait_time
			o_wait_time = s_wait_time / number_of_moves
			
			it=0
			for i in range (int(number_of_moves)):
				
				## Get and write move- an sensordata
				
				# Get the sensor data
				(x, y, z) = optsens.get_sensor_values_hex()
				print "Sensor " + str((x, y, z))
				(time_touch, counter, sensor_status, x, y, z, checksum) = optsens.get_sensor_all()
				print str(time_touch) + "," + str(counter) + "," + str(sensor_status) + "," + str(x) + "," + str(y) + "," + str(z) + "," + str(checksum)
				
				# Write the subsample data to the database
				sql_command = format_str_move.format(subsample_number=subsample_number, subsample_iteration=i,
														l_wrist_x_pos=robot.getAngle("l_wrist_x"),
														l_wrist_z_pos=robot.getAngle("l_wrist_z"),
														l_elbow_y_pos=robot.getAngle("l_elbow_y"),
														l_arm_x_pos=robot.getAngle("l_arm_x_pos"),
														l_shoulder_z_pos=robot.getAngle("l_shoulder_z"),
														l_shoulder_y_pos=robot.getAngle("l_shoulder_y"),
														l_shoulder_y_pos=robot.getCurrent("l_elbow_y"),
														timecode_local=datetime.datetime.now().isoformat(),
														status=sensor_status,
														status_touch=sensor_status,
														counter_touch=counter,
														x_touch=x,
														y_touch=y,
														z_touch=z,
														timecode_touch=time_touch)
														

				print sql_command
				cursor.execute(sql_command)
				subsample_number = cursor.lastrowid
				
				sleep(o_wait_time)
				it=i


			mov.move_file_position(mover_path + "pos_left_arm_ear_2.csv",
                                   subsetfname=mover_path + "subset_left_arm_and_head.csv",
                                   move_speed=0.3)
			
			s_wait_time=3.0
			number_of_moves=move_frequency * s_wait_time
			o_wait_time = s_wait_time / number_of_moves
			
			for i in range (int(number_of_moves)):
				
				## Get and write move- an sensordata
				
				# Get the sensor data
				(x, y, z) = optsens.get_sensor_values_hex()
				print "Sensor " + str((x, y, z))
				(time_touch, counter, sensor_status, x, y, z, checksum) = optsens.get_sensor_all()
				print str(time_touch) + "," + str(counter) + "," + str(sensor_status) + "," + str(x) + "," + str(y) + "," + str(z) + "," + str(checksum)
				sleep(o_wait_time)
			
			########## ToDo:  Stop Audio Recording (Subsample Based) here	
		
		mov.move_file_position(mover_path + "pos_left_arm_ear_3.csv",
                                   subsetfname=mover_path + "subset_left_arm_and_head.csv",
                                   move_speed=0.2)

		sleep(2)
		
		for t in range (number_shakes_per_sample):                              
	
			mov.move_file_position(mover_path + "pos_left_arm_ear_3.csv",
                                   subsetfname=mover_path + "subset_left_arm_and_head.csv",
                                   move_speed=0.3)

			sleep(0.2)

			mov.move_file_position(mover_path + "pos_left_arm_ear_4.csv",
                                   subsetfname=mover_path + "subset_left_arm_and_head.csv",
                                   move_speed=0.3)
			sleep(3)
		
		connection.commit()
		



while (True):



        #Status no overload
        status=0

        # Step 1
        # Close the hand until overload
        for it,pos in enumerate(range (-180,80,10)):
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
            thumb_pos=dxl_io.get_present_position([27])[0]
            finger_pos=dxl_io.get_present_position([29])[0]
            #(x,y,z) = optsens.get_sensor_values_raw()
            #print "Motor Temperature " + str(dxl_io.get_present_temperature([27]))
            #for id in range (30,33):
            #    try:
            id=31
            cur_thumb=dxl_io.get_present_thumb_current([id])[0]
            print "Motor Thumb Current " + str(cur_thumb) + " id " + str(id)
            cur_finger = dxl_io.get_present_finger_current([id])[0]
            print "Motor Finger Current " + str(cur_finger) + " id " + str(id)
            #    except:
            #        pass
            #    sleep(0.1)
            #print "Motor Speed " + str(dxl_io.get_present_speed([27]))
            print "Motor Goal Position " + str(dxl_io.get_goal_position([27]))
            print "Thumb Present Position " + str(thumb_pos)
            print "Finger Present Position " + str(finger_pos)
            #print "Motor Max Torque" + str(dxl_io.get_max_torque([27]))


            #print "Sensor raw" + str((x,y,z))


            if ((cur_thumb>MAX_CUR_THUMB) or (cur_finger>MAX_CUR_FINGER)) and status==0:
                status=1
                print "!!Reached Overload!!"

            

            #take 5 pictures (error in the driver)
            for t in range(5):
                img = cam.get_image()

            pygame.image.save(img, "./egg_data/" + str(subsample_number) + ".jpg")
            #call(["fswebcam", "-r", "640x480", "-d", "/dev/video0", "--jpeg", "95", "-D", "1",
            #      "./data/" + str(subsample_number) + ".jpg"])
            sleep(.1)

            # !!!! Take a photo here - name = ID of the subsample

            # !!!! Take a photo here - name = ID of the subsample

            #raw_input()

        robot = Motion.Motion("../../../../json/nico_humanoid_legged_with_hands_mod.json", vrep=False)

        

        # enable torque of left arm joints
        robot.enableForceControl("head_z", 20)
        robot.enableForceControl("head_y", 20)

        robot.enableForceControl("r_shoulder_z", 20)
        robot.enableForceControl("r_shoulder_y", 20)
        robot.enableForceControl("r_arm_x", 20)
        robot.enableForceControl("r_elbow_y", 20)





        #step over 8 movement steps
        for n in range(8):

            mov.move_file_position(mover_path + "lift_arm_experiment_pos_"+n+".csv",
                                   subsetfname=mover_path + "subset_right_arm.csv",
                                   move_speed=0.05)

            sleep(1)

            # Write the subsample data to the database
            sql_command = format_str_subsample.format(sample_number=sample_number, subsample_iteration=n,
                                                      r_elbow_y_position=robot.getAngle("r_elbow_y"),
                                                        r_arm_x_position = robot.getAngle("r_arm_x"),
                                                      r_shoulder_z_position = robot.getAngle("r_shoulder_z"),
                                                      r_shoulder_y_position = robot.getAngle("r_shoulder_y"),
                                                      r_elbow_y_current=robot.getCurrent("r_elbow_y") ,
                                                      r_arm_x_current=robot.getCurrent("r_arm_x"),
                                                      r_shoulder_z_current=robot.getCurrent("r_shoulder_z"),
                                                      r_shoulder_y_current=robot.getCurrent("r_shoulder_y"),
                                                      timecode_local=datetime.datetime.now().isoformat())

            print sql_command
            cursor.execute(sql_command)
            subsample_number = cursor.lastrowid

            # take 5 pictures (error in the driver)
            for t in range(5):
                img = cam.get_image()

            pygame.image.save(img, "./arm_data/arm_" + str(subsample_number) + ".jpg")
            # call(["fswebcam", "-r", "640x480", "-d", "/dev/video0", "--jpeg", "95", "-D", "1",
            #      "./data/" + str(subsample_number) + ".jpg"])
            sleep(.1)


            #Write data to database here

        for n in range(8,0,-1):
            mov.move_file_position(mover_path + "lift_arm_experiment_pos_" + n + ".csv",
                                   subsetfname=mover_path + "subset_right_arm.csv",
                                   move_speed=0.05)

            sleep(0.5)

            # Write data to database here

        # Step 2
        # Move the arm upwards

        # print "Motor Goal Position " + str(dxl_io.get_goal_position([27]))
        # print "Thumb Present Position " + str(thumb_pos)
        # print "Finger Present Position " + str(finger_pos)
        ###

        # print "Motor Max Torque" + str(dxl_io.get_max_torque([27]))


        # print "Sensor raw" + str((x,y,z))
        # (x, y, z) = optsens.get_sensor_values_hex()
        # print "Sensor " + str((x, y, z))
        # (time_touch, counter, sensor_status, x, y, z, checksum) = optsens.get_sensor_all()
        # print str(time_touch) + "," + str(counter) + "," + str(sensor_status) + "," + str(x) + "," + str(y) + "," + str(z) + "," + str(checksum)


        
	connection.close()
	print "\n\n Great! I got all samples together! Finishing experiment.\n"
	print_progress()

	dxl_io.disable_torque([27, 29])

#set the robot to be compliant














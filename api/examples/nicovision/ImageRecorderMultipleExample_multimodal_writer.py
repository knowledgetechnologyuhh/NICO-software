# Advanced Example using the Imagerecorder class
# takes the args resolution_x, resolution_y, framerate, number_of_cameras_to_use
# modifies the custom_callback function
# to write data from robot joint synchronized by taking the picture
#
# Authors:
# Connor
# Erik

import logging
from nicovision import ImageRecorder
from nicomotion import Motion
import os
from os.path import dirname, abspath
from time import sleep
import datetime
import sys
import cv2

fnl="left_cam_synced_data.csv"
fnr="right_cam_synced_data.csv"
robot=None

import pandas as pd
columns = ["r_arm_x","r_elbow_y","head_z","isotime"]
dfl = pd.DataFrame(columns=columns)
dfr = pd.DataFrame(columns=columns)


def write_joint_data(robot,df,iso_time):
	
	#df = pd.DataFrame.append(data={"r_arm_x":[robot.getAngle("r_arm_x")],"r_elbow_y":[robot.getAngle("r_elbow_y")],"head_z":[robot.getAngle("head_z")],"isotime":[iso_time]})
	dfn = pd.DataFrame(data={"r_arm_x":[robot.getAngle("r_arm_x")],"r_elbow_y":[robot.getAngle("r_elbow_y")],"head_z":[robot.getAngle("head_z")],"isotime":[iso_time]})
	df = pd.concat([df, dfn], ignore_index=True)
	
	#df = pd.DataFrame(data={"r_arm_x":[robot.getAngle("r_arm_x")],"r_elbow_y":[robot.getAngle("r_elbow_y")],"head_z":[robot.getAngle("head_z")]})
	return(df)

class leftcam_ImageRecorder(ImageRecorder.ImageRecorder):
	
	def custom_callback(self, iso_time,frame):
		
		global dfl;
		
		#write joint data
		
		dfl=write_joint_data(robot,dfl,iso_time)
		
		#small = cv2.resize(frame, (0,0), fx=0.5, fy=0.5)
		#small=cv2.flip(small,-1)
		#return(small)
		return(frame) 

class rightcam_ImageRecorder(ImageRecorder.ImageRecorder):
	
	def custom_callback(self, iso_time,frame):
		
		global dfr;
		
		#write joint data
		
		dfr=write_joint_data(robot,dfr,iso_time)
		
		#small = cv2.resize(frame, (0,0), fx=0.5, fy=0.5)		
		#return(small)
		return(frame) 



print len(sys.argv)

if len(sys.argv)!=5:

	print "please call me with the parameters"
	print "resolution_x, resolution_y, framerate, number_of_cameras_to_use"

else:
	
	res_x=int(sys.argv[1])
	res_y=int(sys.argv[2])
	framerate=int(sys.argv[3])
	amount_of_cams=int(sys.argv[4])
	logging.getLogger().setLevel(logging.INFO)
	
		
	try:
		os.mkdir(dirname(abspath(__file__))+'/recorded_images')
	except OSError:
		pass

	try:
		os.mkdir(dirname(abspath(__file__))+'/recorded_images/camera1')
	except OSError:
		pass

	try:
		os.mkdir(dirname(abspath(__file__))+'/recorded_images/camera2')
	except OSError:
		pass

	# Instructions for the experimenter. 
	print "\n Please put the robot in position. Right arm on the table. Left arm hanging down. Give RETURN after finished.\n"
	raw_input()

	#Put the left arm in defined position
	robot = Motion.Motion("../../../json/nico_humanoid_upper_rh7d.json",vrep=False)

	#set the robot to be compliant
	robot.disableTorqueAll()
	# enable torque of left arm joints
	robot.enableForceControl("head_z", 40)
	
	print "devices" + str( ImageRecorder.get_devices() )
	device = ImageRecorder.get_devices()[0]
	ir = leftcam_ImageRecorder(device, res_x, res_y,framerate=framerate,writer_threads=3,pixel_format="UYVY")

	if amount_of_cams>=2:
		device2 = ImageRecorder.get_devices()[1]
		ir2 = rightcam_ImageRecorder(device2, res_x, res_y,framerate=framerate,writer_threads=3,pixel_format="UYVY")

	sleep(2)
	
	print("Start taking pictures")
	print(datetime.datetime.today().isoformat())
	ir.start_recording(
		path=dirname(abspath(__file__))+'/recorded_images/camera1/picture-{}.png')
	if amount_of_cams>=2:
		ir2.start_recording(
			 path=dirname(abspath(__file__))+'/recorded_images/camera2/picture-{}.png')
			 
	#Some motor action here
	for t in range(5):
		robot.setAngle("head_z", 20, 0.05)
		sleep(0.5)
		robot.setAngle("head_z", -20, 0.05)
		sleep(0.5)
	print("Stop recording")
	ir.stop_recording()
	if amount_of_cams>=2:
		ir2.stop_recording()
	print("Finished taking pictures")
	
	for df_set in ((fnl,dfl),(fnr,dfr)):
		fnp,dfp=df_set
		with open(fnp, 'a') as f:
			dfp.to_csv(f, header=True)

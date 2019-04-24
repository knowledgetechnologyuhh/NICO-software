# Advanced Example using the Imagerecorder class
# takes the args resolution_x, resolution_y, framerate, number_of_cameras_to_use
# modifies the custom_callback function
# to crop the picture (right) and to crop and rotate the picture (left)
# from the command line
#
# Authors:
# Connor
# Erik

import logging
from nicovision import ImageRecorder
import os
from os.path import dirname, abspath
from time import sleep
import datetime
import sys
import cv2

class leftcam_ImageRecorder(ImageRecorder.ImageRecorder):
	
	def custom_callback(self, iso_time,frame):
		small = cv2.resize(frame, (0,0), fx=0.5, fy=0.5)
		small=cv2.flip(small,-1)
		return(small) 

class rightcam_ImageRecorder(ImageRecorder.ImageRecorder):
	
	def custom_callback(self, iso_time,frame):
		small = cv2.resize(frame, (0,0), fx=0.5, fy=0.5)
		
		return(small) 

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
	sleep(10)
	print("Stop recording")
	ir.stop_recording()
	if amount_of_cams>=2:
		ir2.stop_recording()
	print("Finished taking pictures")

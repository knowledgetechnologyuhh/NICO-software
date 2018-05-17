import logging
from nicovision import ImageRecorder
import os
from os.path import dirname, abspath
from time import sleep
import datetime
import sys


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
	ir = ImageRecorder.ImageRecorder(device, res_x, res_y,framerate=framerate,writer_threads=5)
	#ir = ImageRecorder.ImageRecorder(device, 1920, 1080,framerate=30,writer_threads=5)
	#ir = ImageRecorder.ImageRecorder(device, 3840, 2160,framerate=30,writer_threads=5)
	#ir = ImageRecorder.ImageRecorder(device, 1280, 720,framerate=20,writer_threads=5)
	if amount_of_cams>=2:
		device2 = ImageRecorder.get_devices()[1]
		ir2 = ImageRecorder.ImageRecorder(device2, res_x, res_y,framerate=framerate,writer_threads=5)
	#ir2 = ImageRecorder.ImageRecorder(device2, 1920, 1080,framerate=30,writer_threads=5)
	#ir2 = ImageRecorder.ImageRecorder(device2, 3840, 2160,framerate=30,writer_threads=5)
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

# Simple Example using the Imagerecorder class
# takes the args
# resolution_x, resolution_y, framerate, number_of_cameras_to_use,
# (optional) zoom
# from the command line
#
# Authors:
# Connor
# Erik

import datetime
import logging
import os
import sys
from os.path import abspath, dirname
from time import sleep

from nicovision import ImageRecorder, MultiCamRecorder

try:
    os.mkdir(dirname(abspath(__file__)) + '/recorded_images')
except OSError:
    pass

print(len(sys.argv))
if len(sys.argv) not in (5, 6):
    print("please call me with the parameters")
    print("resolution_x, resolution_y, framerate, number_of_cameras_to_use, " +
          "(optional) zoom")
else:

    res_x = int(sys.argv[1])
    res_y = int(sys.argv[2])
    framerate = int(sys.argv[3])
    amount_of_cams = int(sys.argv[4])
    zoom = int(sys.argv[5]) if len(sys.argv) == 6 else 100

    for i in range(amount_of_cams):
        try:
            os.mkdir(dirname(abspath(__file__)) +
                     '/recorded_images/camera{}'.format(i))
        except OSError:
            pass

    logging.getLogger().setLevel(logging.INFO)
    print("devices" + str(ImageRecorder.get_devices()))
    if amount_of_cams == 1:
        device = ImageRecorder.get_devices()[0]
        ir = ImageRecorder.ImageRecorder(device, res_x, res_y,
                                         framerate=framerate, zoom=zoom,
                                         writer_threads=3, pixel_format="UYVY")
        # ir = ImageRecorder.ImageRecorder(device, res_x, res_y,framerate=framerate,writer_threads=4,pixel_format="MJPG")
        # ir = ImageRecorder.ImageRecorder(device, 1920, 1080,framerate=30,writer_threads=5)
        # ir = ImageRecorder.ImageRecorder(device, 3840, 2160,framerate=30,writer_threads=5)
        # ir = ImageRecorder.ImageRecorder(device, 1280, 720,framerate=20,writer_threads=5)
    elif amount_of_cams > 2:
        devices = ImageRecorder.get_devices()[0:amount_of_cams]
        ir = MultiCamRecorder.MultiCamRecorder(devices, res_x, res_y,
                                               framerate=framerate, zoom=zoom,
                                               writer_threads=4,
                                               pixel_format="UYVY")
    else:
        devices = MultiCamRecorder.autodetect_nicoeyes()
        ir = MultiCamRecorder.MultiCamRecorder(devices, res_x, res_y,
                                               framerate=framerate, zoom=zoom,
                                               writer_threads=4,
                                               pixel_format="UYVY")
    sleep(2)
    print("Start taking pictures")
    print(datetime.datetime.today().isoformat())
    if amount_of_cams == 1:
        ir.start_recording(path=dirname(abspath(__file__)) +
                           '/recorded_images/camera0/picture-{}.png')
    elif amount_of_cams >= 2:
        ir.start_recording(path=dirname(abspath(__file__)) +
                           '/recorded_images/camera{}/picture-{}.png')
    sleep(10)
    print("Stop recording")
    ir.stop_recording()
    print("Finished taking pictures")

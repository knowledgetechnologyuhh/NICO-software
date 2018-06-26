import datetime
import logging
import os
import random
import sys
from os.path import abspath, dirname
from time import sleep

from nicovision import MultiCamRecorder

logging.getLogger().setLevel(logging.INFO)

try:
    os.mkdir(dirname(abspath(__file__)) + '/random_zoom_images')
except OSError:
    pass

amount_of_cams = 2

for i in range(amount_of_cams):
    try:
        os.mkdir(dirname(abspath(__file__)) +
                 '/random_zoom_images/camera{}'.format(i))
    except OSError:
        pass

devices = MultiCamRecorder.get_devices()[0:amount_of_cams]

ir = MultiCamRecorder.MultiCamRecorder(devices, 1920, 1080, framerate=30,
                                       zoom=400, writer_threads=4,
                                       pixel_format="UYVY")
sleep(2)
print("Start taking pictures")
print(datetime.datetime.today().isoformat())

ir.start_recording(path=dirname(abspath(__file__)) +
                   '/random_zoom_images/camera{}/picture-{}.png')

for i in range(5):
    ir.pan(random.randrange(-648000, 648001, 3600))
    ir.tilt(random.randrange(-648000, 648001, 3600))
    ir.zoom(random.randint(100, 800))
    sleep(1)
print("Stop recording")
ir.stop_recording()
print("Finished taking pictures")

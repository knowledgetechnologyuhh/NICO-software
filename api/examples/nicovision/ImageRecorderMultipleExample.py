import logging
from nicovision import ImageRecorder
import os
from os.path import dirname, abspath
from time import sleep
import datetime

try:
    os.mkdir(dirname(abspath(__file__))+'/recorded_images')
except OSError:
    pass

try:
    os.mkdir(dirname(abspath(__file__))+'/recorded_images/camera1')
except OSError:
    pass

# try:
#     os.mkdir(dirname(abspath(__file__))+'/recorded_images/camera2')
# except OSError:
#     pass

logging.getLogger().setLevel(logging.INFO)
device = ImageRecorder.get_devices()[0]
ir = ImageRecorder.ImageRecorder(device, 1920, 1080)
# device2 = ImageRecorder.get_devices()[1]
# ir2 = ImageRecorder.ImageRecorder(device2, 640, 480)
sleep(1)
print("Start taking pictures")
print(datetime.datetime.today().isoformat())
ir.start_recording(
    path=dirname(abspath(__file__))+'/recorded_images/camera1/picture-{}.png')
# ir2.start_recording(
#     path=dirname(abspath(__file__))+'/recorded_images/camera2/picture-{}.png')
sleep(3)
print("Stop recording")
ir.stop_recording()
# ir2.stop_recording()
print("Finished taking pictures")

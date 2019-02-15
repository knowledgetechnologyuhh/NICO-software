import os
import sys
import time
from os.path import abspath, dirname

from nicovision import MultiCamRecorder

for i in range(2):
    try:
        os.remove(dirname(abspath(__file__)) +
                  '/SettingsExample{}.png'.format(i))
    except OSError:
        pass


settings = dirname(abspath(__file__)) + \
    "/../../../json/nico_vision_Logitech_C905_settings.json"

cameras = MultiCamRecorder.get_devices()[:2]

ir = MultiCamRecorder.MultiCamRecorder(cameras, width=640, height=480,
                                       framerate=30, zoom=None, pan=None,
                                       tilt=None, settings_file=settings,
                                       setting="standard", writer_threads=4)
time.sleep(5)
ir.start_recording(path='SettingsExample{}.png')
time.sleep(0.1)
ir.stop_recording()
time.sleep(1)


import os
import sys
import time
from nicovision import VideoDevice
from nicovision import ImageRecorder

# definition of camera_settings
# name of the setting : just a name of the setting 
# camera_value_name: the name the value like brightness or contrast
# value the value of it
# the position steps are defined in files named pos_[actionname]_[no of step].csv

brightness (int)    : min=-15 max=15 step=1 default=0 value=2
                       contrast (int)    : min=0 max=30 step=1 default=15 value=15
                     saturation (int)    : min=0 max=60 step=1 default=32 value=32
 white_balance_temperature_auto (bool)   : default=1 value=1
                          gamma (int)    : min=40 max=500 step=1 default=220 value=220
                           gain (int)    : min=0 max=100 step=1 default=0 value=2
      white_balance_temperature (int)    : min=1000 max=10000 step=50 default=5000 value=5000 flags=inactive
                      sharpness (int)    : min=0 max=127 step=1 default=16 value=16
                  exposure_auto (menu)   : min=0 max=3 default=1 value=0
              exposure_absolute (int)    : min=1 max=10000 step=1 default=312 value=312 flags=inactive
                   pan_absolute (int)    : min=-648000 max=648000 step=3600 default=0 value=0
                  tilt_absolute (int)    : min=-648000 max=648000 step=3600 default=0 value=0
                  zoom_absolute (int)    : min=100 max=800 step=1 default=100 value=0

camera_settings = {
        "standard": { 
        },
        "multimodal_experiment": {
                "brightness": 2,
                "contrast": 15,
                "saturation": 32,
                "white_balance_temperature_auto": 0,
                "gamma": 220,
                "gain" : 0,
                "white_balance_temperature": 5000,
                "sharpness": 16,
                "exposure_auto" : 0,
                "exposure_absolute" : 312,
                "zoom_absolute": 100,
                "tilt_absolute": 0,
                "pan_absolute": 0
        }
}

def camera_values(setting_name,device):

    setting_set=device.camera_value[setting_name]

    for name,setting in setting_set.iteritems():
        device.camera_settings(name,setting)


if __name__ == '__main__':
    
    if len(sys.argv)!=2:
        print ("\n Set the setting set for cameras.")
        print ("set_camera_values.py [name_of_the_camera_set]")

    else:
        cameras=[VideoDevice.PATH_LEGGED_NICO_LEFT_CAM,VideoDevice.PATH_LEGGED_NICO_RIGHT_CAM]
        for camera in cameras:
            ir = ImageRecorder(
                camera, 1920, 1080, framerate=30, writer_threads=5, pixel_format="UYVY")
            camera_values(sysargv[1],ir)


        

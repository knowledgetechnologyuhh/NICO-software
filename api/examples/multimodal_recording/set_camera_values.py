
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
                "white_balance_temperature": 32000,
                "sharpness": 16,
                "exposure_auto" : 0,
                "exposure_absolute" : 312,
                "zoom_absolute": 200,
                "tilt_absolute": 0,
                "pan_absolute": 0
        }
}

def camera_values(setting_name,device):

    setting_set=camera_settings[setting_name]

    for name,setting in setting_set.iteritems():
        device.camera_value(name,setting)


if __name__ == '__main__':
    
    if len(sys.argv)!=2:
        print ("\n Set the setting set for cameras.")
        print ("set_camera_values.py [name_of_the_camera_set]")

    else:
        cameras=[VideoDevice.ID_STR_LEGGED_NICO_LEFT_CAM,VideoDevice.ID_STR_LEGGED_NICO_RIGHT_CAM]
        for camera in cameras:
            ir = ImageRecorder.ImageRecorder(camera, 1920, 1080, framerate=30, writer_threads=5, pixel_format="UYVY")
            camera_values(sys.argv[1],ir)
            ir.start_recording(path='/tmp/picture.png')
            time.sleep(0.1)
            ir.stop_recording()
            time.sleep(1)
            ir=None


        

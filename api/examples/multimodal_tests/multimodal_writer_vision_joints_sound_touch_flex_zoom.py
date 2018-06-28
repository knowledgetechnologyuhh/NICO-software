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
from nicovision import VideoDevice
from nicomotion import Motion
import os
from os.path import dirname, abspath
from time import sleep
import datetime
import sys
import cv2

from nicotouch.optoforcesensors import optoforce

from nicoaudio import pulse_audio_recorder

import subprocess


fnl = "left_cam_synced_data.csv"
fnr = "right_cam_synced_data.csv"
robot = None

use_touch = True

import pandas as pd
columns = ["r_arm_x", "r_elbow_y", "head_z", "isotime"]
dfl = pd.DataFrame(columns=columns)
dfr = pd.DataFrame(columns=columns)


def write_joint_data(robot, df, iso_time):

    if use_touch:
        (stime, counter, status, x, y, z,
         checksum) = optoforce_sensor.get_sensor_all()
        # stime, counter, status, x, y, z, checksum=10,100,1,1,2,3,10
        dfn = pd.DataFrame(data={"r_arm_x": [robot.getAngle("r_arm_x")], "r_elbow_y": [robot.getAngle("r_elbow_y")], "head_z": [robot.getAngle("head_z")],
                                 "isotime": [iso_time], "isotime_touch": [stime], "count_touch": [counter], "x_touch": [x], "y_touch": [y], "z_touch": [z]})
    else:
        # df = pd.DataFrame.append(data={"r_arm_x":[robot.getAngle("r_arm_x")],"r_elbow_y":[robot.getAngle("r_elbow_y")],"head_z":[robot.getAngle("head_z")],"isotime":[iso_time]})
        dfn = pd.DataFrame(data={"r_arm_x": [robot.getAngle("r_arm_x")], "r_elbow_y": [
            robot.getAngle("r_elbow_y")], "head_z": [robot.getAngle("head_z")], "isotime": [iso_time]})
    df = pd.concat([df, dfn], ignore_index=True)

    # df = pd.DataFrame(data={"r_arm_x":[robot.getAngle("r_arm_x")],"r_elbow_y":[robot.getAngle("r_elbow_y")],"head_z":[robot.getAngle("head_z")]})
    sleep(0.001)
    return(df)





class leftcam_ImageRecorder(ImageRecorder.ImageRecorder):

    def custom_callback(self, iso_time, frame):

        global dfl

        # write joint data

        dfl = write_joint_data(robot, dfl, iso_time)

        # small = cv2.resize(frame, (0,0), fx=0.5, fy=0.5)
        # small=cv2.flip(small,-1)
        # return(small)
        return(frame)


class rightcam_ImageRecorder(ImageRecorder.ImageRecorder):

    def custom_callback(self, iso_time, frame):

        global dfr

        # write joint data

        dfr = write_joint_data(robot, dfr, iso_time)

        # small = cv2.resize(frame, (0,0), fx=0.5, fy=0.5)
        # return(small)
        return(frame)


print len(sys.argv)

if len(sys.argv) != 6:

    print "please call me with the parameters"
    print "resolution_x, resolution_y, framerate, number_of_cameras_to_use, data_directory"

else:

    pulse_device = pulse_audio_recorder.get_pulse_device()

    res_x = int(sys.argv[1])
    res_y = int(sys.argv[2])
    framerate = int(sys.argv[3])
    amount_of_cams = int(sys.argv[4])
    data_directory = str(sys.argv[5])
    logging.getLogger().setLevel(logging.INFO)

    if data_directory == "":
        data_directory = os.mkdir(dirname(abspath(__file__)))

    
    try:
        os.mkdir(data_directory+'/recorded_images')
    except OSError:
        pass

    try:
        os.mkdir(data_directory+'/recorded_images/camera1')
    except OSError:
        pass

    try:
        os.mkdir(data_directory+'/recorded_images/camera2')
    except OSError:
        pass

    # Instructions for the experimenter.
    print "\n Please put the robot in position. Right arm on the table. Left arm hanging down. Give RETURN after finished.\n"
    raw_input()

    # Optoforce_sensor
    optoforce_sensor = optoforce(ser_number=None, cache_frequency=30)

    # Put the left arm in defined position
    # robot = Motion.Motion("../../../json/nico_humanoid_upper_rh7d.json",vrep=False)
    robot = Motion.Motion(
        "../../../json/nico_humanoid_legged_minimal_for_multimodal_recordings.json", vrep=False)

    sleep(3)

    # set the robot to be compliant
    robot.disableTorqueAll()
    # enable torque of left arm joints
    robot.enableForceControl("head_z", 40)

    # zoom level, pan and tilt
    zoom_level = 200
    pan = -648000
    #pan=36000
    # pan=3600*value
    tilt =36000

    print "devices" + str(ImageRecorder.get_devices())
    device = ImageRecorder.get_devices()[0]
    ir = leftcam_ImageRecorder(
        device, res_x, res_y,
        zoom=zoom_level, pan=pan, tilt=tilt, framerate=framerate, writer_threads=4, pixel_format="UYVY")

    #ir.zoom(zoom_level,cam_pathname=VideoDevice.PATH_LEGGED_NICO_LEFT)
    #ir.pan(pan)
    #ir.tilt(tilt)


    if amount_of_cams >= 2:
        device2 = ImageRecorder.get_devices()[1]
        ir2 = rightcam_ImageRecorder(
            device2, res_x, res_y, framerate=framerate, writer_threads=2, pixel_format="UYVY")
        #ir2.zoom(zoom_level)
        ir2.pan(pan)
        ir2.tilt(tilt)

    sleep(2)

    print ("Start sound recording")
    # raw_input()
    ar = pulse_audio_recorder.AudioRecorder(
        audio_channels=2, samplerate=48000, datadir="./.", audio_device=pulse_device)

    # label="s_"+str(sample_number)
    label = datetime.datetime.today().isoformat()
    ar.start_recording(label, fname=label+".wav",
                       dir_name=data_directory+"/audio/")

    print("Start taking pictures")
    print(datetime.datetime.today().isoformat())
    ir.start_recording(
        path=data_directory+'/recorded_images/camera1/picture-{}.png')
    if amount_of_cams >= 2:
        ir2.start_recording(
            path=data_directory+'/recorded_images/camera2/picture-{}.png')

    # Some motor action here
    for t in range(5):
        robot.setAngle("head_z", 20, 0.01)
        sleep(0.5)
        robot.setAngle("head_z", -20, 0.01)
        sleep(0.5)
    print("Stop recording")
    ar.stop_recording(0)
    ir.stop_recording()
    if amount_of_cams >= 2:
        ir2.stop_recording()
    print("Finished taking pictures")

    # Disable and cloe robot
    robot.disableTorqueAll()
    robot = None

    for df_set in ((fnl, dfl), (fnr, dfr)):
        fnp, dfp = df_set
        with open(data_directory + "/" + fnp, 'a') as f:
            dfp.to_csv(f, header=True)

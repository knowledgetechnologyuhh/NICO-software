#!/usr/bin/env python3

import datetime
import time
import os.path

import threading
from nicomotion import Motion
#from nicoaudio import AudioRecorder #TODO check
from nicovision import VideoRecorder
from nicotouch.optoforcesensors import optoforce
from standalone_audio_recorder import AudioRecorder
import pyaudio
import cv2
import numpy as np
import random
import math


class MuliModalRecorder:

    random_seed = 0
    recording_max_time = 10000 #for limit, in ms

    input_actions = "./input_actions_stage1.txt" #looking
    #input_actions = "./input_actions_stage2.txt" #pulling, pushing
    #input_actions = "./input_actions_stage3.txt" #grasping, lifting, shaking, banging
    input_objects = "./input_objects.txt"
    output_successful_datums = "#successful_samples.txt"
    outputfolder_name = "./mmr"
    output_subject = "dave"
    list_datums = []
    gui = None

    # Params for cameras of NICO
    nico_vision_fps = 30
    nico_vision_height = 1600
    nico_vision_width = 1200
    nico_vision_stereo = True
    nico_vision_right_device = "/dev/video0"
    nico_vision_left_device = "/dev/video1"
    nico_vision_format = VideoRecorder.VideoCodec.DIVX
    nico_vision_right_fpostfix = "__nico_vision_right"
    nico_vision_left_fpostfix = "__nico_vision_left"
    nico_vision_recorder_right = None
    nico_vision_recorder_left = None

    # Params for microphones of NICO
    nico_audio_rate = 44100
    nico_audio_stereo = True
    nico_audio_fpostfix = "__nico_audio"
    nico_audio_recorder = None

    # Params for arm of NICO
    nico_sensorim_fps = 30 #60
    nico_sensorim_joints = ("r_shoulder_z", "r_shoulder_y", "r_arm_x", "r_elbow_y", "r_wrist_z", "r_wrist_x", "r_thumb_x", "r_indexfingers_x")
    nico_sensorim_fpostfix = "__nico_sensorim"
    nico_sensorim_recorder = None
    
    # Params for motors of NICO
    nico_motor_max_cur_finger = 120 #Protects the hand from damage
    nico_motor_max_cur_finger = 100 #Protects the hand from damage
    nico_motor_maximum_speed_body = 0.01 #Maximum speed of body motors
    nico_motor_maximum_speed_hand = 0.1  #Maximum speed of hand motors
    
    # Params for teacher HD Webcam
    teacher_hdcam_fps = 30
    teacher_hdcam_height = 1920
    teacher_hdcam_width = 1080
    teacher_hdcam_stereo = True
    teacher_hdcam_device = "/dev/video2"
    teacher_hdcam_format = VideoRecorder.VideoCodec.DIVX
    teacher_hdcam_fpostfix = "__teacher_hdvision"
    teacher_hdcam_recorder = None

    # Params for teacher Intel RealSense
    teacher_rgbdcam_fps = 30
    teacher_rgbdcam_height = 640
    teacher_rgbdcam_width = 480
    teacher_rgbdcam_stereo = True
    teacher_rgbdcam_device = "/dev/video3"
    teacher_rgbdcam_format = VideoRecorder.VideoCodec.DIVX
    teacher_rgbdcam_fpostfix = "__nico_rgbdvision"
    teacher_rgbdcam_recorder = None

    def __init__(self):
        self.ouputfolder_name = self.ouputfolder_name + "_" \
                                + self.output_subject \
                                + "_d" + datetime.datetime.now()("%y%m%d%H%M")

        # Prepare the list of interactions
        act = np.loadtxt(self.input_actions, dtype=np.str, comments="#",
                         delimiter=",", unpack=False).tolist()
        obj = np.loadtxt(self.input_objects, dtype=np.str, comments="#",
                         delimiter=",", unpack=False).tolist()

        self.list_datums = ["" + a + " " + o for a in act for o in obj]

        # Initialise all recorders
        self.nico_vision_recorder_right = VideoRecorder.VideoRecorder(
            device=self.nico_vision_right_device,
            videoformat=self.nico_vision_format,
            height=self.nico_vision_height,
            width=self.nico_vision_width,
            framerate=self.nico_vision_fps)

        if (self.nico_vision_stereo):
            self.nico_vision_recorder_left = VideoRecorder.VideoRecorder(
                device=self.nico_vision_left_device,
                videoformat=self.nico_vision_format,
                height=self.nico_vision_height,
                width=self.nico_vision_width,
                framerate=self.nico_vision_fps)

        self.nico_audio_recorder = AudioRecorder.AudioRecorder()

        self.nico_sensorim_recorder = NicoSensorimotorRecorder(
            joints=self.nico_sensorim_joints,
            framerate=self.nico_sensorim_fps)

        self.teacher_hdcam_recorder = VideoRecorder.VideoRecorder(
            device=self.teacher_hdcam_device,
            videoformat=self.teacher_hdcam_format,
            height=self.teacher_hdcam_height,
            width=self.teacher_hdcam_width,
            framerate=self.teacher_hdcam_fps)

        self.teacher_rgbdcam_recorder = VideoRecorder.VideoRecorder(
            device=self.teacher_rgbdcam_device,
            videoformat=self.teacher_rgbdcam_format,
            height=self.teacher_rgbdcam_height,
            width=self.teacher_rgbdcam_width,
            framerate=self.teacher_rgbdcam_fps)

        # prepare a GUI for the experimenter
        self.gui = np.zeros((400, 1600, 3), np.uint8)
        cv2.imshow('Multi-modal recording', self.gui)
        cv2.moveWindow('Multi-modal recording', 40, 40)

        self.run()


    def start_record(self, datum):

        # record all devices in parallel.
        #self.nico_vision_recorder_right.startRecording(
        #    self.outputfolder_name, datum + self.nico_audio_fpostfix)
        thrds = []
        thrds += [threading.Thread(
            target=self.nico_vision_recorder_right.startRecording,
            args=(self.outputfolder_name, datum + self.nico_vision_right_fpostfix))]
        thrds += [threading.Thread(
            target=self.nico_vision_recorder_left.startRecording,
            args=(self.outputfolder_name, datum + self.nico_vision_left_fpostfix))]
        thrds += [threading.Thread(
            target=self.nico_audio_recorder.startMicrophonesRecording,
            args=("" + self.outputfolder_name + datum + self.nico_audio_fpostfix + ".wav",
                  self.nico_audio_rate, #samplerate
                  (self.nico_audio_stereo,True)))] #channels
        thrds += [threading.Thread(
            target=self.nico_sensorim_recorder.start_recording,
            args=(self.outputfolder_name, datum + self.nico_audio_fpostfix))]
        thrds += [threading.Thread(
            target=self.teacher_hdcam_recorder.startRecording,
            args=(self.outputfolder_name, datum + self.teacher_hdcam_fpostfix))]
        thrds += [threading.Thread(
            target=self.teacher_rgbdcam_recorder.startRecording,
            args=(self.outputfolder_name, datum + self.teacher_rgbdcam_fpostfix))]
        #TODO if necessary, start a motor movement, based on the datum

        for k in range(len(thrds)):
            thrds[k].daemon = True
            thrds[k].start()

        return True


    def stop_record(self, datum):

        thrds = []
        thrds += [threading.Thread(
            target=self.nico_vision_recorder_right.stopRecording)]
        thrds += [threading.Thread(
            target=self.nico_vision_recorder_left.stopRecording)]
        thrds += [threading.Thread(
            target=self.nico_audio_recorder.stopRecording)]
        thrds += [threading.Thread(
            target=self.nico_sensorim_recorder.stop_recording)]
        thrds += [threading.Thread(
            target=self.teacher_hdcam_recorder.stopRecording)]
        thrds += [threading.Thread(
            target=self.teacher_rgbdcam_recorder.stopRecording)]

        for k in range(len(thrds)):
            thrds[k].daemon = True
            thrds[k].start()

        time.sleep(0.2) #just wait a bit for the recorder to stop

        return True


    def run(self):

        listNotEmpty = True

        random.shuffle(self.list_datums)

        while listNotEmpty:
            datum = self.list_datums.pop()

            self.gui = np.zeros((320, 1600, 3), np.uint8)
            cv2.putText(self.gui,
                        "Prepare the following interaction and press any key when ready ...",
                        (80, 40), 10, (255, 255, 128), 2)
            cv2.putText(self.gui,
                        "" + datum,
                        (120, 40), 10, (128, 255, 128), 2)
            cv2.imshow('Multi-modal recording', self.gui)
            c = cv2.waitKey(0)

            self.start_record(datum)
            cv2.putText(self.gui,
                        "... recording - press another key when interaction is finished ...",
                        (160, 40), 10, (255, 128, 128), 2)
            cv2.imshow('Multi-modal recording', self.gui)
            c = cv2.waitKey(0)

            cv2.putText(self.gui,
                        "Are you confident that the recording went well?",
                        (200, 40), 10, (255, 255, 128), 2)
            cv2.putText(self.gui,
                        "Press LEFT for no or press RIGHT for yes!",
                        (240, 40), 10, (255, 255, 128), 2)
            cv2.imshow('Multi-modal recording', self.gui)
            self.stop_record(datum)
            c = cv2.waitKey(0)

            if (c == 2555904): #RIGHT key
                # successful
                cv2.putText(self.gui,
                            "Done. Recordings have been saved. Thank you.",
                            (280, 40), 10, (255, 128, 255), 2)
                cv2.imshow('Multi-modal recording', self.gui)
                with open(self.output_successful_datums, "a") as sf:
                    sf.write(datum + "\n")
            else:
                # unsuccessful
                cv2.putText(self.gui,
                            "No Problem, we will repeat this later. Thank you.",
                            (280, 40), 10, (128, 255, 255), 2)
                cv2.imshow('Multi-modal recording', self.gui)
                self.list_datums.append(datum)

            if not self.list_datums:
                listNotEmpty = False
            time.sleep(2.0)

        cv2.destroyAllWindows()

        return True


class NicoSensorimotorRecorder:

    frame_duration = 1000 #in ms
    joints = None
    robot = None

    time_step = 0
    is_recording = False
    is_file_writing = False

    save_file = None

    def __init__(self, joints=None, framerate=30):
        self.joints = joints
        self.robot = Motion.Motion("../../../../json/nico_arm.json",vrep=False)
        #Hand is included in control, sepatate object needed for haptic sensor
        self.optsens = optoforce("/dev/ttyACM1", "DSE0A125")
        
        #Initializing audio Recorder
        self.p = pyaudio.PyAudio()
        self.info = self.p.get_host_api_info_by_index(0)
        self.numdevices = self.info.get('deviceCount')
        self.audio_device = 0
        for i in range(0, self.numdevices):
            if (self.p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
                device_name = self.p.get_device_info_by_host_api_device_index(0, i).get('name')
                print("Input Device id ", i, " - ", device_name)
                if device_name.find("pulse") != -1:
                    self.audio_device = i

        self.ar = AudioRecorder(audio_channels=2, samplerate=48000, datadir="./.", audio_device=self.audio_device)

        self.time_wait = 1000 / framerate

        # start recorder as a thread in the background
        thrd = threading.Thread(target=self.recorder)
        thrd.daemon = True
        thrd.start()

        return

    def start_recording(self, folder_name, file_name, overwrite = True):

        header = "#sensorimotor measurements - format: timestep"
        for k in range(len(self.joints)):
            header += "; " + self.joints[k] + "_deg"
            header += "; " + self.joints[k] + "_cur"

        if os.path.isfile(folder_name+"/"+file_name):
            if (overwrite):
                self.save_file = open(folder_name + "/" + file_name, "w")
            else:
                self.save_file = open(folder_name+"/"+file_name, "a")
                self.save_file.write(header + "\n")
        else:
            self.save_file = open(folder_name+"/"+file_name, "a")
            self.save_file.write(header + "\n")

        self.is_file_writing = True

        #Audio Recording
        #TODO: Make sure the audiofiles are synched with the rest of the recording
        #TODO: e.g. add name & time stamps of the audio files to the recorded data
        label = "MultimodalDataset"
        self.ar.start_recording(label,fname=label+".wav")

        self.time_step = 0
        self.is_recording = True
        
        return True

    def stop_recording(self):
        self.is_recording = False
        
        #Stop the audio recording
        self.ar.start_recording(0)

        return

    def recorder(self):
        while(True):
            time_start = datetime.datetime.now()

            if (self.is_recording):
                if (self.is_file_writing):

                    dat = "" + str(self.time_step)

                    # get all joint infos
                    for k in range(self.joints):
                        dat += "; " + "{:.6f}".format(self.robot.getAngle(self.joints[k]))
                        dat += "; " + "{:.6f}".format(self.robot.getCurrent(self.joints[k]))

                    #get values from haptic sensor - check if format is ok
                    #saving x,y,z and L2-norm
                    (x, y, z) = self.optsens.get_sensor_values_hex()
                    l2_norm = math.sqrt(math.pow(x,2)+math.pow(y,2)+math.pow(z,2))
                    print "Sensor " + str((x, y, z))
                    dat += "; " + "{:.6f}".format(x)
                    dat += "; " + "{:.6f}".format(y)
                    dat += "; " + "{:.6f}".format(z)
                    dat += "; " + "{:.6f}".format(l2_norm)

                    self.save_file.write(dat + "\n")

                else:
                    # something went wrong, this should not happen
                    self.is_recording = False

                self.time_step += 1
                
            else:
                if (self.is_file_writing):
                    self.save_file.close()

            # wait for remaining time:
            time_fin = datetime.datetime.now()
            time_wait = max(0.0,
               self.frame_duration - (0.001 * (time_fin - time_start).microseconds))
            #print(time_wait, (time_fin - time_start).microseconds) #for debug
            time.sleep(time_wait)


if __name__ == '__main__':

    mmrecord = MuliModalRecorder()





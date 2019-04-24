# -*- coding: utf-8

import time
from os.path import abspath, dirname

from nicoemotionrecognition import EmotionRecognition
from nicoface.FaceExpression import faceExpression
from nicomotion import Motion
from nicovision.VideoDevice import VideoDevice

robot = Motion.Motion(dirname(abspath(__file__)) +
                      "/../../../json/nico_humanoid_upper.json", vrep=False,
                      ignoreMissing=True)
face = faceExpression()
# torso NICO

camera = VideoDevice.autodetect_nicoeyes()[0]  # 0: left_eye, 1: right_eye

emotionRecogniton = EmotionRecognition.EmotionRecognition(
    device=camera, robot=robot, face=face, voiceEnabled=True, german=True)

# emotionRecogniton = EmotionRecognition.EmotionRecognition(
#     device='usb-046d_080a_6C686AA1-video-index0', robot=robot, face=face, voiceEnabled=True, german=True)
# legged NICO

#emotionRecogniton = EmotionRecognition.EmotionRecognition(device='usb-046d_080a_17E79161-video-index0', robot=robot, face=face,faceDetectionDelta=10, voiceEnabled=True)

#emotionRecogniton = EmotionRecognition.EmotionRecognition(device='usb-046d_080a_2DE7B460-video-index0', face=face)

emotionRecogniton.start(showGUI=True, faceTracking=True, mirrorEmotion=True)
raw_input("Press enter to stop\n")
#print emotionRecogniton.getDimensionalData()
#print emotionRecogniton.getCategoricalData()

time.sleep(1.0)

robot.disableTorqueAll()
emotionRecogniton.stop()

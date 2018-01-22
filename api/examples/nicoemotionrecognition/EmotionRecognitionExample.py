import time
from nicoemotionrecognition import EmotionRecognition
from nicomotion import Motion
from nicoface.FaceExpression import faceExpression
from os.path import dirname, abspath

robot = Motion.Motion(dirname(abspath(__file__))+"/../../../json/nico_humanoid_upper.json",vrep=False)
face = faceExpression("/dev/ttyACM0")
emotionRecogniton = EmotionRecognition.EmotionRecognition(device='usb-046d_080a_6C686AA1-video-index0', robot=robot, face=face)
emotionRecogniton.start(showGUI=True, faceTracking=True, mirrorEmotion=True)
raw_input("Press enter to stop\n")
#print emotionRecogniton.getDimensionalData()
#print emotionRecogniton.getCategoricalData()

time.sleep(1.0)

robot.disableTorqueAll()
emotionRecogniton.stop()

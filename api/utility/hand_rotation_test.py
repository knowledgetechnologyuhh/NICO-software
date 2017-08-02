#!/usr/bin/env python
from nicomotion import Motion
import cv2
import time
import thread
import os
from os.path import dirname, abspath

try:
    os.mkdir(dirname(abspath(__file__))+'/rotation_images')
except OSError:
    pass


def cameraThread(camera):
    while camera.isOpened():
        camera.grab()

camera = cv2.VideoCapture(0)
thread.start_new_thread(cameraThread, (camera,))
robot = Motion.Motion("json/nico_humanoid_upper.json",vrep=False)
for rotation in range(80):
    robot.setAngle("r_wrist_z",-180+5*rotation,0.05)
    time.sleep(2)
    angle = robot.getAngle("r_wrist_z")
    _,image = camera.retrieve()
    cv2.putText(image, "setAngle: "+str(-180+5*rotation), (5,20), cv2.cv.CV_FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255))
    cv2.putText(image, "getAngle: "+str(angle), (5,40), cv2.cv.CV_FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255))
    cv2.imwrite(dirname(abspath(__file__))+"/rotation_images/hand_at_"+str(-180+5*rotation)+"_degrees.png", image );
del(camera)
robot.setAngle("r_wrist_z",-180,0.05)
time.sleep(5)
robot.disableTorqueAll()

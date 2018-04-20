#!/usr/bin/env python
import cv2
import time
import thread
import os
from os.path import dirname, abspath

try:
    os.mkdir(dirname(abspath(__file__))+'/test')
except OSError:
    pass


def camera_thread(camera):
    while camera.isOpened():
        camera.grab()

camera = cv2.VideoCapture(0)

camera.set(cv2.CAP_PROP_FRAME_WIDTH, 4096)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,2160)

thread.start_new_thread(camera_thread, (camera,))
cv2.namedWindow('image', cv2.WINDOW_NORMAL)

while camera.isOpened():
    _,image = camera.retrieve()
    cv2.imshow('image',image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
camera.release()
cv2.destroyAllWindows()

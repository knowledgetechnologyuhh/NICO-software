# -*- coding: utf-8 -*-
"""
Created on Wed Sep  7 19:25:08 2016

@author: cruz
"""

import cv2
import numpy as np
import signal
import sys
import time
import threading
from nicomotion import Motion
from os.path import abspath, dirname


class Demo(object):
    def __init__(self):
        self.area = 0
        self.xPosition = 0
        self.yPosition = 0
        self.SPEED = 0.015  # motors speed
        self.SAMPLE_TIME = 0.3  # time to sample a new image (in seconds)
        self.ANGLE_STEP = 5  # angle to turn the motors
        self.MIN_AREA = (
            60000
        )  # 120000      #minimal area to identify a color, the bigger the less noise is consider but the object has to be closer then
        self.MAX_X = 640  # camera resolution in x axis
        self.MAX_Y = 480  # camera resolution in y axis
        self.TOLERANCE = 40  # tolerance to be consider in the middle of the image

        # Uncomment the following line to use the simulated NICO in v-rep
        # self.nico = Motion.Motion('json/nico_humanoid_upper_with_hands.json', True, '127.0.0.1', 19997, 'v-rep/NICO-seated.ttt')
        # Uncomment the following line to use de real NICO with grippers
        config = Motion.Motion.vrepRemoteConfig()
        config["vrep_scene"] = (
            dirname(abspath(__file__)) + "/../../../../v-rep/NICO-seated.ttt"
        )
        self.nico = Motion.Motion(
            dirname(abspath(__file__)) + "/../../../../json/nico_humanoid_vrep.json",
            vrep=True,
            vrepConfig=config,
        )

        def signal_handler(sig, frame):
            self.cleanup()
            sys.exit(0)

        signal.signal(signal.SIGINT, signal_handler)

        threads = list()

        t = threading.Thread(target=self.color_detection)
        threads.append(t)
        t.start()

        t = threading.Thread(target=self.head_movement)
        threads.append(t)
        t.start()

        t = threading.Thread(target=self.arm_movement)
        threads.append(t)
        t.start()

    # end of __init__ method

    def __del__(self):
        self.nico.disableTorqueAll()

    def cleanup(self):
        print("Cleanup")
        self.nico.disableTorqueAll()

    def color_detection(self):
        # Camara initialization
        capture = cv2.VideoCapture(0)

        cv2.namedWindow("mask", 1)
        cv2.moveWindow("mask", 1880, 0)

        cv2.namedWindow("Camera", 1)
        cv2.moveWindow("Camera", 2520, 0)

        while 1:

            # Image capture and conversion from RGB -> HSV
            _, image = capture.read()
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # range of color (pink in this case)
            low_color = np.array([160, 80, 80], dtype=np.uint8)
            high_color = np.array([190, 255, 255], dtype=np.uint8)

            # Mask in range color
            mask = cv2.inRange(hsv, low_color, high_color)

            # Area of the object
            moments = cv2.moments(mask)
            self.area = moments["m00"]

            if self.area > self.MIN_AREA:

                # Center of the object
                x = int(moments["m10"] / moments["m00"])
                y = int(moments["m01"] / moments["m00"])

                self.xPosition = x
                self.yPosition = y

                # Sign at the center of the object
                cv2.rectangle(image, (x, y), (x + 2, y + 2), (0, 0, 255), 2)

            # Show original image with the sign and the mask
            cv2.imshow("mask", mask)
            cv2.imshow("Camera", image)

            key = cv2.waitKey(5) & 0xFF
            if key == 27:
                break
            # time.sleep(0.3)

        cv2.destroyAllWindows()

    # end of color_detection method

    def head_movement(self):
        joint_x = "head_z"
        joint_y = "head_y"
        pos_x = 0
        pos_y = 0
        self.nico.setAngle(joint_x, pos_x, self.SPEED)
        self.nico.setAngle(joint_y, pos_y, self.SPEED)

        while 1:
            if self.area > self.MIN_AREA:

                if (
                    self.xPosition > 0
                    and self.xPosition < (self.MAX_X / 2 - self.TOLERANCE)
                    and pos_x < 65
                ):
                    pos_x = pos_x + self.ANGLE_STEP
                    self.nico.setAngle(joint_x, pos_x, self.SPEED)

                if (
                    self.xPosition > (self.MAX_X / 2 + self.TOLERANCE)
                    and self.xPosition < self.MAX_X
                    and pos_x > -65
                ):
                    pos_x = pos_x - self.ANGLE_STEP
                    self.nico.setAngle(joint_x, pos_x, self.SPEED)

                if (
                    self.yPosition > 0
                    and self.yPosition < (self.MAX_Y / 2 - self.TOLERANCE)
                    and pos_y > -45
                ):
                    pos_y = pos_y - self.ANGLE_STEP
                    self.nico.setAngle(joint_y, pos_y, self.SPEED)

                if (
                    self.yPosition > (self.MAX_Y / 2 + self.TOLERANCE)
                    and self.yPosition < self.MAX_Y
                    and pos_y < 25
                ):
                    pos_y = pos_y + self.ANGLE_STEP
                    self.nico.setAngle(joint_y, pos_y, self.SPEED)

            time.sleep(self.SAMPLE_TIME)

    # end of head_movement method

    def arm_position(self, pos):
        self.nico.setAngle("r_shoulder_z", pos[0], self.SPEED)
        self.nico.setAngle("r_shoulder_y", pos[1], self.SPEED)
        self.nico.setAngle("r_arm_x", pos[2], self.SPEED)
        self.nico.setAngle("r_elbow_y", pos[3], self.SPEED)
        self.nico.setAngle("r_wrist_z", pos[4], self.SPEED)
        # self.nico.setAngle("r_gripper_x", pos[5], self.SPEED)  # FIXME joint

    # end of amr_position method

    def arm_movement(self):
        self.arm_position([0, 0, -15, 0, 0, 0])
        time.sleep(5)

        while True:
            self.arm_position([0, 15, 0, -90, -90, -30])
            time.sleep(5)
            self.arm_position([45, 90, 0, -60, -90, -30])
            time.sleep(5)
            self.arm_position([45, 90, 0, -60, 90, -30])
            time.sleep(3)
            self.arm_position([45, 90, 0, -60, -90, -30])
            time.sleep(3)
            self.arm_position([90, 90, 0, -60, -90, -30])
            time.sleep(5)
            self.arm_position([90, 15, 0, -90, -90, -30])
            time.sleep(5)
            self.arm_position([0, 15, 0, -90, -90, -30])
            time.sleep(5)
            self.arm_position([0, 120, 30, 0, -90, -30])
            time.sleep(10)

            self.arm_position([45, 90, 0, -60, -90, -30])
            time.sleep(5)
            self.arm_position([0, 0, -15, 0, 0, 0])
            time.sleep(5)

            # self.arm_position([45, 90, 0, -60, -90, -30])

    # end of arm_movement method


# run the demo
demo = Demo()

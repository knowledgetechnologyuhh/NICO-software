# -*- coding: utf-8 -*-
"""
Created on Wed Sep  7 19:25:08 2016

@author: cruz, gaede
"""

import argparse
import cv2
import numpy as np
import signal
import sys
import time
import threading
from nicomotion import Motion
from nicovision import VideoDevice
from os.path import abspath, dirname
from nicoface.FaceExpression import faceExpression
import random


class Demo(object):
    def __init__(self, vrep=False):
        self.area = 0
        self.xPosition = 0
        self.yPosition = 0
        self.SPEED = 0.02  # motors speed
        self.SAMPLE_TIME = 0.3  # time to sample a new image (in seconds)
        self.ANGLE_STEP = 5  # angle to turn the motors
        self.ANGLE_STEP_BIG = 7.5  # angle to turn the motors
        self.MIN_AREA = 40000  # 120000      #minimal area to identify a color, the bigger the less noise is consider but the object has to be closer then
        self.MAX_X = 640  # camera resolution in x axis
        self.MAX_Y = 480  # camera resolution in y axis
        self.ZOOM = 200
        self.TOLERANCE = 40  # tolerance to be consider in the middle of the image

        if vrep:
            config = Motion.Motion.vrepRemoteConfig()
            config["vrep_scene"] = (
                dirname(abspath(__file__)) + "/../../../../v-rep/NICO-seated.ttt"
            )
            self.nico = Motion.Motion(
                dirname(abspath(__file__))
                + "/../../../../json/nico_humanoid_vrep.json",
                vrep=True,
                vrepConfig=config,
            )

        else:
            self.nico = Motion.Motion(
                dirname(abspath(__file__))
                + "/../../../../json/nico_humanoid_upper_rh7d.json",
            )
            self.face = faceExpression()

        def signal_handler(sig, frame):
            self.cleanup()
            sys.exit(0)

        signal.signal(signal.SIGINT, signal_handler)

        self.running = True
        self.last_detection = 0

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

        t = threading.Thread(target=self.face_expression)
        threads.append(t)
        t.start()

        self.threads = threads

    # end of __init__ method

    def __del__(self):
        self.nico.disableTorqueAll()

    def cleanup(self):
        print("Cleanup")
        self.running = False
        for t in self.threads:
            t.join()
        self.nico.disableTorqueAll()
        time.sleep(1)

    def color_detection(self):
        # Camara initialization
        device_name = VideoDevice.VideoDevice.autodetect_nicoeyes()[0]
        device = VideoDevice.VideoDevice.from_device(
            device_name, width=self.MAX_X, height=self.MAX_Y, zoom=self.ZOOM
        )

        def callback(rval, image):
            # Image capture and conversion from RGB -> HSV
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # range of color (light green in this case)
            # low_color = np.array([50, 64, 100], dtype=np.uint8)
            # high_color = np.array([70, 191, 255], dtype=np.uint8)
            # range of color (yellow in this case)
            low_color = np.array([20, 100, 100], dtype=np.uint8)
            high_color = np.array([40, 255, 255], dtype=np.uint8)

            # Mask in range color
            mask = cv2.inRange(hsv, low_color, high_color)

            # Area of the object
            moments = cv2.moments(mask)
            self.area = moments["m00"]

            if self.area > self.MIN_AREA:
                # print("detected")
                self.last_detection = time.time()

                # Center of the object
                x = int(moments["m10"] / moments["m00"])
                y = int(moments["m01"] / moments["m00"])

                self.xPosition = x
                self.yPosition = y

                # Sign at the center of the object
                cv2.rectangle(image, (x, y), (x + 2, y + 2), (0, 0, 255), 2)
            else:
                # print("not detected")
                pass

            # Show original image with the sign and the mask
            cv2.imshow("mask", mask)
            cv2.imshow("Camera", image)
            cv2.waitKey(1)
            # time.sleep(0.3)

        device.add_callback(callback)
        print("Camera started")

        while self.running:
            time.sleep(0.1)

        device.close()

        cv2.destroyAllWindows()

    # end of color_detection method

    def head_movement(self):
        joint_x = "head_z"
        joint_y = "head_y"
        pos_x = 0
        pos_y = 0
        self.nico.setAngle(joint_x, pos_x, self.SPEED)
        self.nico.setAngle(joint_y, pos_y, self.SPEED)

        while self.running:
            if self.area > self.MIN_AREA:

                if (
                    self.xPosition > 0
                    and self.xPosition < (self.MAX_X / 2 - self.TOLERANCE)
                    and pos_x < 65
                ):
                    if self.xPosition < self.MAX_X / 3:
                        pos_x += self.ANGLE_STEP_BIG
                    else:
                        pos_x += self.ANGLE_STEP
                    self.nico.setAngle(joint_x, pos_x, self.SPEED)

                if (
                    self.xPosition > (self.MAX_X / 2 + self.TOLERANCE)
                    and self.xPosition < self.MAX_X
                    and pos_x > -65
                ):
                    if self.xPosition > self.MAX_X * 2 / 3:
                        pos_x -= self.ANGLE_STEP_BIG
                    else:
                        pos_x -= self.ANGLE_STEP
                    self.nico.setAngle(joint_x, pos_x, self.SPEED)

                if (
                    self.yPosition > 0
                    and self.yPosition < (self.MAX_Y / 2 - self.TOLERANCE)
                    and pos_y > -45
                ):
                    if self.yPosition < self.MAX_Y / 3:
                        pos_y -= self.ANGLE_STEP_BIG
                    else:
                        pos_y -= self.ANGLE_STEP
                    self.nico.setAngle(joint_y, pos_y, self.SPEED)

                if (
                    self.yPosition > (self.MAX_Y / 2 + self.TOLERANCE)
                    and self.yPosition < self.MAX_Y
                    and pos_y < 25
                ):
                    if self.yPosition > self.MAX_X * 2 / 3:
                        pos_y += self.ANGLE_STEP_BIG
                    else:
                        pos_y += self.ANGLE_STEP
                    self.nico.setAngle(joint_y, pos_y, self.SPEED)
            elif time.time() - self.last_detection > 2:
                self.nico.setAngle(joint_x, 0, self.SPEED)
                self.nico.setAngle(joint_y, 0, self.SPEED)

            time.sleep(self.SAMPLE_TIME)

    # end of head_movement method

    def arm_position(self, pos):
        self.nico.setAngle("r_shoulder_z", pos[0], self.SPEED)
        self.nico.setAngle("r_shoulder_y", pos[1], self.SPEED)
        self.nico.setAngle("r_arm_x", pos[2], self.SPEED)
        self.nico.setAngle("r_elbow_y", pos[3], self.SPEED)
        self.nico.setAngle("r_wrist_z", pos[4], self.SPEED)
        self.nico.setHandPose("RHand", pos[5], self.SPEED, pos[6])

    # end of amr_position method

    def arm_movement(self):
        # r_shoulder_z, r_shoulder_y, r_arm_x, r_elbow_y, r_wrist_z, hand_pose, hand_percentage
        positions = [
            [0, 55, -15, -45, 150, "closeHand", 0.4],
            [22.5, 55, -15, -45, 150, "closeHand", 0.4],
            [45, 55, -15, -45, 150, "closeHand", 0.4],
            [0, 90, -15, -35, 150, "closeHand", 0.4],
            [22.5, 90, -15, -35, 150, "closeHand", 0.4],
            [45, 90, -15, -35, 150, "closeHand", 0.4],
        ]
        while self.running:
            self.arm_position([0, 55, -15, -45, 150, "prepareGrab", 1.0])
            time.sleep(5)
            self.arm_position([0, 55, -15, -45, 150, "closeHand", 0.4])
            time.sleep(3)
            prev_pos = -1
            pos = -1
            for _ in range(5):
                if not self.running:
                    break
                while pos == prev_pos:
                    pos = random.randint(0, len(positions) - 1)
                prev_pos = pos
                self.arm_position(positions[random.randint(0, len(positions) - 1)])
                time.sleep(4)

        self.arm_position([0, 55, -15, -45, 150, "prepareGrab", 1.0])
        time.sleep(5)
        self.arm_position([0, 0, -15, 0, 0, "openHand", 1.0])
        time.sleep(5)

    # end of arm_movement method

    def face_expression(self):
        if self.face:
            while self.running:
                delta_time = time.time() - self.last_detection
                if delta_time < 2:
                    self.face.sendFaceExpression("happiness")
                elif delta_time < 4:
                    self.face.sendFaceExpression("sadness")
                else:
                    self.face.sendFaceExpression("neutral")


# run the demo
parser = argparse.ArgumentParser()
parser.add_argument("-v", "--vrep", help="use vrep simulation", action="store_true")
args = parser.parse_args()

demo = Demo(args.vrep)

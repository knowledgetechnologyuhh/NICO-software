import atexit
import logging
import random
import time
from os.path import abspath, dirname

from nicoface import FaceExpression
from nicomotion import Motion

logging.basicConfig(level=logging.WARNING)


def close(robot):
    robot.disableTorqueAll()
    robot.cleanup()


def initial_position(robot):
    for arm in ("l", "r"):
        prefix = 1. if arm == "l" else -1.

        robot.setAngle(arm + "_shoulder_z", prefix * 0., .03)
        robot.setAngle(arm + "_shoulder_y", prefix * -10., .03)
        robot.setAngle(arm + "_elbow_y", prefix * 80., .03)
        robot.setAngle(arm + "_arm_x", prefix * 15., .03)
        robot.setAngle(arm + "_wrist_x", prefix * 0., .1)
        robot.setAngle(arm + "_wrist_y", prefix * 0., .1)
        robot.setAngle(arm + "_wrist_z", prefix * 0., .1)
        robot.openHand(arm.upper() + "Hand")

    time.sleep(3.)


robot = Motion.Motion(dirname(abspath(__file__)) +
                      "/../../../../json/nico_humanoid_upper_rh7d.json",
                      vrep=False)

atexit.register(close, robot)

face = FaceExpression.faceExpression()

expression = random.choice(('happiness', 'sadness', 'anger', 'disgust',
                            'surprise', 'fear'))  # , 'neutral', 'clear')

initial_position(robot)

face.sendFaceExpression(expression)

if expression == 'happiness':
    for arm in ("r", "l"):
        prefix = 1. if arm == "l" else -1.

        if arm == "r":
            robot.setAngle(arm + "_shoulder_z", prefix * -50., .03)
            robot.setAngle(arm + "_shoulder_y", prefix * -45., .03)
            robot.setAngle(arm + "_elbow_y", prefix * 80., .03)
            robot.setAngle(arm + "_arm_x", prefix * 15., .03)
            robot.setAngle(arm + "_wrist_x", prefix * 0., .1)
            robot.setAngle(arm + "_wrist_y", prefix * 0., .1)
            robot.setAngle(arm + "_wrist_z", prefix * 0., .1)
            robot.setHandPose(arm.upper() + "Hand", "thumbsUp")

elif expression == 'sadness':
    for arm in ("l", "r"):
        prefix = 1. if arm == "l" else -1.

        robot.setAngle(arm + "_shoulder_z", prefix * -40., .03)
        robot.setAngle(arm + "_shoulder_y", prefix * -50., .03)
        robot.setAngle(arm + "_elbow_y", prefix * 100., .03)
        robot.setAngle(arm + "_arm_x", prefix * 140., .03)
        robot.setAngle(arm + "_wrist_x", prefix * 150., .1)
        robot.setAngle(arm + "_wrist_y", prefix * 150., .1)
        robot.setAngle(arm + "_wrist_z", prefix * 50., .1)
        robot.closeHand(arm.upper() + "Hand")

elif expression == 'anger':
    for arm in ("l", "r"):
        prefix = 1. if arm == "l" else -1.

        robot.setAngle(arm + "_shoulder_z", prefix * 40., .03)
        robot.setAngle(arm + "_shoulder_y", prefix * -120., .03)
        robot.setAngle(arm + "_elbow_y", prefix * 80., .03)
        robot.setAngle(arm + "_arm_x", prefix * 0., .03)
        robot.closeHand(arm.upper() + "Hand")

elif expression == 'disgust':
    for arm in ("l", "r"):
        prefix = 1. if arm == "l" else -1.

        if arm == "r":
            robot.setAngle(arm + "_shoulder_z", prefix * -30., .03)
            robot.setAngle(arm + "_shoulder_y", prefix * -70., .03)
            robot.setAngle(arm + "_elbow_y", prefix * 100., .03)
            robot.setAngle(arm + "_arm_x", prefix * 15., .03)
            robot.setAngle(arm + "_wrist_x", prefix * 150., .1)
            robot.setAngle(arm + "_wrist_z", prefix * -100., .1)
        else:
            robot.setAngle(arm + "_shoulder_z", prefix * -75., .03)
            robot.setAngle(arm + "_shoulder_y", prefix * -10., .03)
            robot.setAngle(arm + "_elbow_y", prefix * 80., .03)
            robot.setAngle(arm + "_arm_x", prefix * 10., .03)
            robot.setAngle(arm + "_wrist_x", prefix * 150., .1)

elif expression == 'surprise':
    for arm in ("l", "r"):
        prefix = 1. if arm == "l" else -1.

        if arm == "l":
            robot.setAngle(arm + "_shoulder_z", prefix * -40., .03)
            robot.setAngle(arm + "_shoulder_y", prefix * -60., .03)
            robot.setAngle(arm + "_elbow_y", prefix * 100., .03)
            robot.setAngle(arm + "_arm_x", prefix * 35., .03)
            robot.setAngle(arm + "_wrist_x", prefix * 150., .1)
            robot.setAngle(arm + "_wrist_z", prefix * -100., .1)
        else:
            robot.setAngle(arm + "_shoulder_z", prefix * -10., .03)
            robot.setAngle(arm + "_shoulder_y", prefix * -65., .03)
            robot.setAngle(arm + "_elbow_y", prefix * 35., .03)
            robot.setAngle(arm + "_arm_x", prefix * 15., .03)
            robot.setHandPose(arm.upper() + "Hand", "pointAt")

elif expression == 'fear':
    for arm in ("l", "r"):
        prefix = 1. if arm == "l" else -1.

        # if arm == "r":
        robot.setAngle(arm + "_shoulder_z", prefix * -15., .03)
        robot.setAngle(arm + "_shoulder_y", prefix * -50., .03)
        robot.setAngle(arm + "_elbow_y", prefix * 100., .03)
        robot.setAngle(arm + "_arm_x", prefix * -15., .03)
        robot.setAngle(arm + "_wrist_x", prefix * 150., .1)
        robot.setAngle(arm + "_wrist_z", prefix * -100., .1)
        # else:
        #     robot.setAngle(arm + "_shoulder_z", prefix * -10., .03)
        #     robot.setAngle(arm + "_shoulder_y", prefix * -45., .03)
        #     robot.setAngle(arm + "_elbow_y", prefix * 45., .03)
        #     robot.setAngle(arm + "_arm_x", prefix * 15., .03)
        #     robot.setHandPose(arm.upper() + "Hand", "pointAt")

time.sleep(6.)
face.sendFaceExpression("neutral")
initial_position(robot)
time.sleep(2.)

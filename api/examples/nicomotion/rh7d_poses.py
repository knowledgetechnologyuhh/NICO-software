from nicomotion import Motion
import time
from os.path import dirname, abspath

robot = Motion.Motion(dirname(abspath(__file__))+"/../../../json/rh7d_hands.json",vrep=False)

robot.thumbsUp("LHand")
robot.pointAt("RHand")
#robot.closeHand("LHand")
#robot.okSign("LHand")
#robot.pinchToIndex("LHand")
#robot.pencilGrip("RHand")
#robot.keyGrip("LHand")
time.sleep(10.0)
robot.openHand("LHand")
robot.openHand("RHand")

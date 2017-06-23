from nicomotion import Motion
import time
from os.path import dirname, abspath

robot = Motion.Motion(dirname(abspath(__file__))+"/../../../json/rh7d_hands.json",vrep=False)

#robot.thumbsUp("RHand")
#robot.pointAt("RHand")
#robot.okSign("RHand")
#robot.pinchToIndex("RHand")
#robot.keyGrip("RHand")
#robot.pencilGrip("RHand")
robot.closeHand("RHand")
time.sleep(10.0)
robot.openHand("RHand")
#robot.openHand("LHand")

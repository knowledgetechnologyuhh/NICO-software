from nicomotion import Motion
import time
from os.path import dirname, abspath

robot = Motion.Motion(dirname(abspath(__file__))+"/../../../json/rh7d_hands.json",vrep=False)

#robot.thumbsUp("LHand")
robot.pointAt("RHand")
#robot.okSign("RHand")
#robot.pinchToIndex("RHand")
#robot.keyGrip("RHand")
#robot.pencilGrip("RHand")
#print("Closing hand")
#robot.closeHand("RHand")

time.sleep(10.0)
#robot.openHand("RHand")
print("Opening hand")
robot.openHand("RHand")
time.sleep(10.0)

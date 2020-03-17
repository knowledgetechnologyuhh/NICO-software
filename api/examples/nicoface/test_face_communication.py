import logging
import time

from nicoface.FaceExpression import faceExpression
from nicomotion.Motion import Motion

logging.basicConfig(level=logging.DEBUG)
logging.getLogger("pypot.dynamixel.io.abstract_io").setLevel(logging.WARNING)

face = faceExpression()

face.setCommMode(2)

time.sleep(1)

face.sendFaceExpression("happiness")

time.sleep(1)

face.sendFaceExpression("clear")

time.sleep(1)

face.sendTrainedFaceExpression("happiness")

time.sleep(1)

motion = Motion("../../../json/nico_humanoid_upper.json")
del motion

face.sendFaceExpression("happiness")

time.sleep(1)

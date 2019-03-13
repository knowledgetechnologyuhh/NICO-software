from nicoface.FaceExpression import faceExpression
from nicomotion.Motion import Motion
import time

face = faceExpression()

face.setCommMode(2)

face.sendTrainedFaceExpression("Happy")

time.sleep(1)

face.sendFaceExpression("neutral")

time.sleep(1)

motion = Motion("../../../../json/nico_humanoid_upper.json")

face.sendTrainedFaceExpression("Happy")



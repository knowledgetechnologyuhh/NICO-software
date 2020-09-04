# works only with pillow library ==2.3.0
# pip install pillow==2.3.0

from nicoface.FaceExpression import faceExpression
from time import sleep
from random import choice
import sys
import logging

logging.basicConfig(level=logging.INFO)

if sys.version_info >= (3,):
    raw_input = input

fe = faceExpression()
sleep(1)

# trained expressions
for expression in (("anger"), ("happiness"), ("neutral"), ("sadness"), ("surprise")):
    print("Showing " + expression)
    key = "b"
    face_choice = choice([True, False])
    while key == "b":
        if face_choice:
            print("Trained first")
            fe.sendTrainedFaceExpression(expression)
            raw_input()
            fe.sendFaceExpression(expression)
        else:
            print("Designed first")
            fe.sendFaceExpression(expression)
            key = raw_input()
            fe.sendTrainedFaceExpression(expression)
        key = raw_input()
        # sleep(1)

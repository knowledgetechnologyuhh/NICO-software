from time import sleep
import argparse

from nicoface.FaceExpression import faceExpression

parser = argparse.ArgumentParser()
parser.add_argument(
    "-s",
    action="store_true",
    help="Enables simulated mode, where faces are shown as image instead",
)
args = parser.parse_args()

if args.s:
    fe = faceExpression(simulation=True)
else:
    fe = faceExpression()
    # handcoded presets
    # 'happiness','sadness','anger','disgust','surprise','fear','neutral','clear'
    for expression in (
        "happiness",
        "sadness",
        "anger",
        "disgust",
        "surprise",
        "fear",
        "neutral",
        "clear",
    ):
        fe.sendFaceExpression(expression)  # only works with a real robot
        sleep(1)

# morphable face presets
# 'happiness','sadness','anger','disgust','surprise','fear','neutral'
for expression in (
    "happiness",
    "sadness",
    "anger",
    "disgust",
    "surprise",
    "fear",
    "neutral",
):
    fe.send_morphable_face_expression(expression)
    sleep(1)

# trained expressions
for expression in ("anger", "happiness", "neutral", "sadness", "surprise"):
    fe.sendTrainedFaceExpression(expression)
    sleep(1)

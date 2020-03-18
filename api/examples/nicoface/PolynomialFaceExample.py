from nicoface.FaceExpression import faceExpression
import time
import logging
import argparse

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__file__)

parser = argparse.ArgumentParser()
parser.add_argument(
    "-s",
    action="store_true",
    help="Enables simulated mode, where faces are shown as image instead",
)
args = parser.parse_args()

face = faceExpression(simulation=args.s)

# generate face in one call
face.send_polynomial_face(
    # mouth
    [7, 0, -0.25, 0, 0],
    [0, 0, 0.1, 0, 0],
    7.45,
    3,
    3,
    # left eyebrow
    [2, 0, 0.2, 0, 0],
    3.45,
    0,
    0,
    # right eyebrow
    [2, 0, 0.2, 0, 0],
    3.45,
    0,
    0,
)
time.sleep(4)

# generate individual face parts seperately
face.generate_polynomial_mouth(
    [4, 1.75, 0, -0.05, 0], [4, 1.75, 0, -0.05, 0], 7.45, 0, 0
)
face.generate_polynomial_eyebrow([1, 0, 0.075, 0, 0], 0, 0, 0, left=True)
face.generate_polynomial_eyebrow([4, -0.75, 0.05, 0, 0], 0, 0, 0, left=False)
face.send()

time.sleep(4)

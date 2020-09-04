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

logger.info("Morphing between face presets")

# display morphable face expression to initialize morphable polynomial face
face.send_morphable_face_expression("neutral")
time.sleep(2)

# morph between face expressions modelled after default presets
for preset in (
    "happiness",
    "sadness",
    "anger",
    "disgust",
    "surprise",
    "fear",
    "neutral",
):
    face.morph_face_expression(preset, steps=3, delay=0.0)
    time.sleep(2)

logger.info("Morphing between custom polynomial faces")

# morph between custom polynomial faces
face.morph_polynomial_face(
    m1_target=[4, -1.75, 0, 0.05, 0],
    m2_target=[4, -1.75, 0, 0.05, 0],
    m_x_shift_target=7.45,
    m_crop_left_target=0,
    m_crop_right_target=0,
    left_target=[4, -0.75, 0.05, 0, 0],
    l_x_shift_target=0,
    l_crop_left_target=0,
    l_crop_right_target=0,
    right_target=[1, 0, 0.075, 0, 0],
    r_x_shift_target=0,
    r_crop_left_target=0,
    r_crop_right_target=0,
    steps=3,
    delay=0.0,
)
time.sleep(2)
face.morph_polynomial_face(
    [4, 1.75, 0, -0.05, 0],
    [4, 1.75, 0, -0.05, 0],
    7.45,
    0,
    0,
    [1, 0, 0.075, 0, 0],
    0,
    0,
    0,
    [4, -0.75, 0.05, 0, 0],
    0,
    0,
    0,
    steps=3,
    delay=0.0,
)
time.sleep(2)

logger.info("Morphing between trained presets")

# initialize wavelet face
face.sendTrainedFaceExpression("neutral")
time.sleep(2)
# morph between trained wavelet faces
for preset in ("happiness", "sadness", "anger", "surprise", "neutral"):
    face.morph_face_expression(preset, steps=3, delay=0.0)
    time.sleep(2)

logger.info("Morphing between custom wavelet faces")

# morph between custom wavelet faces
face.morph_wavelet_face(
    (1.0, -0.6, 0.9, 0.02),
    (1.0, -0.6, 0.9, 0.02),
    (-0.5, 0.2, 1.0, 0.0),
    (-0.5, 0.2, 1.0, 0.0),
    steps=3,
    delay=0.0,
)
time.sleep(2)
face.morph_wavelet_face(
    (0.6, 0.1, 1.0, 0.025),
    (-1.1, 0.2, 0.95, 0.025),
    (1.2, -0.5, 1.0, 0.0),
    (1.2, -0.5, 1.0, 0.0),
    steps=3,
    delay=0.0,
)
time.sleep(2)

# Author: Erik
#
# Example for generating different face expression with wavelets
#
from nicoface.FaceExpression import faceExpression
import time
import argparse

parser = argparse.ArgumentParser()
parser.add_argument(
    "-s",
    action="store_true",
    help="Enables simulated mode, where faces are shown as image instead",
)
args = parser.parse_args()

fe = faceExpression(simulation=args.s)

# Generate the mouth form and eyebrowse
# Using the standard values
fe.gen_mouth()
fe.gen_eyebrowse()
fe.gen_eyebrowse(type="r")
fe.send()

time.sleep(1.5)

# Mouth Generator contain 1 tuple for each mouth wavelet
# (ystretch,yoffset,xstretch,xoffset)
fe.gen_mouth((-0.2, -0.2, 1.0, 0), (None, None, None, None))

# This can be displayed on the screen
# fe.show_PIL(fe.mouth)

# And send to the NICO
fe.send()

time.sleep(1.5)

# Using both tuples for two mouth lines
fe.gen_mouth((-1.0, 0.2, 1.0, 0), (0.4, 0.1, 1.0, 0))

fe.send()

time.sleep(1.5)

# Using both tuples for two mouth lines
fe.gen_mouth((1.0, -0.8, 1.0, 0), (None, None, None, None))
# And using the eyebrowse
fe.gen_eyebrowse((-1.2, 0.2, 1.0, 0), type="l")
fe.gen_eyebrowse(type="r")

fe.send()

time.sleep(1.5)

# combined CALLDIR

fe.send_wavelet_face(
    # mouth
    (0.6, 0.1, 1.0, 0.025),
    (-1.1, 0.2, 0.95, 0.025),
    # left eyebrow
    (1.2, -0.5, 1.0, 0.0),
    # right eyebrow
    (1.2, -0.5, 1.0, 0.0),
)

time.sleep(1.5)

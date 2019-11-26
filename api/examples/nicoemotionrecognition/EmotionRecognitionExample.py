# -*- coding: utf-8
import argparse
import logging
import sys
import time
from os.path import abspath, dirname

from nicoemotionrecognition import EmotionRecognition
from nicoface.FaceExpression import faceExpression
from nicomotion import Motion
from nicovision.VideoDevice import VideoDevice

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

if sys.version_info >= (3,):
    raw_input = input

parser = argparse.ArgumentParser(
    description=(
        "NICO looks at a detected face, recognizes and mirrors the emotion "
        "that is shown and says different phrases depending on that emotion."
    )
)
default_config = abspath(
    dirname(abspath(__file__)) + "/../../../json/nico_humanoid_upper_rh7d.json"
)
parser.add_argument(
    "--motor-config",
    dest="json_file",
    type=str,
    default=default_config,
    help="Location of json file with motor definitions. (Default: {})".format(
        default_config
    ),
)
parser.add_argument(
    "--german",
    action="store_true",
    help="Use german phrases for voice feedback (default is english)",
)

parser.add_argument(
    "--disable-voice",
    dest="voice",
    action="store_false",
    help="Disables voice feedback.",
)

parser.add_argument(
    "--disable-motion",
    dest="motion",
    action="store_false",
    help="Disables head movement",
)

parser.add_argument(
    "--disable-gui", dest="gui", action="store_false", help="Disables the GUI."
)

args = parser.parse_args()

robot = None
if args.motion:
    robot = Motion.Motion(args.json_file, ignoreMissing=True)
face = faceExpression()
# torso NICO

camera_path = VideoDevice.autodetect_nicoeyes()[0]  # 0: left_eye, 1: right_eye

logger.info("Using camera %s", camera_path)

camera = VideoDevice.from_device(camera_path)

if "See3CAM" in camera_path:
    camera.zoom(300)

emotionRecogniton = EmotionRecognition.EmotionRecognition(
    device=camera, robot=robot, face=face, voiceEnabled=args.voice, german=args.german
)

emotionRecogniton.start(showGUI=args.gui, faceTracking=args.motion, mirrorEmotion=True)
raw_input("Press enter to stop\n")
# print emotionRecogniton.getDimensionalData()
# print emotionRecogniton.getCategoricalData()

if args.motion:
    robot.disableTorqueAll()
emotionRecogniton.stop()

time.sleep(1.0)

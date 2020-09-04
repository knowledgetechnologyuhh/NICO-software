import argparse
from os import mkdir
from os.path import dirname, abspath, isdir
from nicomotion.Motion import Motion
from nicovision.PyrepRecorder import PyrepRecorder
import cv2

# define directories of config and scene
parent_dir = dirname(dirname(dirname(dirname(abspath(__file__)))))
config = "{}/json/nico_humanoid_vrep.json".format(parent_dir)
scene = "{}/v-rep/NICO-seated-with-table.ttt".format(parent_dir)

# create folders
if not isdir("images"):
    mkdir("images")
if not isdir("videos"):
    mkdir("videos")

# setup argument parser
parser = argparse.ArgumentParser()
parser.add_argument(
    "-g",
    "--gray",
    action="store_true",
    help="Converts the output images into grayscale",
)
parser.add_argument(
    "-s", "--show_ui", action="store_false", help="Disables headless mode"
)

args = parser.parse_args()

# setup robot using pyrep
pyrep_config = Motion.pyrepConfig()
pyrep_config["vrep_scene"] = scene
pyrep_config["headless"] = args.show_ui
robot = Motion(config, vrep=True, vrepConfig=pyrep_config)

# NOTE If you don't want to use Motion you can also launch your own pyrep
# instance (see https://github.com/stepjam/PyRep):
# pr = PyRep()
# pr.launch(scene, headless=True)

# initialize recorder with sensor_names
rec = PyrepRecorder(["left_eye", "right_eye"])
if args.gray:

    def preprocess_image(img):
        """
        Example preprocess function that converts the images to grayscale.
        """
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    rec.preprocess_image = preprocess_image  # override preprocess function

# start simulation and recording
robot.startSimulation()
# record image at each simulation step
rec.start_image_recording("images/{sensor_name}_{timestamp}.png")
# record video including each step (framerate determined by step deltatime)
rec.start_video_recording("videos/{sensor_name}_{timestamp}.avi", fourcc="DIVX")

# robot will look at the table
robot.setAngle("head_y", 45, 0.1)

for i in range(60):
    # each simulation step, an image is recorded for each sensor
    robot.nextSimulationStep()
    if i == 5:
        # take image independant from step call
        rec.take_one_image("images/{sensor_name}_{timestamp}_single.png")

# stop the recording
rec.stop_image_recording()
rec.stop_video_recording()

robot.nextSimulationStep()  # this should not create any more images

# stop the simulation and use cleanup to close
robot.stopSimulation()
robot.cleanup()

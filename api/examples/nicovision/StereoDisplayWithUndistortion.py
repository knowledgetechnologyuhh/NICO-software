import logging
from os.path import abspath, dirname

from nicovision.Display import Display

logging.basicConfig(level=logging.INFO)
display = Display(cam_width=1920, cam_height=1080, framerate=30,
                  window_width_per_cam=768, window_height=432, zoom=200,
                  calibration_file=(
                      dirname(abspath(__file__)) +
                      "/../../../json/" +
                      "nico_vision_calibration_params.json"),
                  undistortion_mode="stereo")
input("Press enter to close the display")
display.close()

import logging
import threading
import time
from os.path import abspath, dirname

import cv2
import numpy as np
from nicovision.Display import Display
from nicovision.MultiCamRecorder import MultiCamRecorder


def disparity_thread(self, stop_event, window_width_per_cam=640,
                     window_height=480):
    # https://github.com/bnascimento/opencv-2.4/blob/master/samples/python2/stereo_match.py
    window_size = 7
    min_disp = 16
    num_disp = 112 - min_disp
    while not stop_event.is_set():
        resized_frames = [cv2.resize(
            frame, (window_width_per_cam, window_height))
            for frame in self._current_frames]

        new_frames = [cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                      for frame in resized_frames]
        stereo = cv2.StereoSGBM_create(minDisparity=min_disp,
                                       numDisparities=num_disp,
                                       blockSize=window_size,
                                       uniquenessRatio=15,
                                       speckleWindowSize=200,
                                       speckleRange=2,
                                       disp12MaxDiff=1,
                                       P1=8 * 3 * window_size**2,
                                       P2=32 * 3 * window_size**2,
                                       # mode=cv2.STEREO_SGBM_MODE_HH
                                       )
        disp = stereo.compute(*new_frames).astype(np.float32) / 16.0
        cv2.imshow("cameras", cv2.hconcat(resized_frames))
        cv2.imshow("disparity", (disp - min_disp) / num_disp)
        cv2.waitKey(1)


logging.basicConfig(level=logging.INFO)
window_width_per_cam = 768
window_height = 432

Display._display_thread = disparity_thread

display = Display(cam_width=1920, cam_height=1080, framerate=30,
                  window_width_per_cam=window_width_per_cam,
                  window_height=window_height, zoom=200,
                  calibration_file=(
                      dirname(abspath(__file__)) +
                      "/../../../json/" +
                      "nico_vision_calibration_params.json"),
                  undistortion_mode="stereo")

input("Press enter to close the display")

display.close()

import json
import logging
import threading
import time
from math import ceil
from os.path import abspath, dirname, isfile

import cv2
import numpy as np

import Barrier
import MultiCamRecorder
from NumpyEncoder import NumpyEncoder


class Display():
    """The Display Class displays multiple cameras in one window"""

    def __init__(self, devices=[], cam_width=640, cam_height=480,
                 window_width_per_cam=640, window_height=480, framerate=20,
                 zoom=None, pan=None, tilt=None, settings_file=None,
                 setting="standard", pixel_format="MJPG",
                 calibration_file=None, undistortion_mode="mono"):
        """
        Initialises the Display with given devices.

        The devices must be contained in :meth:`get_devices`

        :param devices: Device names (autodetected if empty list)
        :type device: list(str)
        :param cam_width: Width of camera image(s)
        :type cam_width: float
        :param cam_height: Height of camera image(s)
        :type cam_height: float
        :param window_width_per_cam: Adjusted width of the displayed image(s)
        :type window_width_per_cam: float
        :param window_height: Adjusted height of the displayed image(s)
        :type window_height: float
        :param framerate: number of frames per second
        :type framerate: int
        :param value: zoom value between 100 and 800 (overwrites settings)
        :type value: int
        :param value: pan value between -648000 and 648000, step 3600
        (overwrites settings)
        :type value: int
        :param value: tilt value between -648000 and 648000, step 3600
        (overwrites settings)
        :type value: int
        :param setttings_file: the settings file
        :type settings_file: str
        :param setting: name of the setting that should be applied
        :type setting: str
        :param writer_threads: Number of worker threads for image writer
        :type writer_threads: int
        :param pixel_format: fourcc codec
        :type pixel_format: string
        :param calibration_file: the calibration_file file
        :type calibration_file: str
        :param undistortion_mode: mono or stereo undistortion
        :type undistortion_mode: str
        """
        self._dim = cam_width, cam_height
        self._recorder = MultiCamRecorder.MultiCamRecorder(devices, cam_width,
                                                           cam_height,
                                                           framerate,
                                                           zoom, pan, tilt,
                                                           settings_file,
                                                           setting, 0,
                                                           pixel_format,
                                                           calibration_file,
                                                           undistortion_mode)
        self._deviceIds = self._recorder._deviceIds
        self._current_frames = [
            np.zeros((window_height, window_width_per_cam, 3),
                     np.uint8)] * len(self._deviceIds)
        self._open = False
        self.open(window_width_per_cam, window_height)

    def __del__(self):
        cv2.destroyAllWindows()

    def _display_thread(self, stop_event, window_width_per_cam=640,
                        window_height=480):
        while not stop_event.is_set():
            resized_frames = [cv2.resize(
                frame, (window_width_per_cam, window_height))
                for frame in self._current_frames]
            cv2.imshow("cameras", cv2.hconcat(resized_frames))
            cv2.waitKey(1)

    def _callback(self, rval, frame, id):
        if (rval):
            self._current_frames[id] = frame

    def open(self, window_width_per_cam=640, window_height=480):
        """
        Opens the display
        :param window_width_per_cam: Adjusted width of the displayed image(s)
        :type window_width_per_cam: float
        :param window_height: Adjusted height of the displayed image(s)
        :type window_height: float
        """
        if not self._open:
            self._stop_event = threading.Event()
            self._display = threading.Thread(
                target=self._display_thread, args=(self._stop_event,
                                                   window_width_per_cam,
                                                   window_height))
            self._display.daemon = True
            self._display.start()
            self._recorder.add_callback(self._callback)
            self._recorder._open = True
            self._open = True

    def close(self):
        """
        Closes the display
        """
        if self._open:
            self._recorder.stop_recording()
            self._stop_event.set()
            self._display.join()
            time.sleep(1)
            cv2.destroyAllWindows()
            self._open = False


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    display = Display(cam_width=1920, cam_height=1080, framerate=10,
                      window_width_per_cam=640, window_height=360,
                      calibration_file=(
                          dirname(abspath(__file__)) +
                          "/../../../../../json/" +
                          "nico_vision_calibration_params.json"),
                      undistortion_mode="stereo")
    input("Press enter to close the display")
    display.close()

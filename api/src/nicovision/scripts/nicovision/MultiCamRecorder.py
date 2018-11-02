import datetime
import inspect
import json
import logging
import os
import subprocess
import sys
import threading
import time

import cv2

import Barrier
from ImageWriter import ImageWriter
from VideoDevice import VideoDevice


def autodetect_nicoeyes():
    """
    Returns a tuple containing the detected path of left and right NICO eye
    camera

    :return: Devicenames as (left, right) tuple (None if not found)
    :rtype: tuple
    """
    return VideoDevice.autodetect_nicoeyes()


def get_devices():
    """
    Returns a list containing the possible path of all video capturing devices

    :return: Video devices
    :rtype: list
    """
    return VideoDevice.get_all_devices()


class MultiCamRecorder(object):
    """
    The MultiCamRecorder class enables simultanious capturing of images from
    multiple cameras.
    """

    def __init__(self, devices=[], width=640, height=480, framerate=20,
                 zoom=None, pan=None, tilt=None, settings_file=None,
                 setting="standard", writer_threads=4,
                 pixel_format="MJPG", calibration_file=None,
                 undistortion_mode="mono"):
        """
        Initialises the MultiCamRecorder with given devices.

        The devices must be contained in :meth:`get_devices`

        :param devices: Device names (autodetected if empty list)
        :type device: list(str)
        :param width: Width of image
        :type width: float
        :param height: Height of image
        :type height: float
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
        self._logger = logging.getLogger(__name__)
        if not devices:
            devices = autodetect_nicoeyes()
        self._deviceIds = []
        for device in devices:
            deviceId = VideoDevice.resolve_device(device)
            if deviceId == -1:
                self._logger.error(
                    'Can not create device from path' + self._device)
                sys.exit()
            self._deviceIds.append(deviceId)

        self._open = False
        self._target = 'picture-{}.png'
        self._image_writer = None
        if writer_threads > 0:
            self._image_writer = ImageWriter(writer_threads)
        self._callback_functions = []
        self._framerate = framerate
        self._width = width
        self._height = height
        self._pixel_format = pixel_format
        if settings_file is not None:
            self.load_settings(settings_file, setting)
        if zoom is not None:
            self.zoom(zoom)
        if pan is not None:
            self.pan(pan)
        if tilt is not None:
            self.tilt(tilt)

        self._rectify_maps = [None] * len(self._deviceIds)
        if calibration_file:
            self.load_callibration(calibration_file, undistortion_mode)

        # Open cameras
        self.open()

    def undistort(self, frame, id):
        """
        Undistorts the frame with the loaded calibration

        :param frame: frame to undistort
        :type frame: cv2 image
        :param id: camera id
        :type id: int
        :return: undistorted frame
        :rtype: cv2 image
        """
        if self._rectify_maps[id]:
            frame = cv2.remap(frame,
                              self._rectify_maps[id][0],
                              self._rectify_maps[id][1],
                              interpolation=cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_CONSTANT)
        return frame

    def load_callibration(self, file_path, undistortion_mode="mono"):
        """
        Loads a calibration json file and prepares undistortion for all cameras

        :param file_path: the calibration file
        :type file_path: str
        :param undistortion_mode: mono or stereo undistortion
        :type undistortion_mode: str
        """
        if mode == "stereo" and len(self._deviceIds != 2):
            self._logger.error(("Stereo undistortion requires exactly 2 " +
                                "devices {} devices initialized"
                                ).format(len(self._deviceIds)))
            return
        # load file
        self._logger.info("Loading calibration file {}".format(file_path))
        if os.path.isfile(file_path):
            with open(file_path, 'r') as file:
                calibration = json.load(file)[undistortion_mode]
        else:
            self._logger.error(
                ("Calibration file {} does not exist").format(file_path))
            return
        # prepare rectify maps
        self._logger.debug("Preparing rectify map")
        devicenames = get_devices()
        devicenames = map(lambda i: devicenames[i], self._deviceIds)
        dim = self._width, self._height
        if undistortion_mode == "stereo":
            if (str(devicenames) in calibration and
                    str(dim) in calibration[str(devicenames)]):
                K_left, D_left, K_right, D_right, R, T, R_left, R_right, \
                    P_left, P_right, Q = map(lambda a: np.array(a),
                                             itemgetter("K_left", "D_left",
                                                        "K_right",
                                                        "D_right", "R", "T",
                                                        "R_left", "R_right",
                                                        "P_left", "P_right",
                                                        "Q")(calibration[
                                                            str(devicenames)][
                                                            str(DIM)]))
                self._rectify_maps[0] = cv2.fisheye.initUndistortRectifyMap(
                    K_left, D_left, R_left, P_left, dim, cv2.CV_16SC2)
                self._rectify_maps[1] = cv2.fisheye.initUndistortRectifyMap(
                    K_right, D_right, R_right, P_right, dim, cv2.CV_16SC2)

        else:
            for i in range(len(devicenames)):
                if (str(devicenames[i]) in calibration and
                        str(dim) in calibration[str(devicenames[i])]):
                    K, D = map(lambda a: np.array(a), itemgetter(
                        "K", "D")(calibration[devicenames[i]][str(dim)]))
                    self._rectify_maps[i] = \
                        cv2.fisheye.initUndistortRectifyMap(
                        K, D, np.eye(3), K, dim, cv2.CV_16SC2)

    def load_settings(self, file_path, setting="standard"):
        """
        Loads a settings json file and applies the given setting to all cameras

        :param file_path: the settings file
        :type file_path: str
        :param setting: name of the setting that should be applied
        :type setting: str
        """
        with open(file_path, 'r') as file:
            settings = json.load(file)
        for value_name, value in settings[setting].iteritems():
            self.camera_value(value_name, value)

    def camera_value(self, value_name, value):
        """
        Sets the a camera value over the v4l-utils. Run 'v4l2-ctl -l' for full
        list of controlls and values. Requires v4l-utils.

        :param value_name: name of the value to set
        :type value_name: str
        :param value: value to set
        :type value: int
        """
        if type(value) is int:
            for id in self._deviceIds:
                subprocess.call(
                    ['v4l2-ctl -d {} -c {}={}'.format(
                        id, value_name, value)], shell=True)
        else:
            self._logger.warning(
                "Invalid value '{}' - value has to be an integer".format(value)
            )

    def zoom(self, value):
        """
        Sets zoom value of all cameras that support it. Requires v4l-utils.

        :param value: zoom value between 100 and 800
        :type value: int
        :return: success
        :rtype: bool
        """
        if type(value) is int and 100 <= value <= 800:
            for id in self._deviceIds:
                subprocess.call(
                    ['v4l2-ctl -d {} -c zoom_absolute={}'.format(id, value)],
                    shell=True)
            return True
        else:
            self._logger.warning(
                "Zoom value has to be an integer between 100 and 800")
            return False

    def pan(self, value):
        """
        Sets pan (x-axis) value of all cameras that support it. Requires
        v4l-utils.

        :param value: pan value between -648000 and 648000, step 3600
        :type value: int
        :return: success
        :rtype: bool
        """
        if(type(value) is int and -648000 <= value <= 648000 and
           value % 3600 == 0):
            for id in self._deviceIds:
                subprocess.call(
                    ['v4l2-ctl -d {} -c pan_absolute={}'.format(id, value)],
                    shell=True)
            return True
        else:
            self._logger.warning(
                "Pan value has to be a multiple of 3600 between -648000 and " +
                "648000")
            return False

    def tilt(self, value):
        """
        Sets tilt (y-axis) value of all cameras that support it. Requires
        v4l-utils.

        :param value: tilt value between -648000 and 648000, step 3600
        :type value: int
        :return: success
        :rtype: bool
        """
        if (type(value) is int and -648000 <= value <= 648000 and
                value % 3600 == 0):
            for id in self._deviceIds:
                subprocess.call(
                    ['v4l2-ctl -d {} -c tilt_absolute={}'.format(id, value)],
                    shell=True)
            return True
        else:
            self._logger.warning(
                "Tilt value has to be a multiple of 3600 between -648000 and" +
                " 648000")
            return False

    def add_callback(self, function):
        """
        Adds a callback for the event loop

        Whenever a new frame arrives, all registered callbacks are called.

        The callback must take exactly 2 arguments: rval and frame. frame can
        be None if any error occures or no image could be grabbed.

        :param function: Function to add as callback
        :type function: function
        """
        if not (inspect.isfunction(function) or inspect.ismethod(function)):
            self._logger.warning('Trying to add non-function callback')
            return
        self._callback_functions += [function]

    def clean_callbacks(self):
        """
        Removes all saved callbacks
        """
        self._callback = []

    def open(self):
        if not self._open:
            # Open cameras
            self._captures = []
            for id in self._deviceIds:
                capture = cv2.VideoCapture(id)
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
                capture.set(cv2.CAP_PROP_FPS, self._framerate)
                fourcc = cv2.VideoWriter_fourcc(*self._pixel_format)
                capture.set(cv2.CAP_PROP_FOURCC, fourcc)
                self._captures.append(capture)
            self._open = True

            # Start thread
            self._barrier = Barrier.Barrier(len(self._deviceIds))
            self._threads = []
            for id in range(len(self._deviceIds)):
                self._threads.append(threading.Thread(
                    target=self._eventloop, args=(id,)))
                self._threads[id].daemon = True
                self._threads[id].start()

    def start_recording(self, path="camera{}/picture-{}.png"):
        """
        Starts continuous image recording.

        :param path: path of the image. First {} is replaced with camera id,
                     the second one with a timestamp.
        :type path: str
        """
        if ImageWriter is not None:
            if not self._image_writer._open:
                self._image_writer.open()
            self._target = path
            self.open()
            self.add_callback(self._callback)
        else:
            self._logger.warning(
                "Could not start recording - No ImageWriter instantiated")

    def stop_recording(self):
        """
        Stops the recording and closes the cameras
        """
        if not self._open:
            self._logger.warning('Trying to close a device which is not open')
            return
        self._barrier.abort()
        self._open = False
        map(lambda t: t.join(), self._threads)
        map(lambda c: c.release(), self._captures)
        self.clean_callbacks()
        self._barrier.reset()
        if self._image_writer is not None:
            self._image_writer.close()

    def enable_write(self, state=True):
        """
        Sets the writing to disk state

        :param state: Write enabled
        :type value: bool
        """
        self._image_writer.enable_write(state)

    def custom_callback(self, iso_time, frame, id):
        # Option to create a custom function, that modifies the frame before
        # saving
        return frame

    def _callback(self, rval, frame, id):
        """
        Internal callback

        :param rval: rval
        :param frame: frame
        """
        if rval:
            iso_time = datetime.datetime.today().isoformat()

            frame = self.custom_callback(
                datetime.datetime.today().isoformat(), frame, id)

            self._image_writer.write_image(self._target.format(id, iso_time),
                                           frame)

    def _singleImageCallback(self, rval, frame, id):
        """
        Internal callback for taking a single image

        :param rval: rval
        :param frame: frame
        """
        if rval and self._once[id]:
            iso_time = datetime.datetime.today().isoformat()

            frame = self.custom_callback(
                datetime.datetime.today().isoformat(), frame, id)

            self._image_writer.write_image(self._target.format(id, iso_time),
                                           frame)
            self._once[id] = False

    def takeSingleImage(self, path="camera{}/picture-{}.png"):
        """
        Takes a single image with each camera. For performance reasons this
        does not close the camera. Please call stop_recording() once if you
        don't take any more images or want to delete the object.

        :param path: path of the image. First {} is replaced with camera id,
                     the second one with a timestamp.
        :type path: str
        """
        if ImageWriter is not None:
            self._once = [True] * len(self._deviceIds)
            self._target = path
            if not self._image_writer._open:
                self._image_writer.open()
            self.open()
            self.add_callback(self._singleImageCallback)
            while reduce(lambda x, y: x and y, self._once):
                continue
            self.clean_callbacks()
        else:
            self._logger.warning(
                "Could not start recording - No ImageWriter instantiated")

    def _eventloop(self, id):
        """
        Internal event loop

        This will call all registered callbacks for each frame
        """
        while self._open:
            t1 = time.time()
            try:
                self._barrier.wait()
            except Barrier.BrokenBarrierError as e:
                break
            self._captures[id].grab()
            rval, frame = self._captures[id].retrieve()
            if rval:
                frame = self.undistort(frame, id)
            for function in self._callback_functions:
                function(rval, frame, id)
            time.sleep(max(0, 1.0 / self._framerate - (time.time() - t1)))

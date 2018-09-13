# TODO
# Erik
# get rid of the java style get and set functions and
# replace it with python style properties

import inspect
import json
import logging
import os
import subprocess
import threading
import time

import cv2

VIDEO_DEVICE_PATH = "/dev/v4l/by-id/"
ID_STR_LEGGED_NICO_LEFT_CAM = "usb-e-con_systems_See3CAM_CU135_09229807-video-index0"
ID_STR_LEGGED_NICO_RIGHT_CAM = "usb-e-con_systems_See3CAM_CU135_36249807-video-index0"
PATH_LEGGED_NICO_LEFT_CAM = VIDEO_DEVICE_PATH + ID_STR_LEGGED_NICO_LEFT_CAM
PATH_LEGGED_NICO_RIGHT_CAM = VIDEO_DEVICE_PATH + ID_STR_LEGGED_NICO_RIGHT_CAM

NICO_EYES = {
    'left': {
        'old': "usb-046d_080a_2DE7B460-video-index0",
        'new': "usb-e-con_systems_See3CAM_CU135_09229807-video-index0"
    },
    'right': {
        'old': 'usb-046d_080a_17E79161-video-index0',
        'new': "usb-e-con_systems_See3CAM_CU135_36249807-video-index0"
    }
}


class VideoDevice:
    """
    The VideoDevice class handles low-level communication with the video
    devices
    """

    _VIDEO_DEVICE_PATH = '/dev/v4l/by-id/'
    """
    This variable holds the base path for the ids of the capturing devices
    """

    @staticmethod
    def get_all_devices():
        """
        Returns a list containing the possible path of all video capturing
        devices

        :return: Paths of video devices
        :rtype: list
        """
        if not os.path.isdir(VideoDevice._VIDEO_DEVICE_PATH):
            logging.error('Video device path does not exists!')
            return []
        paths = []
        for file in os.listdir(VideoDevice._VIDEO_DEVICE_PATH):
            paths += [file]
        return paths

    @staticmethod
    def autodetect_nicoeyes():
        """
        Returns a tuple containing the detected path of left and right NICO eye
        camera

        :return: Devicenames as (left, right) tuple (None if not found)
        :rtype: tuple
        """
        left, right = None, None
        devices = VideoDevice.get_all_devices()
        for _, device in NICO_EYES['left'].iteritems():
            if device in devices:
                left = device
                break
        for _, device in NICO_EYES['right'].iteritems():
            if device in devices:
                right = device
                break
        return (left, right)

    @staticmethod
    def resolve_device(device):
        """
        Returns the id of a device

        :param device: device
        :type device: str
        :return: device id (-1 = error)
        :rtype: int
        """
        if not os.path.isdir(VideoDevice._VIDEO_DEVICE_PATH):
            logging.error('Video device device does not exists!')
            return -1

        candidates = []
        for file in os.listdir(VideoDevice._VIDEO_DEVICE_PATH):
            if device in file:
                candidates += [file]

        if len(candidates) is 0:
            logging.error('No candidates found')
            return -1
        elif len(candidates) is 1:
            return int(os.readlink(
                VideoDevice._VIDEO_DEVICE_PATH + candidates[0])[-1:])
        else:
            logging.error('Multiple candidates found: {}'.format(candidates))
            return -1

    @staticmethod
    def from_device(device, framerate=20, width=640, height=480, zoom=None,
                    pan=None, tilt=None, settings_file=None,
                    setting="standard", compressed=True, pixel_format="MJPG",
                    calibration_file=None):
        """
        Convenience method for creating a VideoDevice from a device

        :param device: device
        :type device: str
        :return: VideoDevice or None if path is not valid / ambiguous
        :rtype: VideoDevice or None
        """
        id = VideoDevice.resolve_device(device)
        if id is -1:
            logging.error('Can not create VideoDevice from ID %s' % id)
            return None
        return VideoDevice(id, framerate, width, height, zoom, pan, tilt,
                           settings_file, setting, compressed, pixel_format,
                           calibration_file)

    def __init__(self, id, framerate=20, width=640, height=480, zoom=None,
                 pan=None, tilt=None, settings_file=None, setting="standard",
                 compressed=True, pixel_format="MJPG", calibration_file=None):
        """
        Initialises the VideoDevice. The device starts open and has to be
        opened.

        If you want to create a VideoDevice from a (partial) ID, use
        :meth:`nicovision.VideoDevice.fromDevice` instead.

        :param id: device id
        :type id: int
        :param uncompressed: using compressed or uncompressed stream of the
        camera
        :type uncompressed: boolean
        :param pixel_format: pixel format like 'UYVY' (econ-camera) or 'YUYV'
        (logitech) or 'MJPG' (both compressed)
        """
        self._logger = logging.getLogger(__name__)
        self._deviceId = id
        self._open = True
        self._callback = []
        self._running = True
        self._framerate = framerate
        self._width = width
        self._height = height
        if settings_file is not None:
            self.load_settings(settings_file, setting)
        if zoom is not None:
            self.zoom(zoom)
        if pan is not None:
            self.pan(pan)
        if tilt is not None:
            self.tilt(tilt)
        self._rectify_map = None
        if calibration_file:
            self.load_callibration(calibration_file)

        # Open camera
        self._capture = cv2.VideoCapture(id)
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self._capture.set(cv2.CAP_PROP_FPS, self._framerate)

        fourcc = cv2.VideoWriter_fourcc(*pixel_format)
        # fourcc = cv2.VideoWriter_fourcc(*'UYVY')
        self._capture.set(cv2.CAP_PROP_FOURCC, fourcc)

        # Start thread
        self._thread = threading.Thread(target=self._eventloop)
        self._thread.start()

        @classmethod
        def from_camera_type(cls, id, camera_type="c905", compressed=True):
            """
            Initializes the video device with the best resulution quality
            avaiable for the cameras e-con See3CAM_CU135 (econ) or Logitech
            Webcam C905 (C905)

            To get the combination of framerate, x_res and y_res out of the
            camera do: v4l2-ctl -d /dev/video0 --list-formats-ext
            """

        # Opens Camera device with maximal resolution

            if camera_type == "c905" and compressed is True:
                return cls(id, framerate=10, width=1600, height=1200,
                           compressed=compressed)
            elif camera_type == "c905" and compressed is False:
                return cls(id, framerate=5, width=1600, height=1200,
                           compressed=compressed)
            elif camera_type == "econ" and compressed is True:
                return cls(id, framerate=20, width=4208, height=3120,
                           compressed=compressed)
            elif camera_type == "econ" and compressed is False:
                return cls(id, framerate=20, width=4208, height=3120,
                           compressed=compressed)

        @classmethod
        def from_nico_vision_version(cls, id, nico_vision_version=1,
                                     compressed=True):
            """
            Initializes the video device with the best resolution quality
            avaiable for the NICO vision version 1 (1) and NICO vision version
            2 (2)
            """
            if nico_vision_version == 1 and compressed is True:
                return cls(id, framerate=10, width=1600, height=1200,
                           compressed=compressed)
            elif nico_vision_version == 1 and compressed is False:
                return cls(id, framerate=5, width=1600, height=1200,
                           compressed=compressed)
            elif nico_vision_version == 2 and compressed is True:
                return cls(id, framerate=20, width=4208, height=3120,
                           compressed=compressed)
            elif nico_vision_version == 2 and compressed is False:
                return cls(id, framerate=20, width=4208, height=3120,
                           compressed=compressed)

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

    def load_callibration(self, file_path):
        """
        Loads a calibration json file and prepares undistortion for all cameras
        :param file_path: the calibration file
        :type file_path: str
        """
        # load file
        self._logger.info("Loading calibration file {}".format(file_path))
        if os.path.isfile(file_path):
            with open(file_path, 'r') as file:
                calibration = json.load(file)["mono"]
        else:
            self._logger.error(
                ("Calibration file {} does not exist").format(file_path))
            return
        # prepare rectify maps
        self._logger.debug("Preparing rectify map")
        devicenames = VideoDevice.get_all_devices()
        devicename = devicenames[self._deviceId]
        dim = self._width, self._height

        if (devicename in calibration and str(dim) in calibration[devicename]):
            K, D = map(lambda a: np.array(a), itemgetter(
                "K", "D")(calibration[devicename][str(dim)]))
            self._rectify_map = cv2.fisheye.initUndistortRectifyMap(
                K, D, np.eye(3), K, dim, cv2.CV_16SC2)

    def undistort(self, frame):
        """
        Undistorts the frame with the loaded calibration
        :param frame: frame to undistort
        :type frame: cv2 image
        :return: undistorted frame
        :rtype: cv2 image
        """
        if self._rectify_map:
            frame = cv2.remap(frame,
                              self._rectify_map[0],
                              self._rectify_map[1],
                              interpolation=cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_CONSTANT)
        return frame

    def camera_value(self, value_name, value):
        """
        Sets the a camera value over the v4l-utils. Run 'v4l2-ctl -l' for full
        list of controlls and values. Requires v4l-utils.
        :param value_name: name of the value to set
        :type value_name str
        :param value: value to set
        :type value: int
        """
        if type(value) is int:
            subprocess.call(
                ['v4l2-ctl -d {} -c {}={}'.format(
                    self._deviceId, value_name, value)], shell=True)
        else:
            self._logger.warning(
                "Invalid value '{}' - value has to be an integer".format(value)
            )

    def zoom(self, value):
        """
        Sets zoom value if camera supports it. Requires v4l-utils.
        :param value: zoom value between 100 and 800
        :type value: int
        """
        if type(value) is int and 100 <= value <= 800:
            call_str = 'v4l2-ctl -d {} -c zoom_absolute={}'.format(
                self._deviceId, value)
            self._logger.debug(
                "Zoom value call with " + call_str)
            subprocess.call([call_str], shell=True)
        else:
            self._logger.warning(
                "Zoom value has to be an integer between 100 and 800")

    def pan(self, value):
        """
        Sets pan (x-axis) value if camera supports it. Requires v4l-utils.
        :param value: pan value between -648000 and 648000, step 3600
        :type value: int
        """
        if(type(value) is int and -648000 <= value <= 648000 and
           value % 3600 == 0):
            subprocess.call(
                ['v4l2-ctl -d {} -c pan_absolute={}'.format(
                    self._deviceId, value)],
                shell=True)
        else:
            self._logger.warning(
                "Pan value has to be a multiple of 3600 between -648000 and " +
                "648000")

    def tilt(self, value):
        """
        Sets tilt (y-axis) value if camera supports it. Requires v4l-utils.
        :param value: tilt value between -648000 and 648000, step 3600
        :type value: int
        """
        if (type(value) is int and -648000 <= value <= 648000 and
                value % 3600 == 0):
            subprocess.call(
                ['v4l2-ctl -d {} -c tilt_absolute={}'.format(
                    self._deviceId, value)],
                shell=True)
        else:
            self._logger.warning(
                "Tilt value has to be a multiple of 3600 between -648000 and" +
                " 648000")

    def set_framerate(self, framerate):
        """
        Sets the current framerate

        :param framerate: Framerate (frames per second)
        :type framerate: int
        """
        self._framerate = framerate
        if self._capture is not None:
            self._capture.set(cv2.CAP_PROP_FPS, self._framerate)

    def get_framerate(self):
        """
        Returns the current frame rate

        :return: Framerate (frames per second)
        :rtype: int
        """
        return self._framerate

    def set_resolution(self, width, height):
        """
        Sets the current resolution

        :param width: Width
        :type width: int
        :param height: Height
        :type height: int
        """
        print("width: {}, heigth: {}".format(width, height))
        self._width = width
        self._height = height
        if self._capture is not None:
            self._running = False
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            self._running = True

    def get_resolution(self):
        """
        Returns the current resolution

        :return: (width, height)
        :rtype: tuple
        """
        return (self._width, self._height)

    def open(self):
        """
        Opens the device and starts the eventloop
        """
        if self._open:
            self._logger.warning('Device is already open')
            return

        # Open camera
        self._capture = cv2.VideoCapture(self._deviceId)
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self._capture.set(cv2.CAP_PROP_FPS, self._framerate)

        # Start eventloop
        self._running = True
        self._open = True
        self._thread = threading.Thread(target=self._eventloop)
        self._thread.start()

    def close(self):
        """
        Stopps the eventloop and closes the device
        """
        if not self._open:
            self._logger.warning('Trying to close a device which is not open')
            return
        self._running = False
        self._open = False
        self._capture.release()
        self._thread.join()

    def is_open(self):
        """
        Checks if the VideoDevice is open

        :return: True if VideoRecorder is open
        :rtype: bool
        """
        return self._open

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
        self._callback += [function]

    def clean_callbacks(self):
        """
        Removes all saved callbacks
        """
        self._callback = []

    def _eventloop(self):
        """
        Internal event loop

        This will call all registered callbacks for each frame
        """
        while self._running:
            t1 = time.time()
            rval, frame = self._capture.read()
            if rval:
                frame = self.undistort(frame)
            for function in self._callback:
                function(rval, frame)
            time.sleep(max(0, 1.0 / self._framerate - (time.time() - t1)))

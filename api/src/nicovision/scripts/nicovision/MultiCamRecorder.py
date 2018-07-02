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


def get_devices():
    """
    Returns a list containing the possible path of all video capturing devices

    :return: Video devices
    :rtype: list
    """
    return VideoDevice.get_all_devices()


class MultiCamRecorder:
    """
    The MultiCamRecorder class enables simultanious capturing of images from
    multiple cameras.
    """

    def __init__(self, devices=[], width=640, height=480, framerate=20,
                 zoom=None, pan=None, tilt=None, settings_file=None,
                 setting="standard", writer_threads=4,
                 pixel_format="MJPG"):
        """
        Initialises the MultiCamRecorder with given devices.

        The devices must be contained in :meth:`get_devices`

        :param devices: Device names
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
        :param file_path: the settings file
        :type file_path: str
        :param setting: name of the setting that should be applied
        :type setting: str
        :param writer_threads: Number of worker threads for image writer
        :type writer_threads: int
        :param pixel_format: fourcc codec
        :type pixel_format: string
        """
        self._deviceIds = []
        for device in devices:
            deviceId = VideoDevice.resolve_device(device)
            if deviceId == -1:
                logging.error('Can not create device from path' + self._device)
                sys.exit()
            self._deviceIds.append(deviceId)

        self._target = 'picture-{}.png'
        self._image_writer = ImageWriter(writer_threads)
        self._open = True
        self._callback_functions = []
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

        # Open cameras
        self._captures = []
        for id in self._deviceIds:
            capture = cv2.VideoCapture(id)
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            capture.set(cv2.CAP_PROP_FPS, self._framerate)
            fourcc = cv2.VideoWriter_fourcc(*pixel_format)
            capture.set(cv2.CAP_PROP_FOURCC, fourcc)
            self._captures.append(capture)

        # Start thread
        self._barrier = Barrier.Barrier(len(self._deviceIds))
        self._threads = []
        for id in range(len(self._deviceIds)):
            self._threads.append(threading.Thread(
                target=self._eventloop, args=(id,)))
            self._threads[id].daemon = True
            self._threads[id].start()

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
        :type value_name str
        :param value: value to set
        :type value: int
        """
        if type(value) is int:
            for id in self._deviceIds:
                subprocess.call(
                    ['v4l2-ctl -d {} -c {}={}'.format(
                        id, value_name, value)], shell=True)
        else:
            logging.warning(
                "Invalid value '{}' - value has to be an integer".format(value)
            )

    def zoom(self, value):
        """
        Sets zoom value of all cameras that support it. Requires v4l-utils.
        :param value: zoom value between 100 and 800
        :type value: int
        """
        if type(value) is int and 100 <= value <= 800:
            for id in self._deviceIds:
                subprocess.call(
                    ['v4l2-ctl -d {} -c zoom_absolute={}'.format(id, value)],
                    shell=True)
        else:
            logging.warning(
                "Zoom value has to be an integer between 100 and 800")

    def pan(self, value):
        """
        Sets pan (x-axis) value of all cameras that support it. Requires
        v4l-utils.
        :param value: pan value between -648000 and 648000, step 3600
        :type value: int
        """
        if(type(value) is int and -648000 <= value <= 648000 and
           value % 3600 == 0):
            for id in self._deviceIds:
                subprocess.call(
                    ['v4l2-ctl -d {} -c pan_absolute={}'.format(id, value)],
                    shell=True)
        else:
            logging.warning(
                "Pan value has to be a multiple of 3600 between -648000 and " +
                "648000")

    def tilt(self, value):
        """
        Sets tilt (y-axis) value of all cameras that support it. Requires
        v4l-utils.
        :param value: tilt value between -648000 and 648000, step 3600
        :type value: int
        """
        if (type(value) is int and -648000 <= value <= 648000 and
                value % 3600 == 0):
            for id in self._deviceIds:
                subprocess.call(
                    ['v4l2-ctl -d {} -c tilt_absolute={}'.format(id, value)],
                    shell=True)
        else:
            logging.warning(
                "Tilt value has to be a multiple of 3600 between -648000 and" +
                " 648000")

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
            logging.warning('Trying to add non-function callback')
            return
        self._callback_functions += [function]

    def clean_callbacks(self):
        """
        Removes all saved callbacks
        """
        self._callback = []

    def start_recording(self, path="camera{}/picture-{}.png"):
        self._target = path
        if not self._open:
            # Open camera
            for capture in self._captures:
                capture = cv2.VideoCapture(self._deviceId)
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
                capture.set(cv2.CAP_PROP_FPS, self._framerate)

            # Start eventloop
            self._running = True
            self._open = True
            self._threads = []
            for id in range(len(self._deviceIds)):
                self._threads.append(threading.Thread(
                    target=self._eventloop, args=(id,)))
                self._threads[id].start()
        self.add_callback(self._callback)

    def stop_recording(self):
        if not self._open:
            logging.warning('Trying to close a device which is not open')
            return
        self._barrier.abort()
        self._running = False
        self._open = False
        map(lambda t: t.join(), self._threads)
        map(lambda c: c.release(), self._captures)
        self.clean_callbacks()
        self._barrier.reset()

    def custom_callback(self, iso_time, frame):
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
                datetime.datetime.today().isoformat(), frame)

            self._image_writer.write_image(self._target.format(id, iso_time),
                                           frame)

    def _eventloop(self, id):
        """
        Internal event loop

        This will call all registered callbacks for each frame
        """
        while self._running:
            t1 = time.time()
            try:
                self._barrier.wait()
            except Barrier.BrokenBarrierError as e:
                break
            self._captures[id].grab()
            rval, frame = self._captures[id].retrieve()
            for function in self._callback_functions:
                function(rval, frame, id)
            time.sleep(max(0, 1.0 / self._framerate - (time.time() - t1)))

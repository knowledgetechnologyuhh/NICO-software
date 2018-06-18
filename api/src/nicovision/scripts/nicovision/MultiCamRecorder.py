import datetime
import inspect
import logging
import os
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
    """docstring for ."""

    def __init__(self, devices=[], width=640, height=480, framerate=20,
                 writer_threads=4, pixel_format="MJPG"):
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

        # Open cameras
        self._captures = []
        for id in self._deviceIds:
            capture = cv2.VideoCapture(id)
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            capture.set(cv2.CAP_PROP_FPS, self._framerate)
            fourcc = cv2.VideoWriter_fourcc(*pixel_format)
            # fourcc = cv2.VideoWriter_fourcc(*'UYVY')
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

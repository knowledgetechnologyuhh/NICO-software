import logging
import cv2
import os
import inspect
import threading
import time

class VideoDevice:
    """
    The VideoDevice class handles low-level communication with the video devices
    """

    _VIDEO_DEVICE_PATH = '/dev/v4l/by-id/'
    """
    This variable holds the base path for the ids of the capturing devices
    """

    @staticmethod
    def getAllPaths():
        """
        Returns a list containing the path of all video capturing devices

        :return: Paths of video devices
        :rtype: list
        """
        if not os.path.isdir(VideoDevice._VIDEO_DEVICE_PATH):
            logging.error('Video device path does not exists!')
            return []
        paths = []
        for file in os.listdir(VideoDevice._VIDEO_DEVICE_PATH):
            paths += [os.path.realpath(VideoDevice._VIDEO_DEVICE_PATH + file)]
        return paths

    @staticmethod
    def resolveID(partID):
        """
        Returns the path of a partial string ID

        :param partID: Partial ID
        :type partID: str
        :return: Path to device
        :rtype: str
        """
        if not os.path.isdir(VideoDevice._VIDEO_DEVICE_PATH):
            logging.error('Video device path does not exists!')
            return ''

        candidates = []
        for file in os.listdir(VideoDevice._VIDEO_DEVICE_PATH):
            if partID in file:
                candidates += [file]

        if len(candidates) is 0:
            logging.error('No candidates found')
            return ''
        elif len(candidates) is 1:
            return int(os.readlink(VideoDevice._VIDEO_DEVICE_PATH + candidates[0])[-1:])
        else:
            logging.error('Multiple candidates found: {}'.format(candidates))
            return ''

    @staticmethod
    def fromPartID(partID):
        """
        Convenience method for creating a VideoDevice from a (partial) string ID

        :param partID: (Partial) ID
        :type partID: str
        :return: VideoDevice or None if ID is not valid / ambiguous
        :rtype: VideoDevice or None
        """
        path = VideoDevice.resolveID(partID)
        if path is '':
            logging.error('Can not create VideoDevice from ID %s' % partID)
            return None
        return VideoDevice(path)

    def __init__(self, id, framerate=20, width=640, height=480):
        """
        Initialises the VideoDevice. The device starts closed and has to be opened.

        If you want to create a VideoDevice from a (partial) ID, use :meth:`nicovision.VideoDevice.fromPartID` instead.

        :param id: device id
        :type id: int
        """
        self._capture = None
        self._deviceId = id
        self._valid = True
        self._open = False
        self._callback = []
        self._running = False
        self._thread = None
        self._framerate = framerate
        self._width = width
        self._height = height

    def setFrameRate(self, framerate):
        """
        Sets the current framerate

        :param framerate: Framerate (frames per second)
        :type framerate: int
        """
        self._framerate = framerate

    def getFrameRate(self):
        """
        Returns the current frame rate

        :return: Framerate (frames per second)
        :rtype: int
        """
        return self._framerate

    def setResolution(self, width, height):
        """
        Sets the current resolution

        :param width: Width
        :type width: int
        :param height: Height
        :type height: int
        """
        self._width = width
        self._height = height

    def getResolution(self):
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
            logging.warning('Device is already oprn')
            return

        # Open camera
        self._capture = cv2.VideoCapture(self._deviceId)
        self._capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, self._width)
        self._capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, self._height)
        self._capture.set(cv2.cv.CV_CAP_PROP_FPS, self._framerate)
        self._capture.open(self._deviceId)

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
            logging.warning('Trying to close a device which is not open')
            return
        self._running = False
        self._open = False
        self._thread.join()
        self._capture.release()
        self._capture = None

    def isOpen(self):
        """
        Checks if the VideoDevice is open

        :return: True if VideoRecorder is open
        :rtype: bool
        """
        return self._open

    def addCallback(self, function):
        """
        Adds a callback for the event loop

        Whenever a new frame arrives, all registered callbacks are called.

        The callback must take exactly 2 arguments: rval and frame. frame can be None if any error occures or no image
        could be grabbed.

        :param function: Function to add as callback
        :type function: function
        """
        if not inspect.isfunction(function):
            logging.warning('Trying to add non-function callback')
            return
        self._callback += [function]

    def cleanCallbacks(self):
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
            rval, frame = self._capture.read()
            for function in self._callback:
                function(rval, frame)
                time.sleep(1.0 / self._framerate)
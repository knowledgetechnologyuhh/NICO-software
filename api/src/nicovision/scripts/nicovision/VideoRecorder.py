import logging
import cv2
import os

from VideoDevice import VideoDevice
from Colorspace import Colorspace

def getDevices():
    """
    Returns a list containing the possible path of all video capturing devices

    :return: Video devices
    :rtype: list
        """
    return VideoDevice.getAllDevices()

class VideoCodec:
    MPEG1 = 1

class VideoRecorder:
    def __init__(self, device='', colorspace=Colorspace.RGB, framerate=20, width=640, height=480, videoformat=VideoCodec.MPEG1):
        """
        Initialises the VideoRecorder

        :param device: Target video capture unit
        :type device: str
        :param colorspace: Used colorspace
        :type colorspace: Colorspace
        :param framerate: Framerate
        :type framerate: float
        :param width: With of captured stream
        :type width: int
        :param height: Height of captured stream
        :type height: int
        :param videoformat: Used video format
        :type videoformat: VideoCodec
        """
        self._deviceName = device
        self._running = False
        self._colorspace = colorspace
        self._framerate = framerate
        self._width = width
        self._height = height
        self._format = videoformat
        self._device = None
        self._encoder = None

    def isRecording(self):
        """
        Returns true if the VideoRecorder is recording

        :return: True if recording
        :rtype: bool
        """
        return self._running

    def getColorSpace(self):
        """
        Returns the currently used colorspace

        :return: Colorspace
        :rtype: :class:`vision.Colorspace`
        """
        return self._colorspace

    def getFrameRate(self):
        """
        Returns the current frame rate

        :return: Framerate (frames per second)
        :rtype: int
        """
        return self._framerate

    def getResolution(self):
        """
        Returns the current resolution

        :return: (width, height)
        :rtype: tuple
        """
        return self._width, self._height


    def getVideoFormat(self):
        """
        Returns the current video format

        :return: Video format
        :rtype: str
        """
        return self._format

    def setColorSpace(self, colorspace):
        """
        Sets the current color space

        :param colorspace: Colorspace
        :type colorspace: :class:`vision.Colorspace`
        """
        self._colorspace = colorspace

    def setFrameRate(self, framerate):
        """
        Sets the current framerate

        :param framerate: Framerate (frames per second)
        :type framerate: int
        """
        self._framerate = framerate

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

    def setVideoFormat(self, format):
        """
        Sets the current video format

        :param format: video format
        :type format: VideoCodec
        """
        self._format = format

    def startRecording(self, folder, file, overwrite = True):
        """
        Starts the recording into folder/file

        :param folder: Target folder. Will be created if none existent and overwrite is set to true
        :type folder: str
        :param file: Target file name
        :type file: str
        :param overwrite: If set to False no files will be overwritten
        :type overwrite: bool
        """
        if not self._running:
            logging.warning('Trying to start recording while already running')
            return
        if folder[-1] is not '/':
            folder += '/'
        if not os.path.isdir(folder):
            if overwrite:
                os.makedirs(folder)
            else:
                logging.error('Folder %s not existing' % folder)
        if not overwrite and os.path.exists(folder + file):
            logging.error('File %s already exists' % folder+file)
            return

        # Resolve codec
        fourcc = None
        try:
            fourcc = {
                VideoCodec.MPEG1: cv2.cv.FOURCC('P','I','M','1')
            }[self._format]
        except:
            logging.error('Unknown codec')

        # Setup video capturing
        self._device = VideoDevice.fromDevice(self._deviceName)
        self._encoder = cv2.VideoWriter(folder + file, fourcc, self._framerate, (self._width, self._height))
        self._device.addCallback(self._callback)
        self._device.setResolution(self._width, self._height)
        self._device.setFrameRate(self._framerate)
        self._device.open()
        self._running = True

    def stopRecording(self):
        """
        Stops the current recording
        """
        if not self._running:
            logging.warning('Trying to stop recording while no recording is running')
            return
        self._device.close()
        self._device = None
        self._encoder = None
        self._running = False

    def _callback(self, rval, frame):
        if frame is not None:
            self._encoder.write(frame)
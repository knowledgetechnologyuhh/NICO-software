import datetime
import os
import time
import cv2
import logging

from VideoDevice import VideoDevice


def getDevices():
    """
    Returns a list containing the possible path of all video capturing devices

    :return: Video devices
    :rtype: list
        """
    return VideoDevice.getAllDevices()

class ImageRecorder:
    def __init__(self, device='', width=640, height=480):
        """
        Initialises the ImageRecorder with a given device.

        The device must be contained in :meth:`getDevices`

        :param device:
        :param width:
        :param height:
        """
        self._device = device
        self._target = 'picture.png'
        self._width = width
        self._height = height

    def saveOneImage(self):
        """
        Saves an image to a unique file

        :return: Path (or '' if an error occured
        :rtype: str
        """
        path = 'picture-' + datetime.datetime.today().isoformat() + '.png'
        return os.path.abspath(path) if self.saveImageTo(path) else ''

    def saveImageTo(self, path):
        """
        Saves an Image to a given file

        :param path: Target file
        :return: True if successful
        :rtype: bool
        """
        self._target = path
        device = VideoDevice.fromDevice(self._device)
        if device is None:
            logging.error('Can not create device from path' + self._device)
            return False
        device.addCallback(self._callback)
        device.setFrameRate(1)
        device.setResolution(self._width, self._height)
        device.open()
        time.sleep(0.1)
        device.close()
        return True

    def _callback(self, rval, frame):
        """
        Internal callback

        :param rval: rval
        :param frame: frame
        """
        if frame is not None:
            cv2.imwrite(self._target, frame)
import datetime
import os
import time
import cv2
import logging

from VideoDevice import VideoDevice


def get_devices():
    """
    Returns a list containing the possible path of all video capturing devices

    :return: Video devices
    :rtype: list
        """
    return VideoDevice.get_all_devices()


class ImageRecorder:
    """
The ImageRecorder class enables the capturing of single images from a camera.
    """

    def __init__(self, device='', width=640, height=480):
        """
        Initialises the ImageRecorder with a given device.

        The device must be contained in :meth:`get_devices`

        :param device: Device name
        :type device: str
        :param width: Width of image
        :type width: float
        :param height: Height of image
        :type height: float
        """
        self._device = device
        self._target = 'picture.png'
        self._width = width
        self._height = height

    def save_one_image(self):
        """
        Saves an image to a unique file

        :return: Path (or '' if an error occured
        :rtype: str
        """
        path = 'picture-' + datetime.datetime.today().isoformat() + '.png'
        return os.path.abspath(path) if self.save_image_to(path) else ''

    def save_image_to(self, path):
        """
        Saves an Image to a given file

        :param path: Target file
        :return: True if successful
        :rtype: bool
        """
        self._target = path
        device = VideoDevice.from_device(self._device)
        if device is None:
            logging.error('Can not create device from path' + self._device)
            return False
        device.set_framerate(1)
        device.set_resolution(self._width, self._height)
        device.open()
        time.sleep(0.1)
        device.add_callback(self._callback)
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

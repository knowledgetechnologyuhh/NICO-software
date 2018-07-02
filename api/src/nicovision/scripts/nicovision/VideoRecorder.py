import logging
import os

import cv2

from Colorspace import Colorspace
from VideoDevice import VideoDevice


def get_devices():
    """
    Returns a list containing the possible path of all video capturing devices

    :return: Video devices
    :rtype: list
        """
    return VideoDevice.get_all_devices()


class VideoCodec:
    """
    The VideoCodec class represents different video codecs.
    """
    MPEG1 = 1
    """
    MPEG-1 codec
    """
    H264 = 2
    """
    H.264 codec
    """
    DIVX = 3
    """
    DivX codec (Also known as MPEG-4)
    """
    XVID = 4
    """
    XVID codec
    """


class VideoRecorder:

    def __init__(self, device='', colorspace=Colorspace.RGB, framerate=20,
                 width=640, height=480, zoom=None, pan=None, tilt=None,
                 settings_file=None, setting="standard",
                 videoformat=VideoCodec.MPEG1):
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
        :param zoom: Zoom of camera (if supported)
        :type zoom: int
        :param videoformat: Used video format
        :type videoformat: VideoCodec
        """
        self._device = VideoDevice.from_device(device, framerate, width,
                                               height, zoom, pan, tilt,
                                               settings_file, setting)
        self._running = False
        self._colorspace = colorspace
        self._framerate = framerate
        self._width = width
        self._height = height
        self._format = videoformat
        self._encoder = None

    def load_settings(self, file_path, setting="standard"):
        """
        Loads a settings json file and applies the given setting to all cameras
        :param file_path: the settings file
        :type file_path: str
        :param setting: name of the setting that should be applied
        :type setting: str
        """
        self._device.load_settings(file_path, setting)

    def zoom(self, value):
        """
        Sets zoom value if camera supports it. Requires v4l-utils.
        :param value: zoom value between 100 and 800
        :type value: int
        """
        self._device.zoom(value)

    def pan(self, value):
        """
        Sets pan (x-axis) value if camera supports it. Requires v4l-utils.
        :param value: pan value between -648000 and 648000, step 3600
        :type value: int
        """
        self._device.pan(value)

    def tilt(self, value):
        """
        Sets tilt (y-axis) value if camera supports it. Requires v4l-utils.
        :param value: tilt value between -648000 and 648000, step 3600
        :type value: int
        """
        self._device.tilt(value)

    def is_recording(self):
        """
        Returns true if the VideoRecorder is recording

        :return: True if recording
        :rtype: bool
        """
        return self._running

    def get_color_space(self):
        """
        Returns the currently used colorspace

        :return: Colorspace
        :rtype: :class:`vision.Colorspace`
        """
        return self._colorspace

    def get_frame_rate(self):
        """
        Returns the current frame rate

        :return: Framerate (frames per second)
        :rtype: int
        """
        return self._framerate

    def get_resolution(self):
        """
        Returns the current resolution

        :return: (width, height)
        :rtype: tuple
        """
        return self._width, self._height

    def get_video_format(self):
        """
        Returns the current video format

        :return: Video format
        :rtype: str
        """
        return self._format

    def set_color_Space(self, colorspace):
        """
        Sets the current color space

        :param colorspace: Colorspace
        :type colorspace: :class:`vision.Colorspace`
        """
        self._colorspace = colorspace

    def set_frame_rate(self, framerate):
        """
        Sets the current framerate

        :param framerate: Framerate (frames per second)
        :type framerate: int
        """
        self._framerate = framerate

    def set_resolution(self, width, height):
        """
        Sets the current resolution

        :param width: Width
        :type width: int
        :param height: Height
        :type height: int
        """
        self._width = width
        self._height = height

    def set_video_format(self, format):
        """
        Sets the current video format

        :param format: video format
        :type format: VideoCodec
        """
        self._format = format

    def start_recording(self, folder, file, overwrite=True):
        """
        Starts the recording into folder/file.
        File has to have the extension '.avi'. Extension will be attached
        automatically.

        :param folder: Target folder. Will be created if none existent and
        overwrite is set to true
        :type folder: str
        :param file: Target file name
        :type file: str
        :param overwrite: If set to False no files will be overwritten
        :type overwrite: bool
        """
        if self._running:
            logging.warning('Trying to start recording while already running')
            return
        if folder[-1] is not '/':
            folder += '/'
        if not file.endswith(('.avi', '.AVI')):
            logging.debug('Attaching ".avi" as file extension')
            file += '.avi'
        if not os.path.isdir(folder):
            if overwrite:
                os.makedirs(folder)
            else:
                logging.error('Folder %s not existing' % folder)
        if not overwrite and os.path.exists(folder + file):
            logging.error('File %s already exists' % folder + file)
            return

        # Resolve codec
        fourcc = None
        try:
            fourcc = {
                VideoCodec.MPEG1: cv2.VideoWriter_fourcc('P', 'I', 'M', '1'),
                VideoCodec.H264: cv2.VideoWriter_fourcc('X', '2', '6', '4'),
                VideoCodec.DIVX: cv2.VideoWriter_fourcc('D', 'I', 'V', 'X'),
                VideoCodec.XVID: cv2.VideoWriter_fourcc('X', 'V', 'I', 'D'),
            }[self._format]
        except:
            logging.error('Unknown codec')
            return

        # Setup video capturing
        self._encoder = cv2.VideoWriter(folder + file, fourcc, self._framerate,
                                        (self._width, self._height))
        self._device.add_callback(self._callback)
        if not self._device._open:
            self._device.set_resolution(self._width, self._height)
            self._device.set_framerate(self._framerate)
            self._device.open()
        self._running = True

    def stop_recording(self):
        """
        Stops the current recording
        """
        if not self._running:
            logging.warning(
                'Trying to stop recording while no recording is running')
            return
        self._device.close()
        self._encoder = None
        self._running = False

    def _callback(self, rval, frame):
        if rval:
            self._encoder.write(frame)

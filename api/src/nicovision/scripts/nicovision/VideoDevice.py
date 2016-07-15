import logging
import cv2
import os
import stat

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
    def getAllIDs():
        """
        Returns a list of all full IDs of video capturing devices

        :return: IDs of capturing devices
        :rtype: list
        """
        if not os.path.isdir(VideoDevice._VIDEO_DEVICE_PATH):
            logging.error('Video device path does not exists!')
            return []
        ids = []
        for file in os.listdir(VideoDevice._VIDEO_DEVICE_PATH):
            ids += [file]
        return ids

    @staticmethod
    def resolveID(partID):
        """
        Returns the path of a partial ID

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
            return os.path.realpath(VideoDevice._VIDEO_DEVICE_PATH + candidates[0])
        else:
            logging.error('Multiple candidates found: {}'.format(candidates))
            return ''

    @staticmethod
    def fromPartID(partID):
        """
        Convenience method for creating a VideoDevice from a (partial) ID

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

    def __init__(self, path):
        """
        Initialises the VideoDevice. The device starts closed and has to be opened.

        If you want to create a VideoDevice from a (partial) ID, use :meth:`nicovision.VideoDevice.fromPartID` instead.

        :param path: Path to device
        :type path: str
        """
        self._capture = None
        self._path = path
        self._valid = True
        self._open = False
        if not os.lstat(path).st_dev:
            logging.error('Path %s is not a valid device path')
            self.valid = False
        pass

    def open(self):
        # TODO: Implement
        pass

    def close(self):
        # TODO: Implement
        pass

    def isOpen(self):
        """
        Checks if the VideoDevice is open

        :return: True if VideoRecorder is open
        :rtype: bool
        """
        return self._open

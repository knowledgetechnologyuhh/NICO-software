import logging

from Colorspace import Colorspace

class VideoRecorder:
    def __init__(self):
        pass

    def isRecording(self):
        """
        Returns true if the VideoRecorder is recording

        :return: True if recording
        :rtype: bool
        """
        # TODO: implement
        pass

    def getColorSpace(self):
        """
        Returns the currently used colorspace

        :return: Colorspace
        :rtype: :class:`Colorspace`
        """
        # TODO: implement
        return Colorspace.RGB

    def getFrameRate(self):
        """
        Returns the current frame rate

        :return: Framerate (frames per second)
        :rtype: int
        """
        # TODO: implement
        pass

    def getResolution(self):
        """
        Returns the current resolution

        :return: (width, height)
        :rtype: tuple
        """
        # TODO: implement


    def getVideoFormat(self):
        """
        Returns the current video format

        :return: Video format
        :rtype: str
        """
        # TODO: implement
        pass

    def setColorSpace(self, colorspace):
        """
        Sets the current color space

        :param colorspace: Colorspace
        :type colorspace: class:`Colorspace`
        """
        # TODO: implement
        pass

    def setFrameRate(self, framerate):
        """
        Sets the current framerate

        :param framerate: Framerate (frames per second)
        :type framerate: int
        """
        # TODO: implement
        pass

    def setResolution(self, width, height):
        """
        Sets the current resolution

        :param width: Width
        :type width: int
        :param height: Height
        :type height: int
        """
        pass

    def setVideoFormat(self, format):
        """
        Sets the current video format

        :param format: video format
        :type format: str
        """
        # TODO: implement
        pass

    def startRecording(self, folder, file, overwrite = True):
        """
        Starts the recording into folder/file

        :param folder: Target folder
        :type folder: str
        :param file: Target file name
        :type file: str
        :param overwrite: If set to False no files will be overwritten
        :type overwrite: bool
        """
        # TODO: implement
        pass

    def stopRecording():
        """
        Stops the current recording
        """
        # TODO: implement
        pass

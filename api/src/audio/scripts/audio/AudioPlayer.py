import logging

class AudioPlayer:
    def __init__(self):
        pass

    def getCurrentPosition(self, task):
        """
        Gets the current position of the file associated with a task

        :param task: Target task
        :type task: int
        :return: Current position in seconds
        :rtype: int
        """
        # TODO: implement
        pass

    def getFileLength(self, task):
        """
        Returns the file length of the file associated with a task

        :param task: Target task
        :type task: int
        :return: File length in seconds
        :rtype: int
        """
        # TODO: implement
        pass

    def getLoadedFilesTasks(self):
        """
        Returns the tasks of all loaded files

        :return: List of tasks
        :rtype: list
        """
        # TODO: implement
        pass

    def getLoadedFilesNames(self):
        """
        Returns the file names of all loaded files

        :return: List of file names
        :rtype: list
        """
        # TODO: implement
        pass

    def getMasterVolume(self):
        """
        Returns the master (default) volume

        :return: Volume [0.0, 1.0]
        :rtype: float
        """
        # TODO: implement
        pass

    def getVolume(self, task):
        """
        Returns the volume of a given task

        :param task: Target task
        :type task: int
        :return: Volume [0.0, 1.0]
        :rtype: float
        """
        # TODO: implement
        pass

    def getMasterPanorama(self):
        """
        Returns the master (default) panorama

        :return: Panorama [-1.0 (left), 1.0 (right)]
        :rtype: float
        """
        # TODO: implement
        pass

    def getPanorama(self, task):
        """
        Returns the panorama of a task

        :param task: Target task
        :type task: int
        :return: Panorama [-1.0 (left), 1.0 (right)]
        :rtype: float
        """
        # TODO: implement
        pass

    def goTo(self, task, position):
        """
        Jumps to a given position in a task

        :param task: Target task
        :type task: int
        :param position: Position in seconds
        :type position: int
        """
        # TODO: implement
        pass

    def loadFile(self, filename):
        """
        Loads a given file

        :param filename: File name
        :type filename: str
        :return: Associated task
        :rtype: int
        """
        # TODO: implement
        pass

    def pause(self, task):
        """
        Pauses the playback of a task

        :param task: Target task
        :type task: int
        """
        # TODO: implement
        pass

    def play(self, task, volume = 1.0, panorama = 0.0):
        """
        Plays a given task

        :param task: Target task
        :type task: int
        :param volume: Volume [0.0, 1.0]
        :type volume: float
        :param panorama: Panorama [-1.0 (left), 1.0 (right)]
        :type panorama: float
        """
        # TODO: implement
        pass

    def playFile(self, filename, position = 0, volume = 1.0, panorama = 0.0):
        """
        Plays a given file

        :param filename: File to play
        :type filename: str
        :param position: Start position in seconds
        :type position: int
        :param volume: Volume [0.0, 1.0]
        :type volume: float
        :param panorama: Panorama [-1.0 (left), 1.0 (right)]
        :type panorama: float
        """
        # TODO: implement
        pass

    def setMasterVolume(self, volume):
        """
        Sets the master (default) volume

        :param volume: Volume [0.0, 1.0]
        :type volume: float
        """
        # TODO: implement
        pass

    def setVolume(self, task, volume):
        """
        Sets the volume of the given task

        :param task: Target task
        :type task: int
        :param volume: Volume [0.0, 1.0]
        :type volume: float
        """
        # TODO: implement
        pass

    def setMasterPanorama(self, panorama):
        """
        Sets the master (default) panorama

        :param panorama: Panorama [-1.0 (left), 1.0 (right)]
        :type panorama: float
        """
        # TODO: implement
        pass

    def setPanorama(self, task, panorama):
        """
        Sets the panorama of a given task

        :param task: Target task
        :type task: int
        :param panorama: Panorama [-1.0 (left), 1.0 (right)]
        :type panorama: float
        """
        # TODO: implement
        pass

    def stopAll(self):
        """
        Stopps all currently playing tasks
        """
        # TODO: implement
        pass

    def unloadAllFiles(self):
        """
        Unloads all files
        """
        # TODO: implement
        pass

    def unloadFile(self, task):
        """
        Unloads the file associated with a task

        :param task: Target task
        :type task: int
        """
        # TODO: implement
        pass

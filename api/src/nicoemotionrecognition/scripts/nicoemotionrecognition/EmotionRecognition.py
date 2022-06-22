#!/usr/bin/env python
from flaskcom.remote_object import RemoteObject
from flaskcom.remote_object import VERBOSITY_ERROR


class EmotionRecognition(object):
    """Connects to and exposes EmotionRecognition instance whithin Docker."""

    def __init__(self):
        # remote object of _nicoemotionrecognition_internal.EmotionRecognitionBackend
        self._emotion_server = RemoteObject(
            server="localhost",  # the remote object is on this computer
            port=50000,  # a port needs to be specified
            start_server=False,  # the server is not started, it should be there already
            new_terminal_window=False,  # forces the code to be run in the terminal it was started from
            time_out=-1,  # the time to wait for the remote terminal to start, -1 means forever
            debug=True,  # keeps the terminal open even if an error occurs
            verbosity_level=VERBOSITY_ERROR,  # makes flaskcom more verbose for debugging, can be changed to VERBOSITY_ERROR to be less verbose
            username="admin",  # the admin username for the remote object
            password="mypassword1",
        )  # the admin password for the remote object

    def send_image(self, frame):
        """
        Performs face detection and emotion recognition on the given image

        :param frame: frame
        """
        self._emotion_server.send_image(frame)

    def get_categorical_data(self):
        """
        Returns the categorical data of the currently detected face (or None if
        there is none)

        :return: Neutral, Happiness, Surprise, Sadness, Anger, Disgust, Fear
                 and Contempt percentages (or None if no face detected)
        :rtype: dict
        """
        return self._emotion_server.get_categorical_data()

    def get_highest_matching_emotion(self):
        """
        Returns the name of the highest matching emotion for the currently
        detected face (or None if there is none)

        :return: Neutral, Happiness, Surprise, Sadness, Anger, Disgust, Fear or
                 Contempt (or None if no face detected)
        :rtype: String
        """
        return self._emotion_server.get_highest_matching_emotion()

    def get_face_center(self):
        """
        Returns the center coordinates of the currently
        detected face (or None if there is none)

        :return: (x, y) coordinates within the image
        :rtype: (int, int)
        """
        return self._emotion_server.get_face_center()

    @property
    def face_detected(self):
        """
        Whether a face was detected in the current frame

        :return: True if a face was detected, False if not
        :rtype: bool
        """
        return self._emotion_server.face_detected

    def close(self):
        self._emotion_server.close()

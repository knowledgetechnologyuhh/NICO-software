#!/usr/bin/env python


import logging
import random
import time

import cv2
import numpy
import tensorflow as tf
from nicovision.VideoDevice import VideoDevice
from nicoaudio.TextToSpeech import TextToSpeech

from ._nicoemotionrecognition_internal import (
    GUIController,
    imageProcessingUtil,
    modelDictionary,
    modelLoader,
)


class EmotionRecognition:
    _voice_reactions = {
        "german": {
            "happiness": (
                "Ich bin froehlich, wenn Du es auch bist!",
                "Was fuer ein schoener Tag heute ist, nicht wahr?",
                "Du bist gerade gluecklich, stimmts?",
            ),
            "surprise": (
                "Du siehst ueberrascht aus. Was ist denn los?",
                "Bist Du ueberrascht, was fuer ein smarter Roboter ich bin?",
                "Das ist eine Ueberraschung, nicht wahr?",
            ),
            "anger": (
                "Du siehst so aergerlich aus, ist alles in Ordnung?",
                "aeh Digga, ich bin auch sauer!",
                "Was ist Dir denn ueber die Leber gelaufen?",
            ),
            "fear": (
                "Hast Du vor etwas Angst? Was ist denn hier gefaehrlich?",
                "Ich bin ganz harmlos! Du musst keine Angst vor mir haben!",
                "Du siehst aus, als haettest Du einen Geist gesehen! "
                "Da bekomme ich auch Angst",
            ),
            "sadness": (
                "Sei nicht traurig, wir können so viel Spaß zusammen haben!",
                "Ich würde ja auch weinen, aber ich habe keine Tränendrüsen.",
                "Warum bist du traurig? Habe ich was falsches gesagt?",
            ),
            "disgust": (
                "Hey! Ich finde dich auch nicht besonders hübsch!",
                "Ich bin von deinem Ekel angewiedert!",
                "Du siehst etwas blass aus. Ist alles in Ordnung?",
            ),
        },
        "english": {
            "happiness": (
                "I am happy, if YOU are happy.",
                "What a nice day, right?",
                "You are happy right now, are you?",
            ),
            "surprise": (
                "You look surprised. Is everything alright?",
                "Are you surprised, what a smart robot I am?",
                "This is a surprise, right?",
            ),
            "anger": (
                "You look angry. Is everything alright?",
                "I am angry as well!",
                "What is wrong with you?",
            ),
            "fear": (
                "Do you fear something?. What is dangerous here?",
                "I am harmless! You do not have to fear me",
                "You look like you have seen a ghost, but that is only me!",
            ),
            "sadness": (
                "Don't be sad, we can have so much fun together!",
                "I'd cry as well, but I don't have tears.",
                "Why are you sad? Did I say something wrong?",
            ),
            "disgust": (
                "Hey! I don't like the way you look either.",
                "I'm disgusted by your disgust!",
                "You look a bit pale. Is everything alright?",
            ),
        },
    }

    def __init__(
        self,
        robot=None,
        face=None,
        faceDetectionDelta=10,
        voiceEnabled=False,
        german=False,
    ):
        """
        Initialises the EmotionRecognition

        :param device: Video capture unit
        :type device: nicovision.VideoDevice
        :param robot: Motion object for face tracking
        :type robot: nicomotion.Motion
        :param face: FaceExpression object to mirror emotions
        :type face: nicoface.FaceExpression
        :param faceDetectionDelta: Number of frames until face detection is
                                   refreshed
        :type faceDetectionDelta: int
        :param voiceEnabled: enables voice feedback based on detected emotions
        :type voiceEnabled: bool
        :param german: switch audio from english to german
        :type german: bool
        """
        self._logger = logging.getLogger(__name__)
        self._finalImageSize = (
            1024,
            768,
        )  # Size of the final image generated by the demo
        # Initial position for adding the categorical graph in the final image
        self._categoricalInitialPosition = 260
        # Input size for both models: categorical and dimensional
        self._faceSize = (64, 64)
        self._categoricalRecognition = None
        self._dimensionalRecognition = None
        self._running = False
        self._facialExpression = face
        self._robot = robot
        self._voiceEnabled = voiceEnabled
        if voiceEnabled:
            self._tts = TextToSpeech()
            self._tts_end = 0
        self._german = german

        self._modelCategorical = modelLoader.modelLoader(
            modelDictionary.CategoricaModel
        )
        # self._modelDimensional = modelLoader.modelLoader(
        #     modelDictionary.DimensionalModel
        # )
        self._graph = tf.get_default_graph()

        self._faceDetectionDelta = faceDetectionDelta
        self._imageProcessing = imageProcessingUtil.imageProcessingUtil(
            faceDetectionDelta
        )

        self._GUIController = GUIController.GUIController()

        self._not_found_counter = 0
        self._same_emotion_counter = 0

        self.last_exp = ""

    def start(
        self,
        showGUI=True,
        faceTracking=False,
        mirrorEmotion=False,
        trackingDelta=10,
        mirrorEmotionDelta=10,
    ):
        """
        Starts the emotion recognition

        :param showGUI: Whether or not the GUI should be displayed
        :type showGUI: bool
        :param faceTracking: Lets the robot follow the detected face with its
                             head (requires motion)
        :type faceTracking: bool
        :param mirrorEmotion: Whether or not the robot should mirror the
                              detected emotion
        :type mirrorEmotion: bool
        :param trackingDelta: Frames before updating face tracking
        :type trackingDelta: int
        :param mirrorEmotionDelta: Consecutive frames with the same emotion
                                   required to update mirrored emotion
        :type mirrorEmotionDelta: int
        """
        if self._running:
            self._logger.warning(
                "Trying to start emotion recognition while already running"
            )
            return
        self._mirrorEmotion = mirrorEmotion
        self._mirrorEmotionDelta = mirrorEmotionDelta
        self._faceTracking = faceTracking
        self._trackingDelta = trackingDelta
        self._trackingCounter = trackingDelta
        self._device.add_callback(self._callback)
        self._device.open()
        self._showGUI = showGUI
        self._running = True

    def stop(self):
        """
        Stops the emotion recognition
        """
        if not self._running:
            self._logger.warning(
                "Trying to stop emotion recognition while it's not running"
            )
            return
        self._device.close()
        self._device = None
        self._categoricalRecognition = None
        self._dimensionalRecognition = None
        cv2.destroyAllWindows()
        self._running = False

    def getDimensionalData(self):
        """
        Returns the dimensional data of the currently detected face (or None
        if there is none)

        :return: Arousal and Valence score (or None if no face detected)
        :rtype: dict
        """
        if not self._running:
            self._logger.warning(
                "Dimensional data requested while emotion recognition not " + "running"
            )
            return None
        if self._dimensionalRecognition is None:
            self._logger.info("No face detected - Dimensional data will be 'None'")
            return None
        return dict(
            zip(
                self._modelDimensional.modelDictionary.classsesOrder,
                map(
                    lambda x: float(float(x[0][0]) * 100), self._dimensionalRecognition
                ),
            )
        )

    def getCategoricalData(self):
        """
        Returns the categorical data of the currently detected face (or None if
        there is none)

        :return: Neutral, Happiness, Surprise, Sadness, Anger, Disgust, Fear
                 and Contempt percentages (or None if no face detected)
        :rtype: dict
        """
        if not self._running:
            self._logger.warning(
                "Categorical data requested while emotion recognition not " + "running"
            )
            return None
        if self._categoricalRecognition is None:
            self._logger.info("No face detected - Categorical data will be 'None'")
            return None
        return dict(
            zip(
                self._modelCategorical.modelDictionary.classsesOrder,
                self._categoricalRecognition[0],
            )
        )

    def getHighestMatchingEmotion(self):
        """
        Returns the name of the highest matching emotion for the currently
        detected face (or None if there is none)

        :return: Neutral, Happiness, Surprise, Sadness, Anger, Disgust, Fear or
                 Contempt (or None if no face detected)
        :rtype: String
        """
        if self._categoricalRecognition is not None:

            if self._categoricalRecognition[0][6] > 15:
                return "fear"
            elif self._categoricalRecognition[0][4] > 20:
                return "anger"
            elif self._categoricalRecognition[0][3] > 15:
                return "sadness"
            elif self._categoricalRecognition[0][3] > 20:
                return "happiness"
            return self._modelCategorical.modelDictionary.classsesOrder[
                numpy.argmax(self._categoricalRecognition[0])
            ].lower()
        return None

    def say(self, sen, delay=5):
        """
        Triggers tts module to play the given sentence and updates internal
        delay until a new voice line is played

        :param sen: the next sentence
        :type sen: str
        :param delay: delay in seconds until another voice line will be played
        :type delay: float
        """
        if self._german:
            duration = self._tts.say(
                sen, language="de", blocking=False, pitch=0.2, speed=2 ** -0.2
            )
        else:
            duration = self._tts.say(sen, blocking=False, pitch=0.2, speed=2 ** -0.2)
        self._tts_end = time.time() + duration + delay

    def voice_reaction(self, emotion):
        """
        Plays random voice feedback for the given emotion

        :param emotion: detected emotion
        :type emotion: str
        """
        language = "german" if self._german else "english"
        if emotion in self._voice_reactions[language]:
            sentences = self._voice_reactions[language][emotion]
            self.say(random.choice(sentences))

    def follow_face_with_head(self, facePoints):
        """
        Moves head towards center of the given facepoints

        :param facePoints: dict containing "center" point of the face
        :type facePoints: dict
        """
        if self._robot is not None:
            if self._trackingCounter == self._trackingDelta:
                # (width - center_x)/width * FOV - FOV/2
                # horizontal
                angle_z = (640 - facePoints["center"].x) / 640.0 * 60 - 60 / 2.0
                # vertikal
                angle_y = (480 - facePoints["center"].y) / 480.0 * 50 - 50 / 2.0
                self._robot.changeAngle("head_z", angle_z, 0.02)
                self._robot.changeAngle("head_y", -angle_y, 0.02)
                self._trackingCounter = 0
            else:
                self._trackingCounter += 1
        else:
            self._logger.warning(
                "No robot given on initialisation - skipping face tracking"
            )

    def show_emotion(self, emotion):
        """
        Updates emotion and plays voice reaction if called enough consecutive
        times with the same emotion.

        :param emotion: detected emotion
        :type emotion: str
        """
        if self.last_exp != emotion:
            self._same_emotion_counter = 0
            self.last_exp = emotion
        else:
            self._same_emotion_counter += 1
            if self._same_emotion_counter > self._mirrorEmotionDelta:
                self._facialExpression.sendFaceExpression(emotion)
                if self._voiceEnabled and time.time() > self._tts_end:
                    self.voice_reaction(emotion)

    def update_GUI(self, frame, facePoints):
        """
        Updates gui with the detected face and corresponding emotion

        :param frame: Current frame from the camera
        :type frame: cv2.image
        :param facePoints: dict containing "top", "left", "bottom", "right"
                           points of the detected face
        :type facePoints: dict
        """
        frame = self._GUIController.createDetectedFacGUI(
            frame,
            facePoints,
            self._modelCategorical.modelDictionary,
            self._categoricalRecognition,
        )
        # frame = self._GUIController.createDimensionalEmotionGUI(
        #     self._dimensionalRecognition,
        #     frame,
        #     self._categoricalRecognition,
        #     self._modelCategorical.modelDictionary,
        # )
        frame = self._GUIController.createCategoricalEmotionGUI(
            self._categoricalRecognition,
            frame,
            self._modelCategorical.modelDictionary,
            initialPosition=self._categoricalInitialPosition,
        )
        return frame

    def _callback(self, rval, frame):
        """
        Callback for the video device.

        :param rval: rval
        :param frame: frame
        """
        if frame is not None:
            facePoints, face = self._imageProcessing.detectFace(frame)

            if self._showGUI:
                image = numpy.zeros(
                    (self._finalImageSize[1], self._finalImageSize[0], 3), numpy.uint8
                )
                image[0:480, 0:640] = frame
                frame = image

            if face is not None and len(face) > 0:
                self._not_found_counter = 0

                if self._faceTracking:
                    self.follow_face_with_head(facePoints)

                face = self._imageProcessing.preProcess(face, self._faceSize)
                with self._graph.as_default():
                    self._categoricalRecognition = self._modelCategorical.classify(face)
                    # self._dimensionalRecognition = self._modelDimensional.classify(face)

                if self._mirrorEmotion and self._facialExpression is not None:
                    expression = self.getHighestMatchingEmotion()
                    self.show_emotion(expression)

                if self._showGUI:
                    frame = self.update_GUI(frame, facePoints)
            else:
                self._categoricalRecognition = None
                self._dimensionalRecognition = None
                if self._mirrorEmotion and self._facialExpression:
                    self._facialExpression.sendFaceExpression("neutral")
                if self._faceTracking:
                    self._not_found_counter += 1
                    self._logger.info("Saw nothing: %i", self._not_found_counter)
                    if self._not_found_counter > 50:
                        # After frames of not detecting something, return to
                        # mid position
                        self._not_found_counter = 0
                        if self._robot is not None:
                            self._robot.setAngle(
                                "head_z", 0 + random.randint(-15, 15), 0.01
                            )
                            self._robot.setAngle(
                                "head_y", random.randint(-10, 10), 0.01
                            )
                        else:
                            self._logger.warning(
                                "No robot given on initialisation - "
                                + "skipping face tracking"
                            )

            if self._showGUI:
                # Display the resulting frame
                cv2.imshow("Visual Emotion Recognition", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    # break
                    return

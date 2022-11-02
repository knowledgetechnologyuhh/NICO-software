# -*- coding: utf-8
import argparse
import logging
import random
import sys
import time
import threading
import yaml
from os.path import abspath, dirname

from nicoaudio.TextToSpeech import TextToSpeech
from nicoemotionrecognition.EmotionRecognition import EmotionRecognition
from nicoface.FaceExpression import faceExpression
from nicomotion import Motion
from nicovision.VideoDevice import VideoDevice

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

if sys.version_info >= (3,):
    raw_input = input


class EmotionDemo(object):
    """NICO looks at a detected face, recognizes and mirrors the emotion that
    is shown and says different phrases depending on that emotion."""

    def __init__(
        self,
        camera=None,
        robot=None,
        face=None,
        voice_enabled=False,
        voice_cache_dir="/tmp",
        voice_use_cuda=False,
        german=False,
        face_tracking=False,
        mirror_emotion=False,
        tracking_delta=10,
        mirror_emotion_delta=10,
    ):
        self._camera = camera
        self._robot = robot
        self._face_expression = face
        self._voice_enabled = voice_enabled
        self._german = german
        self._face_tracking = face_tracking
        self._mirror_emotion = mirror_emotion
        self._tracking_delta = tracking_delta
        self._mirror_emotion_delta = mirror_emotion_delta

        self._same_emotion_counter = 0
        self._not_found_counter = 0
        self._tracking_counter = 0
        self._tts_end = 0
        self._update_face = False
        self._last_exp = "neutral"

        if voice_enabled:
            if german:
                self._tts = TextToSpeech(
                    model_name="tts_models/de/thorsten/vits",
                    cache_dir=voice_cache_dir,
                    use_cuda=voice_use_cuda,
                )
            else:
                self._tts = TextToSpeech(
                    cache_dir=voice_cache_dir, use_cuda=voice_use_cuda
                )
            self._tts_end = 0
            with open(
                dirname(abspath(__file__)) + "/voice_reactions.yml", "r"
            ) as voice_lines:
                self._voice_reactions = yaml.safe_load(voice_lines)[
                    "german" if german else "english"
                ]

        self._emotion_recognition = EmotionRecognition()
        self._camera.add_callback(self._callback)

        self.running = True

        def stop_client():
            raw_input("Press enter to stop\n")
            self.running = False

        t = threading.Thread(target=stop_client)
        t.daemon = True
        t.start()
        # update face in main thread to avoid issues with cv2.imshow
        if face is not None:
            self._face_expression.send_morphable_face_expression("neutral")
            time.sleep(2)
        while self.running:
            if self._update_face and face is not None:
                self._face_expression.morph_face_expression(self._last_exp)
                self._update_face = False

    def show_emotion(self, emotion):
        """
        Updates emotion and plays voice reaction if called enough consecutive
        times with the same emotion.

        :param emotion: detected emotion
        :type emotion: str
        """
        if self._last_exp != emotion:
            self._same_emotion_counter = 0
            self._last_exp = emotion
        else:
            self._same_emotion_counter += 1
            if self._same_emotion_counter > self._mirror_emotion_delta:
                self._update_face = True
                if self._voice_enabled and time.time() > self._tts_end:
                    self.voice_reaction(emotion)

    def voice_reaction(self, emotion, delay=5):
        """
        Plays random voice feedback for the given emotion

        :param emotion: detected emotion
        :type emotion: str
        :param delay: delay in seconds until another voice line will be played
        :type delay: float
        """
        if emotion in self._voice_reactions:
            sentence = random.choice(self._voice_reactions[emotion])
            if self._german:
                duration = self._tts.say(sentence, language="de", blocking=False)
            else:
                duration = self._tts.say(sentence, language="en", blocking=False)
            self._tts_end = time.time() + duration + delay

    def follow_face_with_head(self):
        """
        Moves head towards center of the detected face
        """
        if self._robot is not None:
            if self._tracking_counter == self._tracking_delta:
                center = self._emotion_recognition.get_face_center()
                if center is not None:
                    # (width - center_x)/width * FOV - FOV/2
                    # horizontal
                    angle_z = (640 - center[0]) / 640.0 * 60 - 60 / 2.0
                    # vertikal
                    angle_y = (480 - center[1]) / 480.0 * 50 - 50 / 2.0
                    self._robot.changeAngle("head_z", angle_z, 0.02)
                    self._robot.changeAngle("head_y", -angle_y, 0.02)
                self._tracking_counter = 0
            else:
                self._tracking_counter += 1
        else:
            logger.warning("No robot given on initialisation - skipping face tracking")

    def close(self):
        self._camera.close()
        self._emotion_recognition.close()
        if self._robot:
            self._robot.disableTorqueAll()
            self._robot.cleanup()

    def _callback(self, rval, frame):
        """
        Callback for the video device.

        :param rval: rval
        :param frame: frame
        """
        if frame is not None:
            self._emotion_recognition.send_image(frame)

            if self._emotion_recognition.face_detected:
                self._not_found_counter = 0

                if self._face_tracking:
                    self.follow_face_with_head()

                if self._mirror_emotion:
                    expression = (
                        self._emotion_recognition.get_highest_matching_emotion()
                    )
                    if expression is not None:
                        self.show_emotion(expression)
            else:
                if self._mirror_emotion and self._face_expression:
                    self._last_exp = "neutral"
                    self._update_face = True
                if self._face_tracking:
                    self._not_found_counter += 1
                    logger.info("Saw nothing: %i", self._not_found_counter)
                    if self._not_found_counter > 50:
                        # After 50 frames of not detecting something,
                        # randomly look around mid position
                        self._not_found_counter = 0
                        if self._robot is not None:
                            self._robot.setAngle(
                                "head_z", 0 + random.randint(-15, 15), 0.01
                            )
                            self._robot.setAngle(
                                "head_y", random.randint(-10, 10), 0.01
                            )
                        else:
                            logger.warning(
                                "No robot given on initialisation - "
                                + "skipping face tracking"
                            )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=(
            "NICO looks at a detected face, recognizes and mirrors the emotion "
            "that is shown and says different phrases depending on that emotion."
        )
    )
    default_config = abspath(
        dirname(abspath(__file__)) + "/../../../json/nico_humanoid_upper_rh7d.json"
    )
    parser.add_argument(
        "--motor-config",
        dest="json_file",
        type=str,
        default=default_config,
        help="Location of json file with motor definitions. (Default: {})".format(
            default_config
        ),
    )
    for camera_path in VideoDevice.autodetect_nicoeyes():
        if camera_path is not None:
            break
    parser.add_argument(
        "--camera-path",
        dest="camera_path",
        type=str,
        default=camera_path,
        help="Path of the used camera. (Autodetected: {})".format(camera_path),
    )
    parser.add_argument(
        "--german",
        action="store_true",
        help="Use german phrases for voice feedback (default is english)",
    )

    parser.add_argument(
        "--disable-voice",
        dest="voice",
        action="store_false",
        help="Disables voice feedback.",
    )

    parser.add_argument(
        "--disable-motion",
        dest="motion",
        action="store_false",
        help="Disables head movement",
    )

    parser.add_argument(
        "--disable-face",
        dest="face_enabled",
        action="store_false",
        help="Disables face expressions",
    )

    parser.add_argument(
        "--simulate-face",
        dest="simulate_face",
        action="store_true",
        help="Displays face as image rather than sending it to the real robot",
    )

    parser.add_argument(
        "--cache-dir",
        dest="voice_cache_dir",
        type=str,
        default="/tmp",
        help="Path to cache generated sound files. (Default: /tmp)",
    )
    parser.add_argument(
        "--use-cuda",
        dest="voice_use_cuda",
        action="store_true",
        help="Enable cuda for tts model",
    )
    # parser.add_argument(
    #     "--disable-gui", dest="gui", action="store_false", help="Disables the GUI."
    # )

    args = parser.parse_args()

    robot = None
    face = None
    if args.motion:
        robot = Motion.Motion(args.json_file, ignoreMissing=True)
    if args.face_enabled:
        face = faceExpression(simulation=args.simulate_face)

    logger.info("Using camera %s", args.camera_path)

    camera = VideoDevice.from_device(args.camera_path)

    if "See3CAM" in args.camera_path:
        camera.zoom(300)

    emotion_demo = EmotionDemo(
        camera,
        robot,
        face,
        voice_enabled=args.voice,
        voice_cache_dir=args.voice_cache_dir,
        voice_use_cuda=args.voice_use_cuda,
        german=args.german,
        face_tracking=args.motion,
        mirror_emotion=args.face_enabled,
        tracking_delta=10,
        mirror_emotion_delta=10,
    )

    emotion_demo.close()

    time.sleep(1.0)

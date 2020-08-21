#!/usr/bin/env python

import rospy
import nicomsg.srv as srv
from nicoaudio.TextToSpeech import TextToSpeech


class NicoRosTTS(object):
    """
    The NicoRosTTS class exposes the functions of
    :class:`nicoaudio.TextToSpeech` to ROS
    """

    def __init__(self, cache_dir="/tmp", port="http://0.0.0.0:5002/api/tts"):
        """
        The NicoRosTTS class exposes the functions of
        :class:`nicoaudio.TextToSpeech` to ROS

        :param cache_dir: directory where generated audiofiles should be
                          stored.
        :type cache_dir: str
        :param port: port at which mozilla tts is hosted (uses fallbacks if
                     connection fails)
        :type port: str
        """
        rospy.init_node("text_to_speech", anonymous=True)
        self.tts = TextToSpeech(cache_dir, port)
        rospy.Service("nico/text_to_speech/say", srv.SayText, self._ROSPY_speak)
        rospy.spin()

    def _ROSPY_speak(self, request):
        """
        Callback handle for :meth:`nicoaudio.AudioPlayer.position`

        :param request: ROS service request
        :type request: nicomsg.srv.SayTextRequest
        :return: duration
        :rtype: float
        """
        duration = self.tts.say(
            request.text,
            request.language,
            request.pitch,
            request.speed,
            request.blocking,
        )
        return duration


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--cache_dir",
        default="/tmp",
        nargs="?",
        help=("The directory where generated soundfiles are stored for reuse"),
    )
    parser.add_argument(
        "--port",
        default="http://0.0.0.0:5002/api/tts",
        nargs="?",
        help=("Port of the tts server (if any)"),
    )
    args = parser.parse_args()

    NicoRosTTS(args.cache_dir, args.port)

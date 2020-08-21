#!/usr/bin/env python

import rospy
import uuid
import nicomsg.msg as msg
import nicomsg.srv as srv
from nicoaudio.AudioPlayer import AudioPlayer


class NicoRosAudioPlayer(object):
    """
    The NicoRosAudioPlayer class manages and exposes the functions of
    multiple :class:`nicoaudio.AudioPlayer` instances to ROS
    """

    def __init__(self):
        """
        The NicoRosAudioPlayer class manages and exposes the functions of
        multiple :class:`nicoaudio.AudioPlayer` instances to ROS
        """
        self.audiofiles = {}

        rospy.init_node("audioplayer", anonymous=True)

        # init services
        rospy.Service(
            "nico/audioplayer/load_file", srv.LoadAudio, self._ROSPY_load_audio_segment
        )
        rospy.Service(
            "nico/audioplayer/get_position", srv.GetValue, self._ROSPY_current_position
        )
        rospy.Service(
            "nico/audioplayer/get_duration", srv.GetValue, self._ROSPY_duration
        )
        rospy.Service(
            "nico/audioplayer/get_filename", srv.GetFilename, self._ROSPY_filename
        )
        rospy.Service(
            "nico/audioplayer/get_audio_ids", srv.GetAudioIDs, self._ROSPY_audio_ids
        )

        # init subscribers
        rospy.Subscriber("nico/audioplayer/play", msg.sf, self._ROSPY_play)
        rospy.Subscriber("nico/audioplayer/pause", msg.s, self._ROSPY_pause)
        rospy.Subscriber("nico/audioplayer/resume", msg.s, self._ROSPY_resume)
        rospy.Subscriber(
            "nico/audioplayer/adjust_pitch", msg.sf, self._ROSPY_adjust_pitch
        )
        rospy.Subscriber(
            "nico/audioplayer/adjust_speed", msg.sf, self._ROSPY_adjust_speed
        )

        rospy.spin()

    # ----------------- Services ----------------------------------------------

    def _ROSPY_load_audio_segment(self, request):
        """
        Initializes new AudioPlayer for given file and returns id.

        :param request: ROS service request
        :type request: nicomsg.srv.LoadAudioRequest
        :return: id of the loaded audio file
        :rtype: str
        """
        if request.duration_seconds > 0:
            duration = request.duration_seconds
        else:
            duration = None
        id = uuid.uuid4().hex
        p = AudioPlayer(request.filename, request.start_seconds, duration)
        self.audiofiles[id] = p
        return id

    def _ROSPY_current_position(self, request):
        """
        Callback handle for :meth:`nicoaudio.AudioPlayer.position`

        :param request: ROS service request
        :type request: nicomsg.srv.GetValueRequest
        :return: playback position of the loaded audio segment with given id
        :rtype: float
        """
        p = self.audiofiles[request.param1]
        return p.position

    def _ROSPY_duration(self, request):
        """
        Callback handle for :meth:`nicoaudio.AudioPlayer.duration`

        :param request: ROS service request
        :type request: nicomsg.srv.GetValueRequest
        :return: duration of the loaded audio segment with given id
        :rtype: float
        """
        p = self.audiofiles[request.param1]
        return p.duration

    def _ROSPY_filename(self, request):
        """
        Callback handle for :meth:`nicoaudio.AudioPlayer.duration`

        :param request: ROS service request
        :type request: nicomsg.srv.GetFilenameRequest
        :return: filename of the loaded audio segment with given id
        :rtype: str
        """
        p = self.audiofiles[request.id]
        return p.filename

    def _ROSPY_audio_ids(self, request):
        """
        Returns ids of all loaded audio segments

        :param request: ROS service request
        :type request: nicomsg.srv.GetAudioIDs
        :return: filename of the loaded audio segment with given id
        :rtype: str
        """
        return self.audiofiles.keys()

    # ---------------- Subscriber ---------------------------------------------

    def _ROSPY_play(self, message):
        """
        Callback handle for :meth:`nicoaudio.AudioPlayer.play`

        :param message: ROS message
        :type message: nicomsg.msg.sf
        """
        rospy.loginfo(
            "Starting playback of %s at %f volume", message.param1, message.param2
        )
        p = self.audiofiles[message.param1]
        p.play(message.param2)

    def _ROSPY_pause(self, message):
        """
        Callback handle for :meth:`nicoaudio.AudioPlayer.pause`

        :param message: ROS message
        :type message: nicomsg.msg.s
        """
        rospy.loginfo("Pausing %s", message.param1)
        p = self.audiofiles[message.param1]
        p.pause()

    def _ROSPY_resume(self, message):
        """
        Callback handle for :meth:`nicoaudio.AudioPlayer.resume`

        :param message: ROS message
        :type message: nicomsg.msg.s
        """
        rospy.loginfo("Resume playback of %s", message.param1)
        p = self.audiofiles[message.param1]
        p.resume()

    def _ROSPY_adjust_pitch(self, message):
        """
        Callback handle for :meth:`nicoaudio.AudioPlayer.pitch`

        :param message: ROS message
        :type message: nicomsg.msg.sf
        """
        rospy.loginfo("Adjusting pitch of %s by %f", message.param1, message.param2)
        p = self.audiofiles[message.param1]
        p.pitch(message.param2)

    def _ROSPY_adjust_speed(self, message):
        """
        Callback handle for :meth:`nicoaudio.AudioPlayer.speed`

        :param message: ROS message
        :type message: nicomsg.msg.sf
        """
        rospy.loginfo("Multiplying speed of %s by %f", message.param1, message.param2)
        p = self.audiofiles[message.param1]
        p.speed(message.param2)


if __name__ == "__main__":
    audioplayer = NicoRosAudioPlayer()

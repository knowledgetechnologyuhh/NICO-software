import logging
import threading
import time
from io import BytesIO

from audiotsm import phasevocoder
from audiotsm.io.wav import WavReader, WavWriter
from pyaudio import PyAudio
from pydub import AudioSegment

logger = logging.getLogger(__name__)


def get_pulse_device():
    """
    Detects pulse device

    :return: id of the pulse device
    :rtype: int
    """
    # print("The following sound devices are available.")
    p = PyAudio()
    info = p.get_host_api_info_by_index(0)
    numdevices = info.get("deviceCount")
    audio_device = 0
    for i in range(0, numdevices):
        if (
            p.get_device_info_by_host_api_device_index(0, i).get("maxOutputChannels")
        ) > 0:
            device_name = p.get_device_info_by_host_api_device_index(0, i).get("name")
            # print("Input Device id ", i, " - ", device_name)
            if device_name.find("pulse") != -1:
                audio_device = i
    return audio_device


class AudioPlayer(object):
    """
    The AudioPlayer class allows asynchronous playback of
    (a segment of) a single audio file.
    """

    def __init__(self, filename, start=0, duration=None, audio_device=None):
        """
        The AudioPlayer allows asynchronous playback of (a segment of) a
        single audio file. The same object can be used for subsequent replays.

        :param filename: audio file that should be loaded
        :type filename: str
        :param start: start position in seconds, preceding audio is not loaded
        :type start: float
        :param duration: duration in seconds (relative to start),
                         subsequent audio is not loaded if duration is set.
        :type duration: float
        :param audio_device: id of audio_device, tries to autodetect pulse
                             device if None.
        :type audio_device: int
        """
        if audio_device is None:
            self._audio_device = get_pulse_device()
        else:
            self._audio_device = audio_device
        self._filename = filename
        self._pos = start
        self._thread = None
        self._running = False

        segment = AudioSegment.from_file(filename)
        start *= 1000
        if duration is None:
            stop = len(segment)
        else:
            stop = min(len(segment), start + duration * 1000)

        if start > stop:
            logger.warning(
                "Could not play {} - start position set after "
                "end of audio ({}s > {}s)".format(
                    filename, start / 1000.0, stop / 1000.0
                )
            )

        self._segment = segment[start:stop]
        self._min_dbfs = -100
        self._max_dbfs = segment.dBFS

    @property
    def position(self):
        """
        Current playback position

        :return: Position in seconds
        :rtype: float
        """
        return self._pos / 1000.0

    @property
    def duration(self):
        """
        Duration of the loaded audio segment

        :return: Duration in seconds
        :rtype: float
        """
        return self._segment.duration_seconds

    @property
    def filename(self):
        """
        Name of the loaded file

        :return: filename
        :rtype: str
        """
        return self._filename

    @property
    def volume(self):
        """
        Percentage of volume

        :return: Volume [0.0, 1.0]
        :rtype: float
        """
        return (self._segment.dBFS - self._min_dbfs) / (self._max_dbfs - self._min_dbfs)

    @volume.setter
    def volume(self, percentage):
        """
        Setter for volume

        :param percentage: Volume [0.0, 1.0]
        :type percentage: float
        """
        vol = percentage * (self._max_dbfs - self._min_dbfs) + self._min_dbfs
        self._segment += vol - self._segment.dBFS

    def pitch(self, octaves):
        """
        Shifts pitch in octaves (this affects the speed)

        :param pitch: Pitch in octaves by which to shift the output
                      (this affects the speed)
        :type pitch: float
        """
        if octaves != 0:
            logger.info("Setting pitch to %f", octaves)
            new_sample_rate = int(self._segment.frame_rate * (2.0 ** octaves))
            self._segment = self._segment._spawn(
                self._segment.raw_data, overrides={"frame_rate": new_sample_rate}
            )

    def speed(self, speed):
        """
        Adjusts speed to given percentage without changing pitch

        :param speed: Percentage to increase/decrease speed without changing
                      pitch
        :type speed: float
        """
        if speed != 1:
            logger.info("Setting speed to %f", speed)
            logger.debug("Export file to BytesIO")
            wav_in = BytesIO()
            wav_in = self._segment.export(wav_in, format="wav")
            wav_in.seek(0)
            logger.debug("Initializing reader and writer")
            with WavReader(wav_in) as reader:
                wav_out = BytesIO()
                with WavWriter(wav_out, reader.channels, reader.samplerate) as writer:
                    logger.debug("Adjusting speed with vocoder")
                    tsm = phasevocoder(reader.channels, speed=speed)
                    tsm.run(reader, writer)
                    logger.debug("Reload audio segment")
                    wav_out.seek(44)  # skip metadata and start at first sample
                    self._segment = AudioSegment.from_raw(
                        wav_out,
                        sample_width=self._segment.sample_width,
                        channels=self._segment.channels,
                        frame_rate=self._segment.frame_rate,
                    )

    def play(self, volume=1.0):
        """
        Starts playback of the audio segment from its beginning

        :param percentage: Volume [0.0, 1.0]
        :type percentage: float
        """
        if self._thread is None:
            self.volume = volume
            self._thread = threading.Thread(target=self._playback)
            self._pos = 0
            self._running = True
            self._thread.start()
        else:
            logger.warning("Task for file {} is already running".format(self.filename))

    def pause(self):
        """
        Stops playback of the audio segment
        """
        self._running = False
        self._thread.join()
        self._thread = None

    def resume(self):
        """
        Continues playback of the file from where it was previously stopped.
        """
        if self._thread is None:
            self._thread = threading.Thread(target=self._playback)
            self._thread.start()
            self._running = True
        else:
            logger.warning("Task for file {} is already running".format(self.filename))

    def _playback(self):
        """
        Playback function for thread. Plays a segment until running is set to
        false or end is reached
        """
        # open stream
        p = PyAudio()
        stream = p.open(
            format=p.get_format_from_width(self._segment.sample_width),
            channels=self._segment.channels,
            rate=self._segment.frame_rate,
            output=True,
            output_device_index=self._audio_device,
        )

        # output the file in millisecond steps
        while self._pos < len(self._segment) and self._running:
            stream.write(self._segment[self._pos]._data)
            self._pos += 1

        # close stream
        time.sleep(stream.get_output_latency())
        stream.stop_stream()
        stream.close()

        p.terminate()

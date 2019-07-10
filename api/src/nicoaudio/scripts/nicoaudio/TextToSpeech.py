import datetime
import json
import logging
import os
import subprocess
import time

from gtts import gTTS, lang
from nicoaudio.AudioPlayer import AudioPlayer

logger = logging.getLogger(__name__)


class TextToSpeech(object):
    """
    TextToSpeech is a simple text to speech module for different languages
    """

    def __init__(self, cache_dir="/tmp"):
        """
        TextToSpeech allows simple text to speech using gTTS if google is
        available or pico2wave as fallback if there is no cached file.

        :param cache_dir: directory where generated audiofiles should be
                          stored.
        :type cache_dir: str
        """
        # load cache dictionary
        if not os.path.isdir(cache_dir):
            logger.error(
                "Cache directory '{}' does not exist.".format(cache_dir))
            raise IOError("Directory '{}' does not exist.".format(cache_dir))
        cache_file = "{}/NICO_speech_cache.json".format(cache_dir)
        if os.path.isfile(cache_file):
            logger.info("Loading '{}'".format(cache_file))
            with open(cache_file) as f:
                self._cache = json.load(f)
        else:
            logger.info("File {} not found - initializing empty "
                        "cache".format(cache_file))
            self._cache = {}
        self._cache_dir = cache_dir
        self._cache_file = cache_file

    def say(self, text, language="en-GB", pitch=0, speed=1., blocking=True):
        """
        Generates and plays spoken text using gTTS if google is available or
        pico2wave as fallback if there is no cached file available.

        :param text: text to speak
        :type filename: str
        :param language: Language code (e.g. 'en-GB' or 'de')
        :type language: str
        :param pitch: Pitch in octaves by which to shift the output
                      (this affects the speed)
        :type pitch: float
        :param speed: Percentage to increase/decrease speed without changing
                      pitch
        :type speed: float
        :param blocking: whether this call should block during playback
        :type blocking: bool

        :return: Playback duration left (0 if blocking)
        :rtype: float
        """
        if text in self._cache and False:
            # load cached file if it exists
            filename = self._cache[text]
            logger.info("Using cached audio file {}".format(filename))
            playback = AudioPlayer(filename)
        elif os.system("ping -c 1 google.com > /dev/null 2>&1") == 0:
            # use gTTS if google is available

            # google uses 'de' whereas pico2wave uses 'de-DE'
            if language == "de-DE":
                language = "de"

            if language.lower() not in lang.tts_langs():
                logger.error(
                    "Language '{}' not supported by gTTS - supported languages"
                    " are:\n".format(language) +
                    "\n".join(["{} - {}".format(k, v) for k, v in
                               lang.tts_langs().iteritems()]))
                e = IOError()
                raise e

            # generate audio file
            tts = gTTS(text, language, slow=False)
            filename = '/tmp/NICO_speech_{}.mp3'.format(
                datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S.%f"))
            tts.save(filename)
            playback = AudioPlayer(filename)

            # cache audio file
            self._cache[text] = filename
            with open(self._cache_file, "wb") as f:
                f.write(json.dumps(self._cache))
        else:
            # otherwise use pico2wave as fallback
            logger.warn("Could not connect to google "
                        "- using pico2wave fallback")
            # google uses 'de' whereas pico2wave uses 'de-DE'
            if language == "de":
                language = "de-DE"
            try:
                filename = '/tmp/NICO_speech_{}.wav'.format(
                    datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S.%f"))
                subprocess.call(["pico2wave", "-l", language,
                                 "-w", filename, text])
                playback = AudioPlayer(filename)
                os.remove(filename)
            except OSError as e:
                logger.error(
                    "Could not execute pico2wave - make sure "
                    "'libttspico-utils' apt package is installed")
                raise
            except IOError as e:
                logger.error(
                    "Language '{}' is not supported by pico2wave".format(
                        language))
                raise

        # play audio file
        playback.pitch(pitch)
        playback.speed(speed)
        playback.play()
        if blocking:
            time.sleep(playback.duration)
        return playback.duration - playback.position


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)

    tts = TextToSpeech()
    # default
    tts.say("This is a test sentence.")
    # non-blocking
    duration = tts.say("This is also a test sentence, but the call doesn't "
                       "block.", blocking=False)
    logger.info("Sleep until non-blocking call is done.")
    time.sleep(duration)
    # different language, e.g german
    tts.say("Dies ist ein deutscher Beispielsatz.", language="de")

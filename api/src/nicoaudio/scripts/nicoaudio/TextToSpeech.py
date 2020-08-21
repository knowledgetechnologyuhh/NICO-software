import datetime
import json
import logging
import os
import subprocess
import time
from io import BytesIO

import requests
from gtts import gTTS, lang
from nicoaudio.AudioPlayer import AudioPlayer

logger = logging.getLogger(__name__)


class TextToSpeech(object):
    """
    TextToSpeech is a simple text to speech module for different languages
    """

    def __init__(self, cache_dir="/tmp", port="http://0.0.0.0:5002/api/tts"):
        """
        TextToSpeech allows simple text to speech using locally hosted mozilla
        tts (https://github.com/mozilla/TTS), gTTS if google is available or
        pico2wave as fallback if there is no cached file.

        :param cache_dir: directory where generated audiofiles should be
                          stored.
        :type cache_dir: str
        :param port: port at which mozilla tts is hosted (uses fallbacks if
                     connection fails)
        :type port: str
        """
        # load cache dictionary
        if not os.path.isdir(cache_dir):
            logger.error("Cache directory '%s' does not exist.", cache_dir)
            raise IOError("Directory '{}' does not exist.".format(cache_dir))
        cache_file = "{}/NICO_speech_cache.json".format(cache_dir)
        if os.path.isfile(cache_file):
            logger.info("Loading '%s'", cache_file)
            with open(cache_file) as f:
                self._cache = json.load(f)
        else:
            logger.info("File %s not found - initializing empty cache", cache_file)
            self._cache = {}
        self._cache_dir = cache_dir
        self._cache_file = cache_file
        self._port = port

    def say(self, text, language="en-GB", pitch=0, speed=1.0, blocking=True):
        """
        Generates and plays spoken text using gTTS if google is available or
        pico2wave as fallback if there is no cached file available.

        :param text: text to speak
        :type text: str
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
        # try to use mozilla tts server
        tts_server_online = False
        if language.startswith("en"):
            try:
                response = requests.get(self._port, params={"text": text})
                tts_server_online = True
            except requests.exceptions.ConnectionError:
                logger.warn("Failed to connect to port %s", self._port)

        if tts_server_online and response.headers["Content-Type"] == "audio/wav":
            # play server response if online
            file = BytesIO(response.content)
            playback = AudioPlayer(file)
        elif text + language in self._cache:
            # load cached file if it exists
            file = self._cache[text + language]
            logger.info("Using cached audio file %s", file)
            playback = AudioPlayer(file)
        elif os.system("ping -c 1 google.com > /dev/null 2>&1") == 0:
            # use gTTS if google is available

            # google uses 'de' whereas pico2wave uses 'de-DE'
            if language == "de-DE":
                language = "de"

            if language.lower() not in lang.tts_langs():
                logger.error(
                    "Language '{}' not supported by gTTS - supported languages"
                    " are:\n".format(language)
                    + "\n".join(
                        [
                            "{} - {}".format(k, v)
                            for k, v in lang.tts_langs().iteritems()
                        ]
                    )
                )
                e = IOError()
                raise e

            # generate audio file
            tts = gTTS(text, lang=language, slow=False)
            file = "/tmp/NICO_speech_{}.mp3".format(
                datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S.%f")
            )
            tts.save(file)
            playback = AudioPlayer(file)

            # cache audio file
            self._cache[text + language] = file
            with open(self._cache_file, "w") as f:
                json.dump(self._cache, f)
        else:
            # otherwise use pico2wave as fallback
            logger.warn("Could not connect to google " "- using pico2wave fallback")
            # google uses 'de' whereas pico2wave uses 'de-DE'
            if language == "de":
                language = "de-DE"
            try:
                file = "/tmp/NICO_speech_{}.wav".format(
                    datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S.%f")
                )
                subprocess.call(["pico2wave", "-l", language, "-w", file, text])
                playback = AudioPlayer(file)
                os.remove(file)
            except OSError:
                logger.error(
                    "Could not execute pico2wave - make sure "
                    "'libttspico-utils' apt package is installed"
                )
                raise
            except IOError:
                logger.error(
                    "Language '{}' is not supported by pico2wave".format(language)
                )
                raise

        # play audio file
        playback.pitch(pitch)
        playback.speed(speed)
        playback.play()
        if blocking:
            time.sleep(playback.duration)
        return playback.duration - playback.position


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    tts = TextToSpeech()
    # default
    tts.say("This is a test sentence.")
    # non-blocking
    duration = tts.say(
        "This is also a test sentence, but the call doesn't " "block.", blocking=False
    )
    logger.info("Sleep until non-blocking call is done.")
    time.sleep(duration)
    # different language, e.g german
    tts.say("Dies ist ein deutscher Beispielsatz.", language="de")

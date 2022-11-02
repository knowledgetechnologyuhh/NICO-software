import datetime
import json
import logging
import os
from os.path import dirname, abspath, join
import time

from nicoaudio.AudioPlayer import AudioPlayer

import TTS
from TTS.utils.manage import ModelManager
from TTS.utils.synthesizer import Synthesizer

logger = logging.getLogger(__name__)


class TextToSpeech(object):
    """
    TextToSpeech is a simple text to speech module for different languages
    """

    def __init__(
        self, cache_dir="/tmp", model_name="tts_models/en/vctk/vits", use_cuda=False
    ):
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
        # init TTS
        tts_path = join(dirname(abspath(TTS.__file__)), ".models.json")
        manager = ModelManager(tts_path)
        model_path, config_path, model_item = manager.download_model(model_name)
        self._synthesizer = Synthesizer(
            model_path,
            config_path,
            None,  # speakers_file_path,
            None,  # language_ids_file_path,
            None,  # vocoder_path,
            None,  # vocoder_config_path,
            None,  # encoder_path,
            None,  # encoder_config_path,
            use_cuda,
        )
        self._model_name = model_name

    @staticmethod
    def list_models():
        tts_path = join(dirname(abspath(TTS.__file__)), ".models.json")
        manager = ModelManager(tts_path)
        return manager.list_models()

    def get_speaker_ids(self):
        if self._synthesizer.tts_model.speaker_manager is not None:
            return self._synthesizer.tts_model.speaker_manager.speaker_names
        else:
            return None

    def say(
        self,
        text,
        language="en",
        pitch=0.25,
        speed=2 ** -0.25,
        blocking=True,
        speaker="p336",
    ):
        """
        Generates and plays spoken text using gTTS if google is available or
        pico2wave as fallback if there is no cached file available.

        :param text: text to speak
        :type text: str
        :param language: Language code (e.g. 'en' or 'de')
        :type language: str
        :param pitch: Pitch in octaves by which to shift the output
        :type pitch: float
        :param speed: Percentage to increase/decrease speed without changing
                      pitch
        :type speed: float
        :param blocking: whether this call should block during playback
        :type blocking: bool

        :return: Playback duration left (0 if blocking)
        :rtype: float
        """
        logger.info("Saying: {}".format(text))
        # prevent models without speaker selection from crashing
        if self._synthesizer.tts_model.speaker_manager is None:
            speaker = None
            cached_speaker = "null"
        else:
            cached_speaker = speaker
        # use cached file if possible
        if (
            self._model_name in self._cache
            and language in self._cache[self._model_name]
            and cached_speaker in self._cache[self._model_name][language]
            and text in self._cache[self._model_name][language][cached_speaker]
        ):
            out = self._cache[self._model_name][language][cached_speaker][text]
            logger.info("Using cached file {}".format(out))
        else:
            logger.info("Generating audio file...")
            # generate audio from text
            wav = self._synthesizer.tts(
                text, speaker, language
            )  # TODO other parameters?
            # save result in cache dir
            out = "{}/NICO_speech_{}.wav".format(
                self._cache_dir,
                datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S.%f"),
            )
            self._synthesizer.save_wav(wav, out)
            logger.info("Audio saved as {}".format(out))
            # add missing keys to cache
            if self._model_name not in self._cache:
                self._cache[self._model_name] = {}
            if language not in self._cache[self._model_name]:
                self._cache[self._model_name][language] = {}
            if cached_speaker not in self._cache[self._model_name][language]:
                self._cache[self._model_name][language][cached_speaker] = {}
            # cache filename
            self._cache[self._model_name][language][cached_speaker][text] = out
            with open(self._cache_file, "w") as f:
                json.dump(self._cache, f)
            logger.info("Updated cache")
        # load audio
        playback = AudioPlayer(out)

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
        "This is also a test sentence, but the call doesn't block.", blocking=False
    )
    logger.info("Sleep until non-blocking call is done.")
    time.sleep(duration)
    # different language, e.g german
    tts.say("Dies ist ein deutscher Beispielsatz.", language="de")

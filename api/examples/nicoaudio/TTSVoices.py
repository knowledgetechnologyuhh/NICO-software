import logging
import time

from nicoaudio.TextToSpeech import TextToSpeech

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# change cache_dir for permanent caching of audiofiles
tts = TextToSpeech(cache_dir="/tmp")

for speaker in tts.get_speaker_ids():
    print(speaker)

    # default
    tts.say("This is a test sentence.", speaker=speaker)

    # change pitch and re-adjust speed
    tts.say(
        "This is a test sentence with increased pitch but readjusted speed.",
        pitch=0.2,
        speed=2 ** -0.2,
        speaker=speaker,
    )

    tts.say(
        "Please put the object in my hand.", pitch=0.2, speed=2 ** -0.2, speaker=speaker
    )

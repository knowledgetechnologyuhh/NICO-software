import logging
import time

from nicoaudio.TextToSpeech import TextToSpeech

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# change cache_dir for permanent caching of audiofiles
tts = TextToSpeech(cache_dir="/tmp", use_cuda=True)

# default
tts.say(
    "This is a test sentence with the default NICO voice, which has increased pitch and readjusted speed."
)

# non-blocking
duration = tts.say(
    "This is also a test sentence, but the call doesn't block.", blocking=False
)
logger.info("Sleep until non-blocking call is done.")
time.sleep(duration)

# clean
tts.say(
    "This is a test sentence with the unaltered model result.", pitch=0.0, speed=1.0
)

# change pitch
tts.say("This is a test sentence with a slightly lower pitch.", pitch=-0.125)

# change speed without changing pitch
tts.say("This is a fast test sentence.", speed=1.3)

# show available speaker ids
print("Available speakers for the selected model:")
print(tts.get_speaker_ids())

# use different speaker voice
tts.say("This is a test sentence with another voice.", speaker="p292")

# change model for different language, e.g german
tts = TextToSpeech(
    cache_dir="/tmp", model_name="tts_models/de/thorsten/vits", use_cuda=True
)
tts.say("Dies ist ein deutscher Beispielsatz.")

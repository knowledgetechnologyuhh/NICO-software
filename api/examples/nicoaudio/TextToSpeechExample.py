import logging
import time

from nicoaudio.TextToSpeech import TextToSpeech

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# change cache_dir for permanent caching of audiofiles
tts = TextToSpeech(cache_dir="/tmp")

# default
tts.say("This is a test sentence.")

# change pitch
tts.say("This is a test sentence with a slightly lower pitch.", pitch=-0.1)

# change speed without changing pitch
tts.say("This is a fast test sentence.", speed=1.3)

# change pitch and re-adjust speed
tts.say("This is a test sentence with increased pitch but readjusted speed.",
        pitch=0.2, speed=2**-0.2)

# non-blocking
duration = tts.say("This is also a test sentence, but the call doesn't "
                   "block.", blocking=False)
logger.info("Sleep until non-blocking call is done.")
time.sleep(duration)

# different language, e.g german
tts.say("Dies ist ein deutscher Beispielsatz.", language="de")

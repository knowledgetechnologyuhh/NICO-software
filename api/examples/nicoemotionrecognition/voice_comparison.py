import argparse
import numpy as np
from picotts import PicoTTS
from io import BytesIO
from os.path import dirname, abspath, join
import time
import yaml

from nicoaudio.AudioPlayer import AudioPlayer
from nicoaudio.TextToSpeech import TextToSpeech
from nicoface.FaceExpression import faceExpression

parser = argparse.ArgumentParser()
parser.add_argument(
    "-s",
    action="store_true",
    help="Enables simulated mode, where faces are shown as image instead",
)
args = parser.parse_args()

voice_script = join(dirname(abspath(__file__)), "voice_reactions.yml")

pico_tts = PicoTTS("en-GB")
vits_tts = TextToSpeech(
    load_tts=False, cache_dir=join(dirname(abspath(__file__)), "tts_storage")
)

face = faceExpression(simulation=args.s)
face.send_morphable_face_expression("neutral")

np.random.seed(42)

with open(voice_script, "r") as f:
    voice_reactions = yaml.safe_load(f)
    for emotion, reactions in voice_reactions["english"].items():
        print(emotion)
        face.morph_face_expression(emotion)
        order = np.random.permutation(range(3))
        line = np.random.choice(reactions)
        print(line)
        # for line in reactions:
        for i in order:
            if i == 0:
                wav_bytes = pico_tts.synth_wav(line)
                f = BytesIO(wav_bytes)
                playback = AudioPlayer(f)
                playback.play()
                time.sleep(playback.duration)
            elif i == 1:
                vits_tts.say(line, pitch=0.0, speed=1.0)
            else:
                vits_tts.say(line)

import os
from os.path import join, dirname, abspath, isdir
import yaml

from nicoaudio.TextToSpeech import TextToSpeech

# path for file storage
cache_dir = join(dirname(abspath(__file__)), "tts_storage")

# create directory
if not isdir(cache_dir):
    os.mkdir(cache_dir)

# load voice line dict from voice_reactions.yml
with open(
    join(dirname(abspath(__file__)), "voice_reactions.yml"), "r"
) as reaction_data:
    voice_lines = yaml.safe_load(reaction_data)

# load default tts model
tts = TextToSpeech(cache_dir=cache_dir, use_cuda=True)
# generate english voice lines for each emotion
print("generating english voice lines")
for emotion, reactions in voice_lines["english"].items():
    print(f"generating voice lines for '{emotion}'")
    for reaction in reactions:
        print(f"generating '{reaction}'")
        tts.generate_audio(reaction)
del tts

# load german tts model
tts = TextToSpeech(
    model_name="tts_models/de/thorsten/vits", cache_dir=cache_dir, use_cuda=True
)
# generate german voice lines for each emotion
print("generating german voice lines")
for emotion, reactions in voice_lines["german"].items():
    print(f"generating voice lines for '{emotion}'")
    for reaction in reactions:
        print(f"generating '{reaction}'")
        tts.generate_audio(reaction, language="de")

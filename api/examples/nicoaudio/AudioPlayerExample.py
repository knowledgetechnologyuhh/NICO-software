import time

from nicoaudio.AudioPlayer import AudioPlayer
from nicoaudio.TextToSpeech import TextToSpeech

# generate audio file using tts (see TextToSpeechExample for detailed usage)
tts = TextToSpeech()
file_name = tts.generate_audio(
    "This is an example to showcase the usage of the audio player class, which will be paused for a few seconds now, and then resumes playback until it reaches the end of the audio file."
)

# load file
playback = AudioPlayer(file_name)
# start playback
playback.play(volume=1.0)
# wait until it approximately says "now"
time.sleep(4.75 - playback.position)
# pause playback for 2 seconds
playback.pause()
time.sleep(2.0)
playback.resume()
# wait until playback is finished
time.sleep(playback.duration - playback.position)

import math
import os
import struct
import time
import wave

from nicoaudio.AudioPlayer import AudioPlayer


def generate_audio_file(volume=0.5, sampleRate=44100, duration=5.0,
                        frequency=440.0):
    # taken from http://blog.acipo.com/wave-generation-in-python/
    wav = wave.open("/tmp/NICO_playback_test.wav", "wb")
    wav.setnchannels(1)
    wav.setsampwidth(2)
    wav.setframerate(sampleRate)

    for i in range(int(duration * sampleRate)):
        value = int(volume * 32767.0 * math.cos(frequency *
                                                math.pi *
                                                float(i) / float(sampleRate)))
        data = struct.pack('<h', value)
        wav.writeframesraw(data)
    wav.close()


generate_audio_file()

# load file
playback = AudioPlayer("/tmp/NICO_playback_test.wav")
# start playback
playback.play()
# wait until playback is finished
time.sleep(playback.duration - playback.position)
# delete file
os.remove("/tmp/NICO_playback_test.wav")

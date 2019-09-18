import logging
import sys

from nicoaudio.pulse_audio_recorder import AudioRecorder, get_pulse_device

logging.basicConfig(level=logging.INFO)

audio_device = get_pulse_device()

dev_idx = audio_device
directory = "./."

ar = AudioRecorder(audio_channels=2, samplerate=48000,
                   datadir=directory, audio_device=dev_idx)

if sys.version_info >= (3,):
    raw_input = input

while True:
    print("Enter label for recording or \"0\" to exit")
    label = raw_input()
    if label == "0":
        break
    rec_id = 0
    ar.start_recording(label, fname=label + ".wav", dir_name=directory)
    print("Press Enter to stop recording")
    raw_input()
    ar.stop_recording(0)

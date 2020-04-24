# Authors:
# Manfred Eppe
# adapted to NICO-API
# Erik, Connor

#  Compatibility imports
from __future__ import absolute_import, division, print_function
from functools import wraps

import logging
import os

# from threading import Thread
import threading
import time
import wave

import alsaaudio
import pyaudio

logger = logging.getLogger(__name__)


def ensure_dir(directory):
    """
    Creates directory if it doesn't exist, as well as missing intermediate ones.

    :param directory: path of the directory
    :type directory: str
    """
    # directory = os.path.dirname(file_path)
    if not os.path.isdir(directory):
        os.makedirs(directory)
    else:
        logger.info(
            ("Skipped creating directory {} as it already exists").format(directory)
        )


def get_pulse_device():
    """
    Detects pulse device

    :return: id of the pulse device
    :rtype: int
    """
    # print("The following sound devices are available.")
    p = pyaudio.PyAudio()
    info = p.get_host_api_info_by_index(0)
    numdevices = info.get("deviceCount")
    audio_device = 0
    for i in range(0, numdevices):
        if (
            p.get_device_info_by_host_api_device_index(0, i).get("maxInputChannels")
        ) > 0:
            device_name = p.get_device_info_by_host_api_device_index(0, i).get("name")
            # print("Input Device id ", i, " - ", device_name)
            if device_name.find("pulse") != -1:
                audio_device = i
    return audio_device


def threaded(f, daemon=False):
    """function decorator"""
    import Queue

    def wrapped_f(q, *args, **kwargs):
        """this function calls the decorated function and puts the
        result in a queue"""
        ret = f(*args, **kwargs)
        q.put(ret)

    @wraps(f)
    def wrap(*args, **kwargs):
        """this is the function returned from the decorator. It fires off
        wrapped_f in a new thread and returns the thread object with
        the result queue attached"""

        q = Queue.Queue()

        t = threading.Thread(target=wrapped_f, args=(q,) + args, kwargs=kwargs)
        t.daemon = daemon
        t.start()
        t.result_queue = q
        return t

    return wrap


class AudioRecorder:
    def __init__(self, audio_channels, samplerate, datadir, audio_device):
        """
        Initialize audio recorder

        :param audio_channels: number of channels
        :type audio_channels: int
        :param samplerate: samplerate of the recording
        :type samplerate: int
        :param datadir: data directory
        :type datadir: str
        :param audio_device: device id
        :type audio_device: int
        """
        self.audio_channels = audio_channels
        self.samplerate = samplerate
        self.datadir = datadir
        self.audio_device = audio_device
        logger.info("Selected sound device: {}.".format(self.audio_device))
        self.class_indexes = {}
        self.class_sample_filenames = {}
        ensure_dir(self.datadir)
        class_folders = [d for d in os.listdir(self.datadir) if d.find(" - ") >= 0]
        for d in class_folders:
            classname = d.split("- ")[1]
            class_idx = int(d.split(" -")[0])
            self.class_indexes[classname] = class_idx
            class_files = [
                f for f in os.listdir(self.datadir + "/" + d) if f.find(".wav") >= 0
            ]
            self.class_sample_filenames[classname] = class_files
        self.rec_to_stop = []
        self.rec_running = {}

    @threaded
    def start_recording(
        self, label, rec_id=0, fname="test.wav", dir_name="./data_shake/"
    ):
        """
        Start threaded recording

        :param label: name of the recording
        :type label: str
        :param rec_id: id of the recording
        :type rec_id: int
        :param fname: name of the file
        :type fname: str
        :param dir_name: name of the directory
        :type dir_name: str
        """
        # print("Starting to record {} as id {}.".format(label, rec_id))
        self.rec_running[rec_id] = label
        frames = self.rec_from_mic(n_sec=24 * 60 * 60, rec_id=rec_id)
        self.rec_to_stop.remove(rec_id)
        del self.rec_running[rec_id]

        f_path_name = dir_name + fname
        wfile = wave.open(f_path_name, "wb")
        wfile.setnchannels(self.audio_channels)
        wfile.setframerate(self.samplerate)
        sampwidth = pyaudio.PyAudio().get_sample_size(pyaudio.paInt16)
        wfile.setsampwidth(sampwidth)
        wfile.writeframes(b"".join(frames))
        wfile.close()

    def stop_recording(self, rec_id):
        """
        Stop recording with given id

        :param rec_id: id of the recording
        :type rec_id: int
        """
        self.rec_to_stop.append(rec_id)

    def rec_from_mic(self, n_sec=20, rec_id=0):
        """
        Record for n seconds and returns frames.
        :param n_sec: duration of the recording
        :type n_sec: int
        :param rec_id: id of the recording
        :type rec_id: int

        :return: recorded frames
        :rtype: list
        """
        p = pyaudio.PyAudio()
        chunksize = 1024
        # print("* recording rec_id = {}".format(rec_id))
        frames = []
        stream = p.open(
            format=pyaudio.paInt16,
            channels=self.audio_channels,
            rate=self.samplerate,
            input=True,
            frames_per_buffer=2 * chunksize,
            input_device_index=self.audio_device,
        )
        n_frames = int(self.samplerate * n_sec)
        while n_frames > 0:
            while stream.get_read_available() < chunksize:
                time.sleep(0.01)
            to_read = min(n_frames, chunksize)
            n_frames -= to_read
            data = stream.read(to_read)
            frames.append(data)
            if rec_id in self.rec_to_stop:
                break
        stream.stop_stream()
        stream.close()
        p.terminate()
        # print("* done recording")
        return frames


if __name__ == "__main__":

    audio_device = get_pulse_device()

    dev_idx = audio_device
    directory = "./."

    ar = AudioRecorder(
        audio_channels=2, samplerate=48000, datadir=directory, audio_device=dev_idx
    )

    while True:
        print('Enter label for recording or "0" to exit')
        label = raw_input()
        if label == "0":
            break
        rec_id = 0
        ar.start_recording(label, fname=label + ".wav", dir_name=directory)
        print("Press Enter to stop recording")
        raw_input()
        ar.stop_recording(0)

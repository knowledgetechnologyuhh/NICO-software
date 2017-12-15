#  Compatibility imports
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import alsaaudio, time, audioop, wave, numpy
import struct
import pyaudio
import os
import copy
from thread import start_new_thread
# from threading import Thread
import threading

def ensure_dir(directory):
    # directory = os.path.dirname(file_path)
    try:
        os.makedirs(directory)
    except:
        print("Warning!!! could not create directory {}".format(directory))
        pass

def threaded(f, daemon=False):
    import Queue

    def wrapped_f(q, *args, **kwargs):
        '''this function calls the decorated function and puts the
        result in a queue'''
        ret = f(*args, **kwargs)
        q.put(ret)

    def wrap(*args, **kwargs):
        '''this is the function returned from the decorator. It fires off
        wrapped_f in a new thread and returns the thread object with
        the result queue attached'''

        q = Queue.Queue()

        t = threading.Thread(target=wrapped_f, args=(q,)+args, kwargs=kwargs)
        t.daemon = daemon
        t.start()
        t.result_queue = q
        return t

    return wrap

class AudioRecorder:

    def __init__(self, audio_channels, samplerate, datadir, audio_device):
        self.audio_channels = audio_channels
        self.samplerate = samplerate
        self.datadir=datadir
        self.audio_device=audio_device
        print("Selected sound device: {}.".format(self.audio_device))
        self.class_indexes = {}
        self.class_sample_filenames = {}
        ensure_dir(self.datadir)
        class_folders = [d for d in os.listdir(self.datadir) if d.find(" - ") >= 0]
        for d in class_folders:
            classname = d.split("- ")[1]
            class_idx = int(d.split(" -")[0])
            self.class_indexes[classname] = class_idx
            class_files = [f for f in os.listdir(self.datadir+"/"+d) if f.find(".wav") >= 0]
            self.class_sample_filenames[classname] = class_files
        self.rec_to_stop = []
        self.rec_running = {}

    def generate_data(self):
        continue_generation = True
        while continue_generation:
            label_valid = False
            while not label_valid:
                print("Existing labels are: {}".format(self.class_indexes.keys()))
                print("What is the label of the data that you want to record? (Enter 0 to stop generating data)")
                label = raw_input()
                if label == "0":
                    continue_generation = False
                    break

                if label not in self.class_indexes.keys():
                    print(
                        "{} is a new label that is not yet in the dataset. Are you sure that you want to use this label (y/n)?".format(
                            label))
                    yn = raw_input()
                    if yn == "y":
                        label_valid = True
                        if len(self.class_indexes) > 0:
                            self.class_indexes[label] = max(self.class_indexes.values()) + 1
                        else:
                            self.class_indexes[label] = 0
                        self.class_sample_filenames[label] = []
                else:
                    label_valid = True
            if not continue_generation:
                break
            print("How many seconds of data do you want to record?")
            secs = int(raw_input())
            assert(secs > 0)

            print("Starting top record in...")
            cdown = 3
            while cdown > 0:
                print(cdown)
                time.sleep(1)
                cdown -= 1
            print("go!")
            print("Recording mic data for {} seconds.".format(secs))

            frames = self.rec_from_mic(n_sec=secs)

            print("Do you want to add this recording (y/n)?")
            yn = raw_input()
            if yn != "y":
                continue

            fname = ""
            label_idx = len(self.class_sample_filenames[label])
            # print(self.class_sample_filenames[label])
            while fname == "" or fname in self.class_sample_filenames[label]:
                label_idx += 1
                fname = "{}-{}.wav".format(label_idx, label)

            self.class_sample_filenames[label].append(fname)
            subdirname = "{} - {}".format(self.class_indexes[label], label)
            ensure_dir(self.datadir + "/" + subdirname)
            f_path_name = "{}/{}/{}".format(self.datadir, subdirname, fname)
            wfile = wave.open(f_path_name, 'wb')
            wfile.setnchannels(self.audio_channels)
            wfile.setframerate(self.samplerate)
            sampwidth = pyaudio.PyAudio().get_sample_size(pyaudio.paInt16)
            wfile.setsampwidth(sampwidth)
            wfile.writeframes(b''.join(frames))
            wfile.close()

    @threaded
    def start_recording(self, label, rec_id=0):
        print("Starting to record {} as id {}.".format(label, rec_id))
        self.rec_running[rec_id] = label
        frames = self.rec_from_mic(n_sec=24 * 60 * 60, rec_id=rec_id)
        self.rec_to_stop.remove(rec_id)
        del self.rec_running[rec_id]
        if label not in self.class_indexes.keys():
            label_idx = 0
            if len(self.class_indexes.values()) > 0:
                label_idx = max(self.class_indexes.values()) + 1
            self.class_indexes[label] = label_idx
            self.class_sample_filenames[label] = []
        subdirname = "{} - {}".format(self.class_indexes[label], label)
        ensure_dir(self.datadir + "/" + subdirname)
        fname = ""
        label_idx = len(self.class_sample_filenames[label])
        while fname == "" or fname in self.class_sample_filenames[label]:
            label_idx += 1
            fname = "{}-{}.wav".format(label_idx, label)
        f_path_name = "{}/{}/{}".format(self.datadir, subdirname, fname)
        wfile = wave.open(f_path_name, 'wb')
        wfile.setnchannels(self.audio_channels)
        wfile.setframerate(self.samplerate)
        sampwidth = pyaudio.PyAudio().get_sample_size(pyaudio.paInt16)
        wfile.setsampwidth(sampwidth)
        wfile.writeframes(b''.join(frames))
        wfile.close()


    def stop_recording(self, rec_id):
        self.rec_to_stop.append(rec_id)

    def auto_record_data(self, delay, label, duration_secs):
        print("Starting to record {} for {} sec. in {} secs.".format(label, duration_secs, delay))
        time.sleep(duration_secs)
        print("Recording now!")
        frames = self.rec_from_mic(n_sec=duration_secs)
        # self.class_sample_filenames[label].append(fname)
        subdirname = "{} - {}".format(self.class_indexes[label], label)
        ensure_dir(self.datadir + "/" + subdirname)
        fname = ""
        label_idx = len(self.class_sample_filenames[label])
        # print(self.class_sample_filenames[label])
        while fname == "" or fname in self.class_sample_filenames[label]:
            label_idx += 1
            fname = "{}-{}.wav".format(label_idx, label)
        f_path_name = "{}/{}/{}".format(self.datadir, subdirname, fname)
        wfile = wave.open(f_path_name, 'wb')
        wfile.setnchannels(self.audio_channels)
        wfile.setframerate(self.samplerate)
        sampwidth = pyaudio.PyAudio().get_sample_size(pyaudio.paInt16)
        wfile.setsampwidth(sampwidth)
        wfile.writeframes(b''.join(frames))
        wfile.close()

    def rec_from_mic(self, n_sec=20, rec_id=0):
        p = pyaudio.PyAudio()
        chunksize = 1024
        print("* recording rec_id = {}".format(rec_id))
        frames = []
        stream = p.open(format=pyaudio.paInt16,
                        channels=self.audio_channels,
                        rate=self.samplerate,
                        input=True,
                        frames_per_buffer=2 * chunksize,
                        input_device_index=self.audio_device)
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
        print("* done recording")
        return frames


if __name__ == '__main__':
    print("The following sound devices are available.")
    p = pyaudio.PyAudio()
    info = p.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')
    audio_device = 0
    for i in range(0, numdevices):
        if (p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
            device_name = p.get_device_info_by_host_api_device_index(0, i).get('name')
            print("Input Device id ", i, " - ", device_name)
            if device_name.find("USB") != -1:
                audio_device = i
    dev_idx = audio_device
    ar = AudioRecorder(audio_channels=1, samplerate=48000, datadir="datasets/nico_shaking_0", audio_device=dev_idx)

    # For recoring several streams at once (does not yet work with alsaaudio)
    # while True:
    #     print("\nEnter \"r\" to start a new recording or \"s\" to stop a recording or \"0\" to stop")
    #     rs = raw_input()
    #     if rs == "0":
    #         break
    #     if rs == "r":
    #         print("Enter label for recording")
    #         label = raw_input()
    #         if len(ar.rec_running.keys()) == 0:
    #             rec_id = 0
    #         else:
    #             rec_id = max(ar.rec_running.keys()) + 1
    #         ar.start_recording(label, rec_id)
    #     else:
    #         print("The following recordings are running:")
    #         for rec_id in ar.rec_running.keys():
    #             label = ar.rec_running[rec_id]
    #             print("{} -- index: {}".format(label, rec_id))
    #         print("Enter index of recording that you want to stop")
    #         rec_idx = int(raw_input())
    #         ar.stop_recording(rec_idx)

    while True:
        print("Enter label for recording or \"0\" to exit")
        label = raw_input()
        if label == "0":
            break
        rec_id = 0
        ar.start_recording(label)
        print("Press Enter to stop recording")
        raw_input()
        ar.stop_recording(0)









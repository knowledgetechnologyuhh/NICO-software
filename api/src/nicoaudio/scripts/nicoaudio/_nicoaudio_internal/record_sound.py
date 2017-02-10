#!/usr/bin/env python

"""Threaded audio recording module, developed to be used with the NICO robot

The record_sound module contains the RecordSound class for concurrent sound
recording with the pyAudio library and is both Python 2 and Python 3
compatible. The record_sound module was developed for and tested with PyAudio 
v0.2.9. PyAudio is freely available under the MIT License at 
'https://people.csail.mit.edu/hubert/pyaudio' (Copyright (c) 2006 Hubert Pham)

It was developed with the intend to be used as a recording module for the 
NICO robot with the 'Focusrite' USB recording device. Other possible recording
devices that are connected can be viewed by calling the static get_devices() 
method. Pass another recording device's name to the RecordSound's device_name 
parameter in order to change the used recording device. 

RecordSound is derived from Thread and can thus be used concurrently, although
since a thread can only be started once a new RecordSound instance has to be
initialized in case of multiple recordings. 

A simple threshold may be used to filter noise and crop silent audio samples by
setting the has_threshold parameter to True. 

Be aware that the standard parameter for chunk_size (512), rate (44100) and
channels (2) are possibly dependent on the device and / or computer and may
throw an IOError if modified or used in a different environment.
"""

__author__    = 'Julius Mayer, Connor Gaede'
__copyright__ = 'Copyright (C) 2016 University Hamburg'
__version__   = '1.1'


################################################################################
##########################          Imports          ###########################
################################################################################

import wave
import math
import threading
from warnings import warn
from time import sleep

import pyaudio
import audioop


################################################################################
########################          RecordSound          #########################
################################################################################

class RecordSound(threading.Thread):
    """Threaded audio recording class"""

    def __init__(self, 
                 device_name   = 'default', #TODO real device name
                 has_threshold = False,
                 chunk_size    = 256,
                 audio_format  = pyaudio.paInt16,
                 rate          = 44100,
                 channels      = 2):
        """Initialize a RecordSound instance.
        
        Args:
            device_name:    Optional string specifying the name of the recording 
                            device.
            has_threshold:  Optional integer for a threshold to filter 
                            background noise.
            chunk_size:     Optional integer for the size of chunks of bytes to 
                            be read at once.
            audio_format:   Optional pyaudio format for the audio format.
            rate:           Optional integer for the recording rate.
            channels:       Optional integer for the number of recording 
                            channels.
        """

        # Initialize threading
        super(RecordSound, self).__init__()
        self._stopper       = threading.Event()   

        # Initialize PyAudio
        self._chunk_size    = chunk_size
        self._format        = audio_format
        self._rate          = rate
        self._channels      = channels
        self._pyAudio       = pyaudio.PyAudio()
        self._device_name   = device_name
        self._frames        = []
        self._has_threshold = has_threshold

        try:
            self._stream    = self._pyAudio.open(
                format             = self._format, 
                input_device_index = self._device_index, 
                channels           = self._channels,
                rate               = self._rate,
                input              = True,
                frames_per_buffer  = self._chunk_size)

        # dirty solution for IOError: [Errno Device unavailable] -9985
        except IOError: 
            warn("IOError: Re-initializing RecordSound.")
            self.__init__()


    @property
    def _device_index(self):
        """Return the device index for this instance's given device name.

        Returns:
            Tries to return the index of the device name, specified by this 
            instance's _device_name variable, if found in the device list or 
            else the last device in the list, which is usually the USB device.
        """
        device_count = self._pyAudio.get_device_count()

        if self._device_name in self.get_devices():
            return list(filter(lambda device: device['name'] == 
                self._device_name, map(self._pyAudio.get_device_info_by_index, 
                    range(device_count))))[0]['index']

        else:
            warn("%s not connected. Trying to record with %s" 
                %(self._device_name, self.get_devices()[device_count-1]))
            return device_count-1

    def run(self):
        """Record sound with the PyAudio stream until the stop flag is set."""
        threshold = 3000

        while not self._stopper.is_set():
            data = self._stream.read(self._chunk_size)

            if not self._has_threshold \
            or math.sqrt(abs(audioop.avg(data, 4))) > threshold:
                self._frames.append(data)

        self._stream.stop_stream()
        self._stream.close()
        self._pyAudio.terminate() # responsable for IOError (e.g. with save)?


    def stop(self):
        """Stop the sound recording."""
        self._stopper.set()


    def get_chunk(self,index=-1):
        """Return a specific chunk of streamed data. (default: current chunk)"""
        return self._frames[index]

    def get_number_of_samples(self):
        """Return number of samples taken"""
        return len(self._frames)

    def get_sample_width(self):
        """Return sample width"""
        return self._pyAudio.get_sample_size(self._format)


    def save(self, filename='sound.wav', activeChannels=(True,True)):
        """Save the frames to a WAV file.
        
        Args:
            filename:       An optional string for the name of the sound file.
            activeChannels: An optional boolean tuple to save only specific channels of the signal (left,right)
        """ 
        channels = activeChannels.count(True)
        
        if not channels:
            warn("All channels are disabled: Output file will be empty")
        
        wf = wave.open(filename, 'wb')
        wf.setnchannels(channels)
        wf.setsampwidth(self.get_sample_width())
        wf.setframerate(self._rate)
        if channels == 2:
          wf.writeframes(b''.join(self._frames))
        elif channels:
          wf.writeframes(b''.join(map(
                                  lambda sample: audioop.tomono(sample, self.get_sample_width(), activeChannels[0], activeChannels[1]), 
                                  self._frames)))
        wf.close()
    

    @staticmethod
    def get_devices():
        """Return a list of all connected sound device names."""
        return [device['name'] for device in map(
            pyaudio.PyAudio().get_device_info_by_index,
            range(pyaudio.PyAudio().get_device_count()))]

import logging
from _nicoaudio_internal.record_sound import RecordSound

class AudioRecorder:
    def __init__(self):
        self._record = None
        self._running = False

    def startMicrophonesRecording(self, filename="sound.wav", type="wav(not implemented yet)", samplerate=44100, channels=(True,True)):
        """
        Starts the microphone recording to a given file

        :param filename: Name of target file
        :type filename: str
        :param type: Type of recording (not implemented yet)
        :type type: str
        :param samplerate: Target samplerate
        :type samplerate: int
        :param channels: requested channels (left,right)
        :type channels: tuple(bool, bool)
        """
        if not self._running:
            self._filename = filename
            self._channels = channels
            self._record = RecordSound(rate = samplerate)
            self._record.start()
            self._running = True
        else:
            logging.warning("Start recording failed: Record is already running")

    def stopMicrophonesRecording(self):
        """
        Stops the current microphone recording
        """
        if self._record and self._running:
            self._record.stop()
            self._record.save(self._filename, self._channels)
            self._running = False
        else:
            logging.warning("Stop recording failed: Record is not running")          

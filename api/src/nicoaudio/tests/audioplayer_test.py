import math
import struct
import time
import unittest
import wave

from nicoaudio import AudioPlayer


def generate_audio_file(
    filename="/tmp/NICO_audioplayer_test.wav",
    volume=0.5,
    sampleRate=44100,
    duration=5.0,
    frequency=440.0,
):
    # taken from http://blog.acipo.com/wave-generation-in-python/
    wav = wave.open(filename, "wb")
    wav.setnchannels(1)
    wav.setsampwidth(2)
    wav.setframerate(sampleRate)

    for i in range(int(duration * sampleRate)):
        value = int(
            volume
            * 32767.0
            * math.cos(frequency * math.pi * float(i) / float(sampleRate))
        )
        data = struct.pack("<h", value)
        wav.writeframesraw(data)
    wav.close()


class AudioPlayerTest(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_duration(self):
        generate_audio_file("/tmp/NICO_test_duration_5_secs.wav")
        player = AudioPlayer.AudioPlayer("/tmp/NICO_test_duration_5_secs.wav")
        self.assertEqual(player.duration, 5.0)
        generate_audio_file("/tmp/NICO_test_duration_10_secs.wav", duration=10.0)
        player = AudioPlayer.AudioPlayer("/tmp/NICO_test_duration_10_secs.wav")
        self.assertEqual(player.duration, 10.0)

    def test_duration_5_secs(self):
        generate_audio_file("/tmp/NICO_test_duration_5_secs.wav")
        player = AudioPlayer.AudioPlayer("/tmp/NICO_test_duration_5_secs.wav")
        self.assertEqual(player.duration, 5.0)

    def test_duration_10_secs(self):
        generate_audio_file("/tmp/NICO_test_duration_10_secs.wav", duration=10.0)
        player = AudioPlayer.AudioPlayer("/tmp/NICO_test_duration_10_secs.wav")
        self.assertEqual(player.duration, 10.0)

    def test_duration_start(self):
        generate_audio_file("/tmp/NICO_test_duration_start.wav")
        player = AudioPlayer.AudioPlayer("/tmp/NICO_test_duration_start.wav", start=2.0)
        self.assertEqual(player.duration, 3.0)

    def test_duration_end(self):
        generate_audio_file("/tmp/NICO_test_duration_end.wav")
        player = AudioPlayer.AudioPlayer(
            "/tmp/NICO_test_duration_end.wav", duration=2.0
        )
        self.assertEqual(player.duration, 2.0)

    def test_playback_position(self):
        generate_audio_file("/tmp/NICO_test_postion.wav")
        player = AudioPlayer.AudioPlayer("/tmp/NICO_test_postion.wav")
        self.assertEqual(player.position, 0.0)
        player.play()
        time.sleep(2.0)
        player.pause()
        self.assertAlmostEqual(player.position, 2.0, delta=0.2)
        player.resume()
        time.sleep(3.0)
        player.pause()
        self.assertAlmostEqual(player.position, 5.0, delta=0.2)

    def test_pitch(self):
        generate_audio_file("/tmp/NICO_test_pitch.wav")
        player = AudioPlayer.AudioPlayer("/tmp/NICO_test_pitch.wav")
        original_framerate = player._segment.frame_rate
        original_duration = player.duration
        player.pitch(1.0)
        self.assertEqual(player.duration, original_duration / 2.0)
        self.assertEqual(player._segment.frame_rate, original_framerate * 2)
        player.pitch(-2.0)
        self.assertEqual(player.duration, original_duration * 2.0)
        self.assertEqual(player._segment.frame_rate, original_framerate / 2)

    def test_speed(self):
        generate_audio_file("/tmp/NICO_test_speed.wav")
        player = AudioPlayer.AudioPlayer("/tmp/NICO_test_speed.wav")
        original_framerate = player._segment.frame_rate
        original_duration = player.duration
        player.speed(2.0)
        self.assertAlmostEqual(player.duration, original_duration / 2.0, delta=0.2)
        self.assertEqual(player._segment.frame_rate, original_framerate)
        player.speed(0.5)
        self.assertAlmostEqual(player.duration, original_duration, delta=0.2)
        self.assertEqual(player._segment.frame_rate, original_framerate)

    def test_volume(self):
        generate_audio_file("/tmp/NICO_test_volume.wav")
        player = AudioPlayer.AudioPlayer("/tmp/NICO_test_volume.wav")
        player.volume = 0.5
        self.assertAlmostEqual(player.volume, 0.5, delta=0.1)


if __name__ == "__main__":
    unittest.main()

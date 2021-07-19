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
        """tests if duration matches generated file length"""
        # 5 seconds
        generate_audio_file("/tmp/NICO_test_duration_5_secs.wav")
        player = AudioPlayer.AudioPlayer("/tmp/NICO_test_duration_5_secs.wav")
        self.assertEqual(player.duration, 5.0)
        # 10 seconds to verify its not hardcoded
        generate_audio_file("/tmp/NICO_test_duration_10_secs.wav", duration=10.0)
        player = AudioPlayer.AudioPlayer("/tmp/NICO_test_duration_10_secs.wav")
        self.assertEqual(player.duration, 10.0)

    def test_duration_start(self):
        """tests if file is shortened correctly when defining a starting point"""
        generate_audio_file("/tmp/NICO_test_duration_start.wav")
        player = AudioPlayer.AudioPlayer("/tmp/NICO_test_duration_start.wav", start=2.0)
        self.assertEqual(player.duration, 3.0)

    def test_duration_end(self):
        """tests if file is shortened correctly when defining an end point"""
        generate_audio_file("/tmp/NICO_test_duration_end.wav")
        player = AudioPlayer.AudioPlayer(
            "/tmp/NICO_test_duration_end.wav", duration=2.0
        )
        self.assertEqual(player.duration, 2.0)

    def test_playback_position(self):
        """tests if playback position is correct"""
        generate_audio_file("/tmp/NICO_test_postion.wav")
        player = AudioPlayer.AudioPlayer("/tmp/NICO_test_postion.wav")
        # test start is 0.0
        self.assertEqual(player.position, 0.0)
        # play for 2 seconds
        player.play()
        time.sleep(2.0)
        player.pause()
        self.assertAlmostEqual(player.position, 2.0, delta=0.2)
        # resume to play for another 3 seconds
        player.resume()
        time.sleep(3.0)
        player.pause()
        self.assertAlmostEqual(player.position, 5.0, delta=0.2)

    def test_pitch(self):
        """Tests pitch shifting"""
        generate_audio_file("/tmp/NICO_test_pitch.wav")
        player = AudioPlayer.AudioPlayer("/tmp/NICO_test_pitch.wav")
        # save original framerate and duration
        original_framerate = player._segment.frame_rate
        original_duration = player.duration
        # pitch up by one octave and check if framerate doubled and duration halfed
        player.pitch(1.0)
        self.assertEqual(player.duration, original_duration / 2.0)
        self.assertEqual(player._segment.frame_rate, original_framerate * 2)
        # pitch down by two octaves and check if framerate is half the original and duration doubles
        player.pitch(-2.0)
        self.assertEqual(player.duration, original_duration * 2.0)
        self.assertEqual(player._segment.frame_rate, original_framerate / 2)

    def test_speed(self):
        """tests speed adjustment"""
        generate_audio_file("/tmp/NICO_test_speed.wav")
        player = AudioPlayer.AudioPlayer("/tmp/NICO_test_speed.wav")
        # save original framerate and duration
        original_framerate = player._segment.frame_rate
        original_duration = player.duration
        # double speed, check if duration doubled and framerate is still the same
        player.speed(2.0)
        self.assertAlmostEqual(player.duration, original_duration / 2.0, delta=0.2)
        self.assertEqual(player._segment.frame_rate, original_framerate)
        # half the speed, check if duration is back to the original length and framerate is still the same
        player.speed(0.5)
        self.assertAlmostEqual(player.duration, original_duration, delta=0.2)
        self.assertEqual(player._segment.frame_rate, original_framerate)

    def test_volume(self):
        """tests volume adjustment"""
        generate_audio_file("/tmp/NICO_test_volume.wav")
        player = AudioPlayer.AudioPlayer("/tmp/NICO_test_volume.wav")
        player.volume = 0.5
        self.assertAlmostEqual(player.volume, 0.5, delta=0.1)


if __name__ == "__main__":
    unittest.main()

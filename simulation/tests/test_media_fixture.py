import sys
from pathlib import Path
import tempfile
import unittest
import wave
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'sim-driver'))
from media_fixture import MediaFixture


class MediaFixtureTest(unittest.TestCase):
    def test_replay_validation_and_boundary(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            wav_path = root / 'speech.wav'
            with wave.open(str(wav_path), 'wb') as wav:
                wav.setparams((1, 2, 16000, 0, 'NONE', 'not compressed'))
                wav.writeframes(b'\x01\x00' * 600)
            fixture = MediaFixture(wav_path, 'audio', root)
            first = fixture.audio_chunk()
            self.assertEqual(first, b'\x01\x00' * 512)
            self.assertEqual(fixture.audio_chunk(), b'\x01\x00' * 88 + b'\0' * 848)
            self.assertEqual(fixture.audio_chunk(), first)
            spaced = MediaFixture(wav_path, 'audio', root, silence_ms=64)
            self.assertEqual(spaced.info['silence_ms'], 64)
            self.assertEqual(spaced.audio_chunk(), b'\0' * 1024)
            self.assertEqual(spaced.audio_chunk(), b'\0' * 1024)
            self.assertEqual(spaced.audio_chunk(), first)
            self.assertEqual(spaced.audio_chunk(), b'\x01\x00' * 88 + b'\0' * 848)
            self.assertEqual(spaced.audio_chunk(), b'\0' * 1024)
            spaced.offset = 0  # Sensor restart must begin at the quiet interval.
            self.assertEqual(spaced.audio_chunk(), b'\0' * 1024)
            for invalid in (-1, 60001, 1.5, True):
                with self.assertRaises(ValueError):
                    MediaFixture(wav_path, 'audio', root, silence_ms=invalid)
            image = root / 'camera.jpg'
            Image.new('RGB', (100, 80)).save(image)
            self.assertEqual(MediaFixture(image, 'image', root).data, image.read_bytes())
            with self.assertRaises(ValueError):
                MediaFixture(wav_path, 'audio', root / 'restricted')
            with wave.open(str(wav_path), 'wb') as wav:
                wav.setparams((2, 2, 48000, 0, 'NONE', 'not compressed'))
                wav.writeframes(b'\0' * 100)
            with self.assertRaises(ValueError):
                MediaFixture(wav_path, 'audio', root)


if __name__ == '__main__':
    unittest.main()

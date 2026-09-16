"""Bounded, explicitly synthetic media input; never generates model results."""
import hashlib
from io import BytesIO
from pathlib import Path
import wave

from PIL import Image


class MediaFixture:
    def __init__(self, path, kind, root='/fixtures', *, silence_ms=0):
        if type(silence_ms) is not int or not 0 <= silence_ms <= 60000:
            raise ValueError('fixture silence_ms must be an integer in 0–60000')
        if kind != 'audio' and silence_ms:
            raise ValueError('silence_ms is only supported for audio')
        path = Path(path).resolve()
        path.relative_to(Path(root).resolve())
        if path.stat().st_size > 32 * 1024 * 1024:
            raise ValueError('fixture exceeds 32 MiB')
        data = path.read_bytes()
        self.info = {'source': 'fixture_replay', 'fixture': path.name,
                     'sha256': hashlib.sha256(data).hexdigest()}
        self.offset = 0
        if kind == 'audio':
            with wave.open(BytesIO(data), 'rb') as wav:
                if (wav.getnchannels(), wav.getsampwidth(), wav.getframerate(), wav.getcomptype()) != (1, 2, 16000, 'NONE'):
                    raise ValueError('audio fixture must be PCM_S16_LE 16000 Hz mono WAV')
                if not 0 < wav.getnframes() <= 16000 * 300:
                    raise ValueError('audio fixture must contain 0–300 seconds of audio')
                self.data = wav.readframes(wav.getnframes())
                if len(self.data) != wav.getnframes() * 2:
                    raise ValueError('truncated WAV fixture')
            self.info['silence_ms'] = silence_ms
            # One bounded cycle: quiet startup / inter-utterance gap, then speech.
            self.data = b'\0' * (silence_ms * 32) + self.data
        elif kind == 'image':
            with Image.open(BytesIO(data)) as image:
                if image.format != 'JPEG' or image.width * image.height > 4096 * 4096:
                    raise ValueError('image fixture must be JPEG with at most 16 megapixels')
                image.verify()
            self.data = data
        else:
            raise ValueError('unsupported fixture kind')

    def audio_chunk(self, size=1024):
        # Pad the last chunk; the next call begins a new replay with no stale ROS stamp.
        chunk = self.data[self.offset:self.offset + size]
        self.offset += len(chunk)
        if self.offset == len(self.data):
            self.offset = 0
        return chunk.ljust(size, b'\0')

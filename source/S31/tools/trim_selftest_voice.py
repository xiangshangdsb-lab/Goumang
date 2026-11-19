"""Trim TTS boundary silence from the self-test phrase bank, retaining margins."""
from pathlib import Path
import array
import sys
import wave

directory = Path(sys.argv[1])
pattern = sys.argv[2] if len(sys.argv) > 2 else ""
paths = directory.glob(pattern) if pattern else [
    *directory.glob('st_*.wav'), *directory.glob('greet_*.wav')]
for path in paths:
    if not path.name.startswith(('st_', 'greet_')):
        continue
    with wave.open(str(path), 'rb') as source:
        params = source.getparams()
        assert params.nchannels == 1 and params.sampwidth == 2
        data = array.array('h', source.readframes(source.getnframes()))
    indices = [i for i, sample in enumerate(data) if abs(sample) > 64]
    if not indices:
        raise RuntimeError(f'Silent prompt: {path}')
    before_ms, after_ms = (20, 50) if path.name.startswith('greet_') else (60, 80)
    start = max(0, indices[0] - params.framerate * before_ms // 1000)
    end = min(len(data), indices[-1] + params.framerate * after_ms // 1000 + 1)
    with wave.open(str(path), 'wb') as target:
        target.setparams(params)
        target.writeframes(data[start:end].tobytes())

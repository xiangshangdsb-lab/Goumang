"""Compile the production WAV parser and check actual clips and malformed files."""
import array
import math
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import wave

root = Path(__file__).resolve().parents[1]
checks = 0
with tempfile.TemporaryDirectory(prefix="goumang-wav-") as temp:
    temp = Path(temp)
    runner = temp / "parse.c"
    runner.write_text('''#include "audio/voice_wav.h"
int main(int argc, char **argv) {
    if (argc != 2) return 2;
    FILE *f = fopen(argv[1], "rb");
    if (!f) return 2;
    voice_wav_info_t info;
    int result = voice_wav_parse(f, &info);
    fclose(f);
    return result == 0 ? 0 : 1;
}
''')
    executable = temp / "parse.exe"
    subprocess.run([shutil.which("gcc"), "-std=c99", "-Wall", "-Wextra", "-Werror",
                    "-I", str(root / "main"), str(runner),
                    str(root / "main/audio/voice_wav.c"), "-o", str(executable)], check=True)

    def check(data, valid):
        global checks
        file = temp / "sample.wav"
        file.write_bytes(data)
        result = subprocess.run([str(executable), str(file)]).returncode
        assert result == (0 if valid else 1), (checks, valid, result)
        checks += 1

    for clip in sorted((root / "voice").glob("*.wav")):
        check(clip.read_bytes(), True)
        with wave.open(str(clip)) as audio:
            assert audio.getsampwidth() == 2 and audio.getnframes() > 0
            samples = array.array("h", audio.readframes(audio.getnframes()))
            rms = math.sqrt(sum(s*s for s in samples) / len(samples))
            assert rms > 20, (clip.name, "silent clip")
            print(f"{clip.name}: {audio.getframerate()}Hz {audio.getnchannels()}ch "
                  f"{audio.getnframes()/audio.getframerate():.2f}s rms={rms:.0f}")

    def chunk(tag, data):
        return tag + struct.pack("<I", len(data)) + data + (b"\0" if len(data) % 2 else b"")

    def riff(body):
        return b"RIFF" + struct.pack("<I", len(body) + 4) + b"WAVE" + body

    fmt = struct.pack("<HHIIHH", 1, 1, 16000, 32000, 2, 16)
    pcm = chunk(b"data", b"\1\0" * 16)
    valid = riff(chunk(b"fmt ", fmt) + pcm)
    check(valid, True)
    check(riff(chunk(b"JUNK", b"x") + chunk(b"fmt ", fmt + b"x") + pcm), True)
    for n in range(len(valid)):
        check(valid[:n], False)
    for offset, value in [(0, 3), (2, 0), (2, 3), (12, 4), (14, 8)]:
        bad = bytearray(fmt)
        struct.pack_into("<H", bad, offset, value)
        check(riff(chunk(b"fmt ", bad) + pcm), False)
    check(riff(chunk(b"fmt ", fmt[:6]) + pcm), False)
    check(riff(pcm + chunk(b"fmt ", fmt)), False)
    check(riff(chunk(b"fmt ", fmt) + chunk(b"data", b"")), False)
    check(riff(chunk(b"fmt ", fmt) + chunk(b"data", b"x")), False)
    check(riff(chunk(b"fmt ", fmt) + b"data\xff\xff\xff\xff"), False)
print(f"WAV checks={checks} failures=0")

"""Build and run the S31/C30D software simulation using a local C compiler."""
from pathlib import Path
import argparse
import os
import subprocess
import tempfile

here = Path(__file__).resolve().parent
main = here.parent.parent / "main"
c30d = here.parent.parent.parent / "C30D" / "APP"
sources = [main / name for name in (
    "control/control.c", "protocol/robot_protocol.c", "bus/robot_bus.c", "agent/agent_runtime.c", "agent/agent_runtime_hw.c",
)]
sources.extend((c30d / "line/line_fusion.c", c30d / "protocol/s31_protocol.c",
                here / "sim_control.c"))

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--sanitize", action="store_true", help="use AddressSanitizer and UBSan")
options = parser.parse_args()
sanitizers = ["-fsanitize=address,undefined", "-fno-sanitize-recover=all"] if options.sanitize else []

with tempfile.TemporaryDirectory(prefix="goumang_sim_") as build:
    exe = Path(build) / ("goumang_sim.exe" if os.name == "nt" else "goumang_sim")
    subprocess.run([
        os.environ.get("CC", "gcc"), "-std=c11", "-O1", "-g",
        "-Wall", "-Wextra", "-Werror", *sanitizers,
        "-DCONTROL_HOST_TEST", "-DAGENT_RUNTIME_HOST_TEST",
        "-I", str(here), "-I", str(main), "-I", str(c30d),
        *map(str, sources), "-o", str(exe),
    ], check=True)
    subprocess.run([str(exe)], check=True)

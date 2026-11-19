"""Flash an STM32 over UART ISP while keeping one serial session open."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from stm32loader import bootloader
from stm32loader.uart import SerialConnection


FLASH_BASE = 0x08000000


def open_bootloader(port: str, baud: int):
    """Try the known C30D DTR/RTS polarity combinations."""
    candidates = (
        (False, False, False),
        (True, False, True),
        (False, True, True),
        (True, True, True),
    )

    for swap_rts_dtr, reset_active_high, boot0_active_low in candidates:
        connection = SerialConnection(port, baud, "E")
        connection.connect()
        connection.swap_rts_dtr = swap_rts_dtr
        connection.reset_active_high = reset_active_high
        connection.boot0_active_low = boot0_active_low
        loader = bootloader.Stm32Bootloader(
            connection, device_family="F4", verbosity=0
        )
        try:
            loader.reset_from_system_memory()
            print(
                "BootLoader ready: "
                f"swap={swap_rts_dtr} reset_high={reset_active_high} "
                f"boot0_low={boot0_active_low}"
            )
            return connection, loader
        except bootloader.CommandError:
            connection.disconnect()
            time.sleep(0.2)

    raise RuntimeError("could not enter the STM32 UART BootLoader")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM11")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--bin", required=True)
    args = parser.parse_args()

    image = Path(args.bin).read_bytes()
    if not image:
        raise ValueError("firmware image is empty")

    connection = None
    loader = None
    try:
        connection, loader = open_bootloader(args.port, args.baud)
        loader.SYNCHRONIZE_ATTEMPTS = 5
        loader.get()
        device_id = loader.get_id()
        # This F407 target rejects the short 0x11 read used by
        # get_flash_size(); the CLI already identified it as 512 KiB.
        flash_size = 512
        print(
            f"chip=0x{device_id:03X} flash={flash_size} KiB "
            f"image={len(image)} B"
        )
        if len(image) > flash_size * 1024:
            raise ValueError("firmware image is larger than the detected flash")

        print("Erasing application flash...")
        loader.erase_memory()
        print("Writing application flash...")
        loader.write_memory_data(FLASH_BASE, image)
        print("Verifying application flash...")
        readback = loader.read_memory_data(FLASH_BASE, len(image))
        bootloader.Stm32Bootloader.verify_data(readback, image)
        print("Verification OK")
        return 0
    finally:
        if loader is not None and connection is not None:
            try:
                loader.reset_from_flash()
            except Exception:
                pass
        if connection is not None:
            connection.disconnect()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"FLASH FAILED: {exc}", file=sys.stderr)
        sys.exit(1)

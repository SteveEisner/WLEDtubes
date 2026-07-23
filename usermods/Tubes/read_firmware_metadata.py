#!/usr/bin/env python3

import argparse
import pathlib
import struct
import sys


WLED_METADATA_MAGIC = 0x57535453
WLED_METADATA = struct.Struct("<II48s48sI3B")


def djb2(value: bytes) -> int:
    result = 5381
    for byte in value:
        result = ((result << 5) + result + byte) & 0xFFFFFFFF
    return result


def decode_string(value: bytes, field: str) -> str:
    encoded = value.split(b"\0", 1)[0]
    try:
        return encoded.decode("ascii")
    except UnicodeDecodeError as error:
        raise ValueError(f"firmware {field} is not ASCII") from error


def read_firmware_metadata(path: pathlib.Path) -> tuple[str, str]:
    """Return the embedded WLED version and hardware-family release name.

    A valid record must carry WLED's magic, a supported description version,
    and the release-name hash used by the device-side OTA validator.
    """
    firmware = path.read_bytes()
    magic = struct.pack("<I", WLED_METADATA_MAGIC)
    offset = firmware.find(magic)
    while offset >= 0:
        end = offset + WLED_METADATA.size
        if end <= len(firmware):
            record = WLED_METADATA.unpack(firmware[offset:end])
            _, description_version, raw_version, raw_release, release_hash, *_ = record
            release_bytes = raw_release.split(b"\0", 1)[0]
            if description_version <= 2 and release_bytes and djb2(release_bytes) == release_hash:
                return (
                    decode_string(raw_version, "version"),
                    decode_string(raw_release, "release name"),
                )
        offset = firmware.find(magic, offset + 1)
    raise ValueError("firmware has no valid WLED compatibility metadata")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Print the WLED version and release name embedded in a firmware image."
    )
    parser.add_argument("firmware", type=pathlib.Path)
    args = parser.parse_args()

    try:
        version, release = read_firmware_metadata(args.firmware)
    except (OSError, ValueError) as error:
        print(error, file=sys.stderr)
        return 1

    print(f"{version}\t{release}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

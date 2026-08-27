#!/usr/bin/env python3
"""Fail closed unless two carrier images have exact release/profile identities."""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import pathlib
import struct

MAGIC = b"TUBEUP1\0"
IDENTITY = struct.Struct("<8sBBBH3s")
PROTOCOL = 1
STANDARD = 0
PROFILES = {"dig2go": 1, "athom-c3": 3}


@dataclasses.dataclass(frozen=True)
class Artifact:
    profile: str
    path: pathlib.Path
    family: int
    variant: int
    release: int
    size: int
    md5: str
    sha256: str


def inspect(profile: str, path: pathlib.Path, expected_release: int) -> Artifact:
    contents = path.read_bytes()
    offsets = []
    start = 0
    while (offset := contents.find(MAGIC, start)) >= 0:
        offsets.append(offset)
        start = offset + 1
    if len(offsets) != 1:
        raise ValueError(f"{path}: expected one TUBEUP1 identity, found {len(offsets)}")
    offset = offsets[0]
    if offset + IDENTITY.size > len(contents):
        raise ValueError(f"{path}: truncated TUBEUP1 identity")
    magic, protocol, family, variant, release, reserved = IDENTITY.unpack_from(contents, offset)
    expected_family = PROFILES[profile]
    if (magic != MAGIC or protocol != PROTOCOL or family != expected_family
            or variant != STANDARD or release != expected_release or reserved != b"\0\0\0"):
        raise ValueError(
            f"{path}: identity protocol={protocol} family={family} variant={variant} "
            f"release={release} reserved={reserved.hex()} does not match "
            f"protocol=1 family={expected_family} variant=0 release={expected_release}"
        )
    return Artifact(profile, path, family, variant, release, len(contents),
                    hashlib.md5(contents).hexdigest(),  # noqa: S324, ESP32 HTTPUpdate contract
                    hashlib.sha256(contents).hexdigest())


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--release", required=True, type=int)
    parser.add_argument("--dig2go", required=True, type=pathlib.Path)
    parser.add_argument("--athom-c3", required=True, type=pathlib.Path)
    args = parser.parse_args()
    if not 0 < args.release <= 0xFFFF:
        parser.error("--release must fit a nonzero uint16")
    artifacts = [inspect("dig2go", args.dig2go, args.release),
                 inspect("athom-c3", args.athom_c3, args.release)]
    print(json.dumps({"release": args.release,
                      "artifacts": [dataclasses.asdict(a) | {"path": str(a.path)} for a in artifacts]},
                     indent=2, sort_keys=True))


if __name__ == "__main__":
    main()

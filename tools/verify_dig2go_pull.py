#!/usr/bin/env python3
"""Read-only verifier for the first Dig2Go carrier-to-legacy OTA proof."""

from __future__ import annotations

import argparse
import atexit
import binascii
import hashlib
import json
import shutil
import struct
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

EXPECTED_B_MAC = "5443b2b54c38"
PARTITION_MAGIC = 0x50AA
OTA_STATES = {
    0x0: "new",
    0x1: "pending_verify",
    0x2: "valid",
    0x3: "invalid",
    0x4: "aborted",
    0xFFFFFFFF: "undefined",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def run(command: list[str]) -> str:
    completed = subprocess.run(command, text=True, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, check=False)
    print(completed.stdout, end="")
    if completed.returncode:
        raise RuntimeError(f"command failed ({completed.returncode}): {' '.join(command)}")
    return completed.stdout


def parse_mac(output: str) -> str:
    for line in output.splitlines():
        if line.startswith("MAC:"):
            return "".join(character for character in line[4:].lower() if character in "0123456789abcdef")
    raise ValueError("ROM MAC missing from esptool output")


def parse_partitions(data: bytes) -> list[dict[str, int | str]]:
    entries: list[dict[str, int | str]] = []
    for offset in range(0, min(len(data), 0xC00), 32):
        magic = struct.unpack_from("<H", data, offset)[0]
        if magic in (0xFFFF, 0xEBEB):
            break
        if magic != PARTITION_MAGIC:
            raise ValueError(f"invalid partition magic 0x{magic:04x} at 0x{offset:x}")
        _, part_type, subtype, address, size, raw_label, flags = struct.unpack_from("<HBBII16sI", data, offset)
        label = raw_label.split(b"\0", 1)[0].decode("ascii", errors="strict")
        entries.append({"label": label, "type": part_type, "subtype": subtype,
                        "offset": address, "size": size, "flags": flags})
    if not entries:
        raise ValueError("partition table contains no entries")
    return entries


def require_partition(entries: list[dict[str, int | str]], label: str) -> dict[str, int | str]:
    matches = [entry for entry in entries if entry["label"] == label]
    if len(matches) != 1:
        raise ValueError(f"expected exactly one {label} partition, found {len(matches)}")
    return matches[0]


def parse_otadata(data: bytes, ota_slot_count: int) -> tuple[list[dict[str, int | str | bool]], dict[str, int | str | bool]]:
    entries: list[dict[str, int | str | bool]] = []
    for copy, offset in enumerate((0, 0x1000)):
        sequence, state, stored_crc = struct.unpack_from("<I20xII", data, offset)
        calculated_crc = binascii.crc32(struct.pack("<I", sequence), 0xFFFFFFFF) & 0xFFFFFFFF
        valid_crc = sequence != 0xFFFFFFFF and stored_crc == calculated_crc
        bootable = valid_crc and state not in (0x3, 0x4)
        slot = ((sequence - 1) % ota_slot_count) if bootable else -1
        entries.append({"copy": copy, "sequence": sequence, "state": state,
                        "state_name": OTA_STATES.get(state, "unknown"),
                        "stored_crc": stored_crc, "calculated_crc": calculated_crc,
                        "valid_crc": valid_crc, "bootable": bootable, "slot": slot})
    bootable_entries = [entry for entry in entries if entry["bootable"]]
    if not bootable_entries:
        raise ValueError("otadata has no bootable CRC-valid entry")
    # This bounded two-slot migration cannot approach the uint32 sequence wrap.
    selected = max(bootable_entries, key=lambda entry: int(entry["sequence"]))
    return entries, selected


def read_flash(esptool: str, port: str, offset: int, size: int, destination: Path) -> None:
    run([esptool, "--chip", "esp32", "--port", port, "--before", "default_reset",
         "--after", "no_reset", "read_flash", hex(offset), hex(size), str(destination)])


def reset_device(esptool: str, port: str) -> None:
    try:
        run([esptool, "--chip", "esp32", "--port", port, "--before", "no_reset",
             "--after", "hard_reset", "run"])
    except Exception as error:
        print(f"WARNING: final hard reset failed: {error}", file=sys.stderr)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Explicit serial port resolved for B")
    parser.add_argument("--artifact", required=True, type=Path, help="Exact application image served by A")
    parser.add_argument("--output-dir", type=Path, help="New evidence directory")
    parser.add_argument("--expected-mac", default=EXPECTED_B_MAC)
    args = parser.parse_args()

    esptool = shutil.which("esptool.py") or shutil.which("esptool")
    if not esptool:
        raise RuntimeError("esptool is not available")
    artifact = args.artifact.resolve()
    if not artifact.is_file():
        raise ValueError(f"artifact not found: {artifact}")
    artifact_size = artifact.stat().st_size
    artifact_sha = sha256(artifact)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output_dir = (args.output_dir or Path("build/p2p-verification") / timestamp).resolve()
    output_dir.mkdir(parents=True, exist_ok=False)

    identity = run([esptool, "--chip", "esp32", "--port", args.port, "--before",
                    "default_reset", "--after", "no_reset", "chip_id"])
    observed_mac = parse_mac(identity)
    expected_mac = "".join(character for character in args.expected_mac.lower()
                           if character in "0123456789abcdef")
    if observed_mac != expected_mac:
        raise ValueError(f"B identity gate failed: expected {expected_mac}, observed {observed_mac}")
    atexit.register(reset_device, esptool, args.port)

    partition_path = output_dir / "partition-table.bin"
    read_flash(esptool, args.port, 0x8000, 0x1000, partition_path)
    partitions = parse_partitions(partition_path.read_bytes())
    otadata = require_partition(partitions, "otadata")
    app0 = require_partition(partitions, "app0")
    app1 = require_partition(partitions, "app1")
    if artifact_size > int(app1["size"]):
        raise ValueError(f"artifact ({artifact_size}) exceeds B app1 ({app1['size']})")

    otadata_path = output_dir / "otadata.bin"
    read_flash(esptool, args.port, int(otadata["offset"]), int(otadata["size"]), otadata_path)
    ota_entries, selected = parse_otadata(otadata_path.read_bytes(), 2)

    read_paths = [output_dir / "b-app1-read-1.bin", output_dir / "b-app1-read-2.bin"]
    for path in read_paths:
        read_flash(esptool, args.port, int(app1["offset"]), artifact_size, path)
    read_hashes = [sha256(path) for path in read_paths]
    if read_hashes[0] != read_hashes[1]:
        raise ValueError(f"B app1 reads disagree: {read_hashes}")
    if read_hashes[0] != artifact_sha:
        raise ValueError(f"B app1 does not match A artifact: {read_hashes[0]} != {artifact_sha}")
    if int(selected["slot"]) != 1:
        raise ValueError(f"otadata selects slot {selected['slot']}, not app1")
    image_info = run([esptool, "--chip", "esp32", "image_info", str(read_paths[0])])
    if "Checksum:" not in image_info or "(valid)" not in image_info:
        raise ValueError("B app1 failed esptool image validation")

    receipt = {
        "schema": "tubes-p2p-static-verification-v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "device": {"role": "B", "rom_mac": observed_mac, "port": args.port},
        "artifact": {"path": str(artifact), "size": artifact_size, "sha256": artifact_sha},
        "partition_table": {"path": str(partition_path), "sha256": sha256(partition_path),
                            "entries": partitions},
        "otadata": {"path": str(otadata_path), "entries": ota_entries, "selected": selected},
        "app0": app0,
        "app1": app1,
        "app1_reads": [{"path": str(path), "sha256": digest}
                       for path, digest in zip(read_paths, read_hashes)],
        "result": "exact_app1_and_otadata_verified",
        "runtime_health_required_separately": True,
    }
    receipt_path = output_dir / "receipt.json"
    receipt_path.write_text(json.dumps(receipt, indent=2) + "\n")
    reset_device(esptool, args.port)
    atexit.unregister(reset_device)
    print(f"PASS: exact B app1 and OTA selection verified; receipt={receipt_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"FAIL: {error}", file=sys.stderr)
        raise SystemExit(1)

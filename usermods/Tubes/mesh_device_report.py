#!/usr/bin/env python3

import argparse
import dataclasses
import json
import os
import pathlib
import re
import select
import sys
import termios
import time
from typing import Dict, Optional, Set


FAMILY_IDS = {
    "dig2go": 1,
    "christmas": 1,
    "golden": 1,
    "matrix-m1": 2,
    "athom-c3": 3,
    "gledopto": 4,
    "homelight": 5,
}
MESH_STARTED = 1 << 0
MESH_FOLLOWING = 1 << 1
MASTER_BEHAVIOR = 1 << 2
REPORT_PREFIX = "TUBE_REPORT "
PROBE_PATTERN = re.compile(r"TUBE_PROBE nonce=([0-9A-Fa-f]{8}) mac=([0-9A-Fa-f]{12})")


@dataclasses.dataclass(frozen=True)
class DeviceReport:
    nonce: int
    mac: str
    family: int
    variant: int
    tubes: int
    release: int
    leds: int
    buses: int
    pin: int
    type: int
    role: int
    mesh: int
    node: int
    uplink: int
    uptime: int


def normalize_mac(value: str) -> str:
    normalized = value.lower().replace(":", "").replace("-", "")
    if not re.fullmatch(r"[0-9a-f]{12}", normalized):
        raise ValueError(f"invalid MAC: {value}")
    return normalized


def djb2(value: str) -> int:
    result = 5381
    for byte in value.encode("ascii"):
        result = ((result << 5) + result + byte) & 0xFFFFFFFF
    return result


def parse_report_line(line: str) -> Optional[DeviceReport]:
    report_offset = line.find(REPORT_PREFIX)
    if report_offset < 0:
        return None

    fields: Dict[str, str] = {}
    for item in line[report_offset + len(REPORT_PREFIX):].split():
        key, separator, value = item.partition("=")
        if separator:
            fields[key] = value

    required = {
        "nonce", "mac", "family", "variant", "tubes", "release", "leds", "buses",
        "pin", "type", "role", "mesh", "node", "uplink", "uptime",
    }
    if not required.issubset(fields):
        return None
    try:
        return DeviceReport(
            nonce=int(fields["nonce"], 16),
            mac=normalize_mac(fields["mac"]),
            family=int(fields["family"]),
            variant=int(fields["variant"]),
            tubes=int(fields["tubes"]),
            release=int(fields["release"], 16),
            leds=int(fields["leds"]),
            buses=int(fields["buses"]),
            pin=int(fields["pin"]),
            type=int(fields["type"]),
            role=int(fields["role"]),
            mesh=int(fields["mesh"]),
            node=int(fields["node"], 0),
            uplink=int(fields["uplink"], 0),
            uptime=int(fields["uptime"]),
        )
    except ValueError:
        return None


def report_mismatch(
    report: DeviceReport,
    expected_mac: str,
    expected_family: int,
    expected_variant: int,
    expected_release: str,
    expected_tubes: int,
    expected_leds: int,
    expected_pin: int,
    expected_type: int,
) -> Optional[str]:
    expected = {
        "mac": normalize_mac(expected_mac),
        "family": expected_family,
        "variant": expected_variant,
        "release": djb2(expected_release),
        "tubes": expected_tubes,
        "leds": expected_leds,
        "buses": 1,
        "pin": expected_pin,
        "type": expected_type,
    }
    for field, expected_value in expected.items():
        actual_value = getattr(report, field)
        if actual_value != expected_value:
            return f"{field} is {actual_value!r}, expected {expected_value!r}"

    if not report.mesh & MESH_STARTED:
        return "mesh radio has not started"

    expected_master_behavior = report.role >= 200 or report.variant != 0
    actual_master_behavior = bool(report.mesh & MASTER_BEHAVIOR)
    if actual_master_behavior != expected_master_behavior:
        return (
            f"master behavior is {actual_master_behavior}, but stored role {report.role} "
            f"expects {expected_master_behavior}"
        )

    if not expected_master_behavior:
        if not report.mesh & MESH_FOLLOWING or report.uplink == 0:
            return "non-master device has not joined an uplink"
    return None


def open_serial(path: pathlib.Path) -> int:
    descriptor = os.open(str(path), os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    settings = termios.tcgetattr(descriptor)
    settings[0] = 0
    settings[1] = 0
    settings[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    settings[3] = 0
    settings[4] = termios.B115200
    settings[5] = termios.B115200
    settings[6][termios.VMIN] = 0
    settings[6][termios.VTIME] = 0
    termios.tcsetattr(descriptor, termios.TCSANOW, settings)
    termios.tcflush(descriptor, termios.TCIFLUSH)
    return descriptor


def read_available_lines(descriptor: int, buffer: bytearray, timeout: float):
    readable, _, _ = select.select([descriptor], [], [], timeout)
    if readable:
        try:
            buffer.extend(os.read(descriptor, 4096))
        except BlockingIOError:
            pass

    lines = []
    while b"\n" in buffer:
        raw_line, _, remainder = buffer.partition(b"\n")
        buffer[:] = remainder
        lines.append(raw_line.decode("utf-8", errors="replace").strip())
    return lines


def send_and_confirm(descriptor: int, command: bytes, expected_text: str, timeout: float) -> bool:
    os.write(descriptor, command)
    deadline = time.monotonic() + timeout
    buffer = bytearray()
    while time.monotonic() < deadline:
        for line in read_available_lines(descriptor, buffer, 0.1):
            if expected_text in line:
                return True
    return False


def verify_over_mesh(args) -> int:
    expected_mac = normalize_mac(args.mac)
    descriptor = open_serial(args.serial)
    deadline = time.monotonic() + args.timeout
    next_probe = 0.0
    issued_nonces: Set[int] = set()
    buffer = bytearray()
    last_mismatch = "no report received"
    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_probe:
                os.write(descriptor, f"z{expected_mac}\n".encode("ascii"))
                next_probe = now + 2.0

            for line in read_available_lines(descriptor, buffer, 0.2):
                probe_match = PROBE_PATTERN.search(line)
                if probe_match and normalize_mac(probe_match.group(2)) == expected_mac:
                    issued_nonces.add(int(probe_match.group(1), 16))

                report = parse_report_line(line)
                if report is None or report.mac != expected_mac or report.nonce not in issued_nonces:
                    continue
                last_mismatch = report_mismatch(
                    report,
                    expected_mac,
                    FAMILY_IDS[args.family],
                    args.variant,
                    args.release,
                    args.tubes,
                    args.leds,
                    args.pin,
                    args.type,
                )
                if last_mismatch is None:
                    print(json.dumps(dataclasses.asdict(report), sort_keys=True))
                    return 0
    finally:
        os.close(descriptor)

    print(f"mesh verification timed out: {last_mismatch}", file=sys.stderr)
    return 1


def main() -> int:
    parser = argparse.ArgumentParser(description="Control and verify Tubes devices through a USB mesh node.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    select_parser = subparsers.add_parser("select", help="open the physical-selection window")
    select_parser.add_argument("serial", type=pathlib.Path)

    check_parser = subparsers.add_parser("check", help="require device-report support on the USB controller")
    check_parser.add_argument("serial", type=pathlib.Path)

    offer_parser = subparsers.add_parser("offer", help="wake every mesh device older than a Tubes release")
    offer_parser.add_argument("serial", type=pathlib.Path)
    offer_parser.add_argument("version", type=int)

    verify_parser = subparsers.add_parser("verify", help="probe one MAC until its running state matches")
    verify_parser.add_argument("serial", type=pathlib.Path)
    verify_parser.add_argument("mac")
    verify_parser.add_argument("--family", choices=sorted(FAMILY_IDS), required=True)
    verify_parser.add_argument("--release", required=True)
    verify_parser.add_argument("--variant", type=int, choices=range(3), required=True)
    verify_parser.add_argument("--tubes", type=int, required=True)
    verify_parser.add_argument("--leds", type=int, required=True)
    verify_parser.add_argument("--pin", type=int, required=True)
    verify_parser.add_argument("--type", type=int, required=True)
    verify_parser.add_argument("--timeout", type=float, default=45.0)

    args = parser.parse_args()
    if args.command == "verify":
        return verify_over_mesh(args)

    descriptor = open_serial(args.serial)
    try:
        if args.command == "select":
            if not send_and_confirm(descriptor, b")\n", "[command=)", 2.0):
                print("USB controller did not clear earlier selections", file=sys.stderr)
                return 1
            if not send_and_confirm(descriptor, b"*\n", "[command=*", 2.0):
                print("USB controller did not confirm the selection command", file=sys.stderr)
                return 1
            print("selection window open")
            return 0

        if args.command == "offer":
            if not 1 <= args.version <= 255:
                print("offer version must be between 1 and 255", file=sys.stderr)
                return 2
            command = f"V{args.version}\n".encode("ascii")
            if not send_and_confirm(descriptor, command, "[command=V", 2.0):
                print("USB controller did not confirm the update offer", file=sys.stderr)
                return 1
            print(f"update offer V{args.version} sent")
            return 0

        if not send_and_confirm(descriptor, b"z000000000000\n", "TUBE_PROBE nonce=", 2.0):
            print("USB controller does not support device reports; flash it with the current firmware first", file=sys.stderr)
            return 1
        print("USB controller supports device reports")
        return 0
    finally:
        os.close(descriptor)


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Keep several Dig2Go serial ports open while driving one command channel.

Input lines use ``LABEL:command``.  ``quit`` closes every port.  DTR and RTS
are held inactive before open so the logger itself requests no reset; adapters
whose auto-reset circuitry still pulses on open are opened only once.
"""

import argparse
import selectors
import sys
import time
from pathlib import Path

import serial


def parse_port(value: str) -> tuple[str, str]:
    label, separator, port = value.partition("=")
    if not separator or not label or not port:
        raise argparse.ArgumentTypeError("ports must use LABEL=/dev/cu... form")
    return label, port


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", action="append", type=parse_port, required=True)
    parser.add_argument("--log-dir", type=Path, required=True)
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    args.log_dir.mkdir(parents=True, exist_ok=True)
    selector = selectors.DefaultSelector()
    serial_ports = {}
    logs = {}
    buffers = {}

    try:
        for label, port in args.port:
            channel = serial.Serial(
                port=None,
                baudrate=args.baud,
                timeout=0,
                write_timeout=1,
                exclusive=True,
            )
            channel.dtr = False
            channel.rts = False
            channel.port = port
            channel.open()
            serial_ports[label] = channel
            logs[label] = (args.log_dir / f"{label}.log").open("ab")
            buffers[label] = b""
            selector.register(channel.fileno(), selectors.EVENT_READ, ("serial", label))

        selector.register(sys.stdin.fileno(), selectors.EVENT_READ, ("stdin", ""))
        print("READY " + " ".join(f"{label}={channel.port}" for label, channel in serial_ports.items()), flush=True)

        while True:
            for key, _ in selector.select(timeout=0.25):
                kind, label = key.data
                if kind == "stdin":
                    line = sys.stdin.readline()
                    if not line or line.rstrip("\r\n") == "quit":
                        return 0
                    target, separator, command = line.rstrip("\r\n").partition(":")
                    if not separator or target not in serial_ports:
                        print(f"INPUT_ERROR {line.rstrip()}", flush=True)
                        continue
                    payload = command.encode("utf-8") + b"\n"
                    serial_ports[target].write(payload)
                    serial_ports[target].flush()
                    print(f"TX {target}> {command}", flush=True)
                    continue

                channel = serial_ports[label]
                data = channel.read(channel.in_waiting or 1)
                if not data:
                    continue
                logs[label].write(data)
                logs[label].flush()
                buffers[label] += data
                while b"\n" in buffers[label]:
                    line, buffers[label] = buffers[label].split(b"\n", 1)
                    text = line.decode("utf-8", "replace").rstrip("\r")
                    if text:
                        print(f"{label}> {text}", flush=True)
    finally:
        for log in logs.values():
            log.close()
        for channel in serial_ports.values():
            channel.close()


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""
Guarded OTA updater for Tubes poles.

This intentionally refuses to flash unless it can:
- read the device info and config over HTTP,
- identify the hardware target from config/release markers,
- install and verify a known-safe config,
- verify the local firmware metadata,
- stage old devices through regular WLED 16 before Tubes firmware.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import json
import re
import struct
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple


TARGET_VID = 2605010
DEFAULT_HOST = "4.3.2.1"
REQUEST_TIMEOUT = 8
REBOOT_TIMEOUT = 90
FINAL_VERIFY_SECONDS = 15
METADATA_MAGIC = 0x57535453
METADATA_FORMAT = "<II48s48sI3B"
METADATA_SIZE = struct.calcsize(METADATA_FORMAT)

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
RELEASE_DIR = REPO_ROOT / "build_output" / "release"
PIO_BUILD_DIR = REPO_ROOT / ".pio" / "build"
BACKUP_DIR = Path("/tmp") / "WLED-Tubes-ota-backups"


class AbortUpdate(RuntimeError):
    pass


@dataclass(frozen=True)
class Bus:
    start: int
    length: int
    pin: Tuple[int, ...]
    order: int = 0
    led_type: int = 22
    reversed: bool = False
    skip: int = 0
    refresh: bool = False
    rgbwm: int = 0
    freq: int = 0

    def as_json(self) -> Dict[str, Any]:
        return {
            "start": self.start,
            "len": self.length,
            "pin": list(self.pin),
            "order": self.order,
            "rev": self.reversed,
            "skip": self.skip,
            "type": self.led_type,
            "ref": self.refresh,
            "rgbwm": self.rgbwm,
            "freq": self.freq,
        }


@dataclass(frozen=True)
class Button:
    pin: int
    button_type: int = 2

    def as_json(self) -> Dict[str, Any]:
        return {"type": self.button_type, "pin": [self.pin], "macros": [0, 0, 0]}


@dataclass(frozen=True)
class Target:
    key: str
    display_name: str
    expected_release: str
    regular_release: str
    target_firmware: Tuple[Path, ...]
    regular_firmware: Tuple[Path, ...]
    arch_values: Tuple[str, ...]
    buses: Tuple[Bus, ...]
    buttons: Tuple[Button, ...]
    relay_pin: int
    max_power_ma: int = 700
    allow_legacy_regular_stage: bool = True

    @property
    def total_leds(self) -> int:
        return sum(bus.length for bus in self.buses)


TARGETS: Tuple[Target, ...] = (
    Target(
        key="dig2go",
        display_name="QuinLED Dig2Go Tubes",
        expected_release="DIG2GO_TUBES",
        regular_release="ESP32",
        target_firmware=(
            PIO_BUILD_DIR / "esp32_quinled_dig2go_tubes" / "firmware.bin",
            RELEASE_DIR / "WLED_16.0.1_DIG2GO_TUBES.bin",
        ),
        regular_firmware=(
            PIO_BUILD_DIR / "esp32_quinled_dig2go" / "firmware.bin",
            RELEASE_DIR / "WLED_16.0.1_ESP32.bin",
        ),
        arch_values=("esp32",),
        buses=(Bus(start=0, length=150, pin=(16,)),),
        buttons=(Button(pin=0),),
        relay_pin=12,
    ),
    Target(
        key="dignext2",
        display_name="QuinLED Dig-Next-2 Tubes",
        expected_release="DIGNEXT2_TUBES",
        regular_release="QUINLED_DIG_NEXT_2",
        target_firmware=(
            PIO_BUILD_DIR / "esp32_quinled_dignext2_tubes" / "firmware.bin",
            RELEASE_DIR / "WLED_16.0.1_DIGNEXT2_TUBES.bin",
        ),
        regular_firmware=(
            PIO_BUILD_DIR / "esp32_quinled_dignext2" / "firmware.bin",
        ),
        arch_values=("esp32",),
        buses=(Bus(start=0, length=150, pin=(2,)), Bus(start=150, length=150, pin=(4,))),
        buttons=(Button(pin=34), Button(pin=35)),
        relay_pin=5,
    ),
    Target(
        key="athom-c3",
        display_name="Athom ESP32-C3 Tubes",
        expected_release="ESP32-C3_ATHOM_TUBES",
        regular_release="ESP32-C3",
        target_firmware=(
            PIO_BUILD_DIR / "esp32-c3-athom_tubes" / "firmware.bin",
            RELEASE_DIR / "WLED_16.0.1_ESP32-C3_ATHOM_TUBES.bin",
        ),
        regular_firmware=(
            PIO_BUILD_DIR / "esp32-c3-athom" / "firmware.bin",
            PIO_BUILD_DIR / "esp32c3dev" / "firmware.bin",
        ),
        arch_values=("esp32-c3", "esp32c3", "esp32-c3fn4"),
        buses=(Bus(start=0, length=150, pin=(10,)),),
        buttons=(Button(pin=9),),
        relay_pin=-1,
    ),
)


def log(message: str) -> None:
    print(message, flush=True)


def fail(message: str) -> None:
    raise AbortUpdate(message)


def c_string(raw: bytes) -> str:
    return raw.split(b"\0", 1)[0].decode("utf-8", errors="replace")


def djb2(text: str) -> int:
    value = 5381
    for char in text:
        value = ((value << 5) + value + ord(char)) & 0xFFFFFFFF
    return value


def read_firmware_metadata(path: Path) -> Dict[str, Any]:
    data = path.read_bytes()
    for offset in range(0, len(data) - METADATA_SIZE + 1):
        if data[offset] != (METADATA_MAGIC & 0xFF):
            continue
        magic, desc_version, version, release, checksum, s0, s1, s2 = struct.unpack_from(
            METADATA_FORMAT, data, offset
        )
        if magic != METADATA_MAGIC:
            continue
        release_name = c_string(release)
        if checksum != djb2(release_name):
            continue
        return {
            "offset": offset,
            "desc_version": desc_version,
            "version": c_string(version),
            "release": release_name,
            "safe_update_version": [s0, s1, s2],
            "sha256": hashlib.sha256(data).hexdigest(),
            "size": len(data),
        }
    fail(f"{path} does not contain valid WLED firmware metadata")


def pick_existing(paths: Iterable[Path], label: str, expected_release: str) -> Path:
    candidates = list(paths)
    for path in candidates:
        if not path.exists():
            continue
        metadata = read_firmware_metadata(path)
        if metadata["release"] != expected_release:
            fail(
                f"{label} firmware {path} has release {metadata['release']!r}, "
                f"expected {expected_release!r}"
            )
        log(
            f"Using {label} firmware {path.relative_to(REPO_ROOT)} "
            f"({metadata['release']} {metadata['version']}, sha256 {metadata['sha256'][:12]})"
        )
        return path

    joined = "\n  ".join(str(p.relative_to(REPO_ROOT)) for p in candidates)
    fail(f"Missing {label} firmware for {expected_release}. Checked:\n  {joined}")


def http_bytes(
    host: str,
    path: str,
    *,
    method: str = "GET",
    body: Optional[bytes] = None,
    headers: Optional[Dict[str, str]] = None,
    timeout: int = REQUEST_TIMEOUT,
) -> Tuple[int, bytes, str]:
    url = f"http://{host}{path}"
    req = urllib.request.Request(url, data=body, method=method, headers=headers or {})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as response:
            return response.status, response.read(), response.headers.get_content_type()
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        fail(f"HTTP {exc.code} from {path}: {detail[:400]}")
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        fail(f"Could not reach {url}: {exc}")


def http_json(host: str, path: str, *, data: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    body = None
    headers = {"Connection": "close"}
    method = "GET"
    if data is not None:
        body = json.dumps(data, separators=(",", ":")).encode("utf-8")
        headers["Content-Type"] = "application/json"
        method = "POST"

    status, payload, _ = http_bytes(host, path, method=method, body=body, headers=headers)
    if status != 200:
        fail(f"HTTP {status} from {path}")
    try:
        decoded = json.loads(payload.decode("utf-8"))
    except json.JSONDecodeError as exc:
        fail(f"{path} did not return JSON: {exc}")
    if not isinstance(decoded, dict):
        fail(f"{path} returned a JSON value that is not an object")
    return decoded


def get_info(host: str) -> Dict[str, Any]:
    doc = http_json(host, "/json/si")
    info = doc.get("info", doc)
    if not isinstance(info, dict):
        fail("/json/si did not contain an info object")
    return info


def get_config(host: str) -> Dict[str, Any]:
    return http_json(host, "/json/cfg")


def post_config(host: str, config: Dict[str, Any]) -> None:
    result = http_json(host, "/json/cfg", data=config)
    if result.get("success") is not True:
        fail(f"/json/cfg did not acknowledge success: {result}")


def multipart_upload(
    host: str,
    path: str,
    firmware: Path,
    *,
    skip_validation: bool,
    timeout: int = 180,
) -> str:
    boundary = f"----TubesSafeOTA{int(time.time() * 1000)}"
    body = bytearray()

    def add_field(name: str, value: str) -> None:
        body.extend(f"--{boundary}\r\n".encode("ascii"))
        body.extend(f'Content-Disposition: form-data; name="{name}"\r\n\r\n'.encode("ascii"))
        body.extend(value.encode("utf-8"))
        body.extend(b"\r\n")

    if skip_validation:
        add_field("skipValidation", "1")

    body.extend(f"--{boundary}\r\n".encode("ascii"))
    body.extend(
        (
            f'Content-Disposition: form-data; name="update"; '
            f'filename="{firmware.name}"\r\n'
            "Content-Type: application/octet-stream\r\n\r\n"
        ).encode("ascii")
    )
    body.extend(firmware.read_bytes())
    body.extend(b"\r\n")
    body.extend(f"--{boundary}--\r\n".encode("ascii"))

    status, payload, _ = http_bytes(
        host,
        path,
        method="POST",
        body=bytes(body),
        headers={
            "Content-Type": f"multipart/form-data; boundary={boundary}",
            "Connection": "close",
        },
        timeout=timeout,
    )
    text = payload.decode("utf-8", errors="replace")
    lowered = text.lower()
    if status != 200 or "failed" in lowered or "denied" in lowered or "not support" in lowered:
        fail(f"Firmware upload was not accepted by WLED: HTTP {status}: {text[:500]}")
    return text


def normalize_arch(value: Any) -> str:
    return str(value or "").strip().lower()


def config_buses(config: Dict[str, Any]) -> List[Dict[str, Any]]:
    return list(config.get("hw", {}).get("led", {}).get("ins", []) or [])


def config_buttons(config: Dict[str, Any]) -> List[Dict[str, Any]]:
    return list(config.get("hw", {}).get("btn", {}).get("ins", []) or [])


def json_pin(value: Any) -> Tuple[int, ...]:
    if isinstance(value, list):
        return tuple(int(v) for v in value)
    if value is None:
        return tuple()
    return (int(value),)


def bus_matches_config(bus_config: Dict[str, Any], target_bus: Bus) -> bool:
    return (
        int(bus_config.get("start", -1)) == target_bus.start
        and int(bus_config.get("len", -1)) == target_bus.length
        and json_pin(bus_config.get("pin")) == target_bus.pin
        and int(bus_config.get("type", 22)) == target_bus.led_type
        and int(bus_config.get("order", 0)) == target_bus.order
        and bool(bus_config.get("rev", False)) == target_bus.reversed
        and int(bus_config.get("skip", 0)) == target_bus.skip
    )


def config_has_target_hardware(config: Dict[str, Any], target: Target) -> bool:
    buses = config_buses(config)
    if len(buses) != len(target.buses):
        return False
    return all(bus_matches_config(buses[i], target.buses[i]) for i in range(len(target.buses)))


def config_pins_match_target(config: Dict[str, Any], target: Target) -> bool:
    buses = config_buses(config)
    if len(buses) != len(target.buses):
        return False
    for index, target_bus in enumerate(target.buses):
        if json_pin(buses[index].get("pin")) != target_bus.pin:
            return False
    return True


def config_total_is_plausible(config: Dict[str, Any], target: Target) -> bool:
    total = int(config.get("hw", {}).get("led", {}).get("total", -1))
    if total == target.total_leds:
        return True
    # Early Dig2Go fleet configs used 112 LEDs before the 150 LED Tubes profile.
    return target.key == "dig2go" and total == 112


def config_is_safe(config: Dict[str, Any], target: Target) -> bool:
    hw_led = config.get("hw", {}).get("led", {})
    if int(hw_led.get("total", -1)) != target.total_leds:
        return False
    if not config_has_target_hardware(config, target):
        return False
    if int(config.get("def", {}).get("ps", -1)) != 0:
        return False
    if bool(config.get("def", {}).get("on", True)):
        return False
    if int(config.get("def", {}).get("bri", -1)) > 5:
        return False
    if bool(config.get("light", {}).get("aseg", True)):
        return False
    if bool(config.get("wifi", {}).get("sleep", True)):
        return False
    return True


def target_score(info: Dict[str, Any], config: Dict[str, Any], target: Target) -> int:
    score = 0
    release = str(info.get("release", ""))
    name = str(info.get("name", "")).strip().lower()
    arch = normalize_arch(info.get("arch"))

    if release == target.expected_release:
        score += 100
    if release == target.regular_release:
        score += 25
    if arch in target.arch_values:
        score += 10
    if target.key in name:
        score += 80
    if target.key == "dig2go" and name == "dig2go":
        score += 100
    if target.key == "athom-c3" and "athom" in name:
        score += 80
    if config_has_target_hardware(config, target):
        score += 80
    elif config_pins_match_target(config, target):
        score += 40

    if config_total_is_plausible(config, target):
        score += 20

    relay = config.get("hw", {}).get("relay", {}).get("pin")
    if relay is not None and int(relay) == target.relay_pin:
        score += 20

    buttons = config_buttons(config)
    target_button_pins = tuple(button.pin for button in target.buttons)
    config_button_pins = tuple(
        int(btn.get("pin", [-1])[0] if isinstance(btn.get("pin"), list) else btn.get("pin", -1))
        for btn in buttons[: len(target.buttons)]
    )
    if config_button_pins == target_button_pins:
        score += 20

    return score


def identify_target(info: Dict[str, Any], config: Dict[str, Any]) -> Target:
    scored = [(target_score(info, config, target), target) for target in TARGETS]
    scored.sort(key=lambda item: item[0], reverse=True)
    score, target = scored[0]
    if score < 80:
        release = info.get("release", "unknown")
        arch = info.get("arch", "unknown")
        name = info.get("name", "unknown")
        fail(
            "Cannot prove what hardware this is; refusing OTA. "
            f"info.name={name!r} release={release!r} arch={arch!r}"
        )
    if len(scored) > 1 and scored[1][0] == score:
        fail(f"Target detection is ambiguous between {target.key} and {scored[1][1].key}; refusing OTA")
    return target


def safe_config_patch(target: Target) -> Dict[str, Any]:
    button_entries = [button.as_json() for button in target.buttons]
    while len(button_entries) < 4:
        button_entries.append({"type": 0, "pin": [-1], "macros": [0, 0, 0]})

    return {
        "wifi": {"sleep": False},
        "hw": {
            "led": {
                "total": target.total_leds,
                "maxpwr": target.max_power_ma,
                "ledma": 55,
                "cct": True,
                "cr": False,
                "cb": 0,
                "fps": 60,
                "rgbwm": 3,
                "ld": False,
                "ins": [bus.as_json() for bus in target.buses],
            },
            "com": [],
            "btn": {"max": 4, "pull": True, "ins": button_entries, "tt": 32, "mqtt": False},
            "ir": {"pin": -1, "type": 0, "sel": True},
            "relay": {"pin": target.relay_pin, "rev": False},
            "baud": 1152,
            "if": {"i2c-pin": [-1, -1], "spi-pin": [-1, -1, -1]},
        },
        "light": {
            "scale-bri": 100,
            "pal-mode": 0,
            "aseg": False,
            "gc": {"bri": 1, "col": 2.8, "val": 2.8},
            "tr": {"mode": True, "dur": 80, "pal": 1, "rpc": 5},
            "nl": {"mode": 1, "dur": 60, "tbri": 0, "macro": 0},
        },
        "def": {"ps": 0, "on": False, "bri": 5},
        "if": {
            "sync": {
                "port0": 21324,
                "port1": 65506,
                "recv": {"bri": False, "col": False, "fx": False, "pal": False, "grp": 1, "seg": False, "sb": False},
                "send": {"en": False, "dir": False, "btn": False, "va": False, "hue": False, "macro": False, "grp": 1, "ret": 0},
            },
            "nodes": {"list": False, "bcast": False},
            "live": {
                "en": False,
                "mso": False,
                "port": 5568,
                "mc": False,
                "dmx": {"uni": 1, "seqskip": False, "e131prio": 0, "addr": 1, "dss": 0, "mode": 4},
                "timeout": 25,
                "maxbri": False,
                "no-gc": True,
                "offset": 0,
            },
            "va": {"alexa": False, "macros": [0, 0], "p": 0},
            "ntp": {"en": False, "host": "0.wled.pool.ntp.org", "tz": 0, "offset": 0, "ampm": False, "ln": 0, "lt": 0},
        },
        "ol": {"clock": 0, "cntdwn": False, "min": 0, "max": max(0, target.total_leds - 1), "o12pix": 0, "o5m": False, "osec": False},
        "timers": {"cntdwn": {"goal": [20, 1, 1, 0, 0, 0], "macro": 0}, "ins": []},
        "um": {"AudioReactive": {"enabled": False}},
        "sv": True,
    }


def wait_for_device(
    host: str,
    *,
    timeout: int,
    expected_release: Optional[str] = None,
    min_vid: Optional[int] = None,
    expected_led_count: Optional[int] = None,
) -> Dict[str, Any]:
    deadline = time.monotonic() + timeout
    last_info: Optional[Dict[str, Any]] = None
    last_error = ""

    while time.monotonic() < deadline:
        try:
            info = get_info(host)
            last_info = info
            release_ok = expected_release is None or info.get("release") == expected_release
            vid_ok = min_vid is None or int(info.get("vid", 0)) >= min_vid
            led_count = info.get("leds", {}).get("count") if isinstance(info.get("leds"), dict) else None
            leds_ok = expected_led_count is None or int(led_count or -1) == expected_led_count
            if release_ok and vid_ok and leds_ok:
                return info
        except AbortUpdate as exc:
            last_error = str(exc)
        time.sleep(2)

    if last_info:
        fail(f"Device came back, but not in the expected state: {json.dumps(last_info, sort_keys=True)[:700]}")
    fail(f"Device did not come back within {timeout}s. Last error: {last_error}")


def reset_and_verify_config(host: str, target: Target) -> None:
    log("Resetting once to prove the safe config survives boot")
    try:
        http_bytes(host, "/reset", timeout=3)
    except AbortUpdate:
        pass
    time.sleep(3)
    wait_for_device(host, timeout=REBOOT_TIMEOUT)
    config = get_config(host)
    if not config_is_safe(config, target):
        fail("Safe config did not survive reboot; refusing firmware upload")


def install_safe_config(host: str, target: Target) -> None:
    log(f"Installing known-safe {target.display_name} config")
    post_config(host, safe_config_patch(target))

    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        time.sleep(1)
        config = get_config(host)
        if config_is_safe(config, target):
            reset_and_verify_config(host, target)
            return

    fail("Device accepted /json/cfg but did not report the safe config back")


def backup_device_state(host: str, info: Dict[str, Any], config: Dict[str, Any], target: Target) -> None:
    BACKUP_DIR.mkdir(parents=True, exist_ok=True)
    stamp = _dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    device_id = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(info.get("deviceId", "unknown")))
    base = BACKUP_DIR / f"{stamp}_{device_id}_{target.key}"
    (base.with_suffix(".info.json")).write_text(json.dumps(info, indent=2, sort_keys=True) + "\n")
    (base.with_suffix(".cfg.json")).write_text(json.dumps(config, indent=2, sort_keys=True) + "\n")
    log(f"Backed up current info/config to {base.relative_to(REPO_ROOT)}.*")


def refresh_and_verify_safe(host: str, target: Target) -> Dict[str, Any]:
    info = wait_for_device(host, timeout=REBOOT_TIMEOUT)
    config = get_config(host)
    if not config_is_safe(config, target):
        fail("Device is reachable, but config is no longer safe; refusing next OTA stage")
    return info


def upload_stage(
    host: str,
    *,
    stage_name: str,
    firmware: Path,
    expected_release: str,
    target: Target,
    skip_validation: bool,
) -> Dict[str, Any]:
    log(
        f"Uploading {stage_name}: {firmware.relative_to(REPO_ROOT)}"
        + (" with WLED validation bypass after local checks" if skip_validation else "")
    )
    multipart_upload(host, "/update", firmware, skip_validation=skip_validation)
    log(f"Waiting for {stage_name} to boot as {expected_release}")
    return wait_for_device(
        host,
        timeout=REBOOT_TIMEOUT,
        expected_release=expected_release,
        min_vid=TARGET_VID,
        expected_led_count=target.total_leds,
    )


def verify_stable_final(host: str, target: Target) -> None:
    log(f"Watching final firmware for {FINAL_VERIFY_SECONDS}s")
    end = time.monotonic() + FINAL_VERIFY_SECONDS
    last_uptime = -1
    while time.monotonic() < end:
        info = get_info(host)
        if info.get("release") != target.expected_release:
            fail(f"Final release changed unexpectedly: {info.get('release')!r}")
        if int(info.get("vid", 0)) < TARGET_VID:
            fail(f"Final WLED vid is too old: {info.get('vid')!r}")
        uptime = int(info.get("uptime", 0))
        if uptime < last_uptime:
            fail("Final firmware rebooted during the stability watch")
        last_uptime = uptime
        time.sleep(3)


def run_update(host: str, *, reflash: bool) -> None:
    log(f"Preflighting {host}")
    info = get_info(host)
    config = get_config(host)
    target = identify_target(info, config)
    log(
        f"Identified {target.display_name}: release={info.get('release', 'unknown')} "
        f"vid={info.get('vid', config.get('vid', 'unknown'))} arch={info.get('arch', 'unknown')}"
    )
    backup_device_state(host, info, config, target)

    if not config_is_safe(config, target):
        install_safe_config(host, target)
        info = refresh_and_verify_safe(host, target)
    else:
        log("Existing config already matches the safe OTA profile")

    current_release = str(info.get("release", ""))
    current_vid = int(info.get("vid", config.get("vid", 0)) or 0)

    if current_release == target.expected_release and current_vid >= TARGET_VID and not reflash:
        log("Device is already on the target Tubes firmware; skipping firmware upload")
        verify_stable_final(host, target)
        log("Safe OTA check complete")
        return

    if current_vid < TARGET_VID:
        if not target.allow_legacy_regular_stage:
            fail("This target does not have a safe legacy staging path")
        regular_firmware = pick_existing(target.regular_firmware, "regular WLED 16 staging", target.regular_release)
        upload_stage(
            host,
            stage_name="regular WLED 16 staging firmware",
            firmware=regular_firmware,
            expected_release=target.regular_release,
            target=target,
            skip_validation=(current_release not in ("", target.regular_release)),
        )
        info = refresh_and_verify_safe(host, target)
        current_release = str(info.get("release", ""))
        current_vid = int(info.get("vid", 0) or 0)

    if current_vid < TARGET_VID:
        fail(f"Device is still below WLED vid {TARGET_VID}; refusing Tubes firmware")

    if current_release not in (target.regular_release, target.expected_release):
        fail(
            f"Current release {current_release!r} is not on a safe path to "
            f"{target.expected_release!r}; refusing OTA"
        )

    if current_release != target.expected_release or reflash:
        target_firmware = pick_existing(target.target_firmware, "Tubes target", target.expected_release)
        upload_stage(
            host,
            stage_name="Tubes firmware",
            firmware=target_firmware,
            expected_release=target.expected_release,
            target=target,
            skip_validation=(current_release != target.expected_release),
        )

    final_config = get_config(host)
    if not config_is_safe(final_config, target):
        fail("Final firmware booted, but final config is not safe")
    verify_stable_final(host, target)
    log("Safe OTA complete")


def run_preflight(host: str) -> None:
    info = get_info(host)
    config = get_config(host)
    target = identify_target(info, config)
    log(
        json.dumps(
            {
                "target": target.key,
                "display": target.display_name,
                "release": info.get("release"),
                "vid": info.get("vid", config.get("vid")),
                "config_safe": config_is_safe(config, target),
            },
            indent=2,
            sort_keys=True,
        )
    )


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Guarded Tubes OTA updater")
    parser.add_argument("command", choices=("preflight", "update"))
    parser.add_argument("host", nargs="?", default=DEFAULT_HOST)
    parser.add_argument("--reflash", action="store_true", help="re-upload target firmware even if it is already installed")
    args = parser.parse_args(argv)

    try:
        if args.command == "preflight":
            run_preflight(args.host)
        else:
            run_update(args.host, reflash=args.reflash)
        return 0
    except AbortUpdate as exc:
        print(f"REFUSING OTA: {exc}", file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())

# Remote upgrade procedure

Remote upgrades use ESP-NOW to select one physical device and verify it after
reboot. Firmware and configuration travel over Wi-Fi, either through one selected
device's direct access point or the shared parallel-pull network; firmware is never
carried over the mesh.

Release 22 and newer also support a canary-first parallel pull from one local HTTP
server. That path removes per-device Wi-Fi association and is the preferred workflow
after the one-time migration. See
[`docs/FLEET_PULL_UPDATE.md`](docs/FLEET_PULL_UPDATE.md) for network requirements,
measured results, safety gates, and the fleet command.

## Fast fleet workflow

For a durable job that survives a temporary loss of this computer's normal
network, start either updater through `fleet_update_job.sh`. macOS `launchd`
owns the process, output is kept under `build_output/fleet-update-jobs`, and
the launcher restores the Wi-Fi network that was active when the job began:

```sh
cd usermods/Tubes
./fleet_update_job.sh start-one dig2go /dev/cu.usbserial-8320
./fleet_update_job.sh start-target dig2go /dev/cu.usbserial-8320 1ABC
./fleet_update_job.sh start-fleet /dev/cu.usbserial-8320
./fleet_update_job.sh status
./fleet_update_job.sh log
```

`start-one` requires a physical double-click. On release 15 and newer, first run
`mesh_device_report.py manifest` and pass the fresh four-digit Device ID to
`start-target`; that device enters update mode without a button press. The fleet form is
the unattended canary-first workflow described below. A wired connection or a
second network adapter keeps remote control available throughout the update;
without one, the job continues locally while Wi-Fi is attached to a device and
remote control resumes after the saved network is restored.

Flash the USB-connected controller with current firmware once. That controller
is the trusted mesh bridge for all later upgrades: it opens each selection
window, probes the selected device by stable MAC after reboot, and rejects a
device whose firmware, LED output, stored role, or live mesh state is wrong.

Build the needed images, then start the repeated-device runner on macOS. Use
the `christmas` or `golden` profile instead of `dig2go` when upgrading
one of those intentionally special devices:

```sh
uvx platformio run -e esp32_quinled_dig2go_tubes
uvx platformio run -e christmas -e golden
cd usermods/Tubes
./upgrade_fast.sh dig2go /dev/cu.usbserial-8320 --loop
```

For each device, the runner says when to double-click. It connects to the same
`WLED-UPDATE` SSID and password every time, discovers the selected device's
MAC, enrolls that physical device as a Dig2Go, backs up and validates its
configuration, uploads the image, waits until the update AP disappears, and
then verifies the exact MAC over ESP-NOW. When `FAST_UPGRADE_OK` appears, move
to the next device. The shared SSID is deliberate, so the computer never needs
to choose among per-device network names. Before opening each selection window,
the runner sends a mesh-wide deselect command so an old update AP cannot be
mistaken for the device that was just double-clicked.

Most devices need one double-click and one Wi-Fi upload. A legacy device with
no explicit LED bus needs a safe configuration migration and one extra
selection because WLED reboots after accepting `/cfg.json`.

## Unattended legacy batch

When nobody is available to press buttons, a current USB mesh controller can
wake every device older than the current Tubes release and drain their shared
update SSID:

```sh
cd usermods/Tubes
./upgrade_batch.sh /dev/cu.usbserial-8320
```

The batch validates the standard, Christmas, Golden, and ATHOM-C3 images before sending `V15`. It upgrades
an AP only when its exact MAC is already enrolled or its running release is one
of `DIG2GO_TUBES`, `CHRISTMAS_TUBES`, or `GOLDEN_TUBES`. A legacy `Custom`,
`Light Tube`, or blank release does not distinguish those variants, so an
unenrolled device is backed up, logged, and locally dismissed without a config
or firmware write. This is intentionally fail-closed: an unattended run can
leave devices for later identification, but it cannot guess away a special
installation build.

For a physically confirmed session containing only Dig2Go and ATHOM-C3 poles,
set `TUBES_FLEET_HARDWARE_SET=dig2go,athom-c3`. This permits exact-MAC
enrollment from the deployed hardware signatures: Dig2Go uses LED GPIO 16 and
button GPIO 0, while ATHOM-C3 uses LED GPIO 10 and button GPIO 9. The scope is
deliberately opt-in because Christmas and Golden use Dig2Go hardware pins and
must not be inferred this way when those installations are powered.

Use `TUBES_FLEET_HARDWARE_SET=dig2go` when only standard Dig2Go poles are
powered. In that narrower mode, the ATHOM-C3 signature remains unrecognized and
is skipped rather than enrolled.

The default five rounds handle devices that missed an offer and devices that
needed a configuration migration reboot before OTA. The first known device is
the canary; every other AP is archived and rebooted without an upload until the
canary passes after the update-AP wave closes. Results and captured
`info.json`/`cfg.json` files are stored under a timestamped
`build_output/device-backups/batch-*` directory.

If a firmware upload is accepted but the exact MAC does not rejoin the mesh and
pass its nonce-bound report, the batch stops before uploading another device.
That failure can indicate a fleet-wide compatibility problem, so continuing
unattended would turn a single recoverable device into an unknown number of
detached devices.

Mesh verification must run after the update-AP wave has been drained. Devices
serving `WLED-UPDATE` are temporarily unavailable as normal mesh participants,
so testing a canary while its peers are still selected can look like a failed
join. `drain_update_aps.sh` connects to each remaining shared AP, archives its
identity and config, and reboots only that HTTP-connected device to leave
update mode. The reset endpoint works on legacy WLED builds; the drain never
uploads configuration or firmware.

## Safety model

The mesh node ID is not a device identity because it is randomized at reboot.
The upgrader identifies a selected device by the stable MAC returned from
`/json/si`, then selects firmware from an explicit board-name and architecture
allowlist. A classic `esp32` architecture alone is never enough to distinguish
a Dig2Go from another ESP32 controller.

Every mutating command requires the MAC reported by `inspect`. Reconnecting to
a different `WLED-UPDATE` access point therefore stops before either config or
firmware is written. The uploader also reads the WLED compatibility metadata
from the local binary and requires its release family to match the selected
hardware profile.

Firmware OTA preserves the filesystem and the existing `/cfg.json`. The
upgrader never installs `default_config.json`; that file contains hardware
choices that are not valid for every Tubes controller. Each write first saves a
mode-`600` config backup under `build_output/device-backups/DEVICE_MAC/`.

The post-reboot report carries a fresh request nonce, stable MAC, compiled
hardware family, WLED release hash, Tubes protocol version, active bus shape,
stored role, effective master behavior, node ID, and uplink ID. A non-master
does not pass until it has joined an uplink, so a reachable but detached device
cannot be counted as a successful upgrade.

The general Dig2Go profile does not define `GOLDEN` or `CHRISTMAS`.
Those intentional installation profiles retain their special palettes,
protocol-local value `0xFFF` (Device ID `0x1FFF` in protocol generation 1), and
master behavior under distinct release names. Once a MAC is
enrolled as a special profile, the updater refuses to replace it with standard
Dig2Go firmware. The mesh report also verifies the compiled variant.

## Manual fallback: build the Dig2Go image

From the repository root:

```sh
uvx platformio run -e esp32_quinled_dig2go_tubes
```

The resulting image is
`build_output/firmware/esp32_quinled_dig2go_tubes.bin`. Its embedded release
must be `DIG2GO_TUBES`; `firmware.sh upload` refuses `Custom`, a different board
family, or a binary without valid WLED metadata.

## Manual fallback: select exactly one device

1. Connect a controller to USB and open its 115200-baud serial console.
2. Send `*` followed by a newline. Every mesh device enters selection mode for
   20 seconds.
3. During that window, double-click the button on the physical upgrade target.
   Its selected state is visible as purple LEDs and it advertises
   `WLED-UPDATE` with password `update1234`.
4. Connect the computer to `WLED-UPDATE`. The device is available at
   `http://4.3.2.1`.

Only one device should be selected. If the window expires, a double-click is a
normal BPM toggle; send `*` again before retrying selection.

## Manual fallback: inspect and prepare configuration

Run the read-only inspection first:

```sh
cd usermods/Tubes
./firmware.sh inspect
```

For a supported Dig2Go it reports a stable MAC, `name=dig2go`,
`arch=esp32`, the running version, live LED count, and chosen firmware image.
Copy that MAC into every later command.

Fleet firmware may report `Custom`, `Light Tube`, or a native board name.
Those labels describe build history or mutable configuration and do not prove
the hardware family. After physically identifying a legacy board, enroll that
exact MAC once:

```sh
./firmware.sh enroll 5443b2b542f4 dig2go
./firmware.sh inspect
```

Enrollment changes only the local, mode-`600` device inventory under
`build_output/device-backups/`; it does not write to the controller. Later
commands still require the same MAC and recheck the connected device.

Older WLED configs may contain a correct `hw.led.total` and current limit but an
empty `hw.led.ins` array. WLED 0.15 replaces that implicit bus with its compiled
fallback during first boot, which can reduce a 300-LED tube to 30 LEDs. Prepare
the device before firmware upload:

```sh
./firmware.sh prepare a0b765ca2e28
```

If the output is already explicit, `prepare` only validates and backs it up. If
it is legacy, `prepare` writes one explicit Dig2Go output on GPIO 16 while
preserving the old LED total and current limit, then reboots. After a migration,
repeat the selection and Wi-Fi connection steps before continuing.

If `hw.led.ins` is already explicit, its bus length is authoritative because it
is what the running firmware actually drives. A stale `hw.led.total` is reported
but does not block an upgrade or replace the explicit length; this allows tubes
with different LED counts to retain their existing hardware configuration.
When no explicit bus exists, migration-era `Custom` or `Light Tube` devices use
the fleet default of 112 LEDs; other legacy devices retain their stored total.

## Manual fallback: upload and verify

Upload only after `prepare` reports an explicit, supported output:

```sh
./firmware.sh upload a0b765ca2e28
```

The script checks the selected MAC again, backs up config again, verifies the
local image metadata, and requires an HTTP-success response from `/update`.
Leave the device powered until it reboots and rejoins the mesh.

Builds created before the Dig2Go release identity was fixed report `Custom`.
WLED 0.15 correctly refuses to cross release families, so use the explicit
one-time transition only for a device that `inspect` identifies as that known
Dig2Go build:

```sh
./firmware.sh adopt a0b765ca2e28
```

`adopt` repeats the board, MAC, config, and local-image checks before asking
WLED to skip its redundant release comparison. It refuses any running release
other than `Custom`; after adoption, all later upgrades use `upload` normally.

For final verification, select the same physical device again, reconnect to
`WLED-UPDATE`, and run:

```sh
./firmware.sh verify a0b765ca2e28
```

HTTP verification requires the same MAC, the `DIG2GO_TUBES` running release, a
valid GPIO-16 output, and a live LED count equal to the configured total. It is
a recovery fallback; the fast runner's nonce-bound ESP-NOW report is the full
end-to-end check because it also proves the rebooted firmware joined the mesh.

## Recovery

- Hold button 0 for more than five seconds to reopen `WLED-AP` if Wi-Fi config
  is inaccessible, then restore the saved `/cfg.json` through `/upload`.
- Hold button 0 for more than 30 seconds only when a factory reset is intended.
- If the firmware cannot boot or serve an AP, recover over USB serial flashing.

Do not use the old batch scan to upgrade every visible `WLED-AP`. It cannot
prove physical identity and can select Dig2Go firmware for another classic
ESP32 board.

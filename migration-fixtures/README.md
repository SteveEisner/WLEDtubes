# WLEDTubes migration fixtures

This directory pins firmware inputs and behavior fixtures used to test migrations into the current Tubes base. It is not a public firmware catalog.

## Firmware fixtures

- Stock WLED 0.14.3, 0.15.4, and 16.0.1 are official non-Tubes Dig2Go
  release assets from `intermittech/QuinLED-Firmware`, built from the matching
  upstream `wled/WLED` release.
- Tubes v13 is a reconstructed Dig2Go build from source commit `69f1bd8b` using the earliest recoverable Dig2Go Tubes build configuration at `9e7d3c70`.
- Tubes v14 includes canonical Dig2Go and exact compiled Athom ESP32-C3 OTA
  artifacts used as migration destinations. The Athom profile is the existing
  Steve-main `esp32-c3-athom_tubes` target (LED GPIO 10, button GPIO 9); it is
  not a generic ESP32-C3 image.

The stock binaries are **source fixtures only** even though they carry exact
published Dig2Go build defaults. They model devices being migrated into Tubes;
they are not destination artifacts. The v13 build is also source-only because
its original local build override and historical binary were not committed;
its hash proves this reconstruction, not every deployed v13 unit.

## Required migration order

```text
inspect exact hardware and installed lineage
→ back up configuration and persistent Tubes role state
→ normalize only the explicit hardware output bus when required for safe boot
→ install the exact Tubes hardware artifact when required
→ reboot and verify firmware identity, hardware target, and health
→ apply the runtime configuration/profile supported by that firmware
→ read back and verify effective configuration
```

Never apply a WLED 16/current-schema runtime profile before an older stock-WLED
or Tubes v13 device has booted and verified the destination Tubes base. The
preflash bus normalization is a bounded hardware-safety transform, not general
configuration migration.

## V13 reconstruction

Use a detached worktree at `69f1bd8b`, then copy the files from `build-config/tubes-v13/` into its root. Apply `dependency-repairs.patch`, run the historical `npm ci` and `npm run build`, then build in an isolated PlatformIO core:

```bash
PLATFORMIO_CORE_DIR=.pio-core pio run -e esp32_quinled_dig2go_tubes
```

The dependency repairs compensate for modern package resolution:

1. pin `ESPAsyncWebServer` to its actual `v2.2.1` Git tag;
2. remove the duplicate old `arduinoFFT` dependency from the DigUno base while retaining the Tubes-pinned FFT commit.

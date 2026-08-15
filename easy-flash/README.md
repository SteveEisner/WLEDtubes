# WLEDTubes Easy Flash

Easy Flash is the laptop-local installation and recovery surface for WLEDTubes hardware firmware.

Current branch scope:

- canonical QuinLED Dig2Go Tubes v14 hardware firmware;
- verified complete USB and application-only HTTP OTA artifacts;
- Chrome/Edge Web Serial flashing with chip-family and image-integrity checks, plus a manifest-declared partition and flash-mode contract;
- WLED physical/output configuration planning;
- a deferred Tubes software-profile layer that will consume Steve's canonical runtime packet contracts when they land.

The product boundary is deliberate:

```text
hardware firmware changes rarely
art configuration changes continuously
```

Pattern, Hello, Purple, spatial, Mobile Conductor, and Waveshare S3 experimental firmware remain in their dedicated worktrees and are not bundled here.

No physical write occurs merely by loading the page or preparing an operation receipt. The operator must select a USB port and explicitly approve a write. Safari can download artifacts but cannot use Web Serial; use desktop Chrome or Edge for laptop flashing.

The attached strip cannot be auto-detected. Strip voltage, type, color order, pixel count, wiring, and current ceiling remain human-confirmed inputs.

The ESP32 ROM identifies the MCU family, not the controller board. Easy Flash therefore cannot prove that a connected ESP32 is a QuinLED Dig2Go. Direct USB recovery remains an explicitly confirmed operator action for known Dig2Go hardware; the manifest target metadata must not be treated as connected-device detection.

Browser serial support is provided by the vendored [Espressif esptool-js](https://github.com/espressif/esptool-js) browser bundle. Its upstream license is retained beside the bundle in `vendor/esptool-js/LICENSE`.

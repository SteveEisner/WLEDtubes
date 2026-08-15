# WLEDTubes Easy Flash

Easy Flash is the laptop-local installation and recovery app for WLEDTubes hardware firmware. Ordinary participants use a downloaded macOS or Windows application: no Node.js, Python, terminal, browser flags, or local server is required.

## Participant path

1. **Download** the Easy Flash package for macOS or Windows from the project release.
2. **Open** the application. It loads only the UI, firmware, and serial library bundled inside the app. Opening it never selects a device or starts an install.
3. **Connect** a known QuinLED Dig2Go with a data-capable USB cable, then explicitly choose its serial port.
4. **Confirm** the controller is physically labeled QuinLED Dig2Go and review the strip, color order, pixel count, wiring, and power ceiling. The ESP chip cannot identify the controller board or attached strip.
5. **Backup / Review** by saving the dry-run plan and verified firmware receipt before installation. This release does **not** yet read a complete backup from the device; that remains an authorized-hardware TODO. Preparing or downloading either receipt is side-effect free.
6. **Install** only after checking the Dig2Go confirmation box and approving the final write warning. Easy Flash checks the ESP family and the bundled image size and SHA-256 before writing. It never auto-flashes.

Keep the saved plan with the device. A complete on-device configuration backup, post-flash read-back, and destination boot proof still require authorized hardware validation before this draft is production-ready.

## Unsigned preview warnings

Current desktop packages are unsigned previews.

- **macOS Gatekeeper:** Control-click the app, choose **Open**, then **Open** again. If macOS still blocks it, attempt to open it once and use **System Settings → Privacy & Security → Open Anyway**. Do not disable Gatekeeper. A production release needs an Apple Developer ID certificate, hardened-runtime signing, and Apple notarization.
- **Windows SmartScreen:** Confirm the download came from the official project release and compare its published SHA-256. If SmartScreen shows **Windows protected your PC**, choose **More info → Run anyway** only for that verified download. Do not disable SmartScreen. A production release needs Authenticode signing and publisher reputation.

Unsigned packages display an unknown/unverified publisher and are appropriate for draft testing, not broad participant distribution.

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

No physical write occurs merely by loading the app/page or preparing an operation receipt. The operator must select a USB port, physically confirm Dig2Go hardware, and explicitly approve a write. The canonical browser flow remains available through `npm run easy-flash`; Safari can download artifacts but cannot use Web Serial, so browser-based USB flashing requires desktop Chrome or Edge.

The attached strip cannot be auto-detected. Strip voltage, type, color order, pixel count, wiring, and current ceiling remain human-confirmed inputs.

The ESP32 ROM identifies the MCU family, not the controller board. Easy Flash therefore cannot prove that a connected ESP32 is a QuinLED Dig2Go. Direct USB recovery remains an explicitly confirmed operator action for known Dig2Go hardware; the manifest target metadata must not be treated as connected-device detection.

Browser serial support is provided by the vendored [Espressif esptool-js](https://github.com/espressif/esptool-js) browser bundle. Its upstream license is retained beside the bundle in `vendor/esptool-js/LICENSE`.

## Desktop development and packaging

The desktop shell uses Electron because packaged Chromium retains Web Serial when the application handles Electron's explicit `select-serial-port` event. It serves the same `easy-flash/` UI and verified firmware through a secure, fixed local protocol. Production windows are sandboxed, have no Node.js renderer access or DevTools, deny external navigation/windows, and block HTTP, HTTPS, and `file:` requests.

```sh
npm ci
npm run test:easy-flash
npm run easy-flash:desktop
CSC_IDENTITY_AUTO_DISCOVERY=false npm run dist:easy-flash:mac
```

Windows x64 NSIS and ZIP packages are configured with `npm run dist:easy-flash:win` and built on `windows-2022` by `.github/workflows/easy-flash-desktop.yml`; cross-building the Windows installer is not claimed on macOS. Local/CI unsigned output goes to `dist/easy-flash-desktop/`, which is gitignored. Release artifacts should be copied outside the repository and accompanied by SHA-256 hashes.

The package whitelist contains only the Easy Flash shell/UI, manifest, integrity-pinned firmware, license notices, and package metadata. It does not include generated WLED web headers. See `THIRD_PARTY_NOTICES.md` for the desktop runtime, firmware, and esptool-js dependency audit.

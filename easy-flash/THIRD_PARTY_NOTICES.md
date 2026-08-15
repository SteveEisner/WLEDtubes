# WLEDTubes Easy Flash third-party notices

The packaged application includes the WLEDtubes firmware and Easy Flash source under the EUPL-1.2-or-later. The complete license is shipped as `LICENSE`. The bundled firmware source corresponds to commit `c6522acef3e954b14aad30d6f687cdb99bd1624e` at <https://github.com/SteveEisner/WLEDtubes>.

The desktop runtime is Electron 43.4.0, licensed under the MIT License. Electron packages retain `LICENSE.electron.txt` and Chromium's generated `LICENSES.chromium.html` in the application distribution. Source: <https://github.com/electron/electron>.

The browser serial implementation is Espressif esptool-js 0.6.1, licensed under Apache-2.0. Its complete license is shipped beside the bundle at `vendor/esptool-js/LICENSE`. Source: <https://github.com/espressif/esptool-js/tree/v0.6.1>. The vendored bundle SHA-256 is `ef7d5a237d3f273ecf546bcee65dddad90bd82cf02f22a980d1537e0cd79a152` and was produced from the upstream browser bundle dependency set.

The esptool-js browser bundle includes pako 2.1.0 under its MIT AND Zlib license, and may contain compiled portions of atob-lite 2.0.0 (MIT) and tslib 2.4.1 (0BSD). Sources and license texts are available from their published packages and repositories:

- pako: <https://github.com/nodeca/pako/tree/2.1.0>
- atob-lite: <https://github.com/hughsk/atob-lite/tree/2.0.0>
- tslib: <https://github.com/microsoft/tslib/tree/2.4.1>

electron-builder 26.15.3 is MIT-licensed build tooling and is not shipped as application runtime code. Source: <https://github.com/electron-userland/electron-builder>.

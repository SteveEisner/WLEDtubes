# Waveshare ESP32-S3-Touch-AMOLED-2.16 prototype

This runtime prototype assumes the board has 16 MB QIO flash at 80 MHz and 8 MB
OPI PSRAM (`qio_opi`), with native USB CDC enabled at boot. Peripheral pin assumptions:

- CO5300 AMOLED QSPI: CS 12, SCLK 38, SDIO0..3 4/5/6/7, reset 39
- Shared I2C: SDA 15, SCL 14
- CST92xx touch: IRQ 11, reset 40
- QMI8658 IMU and AXP2101-compatible PMU on the shared I2C bus

The usermod initializes the CO5300 AMOLED and CST9220 touch controller, observes
the power monitor and IMU on the shared I2C bus without configuring them, and
provides local Home, Conductor, Surveyor, and read-only Updater shells. It also
previews the 60-pixel Tubes null framebuffer. It remains authority-neutral,
cannot claim or drive a physical LED bus, and has no peer firmware write path.
Display, touch, and peripheral behavior still require proof on the exact board.

The shell consumes a bounded read-only snapshot of the existing Tubes device-report
semantics. Hardware family, report protocol, Tubes release, WLED version, and
compatibility class are separate fields. Compatibility deliberately displays
`Unknown`: a future policy adapter may classify deployed v14 reports as
Legacy/Current, while a separate v15 adapter can add fields without changing the
v14 wire layout. The UI implementation was AI-assisted using the canonical Tubes
report definitions and the Waveshare board schematic/wiki as its source basis.

This environment changes `platformio_tubes.ini`; inclusion remains explicitly
maintainer-approval-gated. It does not modify `platformio.ini`.

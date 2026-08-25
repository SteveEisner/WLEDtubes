# Waveshare ESP32-S3-Touch-AMOLED-2.16 field runtime

The AMOLED strand reads WLED's canonical `::strip` framebuffer (the previous completed show frame). The S3 `BusTubesNull` remains geometry-only: it provides the 60-pixel topology with no pixel buffer, pins, or transport, so there is one canonical framebuffer.

This board-local smoke usermod assumes the board has 16 MB QIO flash at 80 MHz and 8 MB
OPI PSRAM (`qio_opi`), with native USB CDC enabled at boot. Peripheral pin assumptions:

- CO5300 480 x 480 AMOLED QSPI: CS 12, SCLK 38, SDIO0..3 4/5/6/7, reset 39
- Shared I2C: SDA 15, SCL 14
- CST9217 480 x 480 touch: IRQ 11, reset 40
- QMI8658 IMU and AXP2101-compatible PMU on the shared I2C bus

At boot it initializes those four peripherals, presents live status on the AMOLED, and mirrors
the basic health and power readings in WLED's JSON info. Touch input is diagnostic-only: the
screen retains the latest coordinates without changing brightness or other device state. The
display includes the WLED release and version strings plus compile time so a photographed smoke
result identifies its firmware.

This standalone peripheral canary does not initialize a physical LED bus or add an ESP-NOW send
path. It is separate from the production `waveshare_s3_tubes_remote` environment, which
includes the Tubes usermod, the 60-pixel null output, and normal mesh participation. The canary's
controller and geometry contract is reconstructed from the preserved factory application;
the remaining pin contracts still require confirmation against the physical board and vendor
schematic. The compiled firmware must not be flashed before factory preservation, an exact
write/restore preview, and Greg's explicit approval.

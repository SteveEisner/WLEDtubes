# Waveshare S3 Tubes Remote

The AMOLED strand reads WLED's canonical `::strip` framebuffer (the previous completed show frame). The S3 `BusTubesNull` remains geometry-only: it provides the 60-pixel topology with no pixel buffer, pins, or transport, so there is one canonical framebuffer.

This board-specific Tubes usermod supplies the field interface for the Waveshare
ESP32-S3-Touch-AMOLED-2.16. It assumes 16 MB QIO flash at 80 MHz, 8 MB OPI PSRAM
(`qio_opi`), and native USB CDC at boot. Peripheral assignments are:

- CO5300 480 x 480 AMOLED QSPI: CS 12, SCLK 38, SDIO0..3 4/5/6/7, reset 39
- Shared I2C: SDA 15, SCL 14
- CST9217 480 x 480 touch: IRQ 11, reset 40
- QMI8658 IMU and AXP2101-compatible PMU on the shared I2C bus

The Tubes field OS presents four workspaces: Conductor, Surveyor, Update, and
Channels. Conductor reads WLED's canonical completed framebuffer. Surveyor shows
fresh nearby Tubes nodes. The carrier build embeds the standard Dig2Go and Athom C3
firmware and exposes the bounded one-device update baton. Channels is reserved for
interactions with the release-40 Beat, Pattern, and Palette channel types.

The `waveshare_s3_tubes_remote` environment builds the base field OS. The explicit
`waveshare_s3_tubes_carrier` environment adds the two validated carrier payloads.
Both use the 60-pixel geometry-only null output and participate normally in the
Tubes mesh; neither owns or drives a physical LED output pin.

## Tubes integration boundaries

This usermod is a board adapter over the shared Tubes implementation. It uses the
existing release-40 `ChannelWinnerTable` admission rules, `FleetUpdateOffer` wire
format, device-report probe/reply messages, fleet firmware identity, pull URL, and
`x-MD5` verification contract. It does not define a second channel protocol or a
second receiver-side updater.

The S3-specific code is limited to capabilities the shared implementation does
not provide: the AMOLED/touch interface, a read-only nearby-device view, embedded
Dig2Go/C3 artifact selection, a one-client baton policy, and a temporary access
point plus HTTP response that lets the existing fleet updater pull those bytes.

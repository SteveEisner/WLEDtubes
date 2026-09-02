# Waveshare S3 Tubes Remote

The AMOLED strand reads WLED's canonical `::strip` framebuffer (the previous completed show frame). The S3 `BusTubesNull` remains geometry-only: it provides the 128-pixel topology with no pixel buffer, pins, or transport, so there is one canonical framebuffer.

This board-specific Tubes usermod supplies the field interface for the Waveshare
ESP32-S3-Touch-AMOLED-2.16. It assumes 16 MB QIO flash at 80 MHz, 8 MB OPI PSRAM
(`qio_opi`), and native USB CDC at boot. Peripheral assignments are:

- CO5300 480 x 480 AMOLED QSPI: CS 12, SCLK 38, SDIO0..3 4/5/6/7, reset 39
- Shared I2C: SDA 15, SCL 14
- CST9217 480 x 480 touch: IRQ 11, reset 40
- QMI8658 IMU and AXP2101-compatible PMU on the shared I2C bus
- ES7210 microphone ADC: MCLK 42, BCLK 9, LRCK 45, SDOUT 10

TubeOS keeps a fixed, touchable home header and charging-aware battery meter above
a 128-pixel live pattern preview. The current beat is drawn as a quarter-height
white marker across one sixteenth of that preview. The preview and beat lane use a
single opaque framebuffer transfer because narrow CO5300 writes can leave visible
black seams. The four home workspaces are Patterns, Beats, Colors, and Mesh. Patterns
schedules one of twelve touch-sized choices at the next phrase boundary. Beats
controls microphone tempo listening, shows live BPM plus a 12-band microphone
spectrum, resets the shared downbeat, and queues one of eight curated audio overlays
or turns the overlay off. Mesh shows up to four fresh
nearby Tubes nodes and links to Update. Colors edits and schedules a three-stop
gradient as soon as its continuous hue sweep, brightness row, or a preset changes.
Its eight two-row presets are regenerated around the selected color and preserve
that color at the selected stop. The carrier build embeds the
standard Dig2Go and Athom C3 firmware and exposes the bounded one-device update baton.

The field target bypasses the physical MASTER half-strip compaction so both halves
of the AMOLED preview show the full logical pattern at the same brightness.

The `waveshare_s3_tubes_remote` environment builds the base field OS. The explicit
`waveshare_s3_tubes_carrier` environment adds the two validated carrier payloads.
Both use the 128-pixel geometry-only null output and participate normally in the
Tubes mesh; neither owns or drives a physical LED output pin. The field target is
always the `1FFF` master and enables microphone tempo tracking at boot; the Beats
workspace can turn listening off until it is enabled again. On battery,
the AMOLED drops to a black battery-meter-only screen after 30 seconds. A short
tap on the AXP2101 power key toggles between that battery-only screen and the
full TubeOS UI, and a 10-second power-key hold powers the PMU off until the key
is tapped or held again. The other physical buttons are disabled.

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

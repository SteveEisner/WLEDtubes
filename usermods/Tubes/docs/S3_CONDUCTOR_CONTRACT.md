# S3 Remote Prototype Compatibility Contract

The S3 prototype is a **full logical Tubes node** with **no physical LED
output**. This first milestone renders normal Tubes state to a virtual strip
and previews that frame on the AMOLED. It does not grant conductor authority or
change the deployed wire protocol.

## First implementation

`waveshare_s3_tubes_remote` defines `TUBES_NULL_OUTPUT` and renders 60 logical
pixels through the normal WLED strip, segment, effect, and Tubes overlay paths.
The final bus is `BusVirtualFramebuffer`: a generic addressable packed-RGB
framebuffer whose `show()` is intentionally empty. The Tubes S3 adapter creates
that output idempotently, and target-scoped `BusManager::add()` rejects physical,
network, or additional framebuffer configuration before bus mutation or pin
allocation. It never clears unrelated buses from generic add. AMOLED preview
code can read the rendered frame through the existing `BusManager::getBus()` and
`Bus::getPixelColor()` interfaces.

IDs 24 and above still execute WLED's real effect engine and are cross-faded by
the existing Tubes controller; they are not substituted with fake patterns.
This is compile-tested on the pinned WLED 16 / Arduino-ESP32 2 stack. Until an
S3 device is exercised, framebuffer cadence, effect pixels, display, and touch
remain hardware limitations; the no-transport guarantee is structural and does
not depend on runtime configuration.

## Wire contract

- A deployed v2 `NodeMessage` is immutable: exactly 84 bytes, little-endian, including existing alignment and reserved bytes. Do not repack, extend, or reinterpret it.
- `MasterRole = 200` remains the conductor role boundary and must not be repurposed.
- Pattern IDs `0..23` are native Tubes renderers. Pattern IDs `24+` are WLED-backed entries whose existing IDs and visual intent remain stable.

## Behavioral contract

The prototype runs the existing Tubes scheduler against a virtual strip: shared
BPM/beat and phrase clocks, current and next scheduled state, pattern and
palette selection, fades, effect/transient overlays, node election/following,
and command routing retain their legacy semantics. It must not claim conductor
authority or drive LED buses.

Current/next state is predictive continuity, not a queue to simplify or collapse. Fades, palettes, and transients retain their existing ordering and timing. Node and conductor behavior remains the existing mesh model: a conductor is a high-role node, not a separate protocol authority.

## Stack and fleet limits

The first implementation must use the **same WLED 16.0.1** and **Arduino-ESP32 2.0.18** stack as the fleet. Do not change framework versions as part of conductor work.

A mixed-firmware flock shares only behavior already represented by the immutable v2 frame and understood sidecars. New-only features require capability gating and graceful legacy fallback; old nodes cannot gain new renderers, scheduling semantics, or sidecar behavior merely by hearing a new conductor. Unknown commands or sidecars must remain safely ignorable.

## Hardware write gates

Every physical-output path needs an explicit **hardware write gate**. Headless mode must gate bus initialization, pin ownership, pixel transmission/show calls, current limiting based on physical LEDs, and output-specific teardown. Networking, scheduler state, virtual-strip computation, fades, palettes, and transients stay active behind no such gate. Tests must prove that enabling conductor semantics cannot reach a hardware write.

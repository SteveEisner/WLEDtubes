# Dig2Go A-to-B decision receipt — 2026-08-25

## Bench identity and preservation

- A / PRIME: `/dev/cu.usbserial-2110`, ROM MAC `54:43:B2:B5:49:80`.
- B / receiver: `/dev/cu.usbserial-2120`, ROM MAC `54:43:B2:B5:4C:38`.
- Both are ESP32-D0WD-V3 revision 3.1 with 4 MiB flash.
- B was preserved twice before writes. Both full reads have SHA-256
  `7f5777a68edb2d5971ce22dd518e57868fa964802c5a7666fa90b5563a647e96`.

## Legacy decision

The first-stage failure in `c33b0ed0` was an A-side radio-ownership bug: starting
the migration AP deinitialized ESP-NOW before the wake send. Commit `00eec892`
adds a bounded AP-interface ESP-NOW carrier on the mesh channel. The candidate
built and the full Tubes mesh suite passed.

One physical legacy run was allowed after that repair. B's `app1` SHA-256 was
`8b3080123060ad2411640a5318dd2e4d0b59b467e3ad1eac0c31b9f753cb5585`, not
the served A artifact
`03fbf322d2df7624616e6675fa3b0e87ff901545f88904fa51734290930394f4`.
No legacy pull, reboot, or health report was proven.

The product escape hatch is therefore invoked. Legacy v13/v14 P2P migration is
parked. Easy Flash is the intended one-time USB path into current firmware;
after that, devices enter the modern P2P system. No Easy Flash repository was
touched.

## Modern pivot result

The carrier was extended without defining another protocol: A emits Steve's
existing `FleetUpdateOffer`, B uses the existing fleet updater, and A serves the
existing `/tubes/firmware.bin` contract with exact length and `x-MD5`. Both
devices were exact-MAC gated and app-only flashed to the same current candidate.

The physical modern run did not complete. B remained selected on `app0`; its
`app1` SHA-256 was
`6c6970639d02c4060ee31368888460a9990a6bcdeaa07cce37dbea8906259deb`, not
the served artifact
`9c05bb9a6c3044bc4e726baeb0fd04dd3184256c4fce288d668aaad910b54ed7`.
No reboot health or baton-ready state was proven. The next owning seam is modern
offer acceptance/receiver transition, not the HTTP body or static verifier.

## Evidence boundary

Proven: exact device mapping, matching B preservation, app-only identity-gated
writes, A readback for the AP-carrier build, compile/test success, and two exact
negative B OTA inspections.

Not proven: legacy wake reception, modern offer acceptance, wireless image
commit, fresh post-update health, or baton propagation. A final green/latched
physical result was not reached.

## Final bounded legacy exception

Greg authorized one last cable-telemetry attempt before permanently parking
legacy P2P. The modern diagnostic checkpoint was preserved. B's exact preserved
legacy `app0` was extracted from the matching full backup and restored with an
identity-gated application-only write; its SHA-256 is
`16cf230edca34077ac196a1b4fbae0d94000967148e88b8f8846181992c34db9`.

A reconnecting, DTR/RTS-inactive dual-port logger could not reach the 15-second
offer window. Both devices repeatedly disappeared, re-enumerated, and emitted
fresh `POWERON_RESET` / `SPI_FAST_FLASH_BOOT` lines. B additionally emitted an
`RTCWDT_RTC_RESET`. This is the known cable/power-relay loop, and it prevents
clean proof that A transmitted the legacy offer, B accepted it, or B entered
its updater transition. The required three cable facts were therefore not
established.

Per the authorized hard stop, no externally-powered human test is requested.
Legacy P2P is permanently parked. The product path is one Easy Flash USB
migration for v13/v14, followed by Steve's modern FleetUpdateOffer and baton
system. The Easy Flash repository was not touched.

The preserved modern instrumentation reports A offer validity/send/role/node
state and B offer validity/targeting plus every `startFleet` rejection predicate.
Read-only review identified the next missing breadcrumb at Control-tree ingress
and route rejection: local ESP-NOW enqueue does not prove that B's current
uplink topology admitted the declaration/request.

## USB/power-relay doom-loop fence

The loop received one separate, timeboxed source/telemetry audit. No additional
firmware startup defect was found. The current Dig2Go path resolves retained
`def.on`, `def.bri`, relay polarity, and relay presence once in
`WLED::beginStrip()` through `dig2goRelayStartup()`. It avoids the former forced
off-to-on pulse; subsequent relay writes are normal WLED on/off transitions.

Both devices repeatedly reported `POWERON_RESET` while disappearing and
re-enumerating even with DTR and RTS held inactive. B also reported one
`RTCWDT_RTC_RESET`. There is no matching application restart or Tubes startup
action in source. The remaining ownership seam is electrical: USB bridge modem
control, EN, IO0, USB 5 V, and the board relay/power path.

No firmware change or device-specific delay is justified without schematic or
electrical measurements. The existing relay startup policy remains intact.
P2P testing must ignore this bench artifact by using normal external power and
genuinely passive TX/GND telemetry, or by attaching after the run. The doom loop
is neither a prerequisite for modern P2P nor chargeable against the final
legacy attempt.

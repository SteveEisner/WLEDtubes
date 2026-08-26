# Dig2Go peer update scaffolding

The peer updater is an optional Tubes usermod feature layered over WLED's
existing Wi-Fi/AP and OTA primitives. It is disabled by default. Ordinary WLED
and ordinary Tubes builds do not start a peer host, change saved Wi-Fi data, or
alter their update lifecycle.

## Product boundaries

- P2P reuses the `FleetUpdateOffer` wire and receiver validation, not the laptop
  fleet workflow. Once explicitly seeded, propagation is autonomous.
- Ordinary fleet OTA and P2P fanout are separate modes. A normal offer updates
  receivers but never arms peer hosting.
- `FleetUpdatePropagate` explicitly opts an offer into P2P. A successfully
  updated child stores one durable hosting lease before reboot.
- An exact-target, equal-version propagation command with no download server
  starts one bounded host turn on an already-current root. A wildcard
  equal-version command is invalid, preventing current peers from waking each
  other into a loop.
- Field propagation requires explicit human input. `Q` opens a bounded source
  window and a double-click chooses the one source; ordinary OTA, boot, and
  proximity never start a turn.
- The temporary host reuses the deployed `TubesOTA` / `tubes123` contract in
  RAM. It never serializes those temporary values into WLED configuration.
- Legacy v13/v14 migration remains a Dig2Go-only compatibility adapter. One
  explicitly started P2P turn emits both the deployed legacy wake and the
  modern offer, so old and current Dig2Gos can share the same bounded run.
  Easy Flash remains the supervised USB fallback when wireless migration is
  unsuitable or hardware identity is uncertain.

## Host lifecycle

One host turn inspects and serves the exact running application image at
`4.3.2.1`. It owns the temporary SoftAP and AP-interface ESP-NOW carrier only
for the bounded turn, then restores the prior WLED globals and ordinary Tubes
STA radio.

The host admits at most two receivers per turn and serves them sequentially.
Modern `HTTPUpdate` could tolerate concurrent pulls, but a mixed field run may
contain a deployed legacy client that treats a momentary empty TCP read as
end-of-file. Serialization therefore provides one behavior for old, current,
and mixed Dig2Go populations.

An active transfer is governed by a 20-second no-progress timeout, not the
host's absolute rendezvous timeout. After one completed receiver, the host
leaves a 60-second second-receiver admission window. After two complete bodies
it restores promptly. Partial startup failures restore temporary AP globals,
and ESP-NOW carrier takeover marks the previous owner stopped before
reinitialization.

## Explicit prototype fences

`TUBES_DIG2GO_LEGACY_BOOT_FALLBACK_TEST` exists only for the physically proven
legacy bench image. It allows a newly migrated non-PRIME device to take one
boot-time legacy turn because deployed old firmware cannot persist a modern
lease. The flag requires both the legacy host and dynamic enrollment and must
not be enabled in a production build.

Golden PRIME auto-start and the legacy boot fallback are test activation
mechanisms, not the production API. Production modern fanout is activated only
by an explicit propagation command or a durable lease created by a successful
propagation-marked OTA.

## Verification boundary

Physically proven on August 25, 2026:

- A migrated known B wirelessly and B rebooted onto the exact served image.
- A migrated previously unknown C without a compiled receiver MAC.
- A migrated unknown D and C sequentially in one fanout-two host turn; both
  rebooted onto current firmware and A restored normal operation.

Host tests model A-to-B, A-to-C, A-to-C-plus-D serialization, second-slot
handoff, active-transfer timeout immunity, equal-version rejection, and one
bounded child follow-on turn. Modern command construction, opt-in lease
arming, ordinary-OTA non-propagation, lease replay prevention, credentials,
and equal/newer rejection are also host-tested.

Still unproven physically: an explicit modern command causing a v47 device to
pull v48, reboot, claim its lease, and serve a modern child. Deeper propagation
trees are outside the current validation scope.

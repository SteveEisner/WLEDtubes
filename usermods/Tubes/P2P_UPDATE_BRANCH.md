# P2P Updater Foundations

**Base:** Steve's canonical `main`

**Purpose:** Define same-hardware peer-assisted Tubes firmware update foundations while Steve's split-packet protocol evolves on `main`.

This contribution is intentionally narrow. It owns compatibility gating, image-source seams, transfer-session state, verification contracts, and migration fixtures. It does not invent a competing art/configuration protocol.

These are host-tested foundations, not a live updater. The HTTP source is not registered on WLED's server, no receiver OTA write path is connected, and the current canonical device report does not carry the complete flash and partition identity required for exact-target admission.

## Project pillars

### 1. One firmware artifact per hardware target

Firmware artifacts represent compiled hardware requirements only:

- MCU family and board pinout;
- flash size, flash mode, and partition layout;
- output, display, touch, radio, PSRAM, and peripheral drivers;
- framework/toolchain constraints;
- physical capabilities software cannot change.

Standard, Christmas, Golden, Ruby, Mauve, role, palette policy, event identity, and spatial behavior belong in software configuration.

**Rule:** If two devices have the same hardware target and compiled capabilities, changing their behavior must not require reflashing.

### 2. Art travels as independent software state

Palette, tempo, pattern, pattern variables, effects, role, installation policy, and spatial inputs should be independently changeable and synchronized.

Easy Flash may apply an initial profile, but ongoing art behavior belongs to Steve's canonical runtime configuration and sync packets.

### 3. Small, typed, versioned packet contracts

Preserve the deployed 84-byte state frame during migration. Follow Steve's canonical packet definitions as they land on `main`; do not fork their IDs, layouts, versioning, or fallback semantics here.

Unknown packet kinds or versions must be safely ignored while retaining the last valid state. The current versioned device-report sidecar and old-node relay behavior are the reference compatibility pattern.

### 4. Capabilities fail gracefully

Devices advertise the protocol and rendering capabilities they support. A receiver handles requested state as:

- fully supported: render it;
- partially supported: use a defined fallback when available;
- unsupported: ignore that portion safely and retain coherent prior/fallback state.

An incapable device must not crash, display garbage, corrupt timing, or destabilize the mesh.

### 5. Firmware updates propagate only among compatible targets

A manually seeded device may help update peers only when the exact hardware/update profile matches. Compatibility includes MCU, board, flash/partition profile, release identity, application length, and artifact hash.

Begin with one controlled update baton:

```text
trusted seed
→ select and inspect one compatible target
→ transfer the application image into its inactive OTA slot
→ verify complete image
→ reboot and prove healthy
→ continue or hand off the baton
```

Newly updated devices do not independently broadcast update commands. Cross-target writes fail before erase or write.

The branch currently expresses those checks with `FirmwareTargetContract`, an
internal, non-wire structure covering hardware family, chip family, flash mode,
flash size, partition-table SHA-256, and OTA-slot geometry. The canonical
projection contains static hardware geometry only and is deliberately
receiver-incomplete. A receiver target becomes admissible only when runtime
partition inspection supplies and validates an explicit inactive-slot index;
hardware identity alone is never inactive-slot evidence. Unknown or partial
contracts fail closed. This structure must not be serialized or assigned a mesh
action key until Steve's canonical metadata transport defines that seam.

### 6. Installation and recovery remain separate

Installation tooling chooses a compatible artifact, installs or recovers it, preserves effective configuration, applies an initial runtime profile when Steve's schema supports it, and reports capability/update status.

Both rails consume canonical WLEDTubes hardware IDs, release metadata, generated artifacts, device reports, and WLED JSON configuration rather than defining parallel identities.

## P2P scope

### Current transport boundary

Current deployed behavior is hybrid:

```text
ESP-NOW / Tubes mesh
  update-version orchestration and post-update reporting

Target Wi-Fi + HTTP OTA
  application-image transfer
```

This branch may make a compatible updated device the HTTP image source, but should not push firmware bytes through the ordinary Tubes synchronization frame.

A sender may read and serve its running application partition. A receiver writes only through the platform OTA path into its inactive application slot. Ordinary peer OTA never writes a bootloader, partition table, merged recovery image, or NVS image.

`FirmwareImageSource` is the bounded, read-only source seam. An image artifact
carries its own `FirmwareTargetContract`; the device storing or serving that
artifact is only the carrier and its hardware identity is not used for receiver
admission. Memory/file implementations provide the host-testable carrier seam.
The ESP32 running-image adapter is explicitly limited to the sole registered
running artifact, Dig2Go v14. It verifies the running application with
Espressif's image parser, uses its exact image length rather than partition
capacity, computes SHA-256 across exactly those bytes, and permits only bounded
partition reads. No generic running-image API silently maps other targets to
that artifact. The host-tested HTTP response core provides exact GET, HEAD,
and single bounded byte-range semantics over any verified image source;
malformed, multipart, overflowing, and past-end ranges fail closed. It is not
registered on the live WLED server yet: these seams extend the existing Tubes
update architecture without owning operational session or HTTP endpoint
lifetime. That integration remains intentionally absent pending protocol
authorization; endpoint availability must eventually be bounded by the
selected update session/lease rather than exposing an always-on firmware
download. No receiver write path is connected yet.

### Bootstrap boundary

Existing v12/v13 nodes understand the legacy version action and `WLED-UPDATE` HTTP OTA flow. They do not understand a new lease, hardware descriptor, or chunk protocol merely because v14 does.

Use only proven legacy behavior to wake/bootstrap old devices. Keep richer selection, leases, deduplication, compatibility checks, receipts, and baton handoff in current firmware/tooling.

### Migration corpus and configuration gate

`migration-fixtures/manifest.json` pins stock WLED 14/15/16, reconstructed
Tubes v13, and canonical Tubes v14 inputs for host-side migration tests. Old
stock images and the reconstructed v13 image are source fixtures only; they
must never become automatic installation candidates.

Migration order is mandatory:

```text
inspect and classify exact hardware + installed lineage
→ back up configuration and persistent Tubes state
→ if required, normalize only the explicit hardware output bus
→ install the exact hardware firmware when required
→ reboot and verify destination firmware identity + health
→ apply only configuration supported by that verified firmware
→ read back effective configuration
```

The optional preflash transform is limited to making the LED bus explicit when
newer base firmware could otherwise boot with unsafe fallback geometry. No peer
or installer may send a new runtime configuration schema merely because the old
device reports WLED 16. Full configuration eligibility begins only after the
destination Tubes firmware has booted and passed the health gate.

### Initial concurrency model

Start sequentially:

- one update session;
- one allowed hardware target;
- one application hash;
- one active sender;
- one selected target;
- one bounded lease;
- explicit completion or failure before selecting another target.

Do not add parallel propagation until a real fleet trial shows sequential behavior is inadequate.

`FirmwareUpdateSession` now encodes this sequential control boundary without
defining a wire packet: one known sender MAC, one known target MAC, one exact
artifact/receiver target match, one bounded lease, monotonic byte progress, and
exact transfer-hash verification. Completion remains blocked until a typed
health proof confirms the expected hardware target, release/hash, preserved
runtime/output configuration, mesh rejoin, and stability. Forwarding is hard
disabled even after explicit completion; the completed session only reports
that the baton is ready for a higher-level coordinator.

### Health gate

A transferred image is not a successful update until the target:

- boots the expected release and hardware identity;
- reports the expected application/hash metadata where supported;
- preserves required runtime/output configuration;
- rejoins the Tubes mesh;
- remains healthy long enough to avoid immediate rollback/reset-loop behavior.

Only then may it receive the update baton.

## Keeping pace with Steve's `main`

Before each implementation slice:

1. fetch `origin/main`;
2. inspect new Tubes packet, hardware identity, release metadata, and updater changes;
3. rebase this branch onto `origin/main` when clean;
4. resolve toward Steve's canonical contracts—not a local duplicate;
5. rerun the focused mesh and upgrade checks;
6. update this document only when a pillar or proven transport fact changes.

When Steve's split palette/tempo/pattern/spatial/capability packets land:

- consume their IDs and structs directly;
- keep P2P update control logically separate from art-state packets;
- replace temporary assumptions rather than preserving compatibility with branch-only protocol experiments;
- keep behavior differences in runtime configuration, not new firmware artifacts.

## Explicit non-goals during protocol migration

- no new behavior-specific firmware variants;
- no pattern, Hello, or spatial expansion on this branch;
- no universal binary across incompatible MCU/board targets;
- no firmware bytes inside the deployed Tubes state packet;
- no autonomous update broadcast by every newly updated device;
- no physical writes without Greg's explicit authorization.

## First implementation sequence

1. Freeze a hardware-target/update-manifest contract around existing v14 metadata.
2. Add fail-before-write tests for exact target, partition capacity, image length, and hash.
3. Add the HTTP source around the verified running-image reader and prove it with synthetic clients.
4. Add one sender/one receiver update-session state machine with forwarding disabled.
5. Prove interruption leaves the active receiver image bootable.
6. Prove reboot, health reporting, and explicit baton handoff.
7. Trial on one authorized Dig2Go sender and one expendable matching receiver before any fleet propagation.

## Centralized update contract progress

PR #67 now owns the hand-edited machine-readable source at
`contracts/update/update-contract.json`. It is internal/build data, not a mesh
packet. Deterministic tooling under `tools/update-contract/` validates the
source and produces:

- `contracts/update/generated/update-contract.generated.mjs` for Easy Flash;
- `usermods/Tubes/generated/update_contract_generated.h` for firmware and the
  S3 adapter.

The first bounded contract pins the proven Dig2Go target, USB merged image, OTA
application image, release identity, and byte hashes. It also records proven
Waveshare S3 target and partition geometry but deliberately provides no S3
artifact or hardware-acceptance claim. Missing target, artifact, or release
class identity fails closed; silence is `Unknown`, never `Legacy`.

Artifact source geometry is transport-specific. The USB merged image retains
`writeOffset: 0` and absolute component offsets. The OTA application artifact
records only its distinct build-time component offset (`buildOffset: 0x10000`):
its canonical bytes are position-independent input to the platform Update API,
not bytes bound to OTA slot 0. Session admission separately consumes an
explicit `inactiveOtaSlot` verified from receiver runtime partition evidence
and requires the supplied destination
index, offset, and size to match that canonical slot exactly, with the image
length no larger than the destination. The same OTA artifact ID, full SHA, and
source bytes are therefore admissible to either canonical inactive slot; no
per-slot artifact copies are created.

The generated C++ projection is compile-time data. Firmware does not parse JSON
at runtime. `firmwareStaticTargetFromCanonical()` projects compact target and
vocabulary constants but cannot produce an admissible receiver target;
`firmwareReceiverTargetFromStatic()` requires the separately verified runtime
inactive-slot index. `FirmwareUpdateSession` retains its existing sequential sender,
target, artifact, lease, transfer, health, and disabled-forwarding semantics.

### Centralized-contract TODO

- [ ] Adapt PR #65 Easy Flash to consume the generated JavaScript projection,
  then retire its independent firmware manifest only after migration tests pass.
- [ ] Adapt PR #66 Waveshare S3 target/UI code to consume generated constants;
  keep Updater read-only and omit artifacts until a release image is proven.
- [ ] Replace the temporary device-report hardware-family seam only when Steve's
  additive v15 identity/capability contract lands on `main`.
- [ ] Add receiver writes, lease-scoped serving, and device health evidence only
  in later explicitly authorized slices.

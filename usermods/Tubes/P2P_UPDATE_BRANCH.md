# P2P Update Collaboration Branch

**Branch:** `greg/p2p-update`

**Base:** Steve's canonical `main`

**Purpose:** Develop same-hardware peer-assisted Tubes firmware updates while Steve's split-packet protocol evolves on `main`.

This branch is intentionally narrow. It owns update discovery, compatibility gating, transfer, verification, and bounded propagation. It does not invent a competing art/configuration protocol.

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

### 6. Easy Flash is installation and recovery

Easy Flash detects hardware, chooses the compatible artifact, installs/recovers it, preserves effective configuration, applies an initial runtime profile when Steve's schema supports it, and reports capability/update status.

Easy Flash and this branch consume canonical WLEDTubes hardware IDs, release metadata, generated artifacts, device reports, and WLED JSON configuration rather than defining parallel identities.

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

### Bootstrap boundary

Existing v12/v13 nodes understand the legacy version action and `WLED-UPDATE` HTTP OTA flow. They do not understand a new lease, hardware descriptor, or chunk protocol merely because v14 does.

Use only proven legacy behavior to wake/bootstrap old devices. Keep richer selection, leases, deduplication, compatibility checks, receipts, and baton handoff in current firmware/tooling.

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
3. Trace the ESP32 running-partition read and target HTTP OTA lifecycle with synthetic bytes first.
4. Add one sender/one receiver update-session state machine with forwarding disabled.
5. Prove interruption leaves the active receiver image bootable.
6. Prove reboot, health reporting, and explicit baton handoff.
7. Trial on one authorized Dig2Go sender and one expendable matching receiver before any fleet propagation.

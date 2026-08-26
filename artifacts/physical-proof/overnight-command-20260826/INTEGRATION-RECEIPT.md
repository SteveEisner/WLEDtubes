# Dig2Go P2P overnight integration receipt

Date: 2026-08-26 (America/Los_Angeles)

## Source authority

- Worktree: `/Users/theysayheygreg/Projects/WLEDTubes-p2p-dig2go-push-bridge`
- Branch: `feature/dig2go-p2p-steve-review`
- Reconciled base/current committed HEAD before this pass: `6084307ad13829aa8655c54d46d23d5161762ceb`
- Earlier feature commit: `65439293` (`Add explicit Dig2Go peer update propagation`)
- Overnight recovery and modern-baton implementation: `b1248fd4` (`Complete Dig2Go peer propagation recovery`)
- No PR was opened. Steve's main, laptop OTA, S3 firmware, Easy Flash, WLED Wi-Fi credentials, and the immutable legacy wire were not modified.

## Bench identity map

| Device | Port | ESP32 ROM MAC |
| --- | --- | --- |
| A / golden prime | `/dev/cu.usbserial-310` | `54:43:B2:B5:49:80` |
| B | `/dev/cu.usbserial-110` | `54:43:B2:B5:4C:38` |
| C | `/dev/cu.usbserial-2110` | `54:43:B2:B6:3A:48` |
| D | `/dev/cu.usbserial-2120` | `54:43:B2:B5:49:20` |
| E | `/dev/cu.usbserial-10` | `A0:B7:65:CA:60:80` |

Every direct write in this pass was application-only after an exact live ROM-MAC gate. Preserved partition/slot evidence is under `preservation-e/` and `pre-modern-c/`. C's active app and OTA metadata were each read twice with matching hashes before its v47 staging write.

## Protocol and API surface

This extends the existing Tubes/WLED mechanisms rather than introducing a second OTA system:

- The existing structured `P` command starts a user-authorized propagation turn. A `serverPort == 0` offer means “serve the current application”; ordinary laptop-directed offers remain separate.
- The existing `FleetUpdateOffer` remains the modern offer contract, including release, start window, target, credentials, flags, and nonce.
- `LightNode::sendV3NeighborChannel()` carries that already-validated Control payload directly to nearby peers as well as the established Control/root rail. This makes P2P independent of laptop/root ownership without changing laptop OTA behavior.
- A newer modern receiver uses the existing fleet updater, validates family/variant/release and HTTP identity, persists a propagation lease, reboots, claims it, then serves the same image.
- Deployed legacy receivers still use the immutable wake and `/firmware.bin` pull. Since old firmware cannot persist a modern lease, a freshly rebooted Dig2Go may accept an equal-release propagation offer only inside a 60-second boot window, waits for the predecessor to go quiet, then serves. Already-running current devices ignore that baton.
- A predecessor declares transfer completion from the exact served byte count and recovers without requiring a fragile reboot ACK. It repeats the offer for a bounded 15-second radio grace, then clears its turn.
- Host restore now cancels the separate legacy wake rendezvous so stale AP credentials are not advertised after the server is gone.
- Tubes startup makes a recovered zero-length placeholder segment static for the one loop before WLED rebuilds its LED bus. This prevents a legacy Flow configuration from dividing by zero without changing WLED effect semantics.

## Built application artifacts

| Purpose | File | Bytes | SHA-256 |
| --- | --- | ---: | --- |
| modern receiver fixture | `p2p-release47-modern-receiver.bin` | 1,357,744 | `9a8b2d2100d0e6da757c8911ae6c3aa70b46e2e608e319d69c8eb40a0290692b` |
| final v48 candidate | `p2p-release48-startup-and-rendezvous-fix.bin` | 1,357,744 | `bd31ca02146fcd5fe6a9d98befb84f4a03f60eaaf3ed16d785b94d08de779392` |
| deployed legacy v13 receiver | preserved application | — | `16cf230edca34077ac196a1b4fbae0d94000967148e88b8f8846181992c34db9` |

Both `dig2go_p2p_release47_test` and `dig2go_p2p_release48_test` build successfully. `bash test/tubes_mesh/run.sh` passes, including the propagation lease, neighbor transport, two-completion fanout, rendezvous cancellation, and startup recovery contracts. `git diff --check` passes.

## Physical evidence

### Legacy migration and baton

The bench proved A v48 migrated B from the exact v13 application to v48. B then accepted A's native neighbor `FleetUpdateOffer`. B was explicitly command-seeded and served E, which logged v13, joined `TubesOTA`, downloaded all 1,357,712 bytes of that candidate, logged successful OTA and reboot, reported v48, accepted the equal-release fresh-boot baton, started its own host, and transmitted a fresh offer. Evidence is in `telemetry-native-neighbor-admit-ab/` and `telemetry-b-to-e-legacy-baton/`.

The earlier human-observed three-device run also proved one seed serving two legacy receivers sequentially: D completed and rebooted, then C completed and rebooted; each displayed its own propagation state while A recovered. That visual proof remains human evidence rather than cable-derived identity proof for the C/D labels.

### Modern v47 to v48 propagation

E, ROM MAC `A0:B7:65:CA:60:80`, ran the final v48 candidate and received command nonce `E5000002`. It created offer `D82AE791`, brought up `TubesOTA`, and transmitted the valid existing offer on both rails.

C, ROM MAC `54:43:B2:B6:3A:48`, reported v47 before the run. Its log then records:

- line 175: valid release-48 offer received;
- line 176: existing fleet updater scheduled;
- line 199: HTTP pull from `/tubes/firmware.bin` with nonce, family, variant, and exact MAC;
- lines 297-298: durable propagation lease armed and OTA completed;
- line 334: after reboot, lease claimed with a fresh offer nonce;
- lines 393 and 396: C's host ready and valid `FleetUpdateOffer` transmitted on both Control and neighbor rails.

E independently records an admitted station, the exact 1,357,744-byte request, host completion, recovery without reboot ACK, baton grace, and lease/turn clear. After restore, no further `TUBE_PULL_WAKE attempts=` lines occur; E only logs C's incoming wake/offer and rejects it because E is already current. Evidence: `telemetry-modern-e-to-c/C.log` and `telemetry-e-startup-fix/E.log`.

This closes the previously missing physical boundary: a modern v47 Dig2Go accepts Steve's `FleetUpdateOffer`, uses the existing fleet HTTP updater to install v48, reboots, and propagates the baton.

## Remaining caveats

- C3 is deliberately outside this Dig2Go proof. Family/variant checks prevent image installation, but an old incompatible peer may still briefly consume an AP admission slot before HTTP rejection. Separate hardware-family seed runs remain the product rule.
- The USB power/reset issue is deferred until after Friday. Evidence points to LED-load/current-protection cycling on the 1.5 A-per-port gregbot hub versus a possible 3 A strand load. Bench recommendation remains USB telemetry with LED loads disconnected, or externally power the strands/controllers with a shared safe ground.
- 921600-baud USB writes were unreliable across these adapters; 460800 was repeatably stable. One post-write checksum/header retry was observed before a normal boot. These are recorded bench quirks, not wireless protocol failures.
- The test dependency install reports one pre-existing high-severity npm audit finding; it was not changed by this work.
- The direct-neighbor transmit path can report ring-buffer drops under very noisy five-device telemetry while still delivering repeated valid offers. Capacity/noise tuning is production hardening, not a failed transfer.
- A three-device modern concurrent fanout was not rerun after the final fixes. The two-receiver lifecycle is covered by host tests, legacy two-receiver physical proof, and the modern single-receiver plus baton physical proof.

## S3 and Easy Flash integration contract

No repository changes were made in either consumer.

- S3: an explicit human action selects a same-family Dig2Go seed and sends the existing structured propagation command (`P`) with `FleetUpdatePropagate`, current release, a nonzero target node, and a fresh source nonce. It does not become a laptop OTA proxy and must not seed a cross-family image.
- Easy Flash: remains the one-time USB path for v13/v14 migration when P2P is not appropriate. After installing current Dig2Go firmware, an explicit user action may issue the same seed command. Easy Flash must not silently change WLED credentials, auto-flash from drive insertion, or absorb laptop-specific OTA workflows.

The portable contract is therefore small: deliver a family-correct current application image, obtain the chosen seed's current node identity, and invoke the existing structured propagation command. The device mesh owns discovery, serving, verification, bounded fanout, recovery, and baton continuation.

# Dig2Go five-device overnight receipt — 2026-08-26

## Integration state

- Worktree: `/Users/theysayheygreg/Projects/WLEDTubes-p2p-dig2go-push-bridge`
- Branch: `feature/dig2go-p2p-steve-review`
- Upstream review base: `65439293` (`Add explicit Dig2Go peer update propagation`)
- Existing proof commits: `6084307a`, `b1248fd4`, `18dc8ec6`
- Overnight hardening commit: `f97e51bd` (`Harden Dig2Go multi-peer propagation`)
- No PR was opened. Steve's main, S3, Easy Flash, laptop OTA, Wi-Fi defaults, and the immutable legacy wire were not changed.

## Exact bench identities

| Role | Port | ESP32 ROM MAC | Final role |
| --- | --- | --- | --- |
| A | `/dev/cu.usbserial-310` | `54:43:B2:B5:49:80` | v48 seed |
| B | `/dev/cu.usbserial-110` | `54:43:B2:B5:4C:38` | v48 current-version control |
| C | `/dev/cu.usbserial-2110` | `54:43:B2:B6:3A:48` | v47 receiver, then v48 child host |
| D | `/dev/cu.usbserial-2120` | `54:43:B2:B5:49:20` | v47 third receiver, then v48 grandchild host |
| E | `/dev/cu.usbserial-10` | `A0:B7:65:CA:60:80` | v47 receiver, then v48 child host |

All writes were application-slot-only after exact ROM-MAC gates. Pre-run OTA metadata and active applications were read twice with matching hashes under `preservation-before-rerun/`. The exact v13 D backup remained untouched.

## Final artifacts

| Artifact | Size | SHA-256 |
| --- | ---: | --- |
| `p2p-release47-final.bin` | 1,358,672 | `75f58abe4df284fb45762f02c70952b43faf5815803ee07addb820369f4dc320` |
| `p2p-release48-final.bin` | 1,358,672 | `f8390a8e95d40d17c23f1ce0dc02fa7016e1383be7d7b51b4485b0823bdd72f5` |

## Concrete bugs found and fixed

1. Recently reset, already-current devices could mistake an equal-release legacy baton for proof that they had just migrated. A release-specific LittleFS marker now makes legacy bootstrap eligibility explicit; only a software-reset boot without the marker may claim that bounded baton.
2. A third modern receiver could join A before its two lifetime slots retired, receive 403, and remain failed for 30 seconds—long enough to miss the child host. Propagation-only failure recovery is now 1.5 seconds; the old nonce remains rejected while a distinct child nonce can retry.
3. A propagated receiver kept the Tubes ESP-NOW callback live during synchronous flash writing, overflowing QuickESPNow's small receive queue. Propagation pulls now quiesce that transport and restore it immediately on failure. Ordinary laptop FleetUpdateOffer behavior is unchanged.
4. The async host treated “final source byte entered TCP” as “receiver finalized OTA” and tore the AP down immediately after slot two. A tested three-second terminal response drain now precedes teardown.
5. Two newly migrated child hosts used the same SSID, so a remaining receiver could associate with C while presenting E's nonce and correctly get 403. Modern turns now use the existing Steve credential field to advertise a RAM-only `Tubes-<nonce>` SSID. The v1 envelope remains exactly 22 bytes (`14 + 8`); legacy/default `TubesOTA` and `tubes123` remain unchanged.

## Physical evidence

### Legacy migration boundary

The corrected run in `telemetry-legacy-marker-fix/` proves A emitted the deployed legacy wake, D v13 joined and pulled all 1,358,368 bytes, rebooted v48, claimed the bootstrap baton, and opened a child host. B/C/E rejected equal-release legacy baton offers. This is cable telemetry plus static image evidence; no new legacy wire was invented.

### Final modern two-level fanout

The decisive run is `telemetry-modern-final/`:

- A accepted explicit command source `A5000053`, generated offer `D51560B0`, opened `Tubes-D51560B0`, served two complete 1,358,672-byte bodies, drained the final TCP response, restored normal operation, cleared the lease, and reset the turn.
- C and E both accepted the same existing `FleetUpdateOffer`, completed v47→v48, rebooted, claimed durable leases, and independently opened `Tubes-C70A0F48` and `Tubes-E890DD61`.
- D initially could not join A's one-station transfer rail, recovered, accepted E's distinct offer `E890DD61`, completed v47→v48, rebooted, claimed the lease, and opened `Tubes-C87B3ED9`.
- B remained the current-version control and rejected all equal-release propagation batons.
- Runtime `z` telemetry reported v48 and `OTA=0` on A, C, D, and E after the run.
- Post-run OTA metadata selected C app1 sequence 12, D app0 sequence 9, and E app1 sequence 6. Two independent reads of every selected application matched each other and the exact v48 artifact SHA `f8390a8e...72f5`.

This proves the requested two-level topology: `A -> (C, E)` and `E -> D`, without hardware MAC registration. It does not claim an unbounded E/F/G/H stress tree, RF-range guarantees outside this bench, or C3 compatibility.

## Verification

- `bash test/tubes_mesh/run.sh` — pass, including final-response drain, current-release marker, lease, and unique-session regressions.
- `pio run -e dig2go_p2p_release47_test` — pass; RAM 94,840 bytes, flash 1,351,993 bytes.
- `pio run -e dig2go_p2p_release48_test` — pass; RAM 94,840 bytes, flash 1,351,993 bytes.
- `git diff --check` — pass.
- npm reports the repository's existing one high-severity dependency finding; no dependency mutation was attempted.

## Remaining caveats

- The gregbot USB/app-slot reset quirk remains: A/C sometimes print transient `invalid header: 0xffffffff`, and E has shown a one-time checksum retry after fast USB operations before booting normally. This is recorded beside the deferred USB LED-load/current-protection issue; it did not corrupt the verified wireless result.
- Core ESP-NOW ring-buffer drop diagnostics remain noisy on devices that are still serving/observing a crowded bench. Pull receivers are now quiesced, and correctness proof is based on OTA completion plus exact slot hashes rather than serial silence.
- A final child with no older neighbor keeps its bounded empty-host window before normal recovery. The run proves D opened that terminal host, not that the full five-minute no-receiver timeout elapsed after the final static reset.

## Carry-forward contracts

- S3: invoke the same explicit propagation command/`FleetUpdateOffer` seed contract only after user input; provide the exact Dig2Go artifact identity. Do not make S3 a peer receiver and do not invent a parallel OTA protocol.
- Easy Flash: after a successful manual USB migration, optionally invoke the same explicit seed command. Legacy v13/v14 manual migration remains Easy Flash's product boundary when P2P is unsuitable. Keep its normal explicit-device/laptop workflows separate.
- Both integrations should consume this branch/commit and the existing command contract; neither needs access to these bench MACs or to persistent Wi-Fi credentials.

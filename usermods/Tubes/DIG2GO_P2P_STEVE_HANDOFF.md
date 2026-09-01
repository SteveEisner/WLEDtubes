# Dig2Go peer update handoff

This branch proposes an explicitly triggered, autonomous Dig2Go-to-Dig2Go
update path. A laptop is not a participant: it does not discover receivers,
choose targets, serve firmware, schedule a wave, verify each hop, or pass the
baton. The implementation reuses the `FleetUpdateOffer` wire format and modern
receiver checks without inheriting the laptop fleet workflow. Ordinary laptop
OTA remains a separate update-only operation.

## Runtime shape

1. A Dig2Go already running the desired image is explicitly targeted by a
   `Fleet Update Propagate` command. The command starts its bounded source turn
   directly; it does not enter ordinary OTA selection or require a button.

   The current serial/control form is:

   ```text
   P<release>,0.0.0.0,0,0,<target-device-id>,<nonce>,,
   ```

   `P` is propagation; ordinary server-backed fleet OTA remains `Y`.
2. The source inspects and serves its exact running application image. A
   legacy-only session uses the deployed RAM-only `TubesOTA` / `tubes123`
   network. A mixed modern turn derives a RAM-only `Tubes-<nonce>` SSID and
   carries it in both existing offer envelopes, while retaining the existing
   password and leaving saved WLED credentials unchanged.
3. During one bounded turn it emits both the deployed legacy wake and the
   propagation-marked modern `FleetUpdateOffer`.
4. Old Dig2Gos consume the legacy wake. Current Dig2Gos ignore that equal/older
   wake and consume the modern offer only when the advertised release is newer.
5. The host serves at most two receivers, sequentially, then restores normal
   WLED/Tubes radio and LED operation.
6. A successful modern P2P pull must store one durable at-most-once propagation
   lease before reboot. After reboot that receiver gets one bounded host turn,
   then clears the lease. This is a required P2P contract, not an optional
   laptop-side coordination detail.

The predecessor does not wait for a post-reboot acknowledgment or health report.
Its success boundary is completion of the bounded firmware response bodies,
after which it restores normal operation. The child's first independent
advertisement after reboot is the baton proof. This mirrors the physical
Dig2Go chain demonstrated on the bench and avoids the unreliable reboot-to-
predecessor acknowledgment seam.

After the explicit S3 or Easy Flash user action starts the seed, every runtime
decision is local to the devices. The seed and its children discover eligible
receivers, serve the image, persist continuation, recover, and stop without a
laptop roster or server.

The canonical v52 Dig2Go environment is:

```sh
pio run -e esp32_quinled_dig2go_tubes
```

It retains the standard `DIG2GO_TUBES` firmware identity and now enables the
host and dynamic Dig2Go enrollment by default. A local `P!` command explicitly
starts one turn; merely carrying the capability does not advertise or serve.
The build contains no PRIME MAC, automatic source trigger, or test-only boot
fallback. The old `_p2p` environment is removed so v52 has one canonical
Dig2Go artifact and capability set.

## Evidence boundary

Physically proven on August 25-26, 2026:

- one source migrated a known legacy Dig2Go;
- one source migrated a previously unknown legacy Dig2Go without a compiled
  receiver MAC;
- one source migrated two unknown legacy Dig2Gos sequentially in a single
  fanout-two turn;
- one legacy v13 receiver installed v48, rebooted, and opened its own bounded
  child-host turn;
- in a five-device modern run, A served C and E from v47 to v48, then E passed
  the baton to D;
- C, D, and E were read back twice after the run and every active application
  slot matched the served v48 SHA-256 exactly;
- sources and receivers restored normal operation without a post-reboot ack.

Host/model tests cover the running-image source, strict Dig2Go target contract,
HTTP ranges and transfer completion, A-to-B, A-to-C, A-to-C-plus-D, bounded
fanout, modern offer validation, ordinary-OTA non-propagation, lease claim and
replay prevention, command separation, and mixed legacy/modern wake
construction.

The clean artifact still needs Steve's integration review and a final physical
smoke after reconciliation, but its core legacy migration, modern fanout, and
modern child continuation paths have physical evidence. Legacy continuation
uses a narrowly bounded first-boot marker: only a software-reset boot without
the current-release marker may claim the bootstrap baton. Ordinary current
boots and ordinary laptop OTA do not implicitly propagate.

Not proven: unbounded tree depth, RF range beyond the desk, C3 compatibility,
or final S3/Easy Flash activation UX.

## Verification

```sh
bash test/tubes_mesh/run.sh
node --test tools/fleet-update-protocol-test.js
pio run -e esp32_quinled_dig2go_tubes
```

All canonical v52 Dig2Go images carry the same dormant propagation capability.
C3 family serving and its device flow are intentionally deferred.

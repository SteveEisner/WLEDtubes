# Dig2Go peer update handoff

This branch proposes an explicitly triggered, autonomous Dig2Go-to-Dig2Go
update path. A laptop is not a participant: it does not discover receivers,
choose targets, serve firmware, schedule a wave, verify each hop, or pass the
baton. The implementation reuses the `FleetUpdateOffer` wire format and modern
receiver checks without inheriting the laptop fleet workflow. Ordinary laptop
OTA remains a separate update-only operation.

## Runtime shape

1. A Dig2Go already running the desired image is explicitly chosen as the
   source. The current field prototype uses `Q` followed by a physical
   double-click; S3 and Easy Flash own the eventual user-flow policy.
2. The source inspects and serves its exact running application image over a
   temporary RAM-only `TubesOTA` / `tubes123` network.
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

The production review environment is:

```sh
pio run -e esp32_quinled_dig2go_tubes_p2p
```

It retains the standard `DIG2GO_TUBES` firmware identity. It enables the host
and dynamic Dig2Go enrollment, but contains no PRIME MAC, automatic source
trigger, or legacy boot fallback.

## Evidence boundary

Physically proven on August 25, 2026 with the earlier bench activation:

- one source migrated a known legacy Dig2Go;
- one source migrated a previously unknown legacy Dig2Go without a compiled
  receiver MAC;
- one source migrated two unknown legacy Dig2Gos sequentially in a single
  fanout-two turn;
- receivers rebooted onto the served image and the source restored normal
  operation.

Host/model tests cover the running-image source, strict Dig2Go target contract,
HTTP ranges and transfer completion, A-to-B, A-to-C, A-to-C-plus-D, bounded
fanout, modern offer validation, ordinary-OTA non-propagation, lease claim and
replay prevention, source selection separation, and mixed legacy/modern wake
construction.

Still requiring a small physical proof on this clean artifact:

- explicit field trigger on the production P2P build;
- a modern older-release receiver pulling the newer image, rebooting, claiming
  its lease, and hosting one child;
- one mixed old/current receiver turn.

A receiver that entered through the deployed legacy wake cannot have written a
modern lease before reboot. The physically tested viral legacy chain used a
test-only first-boot fallback, deliberately absent here because it would make
ordinary OTA implicitly propagate. In this clean build a legacy receiver is a
terminal migration result; modern receivers carry the reusable automatic
follow-on turn. Resolving legacy-child continuation requires an explicit
post-reboot command/receipt design and is not disguised as production behavior.

## Verification

```sh
bash test/tubes_mesh/run.sh
node tools/fleet-update-protocol-test.js
pio run -e esp32_quinled_dig2go_tubes
pio run -e esp32_quinled_dig2go_tubes_p2p
```

The ordinary Dig2Go build remains a regression control with P2P disabled.
C3 family propagation and its device flow are intentionally deferred.

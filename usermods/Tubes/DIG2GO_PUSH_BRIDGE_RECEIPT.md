# Dig2Go legacy P2P migration candidate receipt

- Branch: `feature/p2p-dig2go-push-bridge-v1`
- Reconciled code merge: `36c59664`
- Steve base: `d21b3850` (`origin/main`, release 47)
- Environment: `dig2go_push_bridge_test`
- PRIME A: `54:43:B2:B5:49:80`
- enrolled receiver B: `54:43:B2:B5:4C:38`
- Artifact: `build_output/firmware/dig2go_push_bridge_test.bin`
- Size: `1,351,280` bytes
- SHA-256: `ab6aa377386afd50334209e30689112cf425d7933698d583ae669d5dfba68815`
- Embedded fleet identity: protocol 1, Dig2Go family, standard variant,
  release 47, reserved bytes zero
- Release name: `DIG2GO_TUBES_PUSH_TEST`

## Compatibility boundary

Steve's current fleet protocol remains the authority for current firmware. This
candidate only carries a current image across the legacy v13/v14 boundary. A
hosts the image at `4.3.2.1/firmware.bin`; wildcard DNS maps B's deployed
hardcoded `brcac.com` request to A. B performs its own existing pull/update.
Completion is not inferred from HTTP alone: A must restore its mesh and receive
a fresh exact-MAC release-47 device report from B.

The first isolated hardware run showed all five stage pairs blue while B was
off, then red at the rendezvous deadline. Blue is the pending state, so that run
did not claim a request or transfer. A later run reached the same deadline while
B was being manually introduced, exposing the 30-second window as a human race.
The corrected candidate uses a five-minute legacy rendezvous, handles temporary
HTTP-server backpressure without aborting, and still requires exactly one
associated station whose Wi-Fi MAC matches B before serving `/firmware.bin`.

## Reconciliation verification

- `./test/tubes_mesh/run.sh` passed, including legacy wire, rendezvous, running
  image source, HTTP/session, bridge, readiness, inspection, modern fleet,
  channel, tempo, and downbeat coverage.
- `node tools/fleet-update-protocol-test.js` passed.
- `node tools/tempo-tracker-test.js` passed.
- `pio run -e dig2go_push_bridge_test` passed.
- Linked RAM: `94,720 / 327,680` bytes (28.9%).
- Linked flash: `1,342,537 / 1,572,864` bytes (85.4%).
- `git diff --check` passed before this documentation correction.
- No Dig2Go was flashed as part of reconciliation.

## August 25 cable-attached proof

With both LED strands removed, A and B remained continuously enumerated for
65.040 seconds with zero missing samples. USB power diagnosis is deferred until
after Friday's event; the working explanation is LED-load/current-protection
cycling on the 1.5 A-per-port gregbot hub against a Dig2Go load that may reach
3 A. Serial port opens still coincided with controller resets, so the final run
must not use attached serial monitoring.

The final bounded legacy diagnosis then established the complete receiver
boundary in telemetry:

- A prepared the 1,351,264-byte running image, kept ESP-NOW alive on the AP
  carrier, started `TubesOTA`, and reported a radio-accepted legacy wake.
- Legacy B logged `OTA: starting autoupdate`, joined A as exact MAC
  `54:43:B2:B5:4C:38`, requested `/firmware.bin`, accepted content length
  1,351,264 and `application/octet-stream`, recognized a valid OTA BIN, and
  reported increasing write progress.
- On the next observation B booted the current release-47 candidate and logged
  `TUBE_PUSH_AUTO disabled: this device is not PRIME`. This crosses the physical
  legacy migration boundary; it is not inferred from HTTP completion alone.
- Current B then accepted Steve's `FleetUpdateOffer`, joined A again, and began
  a current-firmware fleet download from the existing `/tubes/firmware.bin`
  route. Health-report/baton completion was not captured before serial probing
  stopped, so modern serve/verify/baton remains only transition-proven.

That run also showed A accepting its own wildcard fleet offer. The staged final
candidate therefore sends the unchanged legacy broadcast but targets the
existing modern offer to B's observed DeviceId `0x1E2E`; A's DeviceId is
`0x197C`. The artifact above passed the full Tubes mesh suite and PlatformIO
build, was app-only flashed to exact-ROM-MAC-gated A at `0x10000`, and its
readback SHA-256 matches byte-for-byte. B was not flashed over USB.

## Externally powered wireless proof

Greg ran A and restored-legacy B with USB disconnected and normal external
power. A advanced from two green pairs to four green pairs. B froze its normal
pattern, displayed the first ten pixels yellow, went dark, rebooted, and resumed
a normal pattern. A remained latched at four green pairs and one red pair: wake,
exact receiver admission, firmware request, and complete response body passed;
only the optional post-reboot mesh health callback timed out.

Read-only inspection after the run proved the product result independently:

- B's OTA sequence advanced from 3 to 4, selecting newly written `app1` rather
  than the restored legacy `app0`.
- B's selected `app1` first 1,351,280 bytes have SHA-256
  `ab6aa377386afd50334209e30689112cf425d7933698d583ae669d5dfba68815`,
  byte-for-byte identical to A's served artifact.
- The observed dark reboot and return to normal LEDs therefore correspond to a
  real boot-selection change into the exact current image, not merely a
  completed HTTP response.

Under the event product boundary shared with Easy Flash, this is a successful
legacy-to-current wireless migration. The fifth health callback remains useful
diagnostic debt, but is not required when a device visibly restarts and its
new version/image is subsequently confirmed.

## Golden-prime discovery candidate

The follow-on prototype removes B's compile-time MAC and DeviceId from A while
keeping A's PRIME identity exact and preserving the deployed `TubesOTA` /
`tubes123` credential contract. A maps the requesting client's AP IP to its
station MAC, requires exactly one station, latches that MAC for the session,
and uses it for optional later health verification. The legacy rendezvous does
not send a wildcard modern offer, avoiding host self-acceptance when the future
receiver's DeviceId is unknown.

Candidate artifact SHA-256:
`3e6380ad706ef8f44980bb61380085a371d5c8011a567e04174fed50b4315f82`.
The full Tubes mesh suite and `pio run -e dig2go_push_bridge_test` pass. Physical
proof against a previously unknown legacy Dig2Go is recorded below.

## Unknown-C wireless proof

A was exact-ROM-MAC-gated and app-only flashed with the discovery candidate;
its readback matched the artifact above. B was disconnected and untouched. With
A externally powered, previously unregistered legacy C was powered separately:

- A advanced from two green pairs through six and eight, then latched all five
  pairs green.
- C displayed the legacy yellow updater state, froze its pattern, went dark,
  rebooted, and returned to a normal pattern.
- A's ten-green latch proves its dynamically learned receiver returned the
  expected current release/hash and mesh health after reboot.
- Read-only inspection identified C as ROM MAC `54:43:B2:B6:3A:48`.
- C's OTA sequence advanced from 1 to 2, selecting newly written `app1`.
- C's selected `app1` first 1,351,392 bytes have SHA-256
  `3e6380ad706ef8f44980bb61380085a371d5c8011a567e04174fed50b4315f82`,
  byte-for-byte identical to A's served artifact.

This proves A can discover and migrate one legacy Dig2Go without any compiled
receiver MAC or DeviceId.

## Two-receiver baton candidate

The next candidate preserves the same deployed Wi-Fi credentials and Steve's
current fleet protocol as the control authority. During a legacy rendezvous the
SoftAP admits one station, so the first legacy device to associate wins; this is
an association race, not a claim about which device first sends HTTP. A stops
the repeated legacy wake after that association and serves only the admitted
station.

After the winner reboots and returns a fresh exact-device release/hash health
report, A sends that winner a targeted, validation-restricted baton on the
existing `FleetUpdateOffer` wire. A baton contains no server, start window,
credentials, wildcard target, or force bit. A non-PRIME current Dig2Go may host
one legacy rendezvous only after accepting that exact-target baton. It then uses
the same serve, verify, and baton path for the remaining legacy device.

This section records candidate behavior only. It does not yet claim a physical
two-receiver migration or prove which of C and D wins the association race.
Candidate artifact size: `1,352,528` bytes. SHA-256:
`6529f4f1d258089495153eaa85ead8c36f172359dfb494533a34aa43b5281975`.

The first two-receiver run proved the association winner was C
(`54:43:B2:B6:3A:48`), despite the initial visual identification as D. A reached
full green and static inspection later found C's OTA sequence advanced to 3
with the exact candidate in selected `app0`. D (`54:43:B2:B5:49:20`) remained
on its byte-matching legacy image. C and D were then both returned to verified
legacy state. The run did not prove baton propagation: the one-shot grant had
no acceptance acknowledgment, and non-PRIME baton hosts did not draw the host
diagnostic, so normal LEDs could not distinguish a lost grant from an active
but receiverless host.

The follow-up candidate reuses `DeviceReportReply` as a correlated baton ACK,
without changing either wire structure. A records the exact nonce, DeviceId,
MAC, release/hash, hardware identity, mesh state, and output contract, retries
the identical targeted offer once per second for ten seconds, and accepts only
a matching report. Duplicate same-nonce grants are idempotent and cause another
ACK; a different grant is rejected after ownership is armed. The fifth pair on
A is yellow while ACK is pending, green only after exact acceptance, and red on
timeout. A non-PRIME baton holder now draws the same five-stage host diagnostic.
Follow-up artifact size: `1,353,200` bytes. SHA-256:
`1fca04b86fee1ea6887b4e9aa0478cf64b79e3c53ec74bbd1b023cfde7b670af`.
This behavior is build-verified but not yet physically proven.

The staged chain-lifecycle revision makes hosting a temporary lease. After an
exact baton ACK, the predecessor holds full green for three seconds, retires
its host role, clears the diagnostic, and resumes normal mesh rendering. The
successor alone draws the host stages. A final successor that sees no legacy
station during its bounded rendezvous restores the mesh and returns to normal;
an empty fleet is chain completion, not a red transfer failure. Concrete
association, HTTP, image, or health failures remain visible failures.
Lifecycle artifact size: `1,353,376` bytes. SHA-256:
`387a9d292f69359f1df6ff6615ccc228c02b9719747a63b9b3c6f79d5ff2f095`.
This lifecycle is build-verified but not yet physically proven.

The next physical run established that D (`54:43:B2:B5:49:20`) won: its OTA
sequence advanced from 1 to 2 and selected `app1` matched the lifecycle artifact
byte-for-byte. C remained on legacy with unchanged OTA sequence 3. A served the
complete body but never accepted D's post-reboot health, so the baton path was
not reached. This separates a successful migration from an unreliable
post-reboot mesh callback when a losing legacy device remains present.

For the next cohesive prototype, a non-PRIME node running this candidate takes
one automatic host turn 15 seconds after boot if no acknowledged baton arrived.
Exact health plus acknowledged baton remains optional fast-path telemetry. A
predecessor retires its diagnostic ten seconds after restoring from a complete
transfer, whether or not post-reboot health arrives; the freshly migrated
successor therefore continues independently. This boot fallback is
intentionally prototype-only: production
must replace it with a durable migration/lease marker so ordinary current
devices do not host after every reboot. Artifact size: `1,353,664` bytes.
SHA-256:
`a09849907e75b0d1e5a598bce241870dd645f4ba7e02c6f95b6d838ba3941035`.

## Fanout-two candidate

The propagation run proved A updated D, A recovered, D automatically hosted,
and D updated C after C's legacy updater was re-armed by one power cycle. C then
took its own host turn and timed out cleanly. The power cycle was required only
because the one-client AP limit made C lose association after it had already
consumed the broadcast legacy wake. D also retained a stale ten-green completion
overlay after hearing current C's wake.

The final candidate removes that artificial one-client race. A temporary host
admits two stations and tracks two independent immutable-image responses; it
restores only after every admitted/requesting transfer completes. Wake repeats
until both client slots fill, so two migrated children can each take their own
bounded boot-time fanout turn. An equal-or-older legacy offer is ignored by
current firmware and clears stale migration completion UI; forced modern OTA
remains exclusively on Steve's `FleetUpdateOffer` path. Standard `TubesOTA` /
`tubes123` credentials remain unchanged.

Artifact size: `1,353,856` bytes. SHA-256:
`ce31bfece946061e0b2c495ad8a2147c909ccecdb472e8c86d273d706a459567`.
The full Tubes mesh suite and PlatformIO build pass. Physical fanout-two proof
has not yet been run.

## Fanout-two physical closure and production review

The first concurrent legacy attempt established that both unknown receivers
heard the wake, joined the temporary network, and entered their updater, but
both aborted red when normal two-stream backpressure exposed the deployed
client's empty-read-as-EOF behavior. The compatible host revision therefore
kept two lifetime receiver slots while allowing only one legacy AP association
at a time.

The next externally powered run proved the serialized result end to end. D
joined and pulled first while C continued blinking yellow. D completed, went
dark, rebooted onto the served image, and began its bounded host turn. C then
joined A, completed, went dark, rebooted, and began its bounded host turn. A
showed both complete transfers, restored its ordinary radio, and returned to
normal rendering. Neither receiver MAC was compiled into A.

The subsequent production review separates the proven prototype from reusable
scaffolding:

- ordinary Steve `FleetUpdateOffer` OTA never arms peer propagation;
- `FleetUpdatePropagate` is an explicit opt-in on the existing command;
- an exact-target equal-version command starts a root host without reinstalling;
- a successful newer propagation offer creates one durable child host lease;
- wildcard equal-version activation is invalid and equal/newer peers ignore the
  propagated download offer;
- the legacy non-PRIME boot fallback is explicitly test-only;
- the abandoned baton extension was removed from the fleet wire;
- temporary WLED AP globals and ESP-NOW ownership now fail and restore
  transactionally;
- active streams use a no-progress timeout and cannot be cut off by the absolute
  rendezvous deadline.

Physical modern v47-to-v48 command/update/lease propagation remains the next
hardware proof. No deeper E/F/G/H tree is claimed or required by this receipt.

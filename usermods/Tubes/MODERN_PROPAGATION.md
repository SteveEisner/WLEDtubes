# Modern Dig2Go propagation prototype

Modern release propagation reuses the `FleetUpdateOffer` wire and receiver
validation contract, but not the laptop fleet workflow. After an explicit S3
or Easy Flash user action starts the seed, discovery, serving, download,
continuation, and retirement are device-to-device; no laptop server, roster,
target selection, or per-hop verification participates. It does not persist or
replace WLED's standard Wi-Fi credentials: legacy-only sessions retain the
deployed `TubesOTA` / `tubes123` contract, while mixed modern turns derive a
RAM-only `Tubes-<nonce>` SSID and carry it in both existing offer envelopes.
The per-turn name prevents two child hosts from attracting the wrong receiver;
the password and saved WLED configuration remain unchanged. Nor does an
ordinary reboot turn a current device into a host. Ordinary offers never arm peer hosting;
`FleetUpdatePropagate` is the explicit P2P opt-in carried by the existing wire.

When a device accepts and successfully installs a strictly newer propagation offer,
the updater writes `/tubes-propagate.bin` before scheduling its reboot. The
record contains the installed Tubes release and source offer nonce plus a
checksum. Equal or older offers, forced equal-release reinstalls, legacy wakes,
corrupt records, and records for a different running image cannot arm a
turn.

Lease persistence is additive to OTA success. `HTTPUpdate` has already verified
the image and selected the next boot partition before the lease is written; a
filesystem failure disables propagation for that child but does not misreport
the valid OTA as failed or prevent its reboot.

On the first boot of the new image, the record is changed from `armed` to
`claimed` before radio hosting begins. This at-most-once transition prevents a
reset during a turn from repeatedly amplifying the same update. The claimed
device then reuses the existing immutable running-image HTTP host with capacity
two, but advertises a wildcard, non-forced `FleetUpdateOffer` for its current
release instead of a legacy wake. Older current-firmware peers accept; peers on
the same or newer release reject in `AutoUpdater::startFleet()` without joining
the update network. Each successful child writes its own lease before reboot,
providing bounded fanout-two propagation.
If a reset interrupts a claimed turn, the next boot removes the stale claimed
record without hosting again.

The durable lease is the core continuation contract for modern P2P. It is
written by the receiving device before reboot and consumed locally afterward;
it deliberately replaces laptop-coordinated second acknowledgments and per-hop
commands.

The predecessor never waits for the updated child to reboot, rejoin, report
health, or acknowledge it. Completing the bounded firmware bodies is the
predecessor's terminal success condition; it restores normal operation and
retires. The child's first lease-driven advertisement after reboot is the
observable continuation proof, matching the physically proven legacy chain.

An already-current root starts the same turn from an exact-target propagation
command with no OTA server or credentials. That command does not reinstall the
root. Wildcard equal-version serve commands are invalid, so current peers do not
recursively activate one another.

Field activation is explicit and separate from laptop OTA selection. An S3 or
other controller broadcasts the additive `Q` action, opening a 20-second source
window on capable tubes. A human double-clicks exactly one source; that tube
constructs the exact-target command for its own current Device ID and begins one
bounded turn. The existing `*` / `y####` selection paths retain their
`WLED-UPDATE` behavior and are not used by propagation.

Easy Flash may present "propagate after install" as its default product choice,
but it must still require an explicit user action and invoke this same source
trigger after the new image boots. Firmware does not infer propagation from an
OTA, reboot, proximity, or version change. Ordinary laptop OTA therefore still
installs and stops. Easy Flash integration is a contract only in this repository;
its repository is intentionally untouched.

The shared host admits two receivers per turn and serves them sequentially. A
modern turn emits both the propagation-marked `FleetUpdateOffer` and the
deployed legacy wake. Current Dig2Gos ignore the equal/older legacy offer; old
Dig2Gos ignore the unknown modern command. Serializing the two slots preserves
the deployed client's requirement while allowing old, current, or mixed
Dig2Go populations in one explicit run.

The host clears the claimed record when its bounded turn retires, including an
empty turn. Concrete transfer failures remain visible through the existing host
diagnostic. This prototype shares the image server and bounded host lifecycle
with legacy migration, but the activation mechanisms remain separate:

- legacy migration is activated only by the deployed `COMMAND_UPGRADE` wake;
- modern propagation is activated by an exact-target serve command or a durable
  lease created after a successful propagation-marked installation.

Host tests cover arming, claiming once, release matching, corruption, and the
legacy/equal-release rejection boundary. The native
`dig2go_peer_propagation_test` and production P2P PlatformIO build verify the
integration. A five-device bench run physically proved v47-to-v48 fanout A to
C and E, followed by E to D; two independent reads of each updated active slot
matched the served firmware hash.

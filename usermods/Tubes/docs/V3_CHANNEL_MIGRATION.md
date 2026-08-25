# Tubes V3 Channel Protocol and Migration Plan

This document defines the V3 implementation target and the V2-to-V3 migration.
It also records the projection pattern that a future V4 migration should reuse.
The older claim-based V3 candidate in `PROTOCOL.md` is historical and is not the
implementation contract.

## Model

V2 has one Control ID, one Control uplink, and one downward relay tree. V3 keeps
that transport unchanged. All existing V2 messages are Control messages, and all
upward forwarding, root selection, follower state, leading state, and downward
relaying continue to use the Control ID and Control uplink.

V3 splits the visual part of the V2 `TubeStates` payload into three channels:

- Beat contains the existing BPM and beat-frame fields.
- Pattern contains the existing current and next pattern, sync-mode, effect,
  effect-parameter, and phrase fields.
- Palette contains the existing current and next palette ID and phrase fields.

Pattern includes the V2 Effect state. Position and Debug are Control operations.
There is no Presence channel. The first channel versions add no visual state that
V2 cannot represent.

Each pole stores a Beat Channel ID, Pattern Channel ID, and Palette Channel ID.
They initialize from the pole's Control ID and may later be changed independently
by a Control action. A channel authority is compared lexicographically as
`(channelId, controlId)`. The pair is never stored as a synthetic numeric ID.
Only Control IDs require collision detection; equal channel IDs are resolved by
the higher Control ID.

If a pole's Control ID changes at runtime, every channel that still equals the old
Control ID inherits the new value. A channel that was explicitly assigned a
different value remains independent. This preserves the default relationship
without erasing deliberate per-channel ownership.

The initial firmware exposes local serial operations `B`, `K`, and `C` for changing
the Beat, Pattern, and Palette Channel IDs. They use the same displayed-ID notation
as the existing `i` Control-ID operation, so `B255` assigns Beat Channel ID `FF0`.

## Requests and declarations

A follower sends Beat, Pattern, and Palette state upward through the Control tree
as a request. The Control owner tracks the live winner and last complete state for
each channel.

For each channel, the Control owner applies these rules:

1. With no live winner, the first valid request wins.
2. A request from the current `(channelId, controlId)` with a new sequence refreshes
   the winner and replaces its complete state.
3. A higher `(channelId, controlId)` replaces the winner.
4. A lower pair is ignored.
5. Silence expires the winner after the channel lease.

The Control owner changes an accepted request into a downward declaration. Normal
Control relays repeat that declaration. Every V3 pole mirrors the winning pair,
source boot session, sequence, expiry, and complete channel state so it is ready if
it becomes the Control owner.

A losing source suppresses its channel output while it receives declarations for a
higher effective ID. It resumes requests after the channel lease expires. The first
implementation uses the deployed V2 20-second uplink timeout for all three channels;
later channel versions may tune the leases independently.

Output suppression does not stop local schedule generation. After each Pattern,
Palette, or Effect boundary, every pole generates and retains a speculative next
entry. A winning declaration overwrites that local entry. If the authority becomes
unreachable, the winner in each remaining partition can publish the schedule it
already holds, preserving the failover behavior of the legacy `UPDATE` protocol.

The original Beat body contains only current timing. Pattern and Palette declarations
retain V2's current-and-next look-ahead. A newly accepted winner sends a complete
snapshot immediately and refreshes it on the deployed V2 status cadence even when
the state has not changed.

The first Beat extension uses otherwise-zero bytes after the original six-byte body.
A magic value, extension version, and extension length precede `current` and `next`
sound-program entries. Each entry carries an effective phrase, enabled flag, raw WLED
effect ID, palette, speed, intensity, three custom sliders, three checkbox options,
blend mode, and opacity. The channel carries visual control parameters, never
microphone samples. This intentionally avoids a protocol registry: the Beat owner
may choose from a local candidate table while receivers execute the complete WLED
program sent on the wire.

The remaining eight bytes form an optional transient trailer. It carries a marker,
version, wrapping event sequence, and one-byte kick, snare, and broad-music onset
strengths. The Beat owner derives these values from positive per-bin spectral change:
kick uses WLED FFT bands 0-2 and snare uses bands 9-13. It compares each focused
transient with broad spectral flux so concentrated electronic percussion receives a
strong accent while uncertain full-spectrum activity contributes at only 15% strength.
Total volume never opens the layer. Sustained sound does not keep an overlay visible
because only a rising spectral transient creates a new event. The owner sends that
compact event immediately, and receivers fade kick, snare, and broad activity over
240, 180, and 600 ms respectively while the selected WLED effect keeps rendering
behind it. This keeps the poles synchronized without putting microphone samples on
the mesh.

The optional local tempo tracker follows the same boundary: the Beat owner analyzes
WLED's completed 16-band FFT frames and publishes only the resulting ordinary BPM
and beat frame. It neither extends the Beat-channel packet nor changes what older
receivers interpret. Losing Beat ownership stops local analysis from driving the
channel; the last accepted clock continues free-running as before.

Each known Beat overlay also has the same inclusive `0`-`255` chance used by regular
Tubes effects. Every pole independently rolls once for each accepted transient, so
the selected fraction varies across the fleet without synchronizing a full-fleet
flash. The chance is derived from the overlay effect ID and does not change the wire
entry; unknown or explicitly selected raw WLED effects retain the all-poles default.

The rotating policy resolves each candidate's palette to the Beat owner's current
palette when it schedules the entry. Its candidate table deliberately mixes centered,
traveling, frequency-colored, and sparse effects with additive, screen, overlay,
hard-light, soft-light, lighten, and stencil composition. The opacity in the program
is the peak opacity for a transient rather than a continuously visible layer. The
resolved packet still contains ordinary WLED palette, blend, and opacity values;
receivers do not need to know the local selection policy.

The envelope version and body length remain unchanged, so older generation-1
firmware accepts and relays the entire packet while ignoring the extension bytes.
Updated firmware recognizes each marker and validates the extension before applying
it. Older generation-1 firmware ignores the transient trailer while relaying all 64
bytes unchanged. A Beat declaration without the sound-program marker means that sound programming is disabled.
The sound segment remains active over both WLED-backed and custom Tubes patterns.
Custom Tubes rendering occurs outside WLED's native segment composition and may
overwrite the sound segment on the final LEDs; the sound program continues running
so it is immediately available when a WLED-backed pattern becomes current again.

## Mixed V2 and V3 components

### Device identity and protocol generation

Every pole has one full-word Device ID. Its fields are:

```text
bits 15..12: protocol generation
bits 11..0:  protocol-local value

deviceId = (protocolGeneration << 12) | protocolLocalValue
```

Generation `0` is the deployed legacy protocol, whose Device IDs are the same `0xxx`
values it has always sent. Generation `1` is the channel protocol defined by this
document. A generation-1 pole with protocol-local value `ADC` has Device ID `1ADC` in both
native channel packets and backward-compatible V2 projections.

The existing outer `header.version` remains the packet-layout decoder version. It is
independent of the Device ID generation nibble: a generation-1 pole deliberately sends
a V2-layout projection with `header.version == 2` and Device ID `1ADC`. Old firmware
does not understand the generation field, but its deployed ID field is already a full
16-bit value, so it treats `1ADC` as an ordinary higher ID without truncating it.

Application state, configuration, reports, channel authority, topology, diagnostics,
and operator-facing references use the complete Device ID. `0ADC` and `1ADC` are
different Device IDs and do not collide. ID generation selects a nonzero protocol-local value
inside the running protocol generation. ID-setting operations accept a protocol-local value or an old
ID and overwrite its high nibble with the running generation.

Generation-1 devices form their Control tree from complete Device IDs and do not
follow generation-0 senders. V2 devices hear the V2 projection, follow the adjacent
`1xxx` sender, and send legacy root requests back to that Device ID. A single
generation-1 device in a connected component therefore becomes the effective Control
owner for all generation-0 devices it can reach through normal relaying.

The generation-prefix rule is the mixed-component migration contract. If a connected
component contains any reachable generation-1 pole, legacy election makes the
generation-0 poles follow the generation-1 tree; there is no generation-0 Control
owner in that component. An isolated generation-0 component elects a legacy owner
normally, then yields as soon as it can hear a `1xxx` identity.

A V3 Control owner emits both native V3 declarations and a byte-for-byte compatible
V2 `TubeStates` declaration assembled from the accepted Beat, Pattern, and Palette
snapshots. The V2 payload keeps outer protocol version 2. A validated marker in
unused payload bytes 48 through 63 identifies the packet as a V3-generated gen0
projection. Gen0 receivers ignore those bytes. Gen1 receivers recognize the marker,
relay the packet when needed for gen0 descendants, and never apply its state.

Gen1 receivers do not forward-adapt genuine gen0 `COMMAND_STATE` packets. The Device
ID generation prefix already guarantees that a reachable gen1 pole owns the Control
tree, so accepting gen0 visual authority would reintroduce two competing sources of
truth. A gen1 Control owner arbitrates native channel requests and is the only bridge:
it continuously projects the winning state into gen0 packets for legacy followers.

The legacy-only update offer remains a special Control-master broadcast because its
payload is too large for the gen1 Control body and carries no rendered state. It does
not establish gen0 authority or change channel winners.

## Split, merge, and takeover

Control-tree splits and merges retain the deployed V2 algorithm. A pole follows the
highest directly heard neighbor whose Device ID is greater than
its own. Losing an
uplink causes local reconsideration after the V2 timeout; a pole with no directly
heard higher Device ID becomes a local Control owner.

Each new V3 Control owner starts with its mirrored channel winners and snapshots.
Those winners remain leases, not permanent knowledge. Sources still present in the
new component refresh them. Sources across the split expire, after which the first
valid request wins until a higher effective ID is heard. Gen0 declarations never
supply channel snapshots; the gen1 owner continues projecting its native winners.

## Future channel versions and V4 migration

Channel versions evolve independently. A later Beat, Pattern, or Palette version may
add state that V2 cannot express, but it must define an explicit V2 projection for
mixed components. Unknown channel versions and optional extensions are ignored
without changing the last accepted state.

A future V4 migration should repeat the same structure:

1. Preserve the older packet decoder and relay behavior.
2. Give new messages an unambiguous versioned marker that older firmware ignores.
3. Allocate the next high-nibble generation value so the newer protocol always wins
   Control election within a connected component.
4. Have the newest Control owner emit both its native declarations and an exact
   older-protocol projection.
5. Have newer receivers relay projections for older descendants without applying
   those projections to their own state.
6. Define migration per connected Control component; no pole assumes knowledge of
   devices it cannot currently hear through that component.
7. Retain the prior protocol-local value when practical, but replace the generation
   nibble so the complete Device ID belongs to the new protocol generation.

## Initial validation matrix

- Generation-0 and generation-1 packets preserve the complete Device ID through
  receive, election, forwarding, downward relaying, and reports.
- A legacy pole follows `1xxx`, stores the complete value as its uplink, and accepts
  V2 projections from that Device ID.
- A V3 Control owner accepts the first request, refreshes the same source, rejects a
  lower effective ID, and switches immediately to a higher effective ID.
- Equal channel IDs are won by the higher Control ID without channel conflict repair.
- Losing sources stop requesting and resume after the 20-second lease.
- Pattern and Palette preserve current and next state through request, declaration,
  Control-owner replacement, and V2 packing.
- A genuine V2 declaration never changes gen1 Beat, Pattern, or Palette state.
- A V3-marked V2 projection is relayed for gen0 descendants but never applied by gen1.
- V2-only followers render the packed output of a V3 Control owner unchanged.
- Splits, merges, winner loss, source reboot, and Control-owner replacement converge
  using bounded state and no device list.

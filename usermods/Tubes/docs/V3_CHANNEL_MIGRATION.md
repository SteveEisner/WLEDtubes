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

Beat declarations contain only current timing. Pattern and Palette declarations
retain V2's current-and-next look-ahead. A newly accepted winner sends a complete
snapshot immediately and refreshes it on the deployed V2 status cadence even when
the state has not changed.

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

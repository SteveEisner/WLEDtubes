# Overnight command-path bench artifacts

These are application images only. They are staged for identity-gated writes to
the existing Dig2Go application slots; bootloader, partition table, NVS, and
filesystem are outside the write set.

| Artifact | SHA-256 | Embedded `TUBEUP1` identity |
| --- | --- | --- |
| `p2p-release47-modern-receiver.bin` | `9a8b2d2100d0e6da757c8911ae6c3aa70b46e2e608e319d69c8eb40a0290692b` | protocol 1, family 1 (Dig2Go), variant 0, release 47 |
| `p2p-release48-startup-and-rendezvous-fix.bin` | `bd31ca02146fcd5fe6a9d98befb84f4a03f60eaaf3ed16d785b94d08de779392` | protocol 1, family 1 (Dig2Go), variant 0, release 48 |

The legacy receiver image used for C and D is the previously preserved
application image with SHA-256
`16cf230edca34077ac196a1b4fbae0d94000967148e88b8f8846181992c34db9`.
Its exact source path will be recorded with each identity-gated flash receipt.

Five devices were enumerated and identity mapped on gregbot:

- A: `54:43:B2:B5:49:80`
- C: `54:43:B2:B6:3A:48`
- D: `54:43:B2:B5:49:20`
- B: `54:43:B2:B5:4C:38`
- E: `A0:B7:65:CA:60:80`

The final release-48 image above was served successfully through both the
deployed legacy pull path and Steve's modern `FleetUpdateOffer` path. See
`INTEGRATION-RECEIPT.md` for the evidence boundary.

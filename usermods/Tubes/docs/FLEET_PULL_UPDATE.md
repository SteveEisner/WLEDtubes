# Parallel fleet pull-update proposal

The proposed fast path sends firmware once from a local threaded HTTP server and
lets every eligible pole pull its own board-specific image concurrently. ESP-NOW
carries only a 44-byte offer; it never carries firmware blocks.

This is additive to the existing selected-device updater. Release 22 is the
bootstrap boundary: older poles need one final upgrade through the existing path.
After that, `fleet_pull_update.py` can move a visible release-22-or-newer fleet to
the current release.

## Why this changes the scaling

The current updater creates the same `WLED-UPDATE` AP and `4.3.2.1` address on
every selected pole. One computer radio must associate, inspect, upload, reboot,
and verify each device serially, so elapsed time grows almost directly with fleet
size.

The pull design instead uses one dedicated 2.4 GHz update network:

```text
USB mesh controller --44-byte offer--> visible poles
                                          |
                                          +-- concurrent HTTP pulls -- Ethernet host
```

The host performs a canary first. Once that stable MAC returns on the mesh with
the expected release and unchanged hardware shape, one wildcard offer starts the
rest of the fleet. Stable-MAC hashing spreads starts across a configurable window,
so 50 devices do not begin TCP and flash setup in the same millisecond.

The access point, rather than the host process, becomes the expected bottleneck.
The included server delivered 50 concurrent 1.31 MB responses on localhost in
0.122 seconds; that proves independent server state and concurrency, not Wi-Fi
capacity. A physical Dig2Go pull through another pole's temporary AP transferred
1,312,592 bytes in 16.8 seconds, took 21.7 seconds from offer to final byte, and
rejoined as release 23 with its 112-LED configuration unchanged.

For planning, a 1.31 MB image is about 10.5 megabits. At a deliberately conservative
8-16 Mbit/s aggregate application throughput, the non-canary payload wave is about
13-7 seconds for 10 devices, 26-13 seconds for 20, and 66-33 seconds for 50. Adding
the proven canary, a five-second start window, reboot, and mesh verification gives
a working target of roughly one minute for 10-20 devices and one to two minutes for
50. Those are capacity estimates until a 10-, 20-, and 50-pole venue-AP test records
real results; the orchestrator prints its measured elapsed time and aggregate rate.

## Reliability boundaries

The orchestrator fails closed at every transition:

- A nonce-bound preflight manifest supplies the only MACs the HTTP server accepts.
- Each MAC is bound to its reported compiled hardware family and variant.
- Every binary contains a linker-retained fleet identity with its family, variant,
  protocol, and Tubes release. The server rejects stale or misbuilt artifacts.
- The server supplies `Content-Length` and `x-MD5`; ESP32 `HTTPUpdate` verifies the
  complete body before activating the OTA partition.
- A canary must download, reboot, rejoin the mesh, report the target release, and
  preserve its LED bus shape before the wildcard wave is sent.
- A server-side completed response is only transfer evidence. The final fresh
  manifest is the success authority for the whole fleet.
- Failed devices retain their current OTA partition. Temporary Wi-Fi credentials
  live only in RAM and the previous in-memory station profile is restored on failure.

The local HTTP transport is intentionally consistent with WLED's firewall-isolated
deployment model. MD5 detects corruption but is not authentication. Run the update
network as a dedicated WPA2 network with client isolation disabled, and do not expose
the server outside that network.

## Network requirements

Use a real access point or travel router with enough DHCP leases and associated-client
capacity for the fleet. Connect the update host by Ethernet, reserve a stable IPv4
address, use 2.4 GHz, and keep the combined SSID and password at 22 UTF-8 bytes or
fewer for protocol version 1. A short dedicated profile such as `TubesOTA` is better
than borrowing venue Wi-Fi.

A pole can act as a temporary AP for a one-device diagnostic, which is how the first
physical pull was proven. It is not the 20-50 device plan: one ESP32 radio has limited
station capacity and would have to carry every client's traffic. Multiple peer APs
would also require multiple host radios and routing domains, adding more failure
states than a small dedicated access point removes.

Firmware distribution over ESP-NOW was rejected for this version. Legacy ESP-NOW
payloads are capped at 250 bytes and use a 1 Mbit/s default PHY, before adding chunk
identity, acknowledgements, retransmission, flash backpressure, and mesh relaying.
It would consume the same radio used for Control traffic while delivering a slower
and much more complex updater.

The transport choices follow Espressif's official
[ESP-NOW limits](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/network/esp_now.html)
and Arduino ESP32's
[HTTPUpdate implementation](https://github.com/espressif/arduino-esp32/blob/master/libraries/HTTPUpdate/src/HTTPUpdate.cpp),
which reads `x-MD5` and passes it to `Update.setMD5()` before streaming the body.

## Operation

Build every hardware profile present in the manifest, then run:

```sh
python3 usermods/Tubes/fleet_pull_update.py \
  /dev/tty.usbserial-8310 \
  --advertise 192.168.0.101 \
  --ssid TubesOTA
```

On macOS the tool reads the matching Wi-Fi password from Keychain without printing
it. Other hosts prompt securely; automation can provide
`TUBES_FLEET_WIFI_PASSWORD`. The tool validates artifacts before opening the server,
runs the canary, opens the five-second fleet wave, and verifies stable MACs after
the mesh reforms.

If any visible candidate reports a release below 22, the tool refuses the parallel
wave and lists the MACs requiring the one-time existing updater. That migration gate
prevents an old Control owner or relay from silently partitioning the additive offer.

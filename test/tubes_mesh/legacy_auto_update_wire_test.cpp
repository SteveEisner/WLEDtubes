#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>

#include "legacy_auto_update_wire.h"

namespace {

void expect(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void deployed_receiver_fields_keep_their_offsets() {
  const auto wire = makeLegacyAutoUpdateOfferWire(40, "TubesOTA", "tubes123");
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&wire);
  int32_t version = 0;
  memcpy(&version, bytes, sizeof(version));
  expect(version == 40, "release changed on the wire");
  expect(strcmp(reinterpret_cast<const char*>(bytes + 4), "TubesOTA") == 0,
      "legacy receiver did not find SSID at byte 4");
  expect(strcmp(reinterpret_cast<const char*>(bytes + 29), "tubes123") == 0,
      "legacy receiver did not find password at byte 29");
  for (size_t index = 54; index < sizeof(wire); index++)
    expect(bytes[index] == 0, "ignored historical host bytes were not zero");
}

void credentials_are_bounded_and_terminated() {
  const auto wire = makeLegacyAutoUpdateOfferWire(
      40, "12345678901234567890123456789", "abcdefghijklmnopqrstuvwxyz");
  expect(wire.ssid[24] == '\0', "SSID was not terminated");
  expect(wire.password[24] == '\0', "password was not terminated");
  expect(strlen(wire.ssid) == 24, "SSID bound changed");
  expect(strlen(wire.password) == 24, "password bound changed");
}

void neighbor_recipient_bypasses_legacy_uplink_election() {
  static_assert(RECIPIENTS_NEIGHBORS == 2,
      "current neighbor wire value no longer matches deployed INFO");
  MeshNodeHeader receiver;
  receiver.id = 0x0A11;
  receiver.uplinkId = 0x0B22;
  NodeMessage wake;
  wake.header.id = 0x1A2B;
  wake.recipients = RECIPIENTS_NEIGHBORS;
  const MeshRoutePlan route = planMeshRoute(receiver, true, false, wake);
  expect(route.accepted, "neighbor wake depended on the receiver's uplink");
  expect(route.applyLocally, "neighbor wake was not applied locally");
  expect(!route.relay, "one-hop legacy wake was relayed");
}

} // namespace

int main() {
  try {
    deployed_receiver_fields_keep_their_offsets();
    std::cout << "PASS: deployed receiver fields keep their offsets\n";
    credentials_are_bounded_and_terminated();
    std::cout << "PASS: legacy credentials are bounded and terminated\n";
    neighbor_recipient_bypasses_legacy_uplink_election();
    std::cout << "PASS: neighbor wake bypasses legacy uplink election\n";
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
  return 0;
}

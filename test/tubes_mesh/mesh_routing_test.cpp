#include <algorithm>
#include <array>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "mesh_test_harness.h"

namespace {

constexpr CommandId TEST_BEAT_COMMAND = 0x50;
constexpr MeshId ROOT_ID = 300;
constexpr MeshId RELAY_ID = 200;
constexpr MeshId LEAF_ID = 100;

void expect(bool condition, const std::string& message) {
    if (!condition)
        throw std::runtime_error(message);
}

struct ThreeDeviceMesh {
    MeshTestNetwork network;
    MeshTestDevice& root;
    MeshTestDevice& relay;
    MeshTestDevice& leaf;

    ThreeDeviceMesh()
      : root(network.addDevice(ROOT_ID, 0, false)),
        relay(network.addDevice(RELAY_ID, ROOT_ID, false)),
        leaf(network.addDevice(LEAF_ID, RELAY_ID, false)) {
        network.connect(ROOT_ID, RELAY_ID);
        network.connect(RELAY_ID, LEAF_ID);
        root.localTimebase = 3000;
        relay.localTimebase = 2000;
        leaf.localTimebase = 1000;
    }
};

std::array<uint8_t, MESSAGE_DATA_SIZE> distinctivePayload() {
    std::array<uint8_t, MESSAGE_DATA_SIZE> payload{};
    for (uint8_t index = 0; index < payload.size(); index++)
        payload[index] = uint8_t(index * 3 + 1);
    return payload;
}

void expectPayload(
    const NodeMessage& message,
    const std::array<uint8_t, MESSAGE_DATA_SIZE>& expected,
    const std::string& recipient
) {
    expect(
        std::equal(expected.begin(), expected.end(), message.data),
        recipient + " received a corrupted payload"
    );
}

// When a leaf requests a command through one relay, only the root applies the upward
// request, then all followers apply the root's downward declaration exactly once.
void leaf_request_round_trips_through_root() {
    ThreeDeviceMesh mesh;
    auto payload = distinctivePayload();

    mesh.leaf.sendRootRequest(TEST_BEAT_COMMAND, payload.data(), payload.size());
    mesh.network.deliverAll();

    expect(mesh.root.appliedMessages.size() == 1, "root did not apply the request once");
    expect(mesh.relay.appliedMessages.size() == 1, "relay applied outside the declaration trip");
    expect(mesh.leaf.appliedMessages.size() == 1, "leaf did not apply the declaration once");
    expectPayload(mesh.root.appliedMessages.front(), payload, "root");
    expectPayload(mesh.relay.appliedMessages.front(), payload, "relay");
    expectPayload(mesh.leaf.appliedMessages.front(), payload, "leaf");

    expect(mesh.network.transmissions.size() == 4, "request did not use exactly four tree transmissions");
    expect(mesh.network.transmissions[0].senderId == LEAF_ID, "leaf did not originate the request");
    expect(mesh.network.transmissions[0].message.recipients == RECIPIENTS_ROOT, "leaf request was not root-bound");
    expect(mesh.network.transmissions[1].senderId == RELAY_ID, "relay did not forward the request upward");
    expect(mesh.network.transmissions[1].message.recipients == RECIPIENTS_ROOT, "upward relay changed request direction");
    expect(mesh.network.transmissions[2].senderId == ROOT_ID, "root did not originate the declaration");
    expect(mesh.network.transmissions[2].message.recipients == RECIPIENTS_ALL, "root did not declare to followers");
    expect(mesh.network.transmissions[3].senderId == RELAY_ID, "relay did not forward the declaration downward");
    expect(mesh.network.transmissions[3].message.recipients == RECIPIENTS_ALL, "downward relay changed declaration direction");

    expect(mesh.root.clockSynchronizations == 0, "upward request changed the root clock");
    expect(mesh.relay.clockSynchronizations == 1, "relay did not sync from the root declaration");
    expect(mesh.leaf.clockSynchronizations == 1, "leaf did not sync from the root declaration");
}

// When the root rejects a request, the upward packet stops there and no follower sees
// a declaration, so application validation remains the authorization boundary.
void rejected_root_request_is_not_declared() {
    ThreeDeviceMesh mesh;
    auto payload = distinctivePayload();
    mesh.root.acceptCommands = false;

    mesh.leaf.sendRootRequest(TEST_BEAT_COMMAND, payload.data(), payload.size());
    mesh.network.deliverAll();

    expect(mesh.root.commandAttempts.size() == 1, "root did not inspect the request");
    expect(mesh.root.appliedMessages.empty(), "root applied a rejected request");
    expect(mesh.relay.appliedMessages.empty(), "relay applied a request rejected by the root");
    expect(mesh.leaf.appliedMessages.empty(), "leaf applied a request rejected by the root");
    expect(mesh.network.transmissions.size() == 2, "rejected request was broadcast downward");
}

// An all-recipient packet from any device except the selected uplink is ignored, which
// prevents a leaf from bypassing root declaration by broadcasting directly.
void declaration_from_non_uplink_is_ignored() {
    ThreeDeviceMesh mesh;
    auto payload = distinctivePayload();
    NodeMessage forgedDeclaration;
    forgedDeclaration.header = mesh.leaf.header;
    forgedDeclaration.recipients = RECIPIENTS_ALL;
    forgedDeclaration.command = TEST_BEAT_COMMAND;
    std::copy(payload.begin(), payload.end(), forgedDeclaration.data);

    mesh.network.broadcast(mesh.leaf, forgedDeclaration);
    mesh.network.deliverAll();

    expect(mesh.root.commandAttempts.empty(), "root accepted a declaration from a non-uplink");
    expect(mesh.relay.commandAttempts.empty(), "relay accepted a declaration from a non-uplink");
    expect(mesh.leaf.commandAttempts.empty(), "leaf unexpectedly received its own declaration");
    expect(mesh.network.transmissions.size() == 1, "ignored declaration was relayed");
}

} // namespace

int main() {
    const std::array<std::pair<const char*, void (*)()>, 3> tests = {{
        {"leaf request round trips through root", leaf_request_round_trips_through_root},
        {"rejected root request is not declared", rejected_root_request_is_not_declared},
        {"declaration from non-uplink is ignored", declaration_from_non_uplink_is_ignored},
    }};

    for (const auto& test : tests) {
        try {
            test.second();
            std::cout << "PASS: " << test.first << '\n';
        } catch (const std::exception& error) {
            std::cerr << "FAIL: " << test.first << ": " << error.what() << '\n';
            return 1;
        }
    }
    return 0;
}

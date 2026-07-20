#pragma once

#include <stdint.h>

#define CURRENT_NODE_VERSION 2
#define MESSAGE_DATA_SIZE 64

typedef uint8_t CommandId;

#pragma pack(push,4)
// The four-byte packing is part of the deployed wire format.
typedef enum {
    RECIPIENTS_ALL=0,
    RECIPIENTS_ROOT=1,
    RECIPIENTS_INFO=2,
} MessageRecipients;

typedef uint16_t MeshId;

struct MeshNodeHeader {
    MeshId id = 0;
    MeshId uplinkId = 0;
    uint8_t version = CURRENT_NODE_VERSION;
};

struct NodeMessage {
    MeshNodeHeader header;
    MessageRecipients recipients;
    uint32_t timebase;
    CommandId command;
    uint8_t data[MESSAGE_DATA_SIZE] = {0};
};
#pragma pack(pop)

static_assert(sizeof(NodeMessage) == 84, "NodeMessage wire size changed");

struct MeshRoutePlan {
    bool accepted;
    bool applyLocally;
    bool relay;
    bool synchronizeClock;
};

// A child naming this node as its uplink needs this node to carry traffic. A rootless
// lower-ID neighbor also needs help discovering the stronger leader this node can reach.
inline bool shouldRenewRelayDuty(
    const MeshNodeHeader& localHeader,
    const MeshNodeHeader& peerHeader
) {
    return peerHeader.uplinkId == localHeader.id
        || (peerHeader.uplinkId == 0 && peerHeader.id < localHeader.id);
}

// Decide how one accepted wire packet moves through the current root-oriented tree.
// Root requests travel upward without being applied by intermediate relays. The root
// applies the request, then changes it into an all-recipient declaration for the
// downward trip. Only declarations from the selected uplink may set a follower's clock.
inline MeshRoutePlan planMeshRoute(
    const MeshNodeHeader& localHeader,
    bool localIsFollowing,
    bool localIsLeading,
    const NodeMessage& message
) {
    bool accepted = false;
    switch (message.recipients) {
        case RECIPIENTS_ALL:
            accepted = message.header.id == localHeader.uplinkId;
            break;

        case RECIPIENTS_ROOT:
            accepted = message.header.uplinkId == localHeader.id;
            break;

        case RECIPIENTS_INFO:
            accepted = true;
            break;

        default:
            return {false, false, false, false};
    }

    if (!accepted)
        return {false, false, false, false};

    bool applyLocally = message.recipients != RECIPIENTS_ROOT || !localIsFollowing;
    bool relay = localIsLeading && message.recipients != RECIPIENTS_INFO;
    bool synchronizeClock = applyLocally && message.recipients == RECIPIENTS_ALL;
    return {true, applyLocally, relay, synchronizeClock};
}

// Preserve the command and payload while making the forwarding node the next-hop sender.
// A root converts an upward request into the authoritative downward declaration.
inline NodeMessage makeRelayedMessage(
    const MeshNodeHeader& localHeader,
    bool localIsFollowing,
    const NodeMessage& received
) {
    NodeMessage relayed = received;
    relayed.header = localHeader;
    if (!localIsFollowing)
        relayed.recipients = RECIPIENTS_ALL;
    return relayed;
}

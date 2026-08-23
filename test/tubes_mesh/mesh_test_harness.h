#pragma once

#include <algorithm>
#include <cstdint>
#include <deque>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

#include "mesh_protocol.h"

class MeshTestNetwork;

class MeshTestDevice {
  public:
    MeshNodeHeader header;
    bool leading = false;
    bool acceptCommands = true;
    uint32_t localTimebase = 0;
    uint32_t clockSynchronizations = 0;
    std::vector<NodeMessage> commandAttempts;
    std::vector<NodeMessage> appliedMessages;

    MeshTestDevice(MeshTestNetwork& network, DeviceId id, DeviceId uplinkId, bool leading);

    bool isFollowing() const {
        return header.uplinkId != 0;
    }

    void sendRootRequest(CommandId command, const uint8_t* data, uint8_t length);
    void receive(const NodeMessage& message);

  private:
    MeshTestNetwork& network;
};

class MeshTestNetwork {
  public:
    struct Transmission {
        DeviceId senderId;
        NodeMessage message;
    };

    MeshTestDevice& addDevice(DeviceId id, DeviceId uplinkId, bool leading) {
        auto result = devices.try_emplace(id, *this, id, uplinkId, leading);
        if (!result.second)
            throw std::runtime_error("duplicate mesh device ID");
        return result.first->second;
    }

    void connect(DeviceId first, DeviceId second) {
        neighbors[first].push_back(second);
        neighbors[second].push_back(first);
    }

    void broadcast(const MeshTestDevice& sender, NodeMessage message) {
        // Each forwarding hop stamps its own clock, matching `LightNode::broadcastMessage`.
        message.timebase = sender.localTimebase;
        pending.push_back({sender.header.id, message});
    }

    void deliverAll() {
        uint16_t deliveries = 0;
        while (!pending.empty()) {
            if (++deliveries > 100)
                throw std::runtime_error("mesh delivery did not converge");

            Transmission transmission = pending.front();
            pending.pop_front();
            transmissions.push_back(transmission);

            for (DeviceId neighborId : neighbors.at(transmission.senderId))
                devices.at(neighborId).receive(transmission.message);
        }
    }

    std::vector<Transmission> transmissions;

  private:
    // Ordered maps keep device references stable as a scenario adds more nodes.
    std::map<DeviceId, MeshTestDevice> devices;
    std::map<DeviceId, std::vector<DeviceId>> neighbors;
    std::deque<Transmission> pending;
};

inline MeshTestDevice::MeshTestDevice(
    MeshTestNetwork& network,
    DeviceId id,
    DeviceId uplinkId,
    bool leading
) : leading(leading), network(network) {
    header.id = id;
    header.uplinkId = uplinkId;
}

inline void MeshTestDevice::sendRootRequest(
    CommandId command,
    const uint8_t* data,
    uint8_t length
) {
    if (length > MESSAGE_DATA_SIZE)
        throw std::runtime_error("test command payload is too large");

    NodeMessage message;
    message.header = header;
    message.recipients = isFollowing() ? RECIPIENTS_ROOT : RECIPIENTS_ALL;
    message.command = command;
    std::copy(data, data + length, message.data);
    network.broadcast(*this, message);
}

inline void MeshTestDevice::receive(const NodeMessage& message) {
    if (shouldRenewRelayDuty(header, message.header))
        leading = true;

    MeshRoutePlan route = planMeshRoute(header, isFollowing(), leading, message);
    if (!route.accepted)
        return;

    // Application acceptance at the root authorizes the downward declaration.
    if (route.applyLocally) {
        commandAttempts.push_back(message);
        if (!acceptCommands)
            return;
        appliedMessages.push_back(message);
        if (route.synchronizeClock)
            clockSynchronizations++;
    }

    if (route.relay) {
        NodeMessage relayed = makeRelayedMessage(header, isFollowing(), message);
        network.broadcast(*this, relayed);
    }
}

#pragma once

#include <stddef.h>
#include <stdint.h>

constexpr size_t TUBES_PEER_TELEMETRY_CAPACITY = 24;

struct PeerTelemetryEntry {
  uint16_t nodeId = 0;
  uint16_t uplinkId = 0;
  uint32_t lastSeenMs = 0;
  uint32_t samples = 0;
  int16_t smoothedRssi = 0;
  int8_t latestRssi = 0;
  int8_t minimumRssi = 0;
  int8_t maximumRssi = 0;
  uint8_t protocolGeneration = 0;
  uint16_t tubesVersion = 0;
  bool rssiKnown = false;
};

class PeerTelemetry {
public:
  void observe(uint16_t nodeId, uint16_t uplinkId, uint8_t generation, int8_t rssi, uint32_t now) {
    if (nodeId == 0) return;
    PeerTelemetryEntry *entry = find(nodeId);
    if (entry == nullptr) entry = allocate(now);
    if (entry->nodeId != nodeId) *entry = PeerTelemetryEntry{};
    entry->nodeId = nodeId;
    entry->uplinkId = uplinkId;
    entry->protocolGeneration = generation;
    entry->lastSeenMs = now;
    if (entry->samples != UINT32_MAX) entry->samples++;
    if (rssi == 0) return;
    entry->latestRssi = rssi;
    if (!entry->rssiKnown) {
      entry->minimumRssi = rssi;
      entry->maximumRssi = rssi;
      entry->smoothedRssi = rssi;
      entry->rssiKnown = true;
      return;
    }
    if (rssi < entry->minimumRssi) entry->minimumRssi = rssi;
    if (rssi > entry->maximumRssi) entry->maximumRssi = rssi;
    entry->smoothedRssi = static_cast<int16_t>((entry->smoothedRssi * 3 + rssi) / 4);
  }

  void observeIdentity(uint16_t nodeId, uint16_t uplinkId, uint16_t tubesVersion,
                       uint32_t now) {
    if (nodeId == 0) return;
    PeerTelemetryEntry *entry = find(nodeId);
    if (entry == nullptr) entry = allocate(now);
    if (entry->nodeId != nodeId) *entry = PeerTelemetryEntry{};
    entry->nodeId = nodeId;
    entry->uplinkId = uplinkId;
    entry->tubesVersion = tubesVersion;
    entry->lastSeenMs = now;
  }

  size_t count() const {
    size_t used = 0;
    for (const PeerTelemetryEntry &entry : entries) if (entry.nodeId != 0) used++;
    return used;
  }

  size_t freshCount(uint32_t now, uint32_t maximumAgeMs) const {
    size_t used = 0;
    for (const PeerTelemetryEntry &entry : entries)
      if (entry.nodeId != 0 && now - entry.lastSeenMs <= maximumAgeMs) used++;
    return used;
  }

  const PeerTelemetryEntry *entry(size_t index) const {
    size_t used = 0;
    for (const PeerTelemetryEntry &candidate : entries) {
      if (candidate.nodeId == 0) continue;
      if (used++ == index) return &candidate;
    }
    return nullptr;
  }

  const PeerTelemetryEntry *get(uint16_t nodeId) const {
    for (const PeerTelemetryEntry &candidate : entries) if (candidate.nodeId == nodeId) return &candidate;
    return nullptr;
  }

private:
  PeerTelemetryEntry entries[TUBES_PEER_TELEMETRY_CAPACITY] = {};

  PeerTelemetryEntry *find(uint16_t nodeId) {
    for (PeerTelemetryEntry &entry : entries) if (entry.nodeId == nodeId) return &entry;
    return nullptr;
  }

  PeerTelemetryEntry *allocate(uint32_t now) {
    PeerTelemetryEntry *oldest = &entries[0];
    uint32_t oldestAge = 0;
    for (PeerTelemetryEntry &entry : entries) {
      if (entry.nodeId == 0) return &entry;
      const uint32_t age = now - entry.lastSeenMs;
      if (age >= oldestAge) { oldest = &entry; oldestAge = age; }
    }
    return oldest;
  }
};

#pragma once

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "fleet_update_protocol.h"

// Durable, one-shot evidence that this boot was reached through Steve's modern
// FleetUpdateOffer updater. Legacy COMMAND_UPGRADE never creates this record.
constexpr uint32_t MODERN_PROPAGATION_LEASE_MAGIC = 0x31504C54; // "TLP1"
constexpr uint8_t MODERN_PROPAGATION_LEASE_VERSION = 1;

enum ModernPropagationLeaseState : uint8_t {
  ModernPropagationLeaseEmpty = 0,
  ModernPropagationLeaseArmed = 1,
  ModernPropagationLeaseClaimed = 2,
};

#pragma pack(push, 1)
struct ModernPropagationLeaseRecord {
  uint32_t magic = MODERN_PROPAGATION_LEASE_MAGIC;
  uint8_t formatVersion = MODERN_PROPAGATION_LEASE_VERSION;
  uint8_t state = ModernPropagationLeaseEmpty;
  uint16_t tubesVersion = 0;
  uint32_t sourceNonce = 0;
  uint32_t checksum = 0;
};
#pragma pack(pop)

static_assert(sizeof(ModernPropagationLeaseRecord) == 16,
    "modern propagation lease wire size changed");

inline uint32_t modernPropagationLeaseChecksum(
    const ModernPropagationLeaseRecord& record
) {
  uint32_t hash = 2166136261UL;
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&record);
  for (size_t index = 0; index < sizeof(record) - sizeof(record.checksum); index++)
    hash = (hash ^ bytes[index]) * 16777619UL;
  return hash;
}

inline bool isValidModernPropagationLease(
    const ModernPropagationLeaseRecord& record
) {
  return record.magic == MODERN_PROPAGATION_LEASE_MAGIC
      && record.formatVersion == MODERN_PROPAGATION_LEASE_VERSION
      && (record.state == ModernPropagationLeaseArmed
          || record.state == ModernPropagationLeaseClaimed)
      && record.tubesVersion > 0
      && record.sourceNonce != 0
      && record.checksum == modernPropagationLeaseChecksum(record);
}

inline bool shouldArmModernPropagationLease(
    const FleetUpdateOffer& offer,
    uint16_t runningVersion
) {
  return isValidFleetUpdateOffer(offer)
      && (offer.flags & FleetUpdatePropagate)
      && offer.tubesVersion > runningVersion;
}

// Deployed legacy firmware cannot write a modern lease before reboot. Its
// freshly installed image can still hear the predecessor's continuing,
// propagation-marked download offer and use that equal-release offer as the
// baton. The boot window keeps established current devices out of later waves.
inline bool isFreshLegacyBootstrapBaton(
    const FleetUpdateOffer& offer,
    uint16_t runningVersion,
    uint32_t uptimeMs,
    uint32_t bootWindowMs,
    bool legacyMigrationBoot
) {
  return isValidFleetUpdateOffer(offer)
      && legacyMigrationBoot
      && (offer.flags & FleetUpdatePropagate)
      && offer.serverPort != 0
      && offer.targetDeviceId == 0
      && offer.tubesVersion == runningVersion
      && uptimeMs <= bootWindowMs;
}

inline bool makeModernPropagationSessionSSID(
    char* destination,
    size_t capacity,
    uint32_t nonce
) {
  if (!destination || capacity < 15 || nonce == 0) return false;
  // Steve's v1 offer has 22 combined credential bytes. Fourteen bytes here
  // leave the unchanged eight-byte Tubes password intact.
  return snprintf(destination, capacity, "Tubes-%08lX",
      static_cast<unsigned long>(nonce)) == 14;
}

inline ModernPropagationLeaseRecord makeModernPropagationLease(
    const FleetUpdateOffer& offer
) {
  ModernPropagationLeaseRecord record;
  record.state = ModernPropagationLeaseArmed;
  record.tubesVersion = offer.tubesVersion;
  record.sourceNonce = offer.nonce;
  record.checksum = modernPropagationLeaseChecksum(record);
  return record;
}

inline bool claimModernPropagationLease(
    ModernPropagationLeaseRecord& record,
    uint16_t runningVersion
) {
  if (!isValidModernPropagationLease(record)
      || record.state != ModernPropagationLeaseArmed
      || record.tubesVersion != runningVersion)
    return false;
  record.state = ModernPropagationLeaseClaimed;
  record.checksum = modernPropagationLeaseChecksum(record);
  return true;
}

// A propagation turn stays on Steve's existing download offer. It is wildcard
// and non-forced so only genuinely older peers enter the updater.
inline bool makeModernPropagationOffer(
    FleetUpdateOffer& offer,
    uint16_t runningVersion,
    uint32_t nonce,
    const uint8_t serverAddress[4],
    uint16_t serverPort,
    uint16_t startWindowMs,
    const char* ssid,
    const char* password
) {
  offer = FleetUpdateOffer();
  offer.flags = FleetUpdatePropagate;
  offer.tubesVersion = runningVersion;
  offer.nonce = nonce;
  memcpy(offer.serverAddress, serverAddress, sizeof(offer.serverAddress));
  offer.serverPort = serverPort;
  offer.startWindowMs = startWindowMs;
  offer.targetDeviceId = 0;
  return setFleetUpdateCredentials(offer, ssid, password)
      && isValidFleetUpdateOffer(offer)
      && offer.flags == FleetUpdatePropagate;
}

// Exact-target command that asks an already-current device to take one host
// turn. It contains no download server or credentials and never reinstalls.
inline bool makeModernPropagationServeCommand(
    FleetUpdateOffer& command,
    uint16_t runningVersion,
    uint32_t nonce,
    DeviceId targetDeviceId
) {
  command = FleetUpdateOffer();
  command.flags = FleetUpdatePropagate;
  command.tubesVersion = runningVersion;
  command.nonce = nonce;
  command.serverPort = 0;
  command.targetDeviceId = targetDeviceId;
  return isValidFleetUpdateOffer(command);
}

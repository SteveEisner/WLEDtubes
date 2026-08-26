#pragma once

#include "wled.h"
#include "modern_propagation_lease.h"

constexpr char MODERN_PROPAGATION_LEASE_PATH[] = "/tubes-propagate.bin";
constexpr char MODERN_PROPAGATION_LEASE_TEMP_PATH[] = "/tubes-propagate.tmp";
constexpr char CURRENT_RELEASE_MARKER_PATH[] = "/tubes-current.bin";
constexpr char CURRENT_RELEASE_MARKER_TEMP_PATH[] = "/tubes-current.tmp";
constexpr uint32_t CURRENT_RELEASE_MARKER_MAGIC = 0x31524354; // "TCR1"

#pragma pack(push, 1)
struct CurrentReleaseMarker {
  uint32_t magic = CURRENT_RELEASE_MARKER_MAGIC;
  uint16_t tubesVersion = 0;
  uint16_t invertedVersion = 0;
};
#pragma pack(pop)

static_assert(sizeof(CurrentReleaseMarker) == 8,
    "current release marker size changed");

inline bool hasCurrentReleaseMarker(uint16_t runningVersion) {
  CurrentReleaseMarker marker;
  File file = WLED_FS.open(CURRENT_RELEASE_MARKER_PATH, "r");
  if (!file) return false;
  const bool read = file.size() == sizeof(marker)
      && file.read(reinterpret_cast<uint8_t*>(&marker), sizeof(marker)) == sizeof(marker);
  file.close();
  return read && marker.magic == CURRENT_RELEASE_MARKER_MAGIC
      && marker.tubesVersion == runningVersion
      && marker.invertedVersion == static_cast<uint16_t>(~runningVersion);
}

inline bool writeCurrentReleaseMarker(uint16_t runningVersion) {
  CurrentReleaseMarker marker;
  marker.tubesVersion = runningVersion;
  marker.invertedVersion = static_cast<uint16_t>(~runningVersion);
  File file = WLED_FS.open(CURRENT_RELEASE_MARKER_TEMP_PATH, "w");
  if (!file) return false;
  const bool written = file.write(
      reinterpret_cast<const uint8_t*>(&marker), sizeof(marker)) == sizeof(marker);
  file.close();
  if (!written) {
    WLED_FS.remove(CURRENT_RELEASE_MARKER_TEMP_PATH);
    return false;
  }
  WLED_FS.remove(CURRENT_RELEASE_MARKER_PATH);
  return WLED_FS.rename(CURRENT_RELEASE_MARKER_TEMP_PATH,
      CURRENT_RELEASE_MARKER_PATH);
}

inline bool writeModernPropagationLease(
    const ModernPropagationLeaseRecord& record
) {
  if (!isValidModernPropagationLease(record)) return false;
  File file = WLED_FS.open(MODERN_PROPAGATION_LEASE_TEMP_PATH, "w");
  if (!file) return false;
  const bool written = file.write(
      reinterpret_cast<const uint8_t*>(&record), sizeof(record)) == sizeof(record);
  file.close();
  if (!written) {
    WLED_FS.remove(MODERN_PROPAGATION_LEASE_TEMP_PATH);
    return false;
  }
  WLED_FS.remove(MODERN_PROPAGATION_LEASE_PATH);
  return WLED_FS.rename(
      MODERN_PROPAGATION_LEASE_TEMP_PATH, MODERN_PROPAGATION_LEASE_PATH);
}

inline bool readModernPropagationLease(ModernPropagationLeaseRecord& record) {
  File file = WLED_FS.open(MODERN_PROPAGATION_LEASE_PATH, "r");
  if (!file) return false;
  const bool read = file.size() == sizeof(record)
      && file.read(reinterpret_cast<uint8_t*>(&record), sizeof(record)) == sizeof(record);
  file.close();
  return read && isValidModernPropagationLease(record);
}

inline void clearModernPropagationLease() {
  WLED_FS.remove(MODERN_PROPAGATION_LEASE_TEMP_PATH);
  WLED_FS.remove(MODERN_PROPAGATION_LEASE_PATH);
}

// Claim is persisted before the host turn starts. A reset during that turn will
// therefore not amplify the same update repeatedly.
inline bool claimStoredModernPropagationLease(
    ModernPropagationLeaseRecord& record,
    uint16_t runningVersion
) {
  if (!readModernPropagationLease(record))
    return false;
  // Claimed means a prior boot already consumed the one shot and reset before
  // cleanup. Remove it instead of repeating or retaining a stale lease.
  if (record.state == ModernPropagationLeaseClaimed) {
    clearModernPropagationLease();
    return false;
  }
  if (!claimModernPropagationLease(record, runningVersion)
      || !writeModernPropagationLease(record)) {
    clearModernPropagationLease();
    return false;
  }
  return true;
}

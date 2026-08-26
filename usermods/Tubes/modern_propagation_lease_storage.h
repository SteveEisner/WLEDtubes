#pragma once

#include "wled.h"
#include "modern_propagation_lease.h"

constexpr char MODERN_PROPAGATION_LEASE_PATH[] = "/tubes-propagate.bin";
constexpr char MODERN_PROPAGATION_LEASE_TEMP_PATH[] = "/tubes-propagate.tmp";

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

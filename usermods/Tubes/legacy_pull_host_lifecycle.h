#pragma once

#include <stdint.h>

enum LegacyPullHostRestoreReason : uint8_t {
  LegacyPullHostKeepServing = 0,
  LegacyPullHostRestoreRequested,
  LegacyPullHostReadFailed,
  LegacyPullHostStreamStalled,
  LegacyPullHostAllSlotsComplete,
  LegacyPullHostSecondReceiverGraceElapsed,
  LegacyPullHostAssociatedWithoutRequest,
  LegacyPullHostRendezvousTimedOut,
};

struct LegacyPullHostLifecycle {
  uint32_t startedAt = 0;
  uint32_t stationSeenAt = 0;
  uint32_t lastProgressAt = 0;
  uint32_t completedAt = 0;
  bool restoreRequested = false;
  bool readFailed = false;
  bool stationSeen = false;
  bool requestSeen = false;
  bool incompleteRequest = false;
  bool bodyComplete = false;
  bool allLifetimeSlotsUsed = false;
};

inline bool legacyPullDeadlineReached(uint32_t now, uint32_t then, uint32_t timeout) {
  return static_cast<int32_t>(now - then) >= static_cast<int32_t>(timeout);
}

inline LegacyPullHostRestoreReason legacyPullHostRestoreReason(
    const LegacyPullHostLifecycle& state,
    uint32_t now,
    uint32_t requestTimeoutMs,
    uint32_t streamIdleTimeoutMs,
    uint32_t associatedRequestTimeoutMs,
    uint32_t finalResponseDrainMs,
    uint32_t secondReceiverGraceMs
) {
  if (state.restoreRequested) return LegacyPullHostRestoreRequested;
  if (state.readFailed) return LegacyPullHostReadFailed;
  if (state.incompleteRequest) {
    if (legacyPullDeadlineReached(now, state.lastProgressAt, streamIdleTimeoutMs))
      return LegacyPullHostStreamStalled;
    return LegacyPullHostKeepServing;
  }
  // AsyncAbstractResponse reports complete when the final source bytes have
  // entered its TCP send buffer. Keep the AP alive briefly so the last client
  // can consume them, verify the image, and commit its OTA slot before teardown.
  if (state.bodyComplete && state.allLifetimeSlotsUsed) {
    if (legacyPullDeadlineReached(now, state.completedAt, finalResponseDrainMs))
      return LegacyPullHostAllSlotsComplete;
    return LegacyPullHostKeepServing;
  }
  if (state.bodyComplete) {
    if (legacyPullDeadlineReached(now, state.completedAt, secondReceiverGraceMs))
      return LegacyPullHostSecondReceiverGraceElapsed;
    return LegacyPullHostKeepServing;
  }
  if (state.stationSeen && !state.requestSeen) {
    if (legacyPullDeadlineReached(now, state.stationSeenAt, associatedRequestTimeoutMs))
      return LegacyPullHostAssociatedWithoutRequest;
    return LegacyPullHostKeepServing;
  }
  if (legacyPullDeadlineReached(now, state.startedAt, requestTimeoutMs))
    return LegacyPullHostRendezvousTimedOut;
  return LegacyPullHostKeepServing;
}

inline bool legacyPullPropagationTurnFinished(
    bool modernTurn,
    bool restoreStarted,
    bool meshRestored,
    bool hostRetired,
    bool restoreNeeded,
    bool bodyServed
) {
  return modernTurn && restoreStarted && meshRestored
      && (hostRetired || (restoreNeeded && !bodyServed));
}

inline bool legacyPullCanAcceptExplicitTurn(bool modernTurnActive) {
  return !modernTurnActive;
}

inline bool legacyPullAutomaticHostEligible(
    bool bootEligible,
    bool modernTurnActive,
    bool offerSent,
    bool hostRetired
) {
  return (bootEligible || modernTurnActive) && !offerSent && !hostRetired;
}

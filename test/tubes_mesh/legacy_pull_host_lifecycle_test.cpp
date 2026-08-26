#include <stdexcept>
#include <string>

#include "legacy_pull_host_lifecycle.h"

namespace {

void expect(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

constexpr uint32_t REQUEST_TIMEOUT = 360000;
constexpr uint32_t STREAM_TIMEOUT = 20000;
constexpr uint32_t ASSOCIATED_TIMEOUT = 20000;
constexpr uint32_t SECOND_GRACE = 60000;

LegacyPullHostRestoreReason reason(const LegacyPullHostLifecycle& state, uint32_t now) {
  return legacyPullHostRestoreReason(state, now, REQUEST_TIMEOUT, STREAM_TIMEOUT,
      ASSOCIATED_TIMEOUT, SECOND_GRACE);
}

void associatedWithoutRequestRecoversBoundedly() {
  LegacyPullHostLifecycle state;
  state.startedAt = 100;
  state.stationSeen = true;
  state.stationSeenAt = 1000;
  expect(reason(state, 1000 + ASSOCIATED_TIMEOUT - 1) == LegacyPullHostKeepServing,
      "associated receiver was evicted before its request window ended");
  expect(reason(state, 1000 + ASSOCIATED_TIMEOUT) == LegacyPullHostAssociatedWithoutRequest,
      "associated receiver pinned the host until the rendezvous timeout");
}

void bothCompletedLifetimeSlotsRestoreImmediately() {
  LegacyPullHostLifecycle state;
  state.startedAt = 100;
  state.requestSeen = true;
  state.bodyComplete = true;
  state.completedAt = 2000;
  state.allLifetimeSlotsUsed = true;
  expect(reason(state, 2000) == LegacyPullHostAllSlotsComplete,
      "two completed lifetime slots waited through second-receiver grace");
}

void oneCompletedSlotRetainsSecondReceiverGrace() {
  LegacyPullHostLifecycle state;
  state.requestSeen = true;
  state.bodyComplete = true;
  state.completedAt = 2000;
  expect(reason(state, 2000 + SECOND_GRACE - 1) == LegacyPullHostKeepServing,
      "single completion lost its second-receiver window");
  expect(reason(state, 2000 + SECOND_GRACE) == LegacyPullHostSecondReceiverGraceElapsed,
      "single completion did not close after second-receiver grace");
}

void activePartialBodyUsesProgressDeadline() {
  LegacyPullHostLifecycle state;
  state.startedAt = 0;
  state.stationSeen = true;
  state.stationSeenAt = 1;
  state.requestSeen = true;
  state.incompleteRequest = true;
  state.lastProgressAt = REQUEST_TIMEOUT + 10;
  expect(reason(state, REQUEST_TIMEOUT + 10) == LegacyPullHostKeepServing,
      "absolute rendezvous timeout interrupted an active body");
  expect(reason(state, REQUEST_TIMEOUT + 10 + STREAM_TIMEOUT)
      == LegacyPullHostStreamStalled, "stalled body did not recover");
}

void deadlinesRemainCorrectAcrossMillisWrap() {
  LegacyPullHostLifecycle state;
  state.startedAt = UINT32_MAX - 10;
  state.stationSeen = true;
  state.stationSeenAt = UINT32_MAX - 10;
  expect(reason(state, ASSOCIATED_TIMEOUT - 12) == LegacyPullHostKeepServing,
      "association timeout fired early across millis wrap");
  expect(reason(state, ASSOCIATED_TIMEOUT - 11) == LegacyPullHostAssociatedWithoutRequest,
      "association timeout failed across millis wrap");
}

void completedAndFailedTurnsBecomeRearmableAfterRestore() {
  expect(legacyPullPropagationTurnFinished(true, true, true, true, true, true),
      "completed turn did not release its explicit-trigger latch");
  expect(legacyPullPropagationTurnFinished(true, true, true, false, true, false),
      "failed turn did not release its explicit-trigger latch");
  expect(!legacyPullPropagationTurnFinished(true, true, false, true, true, true),
      "turn reset before mesh recovery completed");
  expect(!legacyPullPropagationTurnFinished(false, true, true, true, true, true),
      "ordinary legacy host was treated as an explicit modern turn");
}

void terminalTurnCannotAutoRepeatButExplicitTurnCanRearm() {
  expect(!legacyPullAutomaticHostEligible(true, false, true, true),
      "retired PRIME/test turn automatically repeated");
  expect(legacyPullCanAcceptExplicitTurn(false),
      "retired latches incorrectly blocked a fresh human command");
  expect(!legacyPullCanAcceptExplicitTurn(true),
      "overlapping explicit turn was accepted");
  expect(legacyPullAutomaticHostEligible(false, true, false, false),
      "fresh explicit turn did not become host eligible");
}

} // namespace

int main() {
  associatedWithoutRequestRecoversBoundedly();
  bothCompletedLifetimeSlotsRestoreImmediately();
  oneCompletedSlotRetainsSecondReceiverGrace();
  activePartialBodyUsesProgressDeadline();
  deadlinesRemainCorrectAcrossMillisWrap();
  completedAndFailedTurnsBecomeRearmableAfterRestore();
  terminalTurnCannotAutoRepeatButExplicitTurnCanRearm();
  return 0;
}

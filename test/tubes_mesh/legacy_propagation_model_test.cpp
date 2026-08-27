#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr uint16_t CURRENT_RELEASE = 47;
constexpr uint32_t TRANSFER_MS = 3000;
constexpr uint32_t SECOND_RECEIVER_GRACE_MS = 60000;
constexpr uint32_t HOST_TIMEOUT_MS = 360000;

void expect(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

struct Receiver {
  uint16_t id;
  uint16_t release;
  bool updateInProgress = false;
  bool propagationTurnUsed = false;
};

// Deterministic model of the deployed legacy boundary: an offer is useful only
// to an older idle receiver. Equal/older offers are silent no-ops.
bool acceptsLegacyOffer(const Receiver& receiver, uint16_t offeredRelease) {
  return !receiver.updateInProgress && offeredRelease > receiver.release;
}

class SerializedLegacyHost {
public:
  SerializedLegacyHost(uint32_t startedAt, bool dynamic, uint16_t known = 0)
      : _startedAt(startedAt), _dynamic(dynamic), _known(known) {}

  bool offer(Receiver& receiver, uint32_t now) {
    if (_closed || _active || _served.size() == 2) return false;
    if (!_dynamic && receiver.id != _known) return false;
    if (!acceptsLegacyOffer(receiver, CURRENT_RELEASE)) return false;
    receiver.updateInProgress = true;
    _active = &receiver;
    _activeStartedAt = now;
    return true;
  }

  bool complete(uint32_t now) {
    if (!_active || now - _activeStartedAt < TRANSFER_MS) return false;
    _active->release = CURRENT_RELEASE;
    _active->updateInProgress = false;
    _served.push_back(_active->id);
    _active = nullptr;
    _lastCompleteAt = now;
    return true;
  }

  bool shouldClose(uint32_t now) const {
    if (_active) return false;
    if (_served.size() == 2) return true;
    if (!_served.empty() && now - _lastCompleteAt >= SECOND_RECEIVER_GRACE_MS)
      return true;
    return now - _startedAt >= HOST_TIMEOUT_MS;
  }

  void close() { _closed = true; }
  size_t servedCount() const { return _served.size(); }
  bool active() const { return _active != nullptr; }

private:
  uint32_t _startedAt;
  uint32_t _activeStartedAt = 0;
  uint32_t _lastCompleteAt = 0;
  bool _dynamic;
  uint16_t _known;
  bool _closed = false;
  Receiver* _active = nullptr;
  std::vector<uint16_t> _served;
};

void knownReceiverPathIsClosedToUnknownDevices() {
  Receiver b{2, 14};
  Receiver c{3, 14};
  SerializedLegacyHost a(0, false, b.id);
  expect(!a.offer(c, 0), "registered host admitted an unknown receiver");
  expect(a.offer(b, 0), "registered host rejected B");
  expect(!a.complete(TRANSFER_MS - 1), "B completed before its transfer ended");
  expect(a.complete(TRANSFER_MS), "B did not complete its transfer");
  expect(b.release == CURRENT_RELEASE, "B did not reach the offered release");
}

void dynamicEnrollmentAcceptsOnePreviouslyUnknownReceiver() {
  Receiver c{3, 13};
  SerializedLegacyHost a(100, true);
  expect(a.offer(c, 100), "dynamic host rejected C");
  expect(a.complete(100 + TRANSFER_MS), "C did not complete");
  expect(!a.shouldClose(100 + TRANSFER_MS + SECOND_RECEIVER_GRACE_MS - 1),
      "host closed before the second-receiver grace elapsed");
  expect(a.shouldClose(100 + TRANSFER_MS + SECOND_RECEIVER_GRACE_MS),
      "one-receiver host did not close at its bounded grace");
}

void twoLegacyReceiversAreSerializedWithoutDeadlineRaces() {
  Receiver c{3, 13};
  Receiver d{4, 14};
  SerializedLegacyHost a(0, true);
  expect(a.offer(c, 0), "C did not start");
  expect(!a.offer(d, 1), "D overlapped C's deployed legacy stream");
  expect(!a.shouldClose(HOST_TIMEOUT_MS + 1),
      "hard timeout interrupted an active legacy stream");
  expect(a.complete(HOST_TIMEOUT_MS + TRANSFER_MS), "C did not complete");
  expect(a.offer(d, HOST_TIMEOUT_MS + TRANSFER_MS + 1),
      "D could not claim the released serialized slot");
  expect(!a.offer(c, HOST_TIMEOUT_MS + TRANSFER_MS + 2),
      "equal-version C re-entered the transfer");
  expect(a.complete(HOST_TIMEOUT_MS + 2 * TRANSFER_MS + 1), "D did not complete");
  expect(a.servedCount() == 2, "fanout did not retain both completed receivers");
  expect(a.shouldClose(HOST_TIMEOUT_MS + 2 * TRANSFER_MS + 1),
      "full two-receiver host did not close promptly");
}

void equalAndOlderOffersNeverRestartAnUpdater() {
  Receiver current{7, CURRENT_RELEASE};
  Receiver newer{8, uint16_t(CURRENT_RELEASE + 1)};
  expect(!acceptsLegacyOffer(current, CURRENT_RELEASE),
      "equal release restarted the updater");
  expect(!acceptsLegacyOffer(newer, CURRENT_RELEASE),
      "older release downgraded a receiver");
  current.updateInProgress = true;
  expect(!acceptsLegacyOffer(current, uint16_t(CURRENT_RELEASE + 1)),
      "an in-progress receiver accepted a competing offer");
}

void oneChildTakesOneBoundedFollowOnTurn() {
  Receiver a{1, CURRENT_RELEASE, false, true};
  Receiver c{3, 13};
  Receiver d{4, 14};
  Receiver legacyNeighbor{5, 13};

  SerializedLegacyHost firstTurn(0, true);
  expect(firstTurn.offer(c, 0), "A could not enroll C");
  expect(firstTurn.complete(TRANSFER_MS), "A did not finish C");
  expect(firstTurn.offer(d, TRANSFER_MS + 1), "A could not enroll D");
  expect(firstTurn.complete(2 * TRANSFER_MS + 1), "A did not finish D");

  expect(!c.propagationTurnUsed, "C arrived with its turn consumed");
  c.propagationTurnUsed = true;
  SerializedLegacyHost secondTurn(0, true);
  expect(!secondTurn.offer(a, 0), "current A re-entered C's wake");
  expect(!secondTurn.offer(d, 0), "current D re-entered C's wake");
  expect(secondTurn.offer(legacyNeighbor, 0), "C could not serve one legacy neighbor");
  expect(secondTurn.complete(TRANSFER_MS), "C did not finish its bounded child");
  expect(c.propagationTurnUsed, "C lost its consumed-turn marker");
}

} // namespace

int main() {
  knownReceiverPathIsClosedToUnknownDevices();
  dynamicEnrollmentAcceptsOnePreviouslyUnknownReceiver();
  twoLegacyReceiversAreSerializedWithoutDeadlineRaces();
  equalAndOlderOffersNeverRestartAnUpdater();
  oneChildTakesOneBoundedFollowOnTurn();
  return 0;
}

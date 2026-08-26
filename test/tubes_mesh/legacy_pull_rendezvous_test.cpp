#include <iostream>
#include <stdexcept>

#include "legacy_pull_rendezvous.h"

void expect(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

int main() {
  try {
    LegacyPullRendezvous rendezvous;
    rendezvous.begin(1000);
    expect(rendezvous.update(1000, false) == LegacyPullRendezvousSendWake,
        "first wake was not immediate after AP readiness");
    expect(rendezvous.update(1499, false) == LegacyPullRendezvousIdle,
        "wake repeated before its interval");
    expect(rendezvous.update(1500, false) == LegacyPullRendezvousSendWake,
        "wake did not repeat at its interval");
    expect(rendezvous.update(1750, true) == LegacyPullRendezvousStationArrived,
        "station did not end advertising");
    expect(!rendezvous.active(), "rendezvous stayed active after a station arrived");
    expect(rendezvous.update(2000, false) == LegacyPullRendezvousIdle,
        "wake continued after station arrival");
    std::cout << "PASS: station arrival ends repeated legacy wake\n";

    rendezvous.begin(0xFFFFFF00U);
    expect(rendezvous.update(0xFFFFFF00U, false) == LegacyPullRendezvousSendWake,
        "wraparound run missed first wake");
    expect(rendezvous.update(
        0xFFFFFF00U + LegacyPullRendezvous::WINDOW_MS, false)
        == LegacyPullRendezvousTimedOut,
        "bounded rendezvous did not time out across millis wrap");
    expect(!rendezvous.active(), "timed-out rendezvous stayed active");
    std::cout << "PASS: rendezvous timeout is bounded across millis wrap\n";
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
  return 0;
}

#pragma once

#include <stdint.h>

enum LegacyPullRendezvousAction : uint8_t {
  LegacyPullRendezvousIdle = 0,
  LegacyPullRendezvousSendWake,
  LegacyPullRendezvousStationArrived,
  LegacyPullRendezvousTimedOut,
};

// Host-testable policy for a legacy receiver that cannot acknowledge the wake.
// A keeps the already-ready AP live and repeats a one-hop offer until exactly
// one station arrives or the bounded migration window closes.
class LegacyPullRendezvous {
public:
  static constexpr uint32_t WAKE_INTERVAL_MS = 500;
  // This is a human-operated migration window, not the modern fleet protocol's
  // synchronized start window. Keep it long enough that powering the receiver
  // is not a race against boot, observation, or conversation latency.
  static constexpr uint32_t WINDOW_MS = 300000;

  void begin(uint32_t now) {
    _active = true;
    _startedAt = now;
    _nextWakeAt = now;
    _wakeAttempts = 0;
  }

  // The HTTP host owns the rendezvous lifetime. Once it restores the normal
  // mesh/AP configuration, no further wake may advertise stale credentials.
  void cancel() { _active = false; }

  LegacyPullRendezvousAction update(uint32_t now, bool stationSeen) {
    if (!_active) return LegacyPullRendezvousIdle;
    if (stationSeen) {
      _active = false;
      return LegacyPullRendezvousStationArrived;
    }
    if (static_cast<int32_t>(now - _startedAt) >= static_cast<int32_t>(WINDOW_MS)) {
      _active = false;
      return LegacyPullRendezvousTimedOut;
    }
    if (static_cast<int32_t>(now - _nextWakeAt) >= 0) {
      _nextWakeAt = now + WAKE_INTERVAL_MS;
      _wakeAttempts++;
      return LegacyPullRendezvousSendWake;
    }
    return LegacyPullRendezvousIdle;
  }

  bool active() const { return _active; }
  uint16_t wakeAttempts() const { return _wakeAttempts; }

private:
  bool _active = false;
  uint32_t _startedAt = 0;
  uint32_t _nextWakeAt = 0;
  uint16_t _wakeAttempts = 0;
};

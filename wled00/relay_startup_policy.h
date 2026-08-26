#pragma once
#include <stdint.h>

// Dig2Go startup policy: preserve the retained relay contract while avoiding
// an unconditional off->on power cycle during an application-only update.
struct RelayStartupDecision {
  bool relayPresent;
  bool relayOn;
  bool outputLevel;
  bool offMode;
};

inline RelayStartupDecision dig2goRelayStartup(bool relayPresent, bool turnOnAtBoot,
                                                   uint8_t startupBrightness, bool relayMode) {
  const bool relayOn = turnOnAtBoot && startupBrightness > 0;
  return {relayPresent, relayOn, relayMode ? relayOn : !relayOn, !relayOn};
}

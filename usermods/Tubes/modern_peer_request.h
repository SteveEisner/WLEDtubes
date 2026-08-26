#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

struct ModernPeerRequestIdentity {
  uint32_t nonce = 0;
  uint16_t release = 0;
  uint8_t hardwareFamily = 0;
  uint8_t firmwareVariant = 0;
  uint8_t mac[6] = {0};
};

inline int modernPeerHexDigit(char value) {
  if (value >= '0' && value <= '9') return value - '0';
  if (value >= 'a' && value <= 'f') return value - 'a' + 10;
  if (value >= 'A' && value <= 'F') return value - 'A' + 10;
  return -1;
}

inline bool parseModernPeerMac(const char* text, uint8_t mac[6]) {
  if (!text || strlen(text) != 12) return false;
  for (uint8_t index = 0; index < 6; index++) {
    const int high = modernPeerHexDigit(text[index * 2]);
    const int low = modernPeerHexDigit(text[index * 2 + 1]);
    if (high < 0 || low < 0) return false;
    mac[index] = uint8_t((high << 4) | low);
  }
  return text[12] == '\0';
}

inline bool authorizeModernPeerRequest(
    const ModernPeerRequestIdentity& request,
    const ModernPeerRequestIdentity& activeTurn,
    const uint8_t stationMac[6]
) {
  if (request.nonce == 0 || activeTurn.nonce == 0
      || request.release == 0 || activeTurn.release == 0)
    return false;
  if (request.nonce != activeTurn.nonce
      || request.release != activeTurn.release
      || request.hardwareFamily != activeTurn.hardwareFamily
      || request.firmwareVariant != activeTurn.firmwareVariant)
    return false;
  for (uint8_t index = 0; index < 6; index++)
    if (request.mac[index] != stationMac[index]) return false;
  return true;
}

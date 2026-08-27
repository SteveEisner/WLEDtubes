#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "mesh_protocol.h"

// Release 13/14 consumed only these three fields from its 64-byte update
// command. The old trailing IPAddress object was an Arduino-core ABI detail and
// is ignored by the deployed HTTP client, which connects to brcac.com.
#pragma pack(push, 4)
struct LegacyAutoUpdateOfferWire {
  int32_t version = 0;
  char ssid[25] = {0};
  char password[25] = {0};
  uint8_t ignoredLegacyHost[10] = {0};
};
#pragma pack(pop)

static_assert(sizeof(LegacyAutoUpdateOfferWire) == MESSAGE_DATA_SIZE,
    "legacy update wire payload must remain exactly 64 bytes");
static_assert(offsetof(LegacyAutoUpdateOfferWire, ssid) == 4,
    "legacy update SSID offset changed");
static_assert(offsetof(LegacyAutoUpdateOfferWire, password) == 29,
    "legacy update password offset changed");

inline void copyLegacyUpdateField(char* output, size_t capacity, const char* input) {
  if (capacity == 0) return;
  size_t length = input ? strnlen(input, capacity - 1) : 0;
  if (length) memcpy(output, input, length);
  output[length] = '\0';
}

inline LegacyAutoUpdateOfferWire makeLegacyAutoUpdateOfferWire(
    int32_t version, const char* ssid, const char* password) {
  LegacyAutoUpdateOfferWire wire;
  wire.version = version;
  copyLegacyUpdateField(wire.ssid, sizeof(wire.ssid), ssid);
  copyLegacyUpdateField(wire.password, sizeof(wire.password), password);
  return wire;
}

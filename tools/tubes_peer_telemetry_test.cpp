#include <assert.h>
#include <stdint.h>
#include "usermods/Tubes/peer_telemetry.h"

int main() {
  PeerTelemetry telemetry;
  telemetry.observe(0, 0, 1, -50, 10);
  assert(telemetry.count() == 0);
  telemetry.observe(0x1101, 0x1100, 1, 0, 100);
  const PeerTelemetryEntry *unknown = telemetry.get(0x1101);
  assert(unknown && !unknown->rssiKnown && unknown->samples == 1);
  telemetry.observe(0x1101, 0x1102, 1, -70, 200);
  telemetry.observe(0x1101, 0x1102, 1, -50, 300);
  assert(unknown->rssiKnown && unknown->minimumRssi == -70 && unknown->maximumRssi == -50);
  assert(unknown->smoothedRssi == -65 && unknown->uplinkId == 0x1102);
  for (uint16_t i = 0; i < TUBES_PEER_TELEMETRY_CAPACITY - 1; i++)
    telemetry.observe(0x1200 + i, 0x1100, 1, -60, 400 + i);
  assert(telemetry.count() == TUBES_PEER_TELEMETRY_CAPACITY);
  telemetry.observe(0x1FFF, 0x1100, 1, -40, 1000);
  assert(telemetry.get(0x1101) == nullptr);
  PeerTelemetry wrap;
  wrap.observe(1, 0, 0, -1, UINT32_MAX - 4);
  for (uint16_t i = 2; i <= TUBES_PEER_TELEMETRY_CAPACITY; i++) wrap.observe(i, 0, 0, -1, UINT32_MAX - 3 + i);
  wrap.observe(99, 0, 0, -1, 20);
  assert(wrap.get(1) == nullptr && wrap.get(99));
}

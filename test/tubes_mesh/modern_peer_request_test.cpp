#include <cstring>
#include <stdexcept>
#include <string>

#include "modern_peer_request.h"

namespace {

void expect(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

ModernPeerRequestIdentity identity() {
  ModernPeerRequestIdentity value;
  value.nonce = 0x1234ABCD;
  value.release = 48;
  value.hardwareFamily = 1;
  value.firmwareVariant = 0;
  expect(parseModernPeerMac("5443B2B54980", value.mac), "valid MAC did not parse");
  return value;
}

void exactActiveTurnAndStationAreRequired() {
  const ModernPeerRequestIdentity active = identity();
  ModernPeerRequestIdentity request = active;
  expect(authorizeModernPeerRequest(request, active, active.mac),
      "exact modern peer request was rejected");
  request.nonce++;
  expect(!authorizeModernPeerRequest(request, active, active.mac), "wrong nonce was accepted");
  request = active; request.release--;
  expect(!authorizeModernPeerRequest(request, active, active.mac), "wrong release was accepted");
  request = active; request.hardwareFamily++;
  expect(!authorizeModernPeerRequest(request, active, active.mac), "wrong family was accepted");
  request = active; request.firmwareVariant++;
  expect(!authorizeModernPeerRequest(request, active, active.mac), "wrong variant was accepted");
  uint8_t other[6]; memcpy(other, active.mac, sizeof(other)); other[5]++;
  expect(!authorizeModernPeerRequest(active, active, other),
      "query MAC was not bound to the associated station");
}

void malformedMacsFailClosed() {
  uint8_t mac[6];
  expect(!parseModernPeerMac(nullptr, mac), "null MAC parsed");
  expect(!parseModernPeerMac("5443B2B5498", mac), "short MAC parsed");
  expect(!parseModernPeerMac("5443B2B549800", mac), "long MAC parsed");
  expect(!parseModernPeerMac("5443B2B5498Z", mac), "non-hex MAC parsed");
}

} // namespace

int main() {
  exactActiveTurnAndStationAreRequired();
  malformedMacsFailClosed();
  return 0;
}

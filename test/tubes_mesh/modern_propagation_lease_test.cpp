#include <stdexcept>
#include <string>

#include "modern_propagation_lease.h"

namespace {

void expect(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

FleetUpdateOffer offer(uint16_t version = 48) {
  FleetUpdateOffer value;
  value.flags = FleetUpdatePropagate;
  value.tubesVersion = version;
  value.nonce = 0x12345678;
  value.serverAddress[0] = 4;
  value.serverAddress[1] = 3;
  value.serverAddress[2] = 2;
  value.serverAddress[3] = 1;
  value.serverPort = 80;
  return value;
}

void newerModernOfferArmsOneShotLease() {
  FleetUpdateOffer update = offer();
  expect(shouldArmModernPropagationLease(update, 47),
      "newer modern offer did not arm propagation");
  ModernPropagationLeaseRecord lease = makeModernPropagationLease(update);
  expect(isValidModernPropagationLease(lease), "created lease was invalid");
  expect(claimModernPropagationLease(lease, 48), "new image could not claim lease");
  expect(lease.state == ModernPropagationLeaseClaimed, "lease was not claimed");
  expect(!claimModernPropagationLease(lease, 48), "claimed lease was reusable");
}

void equalOlderAndForcedOffersDoNotPropagate() {
  FleetUpdateOffer equal = offer(47);
  expect(!shouldArmModernPropagationLease(equal, 47),
      "equal release armed propagation");
  FleetUpdateOffer older = offer(46);
  expect(!shouldArmModernPropagationLease(older, 47),
      "older release armed propagation");
  FleetUpdateOffer forced = offer(47);
  forced.flags = FleetUpdateForce;
  expect(!shouldArmModernPropagationLease(forced, 47),
      "forced equal reinstall armed propagation");
}

void ordinaryFleetOfferNeverArmsPeerPropagation() {
  FleetUpdateOffer ordinary = offer(48);
  ordinary.flags = 0;
  expect(isValidFleetUpdateOffer(ordinary), "ordinary fleet offer became invalid");
  expect(!shouldArmModernPropagationLease(ordinary, 47),
      "ordinary fleet OTA armed peer propagation");
}

void wrongImageAndCorruptionFailClosed() {
  ModernPropagationLeaseRecord lease = makeModernPropagationLease(offer());
  expect(!claimModernPropagationLease(lease, 49),
      "different running image claimed lease");
  lease.checksum ^= 1;
  expect(!isValidModernPropagationLease(lease), "corrupt lease was valid");
  expect(!claimModernPropagationLease(lease, 48), "corrupt lease was claimed");
}

void propagationOfferPreservesModernAuthorityAndStandardCredentials() {
  FleetUpdateOffer propagation;
  const uint8_t address[4] = {4, 3, 2, 1};
  expect(makeModernPropagationOffer(
      propagation, 48, 0xCAFEBABE, address, 80, 1000,
      "TubesOTA", "tubes123"), "propagation offer was not constructed");
  expect(isValidFleetUpdateOffer(propagation), "propagation offer was invalid");
  expect(propagation.flags == FleetUpdatePropagate,
      "propagation offer omitted its explicit opt-in");
  expect(propagation.targetDeviceId == 0, "propagation offer was MAC targeted");
  expect(propagation.tubesVersion == 48, "propagation release changed");
  expect(propagation.ssidLength == 8 && propagation.passwordLength == 8,
      "standard credential lengths changed");
  expect(memcmp(propagation.credentials, "TubesOTAtubes123", 16) == 0,
      "standard credentials changed");
  expect(shouldArmModernPropagationLease(propagation, 47),
      "older peer would not propagate after installing offer");
  expect(!shouldArmModernPropagationLease(propagation, 48),
      "equal peer would amplify propagation");
}

void exactCurrentCommandStartsHostingWithoutAnOtaServer() {
  FleetUpdateOffer command;
  expect(makeModernPropagationServeCommand(command, 48, 0x10203040, 0x1234),
      "exact propagation command was not constructed");
  expect(command.flags == FleetUpdatePropagate, "command lost P2P opt-in");
  expect(command.targetDeviceId == 0x1234, "command lost exact target");
  expect(command.serverPort == 0 && command.ssidLength == 0
      && command.passwordLength == 0, "serve command carried OTA transport");
  FleetUpdateOffer wildcard = command;
  wildcard.targetDeviceId = 0;
  expect(!isValidFleetUpdateOffer(wildcard), "wildcard equal-version serve was valid");
}

} // namespace

int main() {
  newerModernOfferArmsOneShotLease();
  equalOlderAndForcedOffersDoNotPropagate();
  ordinaryFleetOfferNeverArmsPeerPropagation();
  wrongImageAndCorruptionFailClosed();
  propagationOfferPreservesModernAuthorityAndStandardCredentials();
  exactCurrentCommandStartsHostingWithoutAnOtaServer();
  return 0;
}

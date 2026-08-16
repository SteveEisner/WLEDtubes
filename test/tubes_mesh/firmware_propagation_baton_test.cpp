#include <array>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "firmware_propagation_baton.h"

namespace {

void expect(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

constexpr uint8_t SENDER[6] = {1, 2, 3, 4, 5, 6};
constexpr uint8_t RECEIVER[6] = {7, 8, 9, 10, 11, 12};
constexpr uint8_t OTHER[6] = {13, 14, 15, 16, 17, 18};

FirmwareTargetContract exactTarget() {
  const FirmwareTargetContract staticTarget = firmwareStaticTargetFromCanonical(CANONICAL_TARGET_QUINLED_DIG2GO, TubeHardwareDig2Go);
  FirmwareTargetContract target;
  expect(firmwareReceiverTargetFromStatic(staticTarget, 0, target), "receiver target construction failed");
  return target;
}

FirmwareImageArtifact exactArtifact() {
  const FirmwareTargetContract staticTarget = firmwareStaticTargetFromCanonical(CANONICAL_TARGET_QUINLED_DIG2GO, TubeHardwareDig2Go);
  return firmwareArtifactFromCanonical(CANONICAL_ARTIFACT_DIG2GO_V14_OTA_APPLICATION, staticTarget, 0x12345678);
}

FirmwareUpdateCompletionProof completedProof(uint32_t sessionNonce = 0x1234) {
  FirmwareUpdateSession session; const FirmwareImageArtifact artifact=exactArtifact(); const FirmwareTargetContract target=exactTarget();
  expect(session.select(SENDER,RECEIVER,artifact,target,firmwareInactiveSlotDestination(target),0,100,sessionNonce),"session select failed");
  expect(session.startTransfer(RECEIVER,1),"session transfer failed");expect(session.recordProgress(RECEIVER,artifact.imageLengthBytes,2),"session progress failed");
  expect(session.verifyTransfer(RECEIVER,artifact.imageSha256,3),"session hash failed");FirmwareUpdateHealthProof proof;proof.target=target;proof.releaseHash=artifact.releaseHash;
  memcpy(proof.imageSha256,artifact.imageSha256,sizeof(proof.imageSha256));proof.runtimeConfigurationPreserved=true;proof.meshRejoined=true;proof.stable=true;
  expect(session.proveHealthy(RECEIVER,proof,4)&&session.complete(RECEIVER),"session health failed");
  FirmwareUpdateCompletionProof completion;
  expect(session.completionProof(completion), "typed completion proof was unavailable");
  return completion;
}

void admits_only_one_targetable_receiver_for_the_frozen_artifact() {
  FirmwarePropagationBaton baton;
  expect(baton.start(SENDER, exactArtifact(), 100), "seed did not start");
  expect(!baton.selectReceiver(RECEIVER, FirmwarePeerLegacy, 0x1234, 101), "legacy peer entered targetable path");
  expect(baton.selectReceiver(RECEIVER, FirmwarePeerTargetable, 0x1234, 101), "targetable peer was rejected");
  expect(!baton.selectReceiver(OTHER, FirmwarePeerTargetable, 0x5678, 102), "second receiver entered active transfer");
  expect(baton.senderIs(SENDER) && baton.receiverIs(RECEIVER), "exact endpoints changed");
  expect(baton.artifactId() == CanonicalArtifactDig2goV14OtaApplication, "artifact changed");
}

void expires_after_sixty_idle_seconds_without_claiming_fleet_completion() {
  FirmwarePropagationBaton baton;
  expect(baton.start(SENDER, exactArtifact(), 100), "seed did not start");
  expect(!baton.expireIfIdle(159), "baton expired early");
  expect(baton.expireIfIdle(160), "baton did not expire at sixty seconds");
  expect(baton.state() == FirmwarePropagationExpired, "expiry state changed");
  expect(!baton.fleetIsCurrent(), "idle expiry claimed absent fleet was current");
}

void one_retry_restarts_from_byte_zero() {
  FirmwarePropagationBaton baton;
  expect(baton.start(SENDER, exactArtifact(), 0), "seed did not start");
  expect(baton.selectReceiver(RECEIVER, FirmwarePeerTargetable, 0x1234, 1), "receiver was rejected");
  expect(baton.recordProgress(RECEIVER, 4096), "progress was rejected");
  expect(baton.retryFromZero(RECEIVER), "first retry was rejected");
  expect(baton.transferredBytes() == 0, "retry did not reset to byte zero");
  expect(!baton.retryFromZero(RECEIVER), "second retry was admitted");
}

void health_failure_never_hands_off_the_baton() {
  FirmwarePropagationBaton baton;
  expect(baton.start(SENDER, exactArtifact(), 0), "seed did not start");
  expect(baton.selectReceiver(RECEIVER, FirmwarePeerTargetable, 0x1234, 1), "receiver was rejected");
  expect(baton.awaitHealth(RECEIVER), "health wait was rejected");
  FirmwareUpdateCompletionProof missing;
  expect(!baton.handoffFromCompletionProof(missing, 2), "missing typed health proof was accepted");
  expect(!baton.handoffFromCompletionProof(completedProof(0x5678), 2), "stale session proof was accepted");
  expect(baton.senderIs(SENDER), "failed receiver became sender");
  expect(!baton.batonHandedOff(), "failed receiver received baton");
}

void successful_health_hands_off_and_refreshes_one_idle_window() {
  FirmwarePropagationBaton baton;
  expect(baton.start(SENDER, exactArtifact(), 0), "seed did not start");
  expect(baton.selectReceiver(RECEIVER, FirmwarePeerTargetable, 0x1234, 1), "receiver was rejected");
  expect(baton.awaitHealth(RECEIVER), "health wait was rejected");
  const FirmwareUpdateCompletionProof proof=completedProof();
  expect(baton.handoffFromCompletionProof(proof, 20), "healthy receiver was rejected");
  expect(baton.senderIs(RECEIVER), "healthy receiver did not become sender");
  expect(baton.batonHandedOff(), "handoff was not recorded");
  expect(!baton.expireIfIdle(79) && baton.expireIfIdle(80), "fresh sixty-second window changed");
}

void first_updated_boot_arms_once_but_ordinary_reboot_does_not() {
  FirmwareUpdatedBootLatch ordinary{FirmwareUpdatedBootMarker()};
  expect(ordinary.consumeSuccessfulUpdateBoot(exactArtifact(),true) == FirmwareBootNoPropagation, "ordinary boot started propagation");
  FirmwareUpdatedBootMarker marker = firmwareUpdatedBootMarkerFor(exactArtifact());
  FirmwareUpdatedBootLatch updated(marker);
  FirmwareImageArtifact wrongBytes=exactArtifact(); wrongBytes.imageSha256[0]^=0xff;
  expect(updated.consumeSuccessfulUpdateBoot(wrongBytes,true) == FirmwareBootNoPropagation, "different artifact bytes consumed update marker");
  expect(updated.consumeSuccessfulUpdateBoot(exactArtifact(),false) == FirmwareBootNoPropagation, "unhealthy boot consumed update marker");
  expect(updated.consumeSuccessfulUpdateBoot(exactArtifact(),true) == FirmwareBootStartPropagation, "first updated boot did not start");
  expect(updated.marker().state == FirmwareBootMarkerConsumed, "updated boot marker was not consumed");
  FirmwareUpdatedBootLatch rebooted(updated.marker());
  expect(rebooted.consumeSuccessfulUpdateBoot(exactArtifact(),true) == FirmwareBootNoPropagation, "ordinary reboot restarted propagation");
}

} // namespace

int main() {
  const std::array<std::pair<const char*, void (*)()>, 6> tests = {{
    {"admits only one targetable receiver for the frozen artifact", admits_only_one_targetable_receiver_for_the_frozen_artifact},
    {"expires after sixty idle seconds without claiming fleet completion", expires_after_sixty_idle_seconds_without_claiming_fleet_completion},
    {"one retry restarts from byte zero", one_retry_restarts_from_byte_zero},
    {"health failure never hands off the baton", health_failure_never_hands_off_the_baton},
    {"successful health hands off and refreshes one idle window", successful_health_hands_off_and_refreshes_one_idle_window},
    {"first updated boot arms once but ordinary reboot does not", first_updated_boot_arms_once_but_ordinary_reboot_does_not},
  }};
  for (const auto& test : tests) {
    try { test.second(); std::cout << "PASS: " << test.first << '\n'; }
    catch (const std::exception& error) { std::cerr << "FAIL: " << test.first << ": " << error.what() << '\n'; return 1; }
  }
  return 0;
}

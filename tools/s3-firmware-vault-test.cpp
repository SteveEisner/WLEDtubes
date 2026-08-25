#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "../usermods/Tubes/s3_firmware_vault.h"

static void require(bool condition, const char* message) {
  if (!condition) { fprintf(stderr, "%s\n", message); exit(1); }
}

static S3VaultObservedDevice observed(uint8_t family, uint16_t release, uint32_t at) {
  S3VaultObservedDevice device;
  const uint8_t mac[6] = {0x24, 0x6f, 0x28, 0, 0, family};
  memcpy(device.mac, mac, 6);
  device.nonce = 0x1234abcd;
  device.family = family;
  device.variant = TubeVariantStandard;
  device.tubesVersion = release;
  device.observedAtMs = at;
  return device;
}

static S3VaultRequest requestFor(const S3VaultObservedDevice& device) {
  S3VaultRequest request;
  request.nonce = 0x1234abcd;
  request.release = 47;
  request.family = device.family;
  request.variant = device.variant;
  memcpy(request.mac, device.mac, 6);
  return request;
}

int main() {
  S3VaultArtifact dig2goArtifact;
  dig2goArtifact.family = TubeHardwareDig2Go;
  dig2goArtifact.tubesVersion = 47;
  dig2goArtifact.size = 1300000;
  strcpy(dig2goArtifact.md5, "0123456789abcdef0123456789abcdef");
  S3VaultArtifact c3Artifact = dig2goArtifact;
  c3Artifact.family = TubeHardwareAthomC3;
  S3FirmwareVaultCatalog catalog;
  require(catalog.configure(dig2goArtifact, c3Artifact, 47), "exact catalog was rejected");
  require(catalog.select(TubeHardwareDig2Go, TubeVariantStandard, 47) != nullptr,
          "Dig2Go artifact was not selected");
  require(catalog.select(TubeHardwareAthomC3, TubeVariantStandard, 47) != nullptr,
          "Athom C3 artifact was not selected");
  require(catalog.select(TubeHardwareGledopto, TubeVariantStandard, 47) == nullptr,
          "third family was selected");
  require(catalog.select(TubeHardwareDig2Go, TubeVariantStandard, 39) == nullptr,
          "stale release was selected");

  FleetUpdateOffer offer;
  const uint8_t address[4] = {192, 168, 4, 1};
  require(S3VaultOfferFactory::make(offer, 0x1234abcd, 47, address, 8080,
                                    "TubesOTA", "update1234"),
          "valid wildcard baton offer was rejected");
  require(offer.targetDeviceId == 0 && isValidFleetUpdateOffer(offer),
          "offer factory emitted an invalid or targeted offer");

  S3FirmwareVaultPolicy vault;
  vault.arm(0x1234abcd, 47, 1000);
  auto dig2go = observed(TubeHardwareDig2Go, 22, 1010);
  auto request = requestFor(dig2go);
  require(vault.claim(request, &dig2go, 1020) == S3VaultDecision::Accepted,
          "fresh Dig2Go was not accepted");
  require(vault.state() == S3VaultState::Claimed, "claim did not stop the open wave");
  require(vault.claim(request, &dig2go, 1030) == S3VaultDecision::RetryAccepted,
          "same-device retry was rejected");

  auto c3 = observed(TubeHardwareAthomC3, 22, 1030);
  auto c3Request = requestFor(c3);
  require(vault.claim(c3Request, &c3, 1040) == S3VaultDecision::ClaimedByAnotherDevice,
          "second device entered a claimed wave");
  require(vault.bodyCompleted(dig2go.mac, 2000), "body completion was rejected");
  require(vault.state() == S3VaultState::AwaitingFreshReport,
          "body completion was incorrectly treated as success");
  dig2go.tubesVersion = 47;
  dig2go.observedAtMs = 5000;
  require(vault.acceptFreshReport(dig2go), "fresh target report was rejected");
  require(vault.state() == S3VaultState::Complete, "fresh report did not complete baton");

  vault.disarm();
  vault.arm(0x1234abcd, 47, 0);
  auto current = observed(TubeHardwareDig2Go, 47, 10);
  require(vault.claim(requestFor(current), &current, 20) == S3VaultDecision::DeviceAlreadyCurrent,
          "current device was allowed to claim");
  auto unsupported = observed(TubeHardwareGledopto, 22, 10);
  require(vault.claim(requestFor(unsupported), &unsupported, 20) == S3VaultDecision::UnsupportedProfile,
          "unsupported family was accepted");
  auto stale = observed(TubeHardwareAthomC3, 22, 1);
  vault.setObservationMaxAgeMs(10);
  require(vault.claim(requestFor(stale), &stale, 20) == S3VaultDecision::DeviceNotObserved,
          "stale observation was accepted");

  vault.disarm();
  vault.setArmTimeoutMs(100);
  vault.arm(0x1234abcd, 47, 1000);
  require(!vault.expire(1100), "arm expired at its inclusive deadline");
  require(vault.expire(1101) && vault.state() == S3VaultState::Failed,
          "unclaimed arm window did not fail closed");

  printf("s3 firmware vault policy scenarios passed\n");
}

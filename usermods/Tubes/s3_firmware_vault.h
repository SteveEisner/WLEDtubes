#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "device_report_protocol.h"
#include "fleet_update_protocol.h"

// Pure policy for the S3 carrier. Transport and storage adapters deliberately
// live outside this class so host tests exercise the same authorization rules.
enum class S3VaultState : uint8_t {
  Idle,
  Armed,
  Claimed,
  AwaitingFreshReport,
  Complete,
  Failed,
};

enum class S3VaultDecision : uint8_t {
  Accepted,
  RetryAccepted,
  NotArmed,
  StaleWave,
  UnsupportedProfile,
  DeviceNotObserved,
  DeviceIdentityMismatch,
  DeviceAlreadyCurrent,
  ClaimedByAnotherDevice,
};

struct S3VaultArtifact {
  uint8_t family = TubeHardwareUnknown;
  uint8_t variant = TubeVariantStandard;
  uint16_t tubesVersion = 0;
  size_t size = 0;
  char md5[33] = {0};
};

struct S3VaultRequest {
  uint32_t nonce = 0;
  uint16_t release = 0;
  uint8_t family = TubeHardwareUnknown;
  uint8_t variant = TubeVariantStandard;
  uint8_t mac[6] = {0};
};

struct S3VaultObservedDevice {
  uint8_t mac[6] = {0};
  uint32_t nonce = 0;
  uint8_t family = TubeHardwareUnknown;
  uint8_t variant = TubeVariantStandard;
  uint16_t tubesVersion = 0;
  uint32_t observedAtMs = 0;
};

class S3FirmwareVaultCatalog {
public:
  bool configure(const S3VaultArtifact& dig2go, const S3VaultArtifact& athomC3,
                 uint16_t release) {
    valid_ = validArtifact(dig2go, TubeHardwareDig2Go, release)
        && validArtifact(athomC3, TubeHardwareAthomC3, release);
    if (valid_) {
      artifacts_[0] = dig2go;
      artifacts_[1] = athomC3;
    }
    return valid_;
  }

  const S3VaultArtifact* select(uint8_t family, uint8_t variant,
                               uint16_t release) const {
    if (!valid_ || variant != TubeVariantStandard) return nullptr;
    for (const auto& artifact : artifacts_)
      if (artifact.family == family && artifact.variant == variant
          && artifact.tubesVersion == release)
        return &artifact;
    return nullptr;
  }

private:
  static bool validArtifact(const S3VaultArtifact& artifact, uint8_t family,
                            uint16_t release) {
    if (artifact.family != family || artifact.variant != TubeVariantStandard
        || artifact.tubesVersion != release || !artifact.size
        || artifact.md5[32] != '\0') return false;
    for (uint8_t index = 0; index < 32; index++) {
      const char digit = artifact.md5[index];
      if (!((digit >= '0' && digit <= '9') || (digit >= 'a' && digit <= 'f')))
        return false;
    }
    return true;
  }

  S3VaultArtifact artifacts_[2];
  bool valid_ = false;
};

class S3VaultOfferFactory {
public:
  static bool make(FleetUpdateOffer& offer, uint32_t nonce, uint16_t release,
                   const uint8_t serverAddress[4], uint16_t serverPort,
                   const char* ssid, const char* password) {
    offer = FleetUpdateOffer();
    offer.nonce = nonce;
    offer.tubesVersion = release;
    memcpy(offer.serverAddress, serverAddress, sizeof(offer.serverAddress));
    offer.serverPort = serverPort;
    offer.targetDeviceId = 0; // Baton claim is authorized by the HTTP policy.
    return setFleetUpdateCredentials(offer, ssid, password)
        && isValidFleetUpdateOffer(offer);
  }
};

class S3FirmwareVaultPolicy {
public:
  static constexpr uint32_t DEFAULT_OBSERVATION_MAX_AGE_MS = 30000;
  static constexpr uint32_t DEFAULT_CLAIM_TIMEOUT_MS = 90000;

  void arm(uint32_t nonce, uint16_t release, uint32_t nowMs) {
    resetClaim();
    nonce_ = nonce;
    release_ = release;
    armedAtMs_ = nowMs;
    state_ = nonce && release ? S3VaultState::Armed : S3VaultState::Failed;
  }

  void disarm() {
    nonce_ = 0;
    release_ = 0;
    resetClaim();
    state_ = S3VaultState::Idle;
  }

  S3VaultDecision claim(
      const S3VaultRequest& request,
      const S3VaultObservedDevice* observed,
      uint32_t nowMs
  ) {
    if (state_ == S3VaultState::Claimed && sameMac(request.mac, claimedMac_))
      return requestMatchesClaim(request) ? S3VaultDecision::RetryAccepted
                                          : S3VaultDecision::StaleWave;
    if (state_ != S3VaultState::Armed)
      return state_ == S3VaultState::Claimed
          ? S3VaultDecision::ClaimedByAnotherDevice
          : S3VaultDecision::NotArmed;
    if (request.nonce != nonce_ || request.release != release_)
      return S3VaultDecision::StaleWave;
    if (!isSupportedProfile(request.family, request.variant))
      return S3VaultDecision::UnsupportedProfile;
    if (!observed || elapsed(nowMs, observed->observedAtMs) > observationMaxAgeMs_)
      return S3VaultDecision::DeviceNotObserved;
    if (!sameMac(request.mac, observed->mac)
        || request.nonce != observed->nonce
        || request.family != observed->family
        || request.variant != observed->variant)
      return S3VaultDecision::DeviceIdentityMismatch;
    if (observed->tubesVersion >= release_)
      return S3VaultDecision::DeviceAlreadyCurrent;

    memcpy(claimedMac_, request.mac, sizeof(claimedMac_));
    claimedFamily_ = request.family;
    claimedVariant_ = request.variant;
    claimedAtMs_ = nowMs;
    state_ = S3VaultState::Claimed;
    return S3VaultDecision::Accepted;
  }

  bool bodyCompleted(const uint8_t mac[6], uint32_t nowMs) {
    if (state_ != S3VaultState::Claimed || !sameMac(mac, claimedMac_)) return false;
    bodyCompletedAtMs_ = nowMs;
    state_ = S3VaultState::AwaitingFreshReport;
    return true;
  }

  bool acceptFreshReport(const S3VaultObservedDevice& report) {
    if (state_ != S3VaultState::AwaitingFreshReport
        || !sameMac(report.mac, claimedMac_)
        || report.nonce != nonce_
        || report.family != claimedFamily_
        || report.variant != claimedVariant_
        || report.tubesVersion != release_
        || elapsed(report.observedAtMs, bodyCompletedAtMs_) > claimTimeoutMs_)
      return false;
    state_ = S3VaultState::Complete;
    return true;
  }

  bool expire(uint32_t nowMs) {
    if ((state_ == S3VaultState::Claimed
         && elapsed(nowMs, claimedAtMs_) > claimTimeoutMs_)
        || (state_ == S3VaultState::AwaitingFreshReport
            && elapsed(nowMs, bodyCompletedAtMs_) > claimTimeoutMs_)) {
      state_ = S3VaultState::Failed;
      return true;
    }
    return false;
  }

  static bool isSupportedProfile(uint8_t family, uint8_t variant) {
    return variant == TubeVariantStandard
        && (family == TubeHardwareDig2Go || family == TubeHardwareAthomC3);
  }

  S3VaultState state() const { return state_; }
  uint32_t nonce() const { return nonce_; }
  uint16_t release() const { return release_; }
  const uint8_t* claimedMac() const { return claimedMac_; }
  void setObservationMaxAgeMs(uint32_t value) { observationMaxAgeMs_ = value; }
  void setClaimTimeoutMs(uint32_t value) { claimTimeoutMs_ = value; }

private:
  static uint32_t elapsed(uint32_t later, uint32_t earlier) { return later - earlier; }
  static bool sameMac(const uint8_t left[6], const uint8_t right[6]) {
    return memcmp(left, right, 6) == 0;
  }
  bool requestMatchesClaim(const S3VaultRequest& request) const {
    return request.nonce == nonce_ && request.release == release_
        && request.family == claimedFamily_ && request.variant == claimedVariant_;
  }
  void resetClaim() {
    memset(claimedMac_, 0, sizeof(claimedMac_));
    claimedFamily_ = TubeHardwareUnknown;
    claimedVariant_ = TubeVariantStandard;
    claimedAtMs_ = 0;
    bodyCompletedAtMs_ = 0;
  }

  S3VaultState state_ = S3VaultState::Idle;
  uint32_t nonce_ = 0;
  uint16_t release_ = 0;
  uint32_t armedAtMs_ = 0;
  uint8_t claimedMac_[6] = {0};
  uint8_t claimedFamily_ = TubeHardwareUnknown;
  uint8_t claimedVariant_ = TubeVariantStandard;
  uint32_t claimedAtMs_ = 0;
  uint32_t bodyCompletedAtMs_ = 0;
  uint32_t observationMaxAgeMs_ = DEFAULT_OBSERVATION_MAX_AGE_MS;
  uint32_t claimTimeoutMs_ = DEFAULT_CLAIM_TIMEOUT_MS;
};

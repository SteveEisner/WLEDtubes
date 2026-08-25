// Dig2Go and Athom C3 update carrier for the Waveshare S3 Tubes Remote.
#if defined(WAVESHARE_S3_TUBES_REMOTE) && defined(TUBES_S3_FIRMWARE_CARRIER)

#include "wled.h"
#include "../Tubes/s3_field_api.h"
#include "../Tubes/s3_firmware_vault.h"
#include "s3_vault_artifacts.h"

extern const uint8_t dig2goStart[] asm("_binary_build_output_s3_vault_esp32_quinled_dig2go_tubes_bin_start");
extern const uint8_t dig2goEnd[] asm("_binary_build_output_s3_vault_esp32_quinled_dig2go_tubes_bin_end");
extern const uint8_t athomC3Start[] asm("_binary_build_output_s3_vault_esp32_c3_athom_tubes_bin_start");
extern const uint8_t athomC3End[] asm("_binary_build_output_s3_vault_esp32_c3_athom_tubes_bin_end");

namespace {
constexpr char FIRMWARE_PATH[] = "/tubes/firmware.bin";
constexpr char ARM_PATH[] = "/tubes/carrier/arm";
constexpr uint16_t CARRIER_RELEASE = S3_VAULT_RELEASE;
constexpr char CARRIER_SSID[] = "Tubes40";
constexpr char CARRIER_PASSWORD[] = "baton4040";
static_assert(sizeof(CARRIER_SSID) - 1 + sizeof(CARRIER_PASSWORD) - 1
              <= FLEET_UPDATE_CREDENTIAL_BYTES,
              "carrier credentials exceed FleetUpdateOffer capacity");
constexpr uint32_t PROBE_TIMEOUT_MS = 10000;
constexpr uint32_t POST_REPORT_DELAY_MS = 3000;
constexpr uint32_t POST_REPORT_INTERVAL_MS = 2000;
constexpr uint32_t TCP_DRAIN_GRACE_MS = 100;
constexpr size_t TARGET_CAPACITY = 7;
constexpr uint32_t TARGET_MAX_AGE_MS = 60000;

S3FirmwareVaultPolicy policy;
S3FirmwareVaultCatalog catalog;
S3VaultObservedDevice armedDevice;
bool probePending = false;
uint8_t probeMac[6] = {0};
uint8_t probeFamily = 0;
uint8_t probeVariant = 0;
uint16_t probeCurrentRelease = 0;
uint32_t probeNonce = 0;
uint32_t probeDeadline = 0;
uint32_t nextPostReportAt = 0;
bool carrierApActive = false;
wifi_mode_t previousWifiMode = WIFI_MODE_STA;
TubesS3CarrierTarget targets[TARGET_CAPACITY];
size_t targetCount = 0;
volatile int8_t pendingResponseOutcome = 0;
uint8_t pendingResponseMac[6] = {0};
uint32_t pendingResponseAt = 0;

void stopCarrierAP() {
  if (!carrierApActive) return;
  WiFi.softAPdisconnect(true);
  WiFi.mode(previousWifiMode);
  carrierApActive = false;
}

bool startCarrierAP() {
  if (carrierApActive || apActive) return false;
  previousWifiMode = WiFi.getMode();
  uint8_t channel = WiFi.channel();
  if (!channel) channel = 1;
  WiFi.mode(WIFI_AP_STA);
  if (!WiFi.softAP(CARRIER_SSID, CARRIER_PASSWORD, channel, false, 1)) {
    WiFi.mode(previousWifiMode);
    return false;
  }
  carrierApActive = true;
  return WiFi.softAPIP() != IPAddress(0, 0, 0, 0);
}

void responseFinished(bool acknowledged, const uint8_t mac[6]) {
  memcpy(pendingResponseMac, mac, sizeof(pendingResponseMac));
  pendingResponseAt = millis();
  pendingResponseOutcome = acknowledged ? 1 : -1;
}

void rememberTarget(const DeviceReportMessage& report) {
  if (!S3FirmwareVaultPolicy::isSupportedProfile(report.hardwareFamily,
                                                  report.firmwareVariant)
      || report.tubesVersion >= CARRIER_RELEASE) return;
  size_t slot = targetCount;
  for (size_t index = 0; index < targetCount; index++)
    if (!memcmp(targets[index].mac, report.mac, 6)) { slot = index; break; }
  if (slot == targetCount) {
    if (targetCount < TARGET_CAPACITY) targetCount++;
    else {
      slot = 0;
      for (size_t index = 1; index < TARGET_CAPACITY; index++)
        if (targets[index].lastSeenMs < targets[slot].lastSeenMs) slot = index;
    }
  }
  memcpy(targets[slot].mac, report.mac, 6);
  targets[slot].family = report.hardwareFamily;
  targets[slot].variant = report.firmwareVariant;
  targets[slot].release = report.tubesVersion;
  targets[slot].lastSeenMs = millis();
}

class AcknowledgedProgmemResponse : public AsyncProgmemResponse {
public:
  AcknowledgedProgmemResponse(const uint8_t* content, size_t length,
                              const uint8_t mac[6])
      : AsyncProgmemResponse(200, "application/octet-stream", content, length) {
    memcpy(mac_, mac, sizeof(mac_));
  }

  ~AcknowledgedProgmemResponse() override {
    if (!reported_) responseFinished(false, mac_);
  }

  size_t _ack(AsyncWebServerRequest* request, size_t length, uint32_t time) override {
    size_t result = AsyncProgmemResponse::_ack(request, length, time);
    if (!reported_ && _state == RESPONSE_END && _ackedLength >= _writtenLength) {
      reported_ = true;
      responseFinished(true, mac_);
    } else if (!reported_ && _state == RESPONSE_FAILED) {
      reported_ = true;
      responseFinished(false, mac_);
    }
    return result;
  }

private:
  uint8_t mac_[6] = {0};
  bool reported_ = false;
};

bool parseUnsigned(const String& text, uint32_t maximum, uint32_t& value, int base = 10) {
  if (!text.length()) return false;
  char* end = nullptr;
  unsigned long parsed = strtoul(text.c_str(), &end, base);
  if (!end || *end || parsed > maximum) return false;
  value = parsed;
  return true;
}

bool parseMac(const String& text, uint8_t mac[6]) {
  if (text.length() != 12) return false;
  for (uint8_t index = 0; index < 6; index++) {
    char pair[3] = {text[index * 2], text[index * 2 + 1], 0};
    char* end = nullptr;
    unsigned long value = strtoul(pair, &end, 16);
    if (!end || *end) return false;
    mac[index] = uint8_t(value);
  }
  return true;
}

bool exactArguments(AsyncWebServerRequest* request,
                    const char* const* names, size_t count) {
  if (request->args() != count) return false;
  for (size_t index = 0; index < count; index++)
    if (!request->hasArg(names[index])) return false;
  return true;
}

const uint8_t* artifactData(uint8_t family, size_t& size) {
  if (family == TubeHardwareDig2Go) {
    size = size_t(dig2goEnd - dig2goStart);
    return dig2goStart;
  }
  if (family == TubeHardwareAthomC3) {
    size = size_t(athomC3End - athomC3Start);
    return athomC3Start;
  }
  size = 0;
  return nullptr;
}

void sendError(AsyncWebServerRequest* request, int code, const char* message) {
  request->send(code, "text/plain", message);
}

class S3FirmwareCarrier : public Usermod {
public:
  void setup() override {
    S3VaultArtifact dig2go;
    dig2go.family = TubeHardwareDig2Go;
    dig2go.variant = TubeVariantStandard;
    dig2go.tubesVersion = CARRIER_RELEASE;
    dig2go.size = S3_VAULT_DIG2GO_SIZE;
    strlcpy(dig2go.md5, S3_VAULT_DIG2GO_MD5, sizeof(dig2go.md5));
    S3VaultArtifact c3;
    c3.family = TubeHardwareAthomC3;
    c3.variant = TubeVariantStandard;
    c3.tubesVersion = CARRIER_RELEASE;
    c3.size = S3_VAULT_ATHOM_C3_SIZE;
    strlcpy(c3.md5, S3_VAULT_ATHOM_C3_MD5, sizeof(c3.md5));
    if (!catalog.configure(dig2go, c3, CARRIER_RELEASE)
        || size_t(dig2goEnd - dig2goStart) != dig2go.size
        || size_t(athomC3End - athomC3Start) != c3.size) {
      policy.arm(0, 0, millis());
      return;
    }

    server.on(ARM_PATH, HTTP_POST, [](AsyncWebServerRequest* request) {
      static const char* const names[] = {"mac", "family", "variant", "current"};
      uint32_t family, variant, current;
      uint8_t mac[6];
      if (!exactArguments(request, names, 4)
          || !parseMac(request->arg("mac"), mac)
          || !parseUnsigned(request->arg("family"), UINT8_MAX, family)
          || !parseUnsigned(request->arg("variant"), UINT8_MAX, variant)
          || !parseUnsigned(request->arg("current"), UINT16_MAX, current)
          || !S3FirmwareVaultPolicy::isSupportedProfile(family, variant)
          || current >= CARRIER_RELEASE) {
        sendError(request, 400, "invalid carrier target");
        return;
      }
      if (!tubesS3ArmCarrier(mac, family, variant, current)) {
        sendError(request, 503, "carrier could not request target report");
        return;
      }
      char response[48];
      snprintf(response, sizeof(response), "probing nonce=%08lX release=%u\n",
               (unsigned long)probeNonce, CARRIER_RELEASE);
      request->send(202, "text/plain", response);
    });

    server.on(FIRMWARE_PATH, HTTP_GET, [](AsyncWebServerRequest* request) {
      static const char* const names[] = {"nonce", "release", "family", "variant", "mac"};
      S3VaultRequest incoming;
      uint32_t release, family, variant;
      if (!exactArguments(request, names, 5)
          || !parseUnsigned(request->arg("nonce"), UINT32_MAX, incoming.nonce, 16)
          || !parseUnsigned(request->arg("release"), UINT16_MAX, release)
          || !parseUnsigned(request->arg("family"), UINT8_MAX, family)
          || !parseUnsigned(request->arg("variant"), UINT8_MAX, variant)
          || !parseMac(request->arg("mac"), incoming.mac)) {
        sendError(request, 400, "invalid update query");
        return;
      }
      incoming.release = release;
      incoming.family = family;
      incoming.variant = variant;
      S3VaultDecision decision = policy.claim(incoming, &armedDevice, millis());
      if (decision != S3VaultDecision::Accepted
          && decision != S3VaultDecision::RetryAccepted) {
        sendError(request, decision == S3VaultDecision::UnsupportedProfile ? 404 : 403,
                  "firmware request refused");
        return;
      }
      const S3VaultArtifact* artifact = catalog.select(family, variant, release);
      size_t embeddedSize;
      const uint8_t* data = artifactData(family, embeddedSize);
      if (!artifact || !data || embeddedSize != artifact->size) {
        sendError(request, 500, "carrier artifact unavailable");
        return;
      }
      AsyncWebServerResponse* response = new AcknowledgedProgmemResponse(
          data, embeddedSize, incoming.mac);
      response->addHeader("x-MD5", artifact->md5);
      response->addHeader("Cache-Control", "no-store");
      response->addHeader("Connection", "close");
      request->send(response);
    });
  }

  void loop() override {
    const uint32_t now = millis();
    if (pendingResponseOutcome && now - pendingResponseAt >= TCP_DRAIN_GRACE_MS) {
      const int8_t outcome = pendingResponseOutcome;
      pendingResponseOutcome = 0;
      if (outcome > 0 && policy.bodyCompleted(pendingResponseMac, now)) {
        stopCarrierAP();
        nextPostReportAt = now + POST_REPORT_DELAY_MS;
      } else {
        policy.fail();
        stopCarrierAP();
      }
    }
    if (probePending && int32_t(now - probeDeadline) >= 0) {
      probePending = false;
      policy.fail();
      stopCarrierAP();
    }
    if (policy.expire(now)) stopCarrierAP();
    if (policy.state() == S3VaultState::AwaitingFreshReport
        && int32_t(now - nextPostReportAt) >= 0) {
      tubesS3RequestDeviceReport(policy.claimedMac(), policy.nonce());
      nextPostReportAt = now + POST_REPORT_INTERVAL_MS;
    }
  }

  void addToJsonInfo(JsonObject& root) override {
    JsonObject user = root[F("u")];
    if (user.isNull()) user = root.createNestedObject(F("u"));
    JsonArray carrier = user.createNestedArray(F("S3 carrier"));
    carrier.add(probePending ? 250 : uint8_t(policy.state()));
    carrier.add(probePending ? CARRIER_RELEASE : policy.release());
  }
};

S3FirmwareCarrier carrier;
REGISTER_USERMOD(carrier);
} // namespace

bool tubesS3ReadCarrierStatus(TubesS3CarrierStatus& status) {
  status.state = probePending ? 250 : uint8_t(policy.state());
  status.nonce = probePending ? probeNonce : policy.nonce();
  status.release = probePending ? CARRIER_RELEASE : policy.release();
  memcpy(status.claimedMac, policy.claimedMac(), sizeof(status.claimedMac));
  return true;
}

bool tubesS3ArmCarrier(const uint8_t mac[6], uint8_t family, uint8_t variant,
                       uint16_t currentRelease) {
  if (!S3FirmwareVaultPolicy::isSupportedProfile(family, variant)
      || currentRelease >= CARRIER_RELEASE || probePending
      || policy.state() == S3VaultState::Armed
      || policy.state() == S3VaultState::Claimed
      || policy.state() == S3VaultState::AwaitingFreshReport) return false;
  uint32_t nonce = esp_random();
  if (!nonce) nonce = 1;
  memcpy(probeMac, mac, sizeof(probeMac));
  probeFamily = family;
  probeVariant = variant;
  probeCurrentRelease = currentRelease;
  probeNonce = nonce;
  probeDeadline = millis() + PROBE_TIMEOUT_MS;
  probePending = tubesS3RequestDeviceReport(probeMac, probeNonce);
  return probePending;
}

void tubesS3DisarmCarrier() {
  probePending = false;
  policy.disarm();
  stopCarrierAP();
}

void tubesS3CarrierObserveDeviceReport(const DeviceReportMessage& report) {
  if (report.kind != DeviceReportReply) return;
  rememberTarget(report);
  if (probePending) {
    if (report.nonce != probeNonce || memcmp(report.mac, probeMac, sizeof(probeMac))
        || report.hardwareFamily != probeFamily
        || report.firmwareVariant != probeVariant
        || report.tubesVersion != probeCurrentRelease)
      return;
    probePending = false;
    memset(&armedDevice, 0, sizeof(armedDevice));
    memcpy(armedDevice.mac, report.mac, sizeof(armedDevice.mac));
    armedDevice.nonce = report.nonce;
    armedDevice.family = report.hardwareFamily;
    armedDevice.variant = report.firmwareVariant;
    armedDevice.tubesVersion = report.tubesVersion;
    armedDevice.observedAtMs = millis();
    if (!startCarrierAP()) {
      policy.fail();
      stopCarrierAP();
      return;
    }
    IPAddress addressIp = WiFi.softAPIP();
    const uint8_t address[4] = {addressIp[0], addressIp[1], addressIp[2], addressIp[3]};
    FleetUpdateOffer offer;
    policy.arm(probeNonce, CARRIER_RELEASE, millis());
    if (!S3VaultOfferFactory::make(offer, probeNonce, CARRIER_RELEASE, address, 80,
                                  CARRIER_SSID, CARRIER_PASSWORD)
        || !tubesS3BroadcastFleetOffer(offer)) {
      policy.fail();
      stopCarrierAP();
    }
    return;
  }

  if (policy.state() == S3VaultState::AwaitingFreshReport) {
    S3VaultObservedDevice observed;
    memcpy(observed.mac, report.mac, sizeof(observed.mac));
    observed.nonce = report.nonce;
    observed.family = report.hardwareFamily;
    observed.variant = report.firmwareVariant;
    observed.tubesVersion = report.tubesVersion;
    observed.observedAtMs = millis();
    policy.acceptFreshReport(observed);
  }
}

bool tubesS3ScanCarrierTargets() {
  uint8_t wildcard[6] = {0};
  uint32_t nonce = esp_random();
  if (!nonce) nonce = 1;
  return tubesS3RequestDeviceReport(wildcard, nonce);
}

size_t tubesS3CarrierTargetCount() {
  const uint32_t now = millis();
  size_t count = 0;
  for (size_t index = 0; index < targetCount; index++)
    if (now - targets[index].lastSeenMs <= TARGET_MAX_AGE_MS) count++;
  return count;
}

bool tubesS3ReadCarrierTarget(size_t requested, TubesS3CarrierTarget& target) {
  const uint32_t now = millis();
  size_t visible = 0;
  for (size_t index = 0; index < targetCount; index++) {
    if (now - targets[index].lastSeenMs > TARGET_MAX_AGE_MS) continue;
    if (visible++ == requested) { target = targets[index]; return true; }
  }
  return false;
}

#endif

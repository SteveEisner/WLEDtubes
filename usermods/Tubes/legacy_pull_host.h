#pragma once

#include "wled.h"
#include "firmware_http_source.h"
#include "running_image_source.h"
#include "legacy_pull_host_lifecycle.h"
#include "modern_peer_request.h"
#include <esp_wifi.h>
#include <esp_netif_sta_list.h>

#if defined(ARDUINO_ARCH_ESP32)

struct LegacyPullTelemetry {
  struct TransferSlot {
    uint8_t mac[6] = {0};
    uint32_t servedBytes = 0;
    bool admitted = false;
    bool requested = false;
    bool complete = false;
  };

  static TransferSlot* slots() { static TransferSlot v[2]; return v; }
  static volatile uint32_t& expectedBytes() { static volatile uint32_t v = 0; return v; }
  static volatile uint32_t& servedBytes() { static volatile uint32_t v = 0; return v; }
  static volatile uint32_t& completedAt() { static volatile uint32_t v = 0; return v; }
  static volatile uint32_t& lastProgressAt() { static volatile uint32_t v = 0; return v; }
  static volatile uint32_t& stationSeenAt() { static volatile uint32_t v = 0; return v; }
  static volatile bool& requestSeen() { static volatile bool v = false; return v; }
  static volatile bool& stationSeen() { static volatile bool v = false; return v; }
  static volatile bool& readFailed() { static volatile bool v = false; return v; }

  static void reset(uint32_t expected) {
    expectedBytes() = expected;
    servedBytes() = 0;
    completedAt() = 0;
    lastProgressAt() = 0;
    stationSeenAt() = 0;
    requestSeen() = false;
    stationSeen() = false;
    readFailed() = false;
    slots()[0] = TransferSlot();
    slots()[1] = TransferSlot();
  }

  static int admit(const uint8_t mac[6]) {
    for (uint8_t index = 0; index < 2; index++)
      if (slots()[index].admitted && memcmp(slots()[index].mac, mac, 6) == 0)
        return index;
    for (uint8_t index = 0; index < 2; index++) {
      if (slots()[index].admitted) continue;
      slots()[index].admitted = true;
      memcpy(slots()[index].mac, mac, 6);
      return index;
    }
    return -1;
  }

  static uint8_t admittedCount() {
    return uint8_t(slots()[0].admitted) + uint8_t(slots()[1].admitted);
  }

  static uint8_t completedCount() {
    return uint8_t(slots()[0].complete) + uint8_t(slots()[1].complete);
  }

  static void observeStation() {
    if (stationSeen() || WiFi.softAPgetStationNum() == 0) return;
    stationSeen() = true;
    stationSeenAt() = millis();
    Serial.println(F("TUBE_PULL_WIFI station_connected"));
  }

  static bool beginRequest(uint8_t slot, uint32_t expected) {
    if (slot >= 2 || !slots()[slot].admitted || slots()[slot].requested)
      return false;
    slots()[slot].requested = true;
    lastProgressAt() = millis();
    requestSeen() = true;
    expectedBytes() = expected;
    Serial.printf("TUBE_PULL_HTTP request slot=%u expected=%lu\n", slot,
        static_cast<unsigned long>(expected));
    return true;
  }

  static void addBytes(uint8_t slot, size_t count, bool complete) {
    if (slot >= 2) return;
    slots()[slot].servedBytes += static_cast<uint32_t>(count);
    servedBytes() += static_cast<uint32_t>(count);
    if (count) lastProgressAt() = millis();
    if (complete && !slots()[slot].complete) {
      slots()[slot].complete = true;
      completedAt() = millis();
      Serial.printf("TUBE_PULL_HTTP body_complete slot=%u bytes=%lu\n", slot,
          static_cast<unsigned long>(slots()[slot].servedBytes));
    }
  }

  static bool allRequestedComplete() {
    bool any = false;
    for (uint8_t index = 0; index < 2; index++) {
      if (!slots()[index].requested) continue;
      any = true;
      if (!slots()[index].complete || slots()[index].servedBytes != expectedBytes())
        return false;
    }
    return any;
  }

  static bool hasIncompleteRequest() {
    for (uint8_t index = 0; index < 2; index++)
      if (slots()[index].requested && !slots()[index].complete)
        return true;
    return false;
  }

  static void failRead() {
    readFailed() = true;
    Serial.printf("TUBE_PULL_HTTP read_failed bytes=%lu expected=%lu\n",
        static_cast<unsigned long>(servedBytes()),
        static_cast<unsigned long>(expectedBytes()));
  }
};

class LegacyFirmwareResponse : public AsyncAbstractResponse {
public:
  LegacyFirmwareResponse(FirmwareImageSource& source, uint8_t slot) : _http(source), _slot(slot) {
    _code = 503;
    _contentType = F("text/plain");
    _contentLength = 0;
    if (_http.begin(FirmwareHttpMethodGet, nullptr)
        && LegacyPullTelemetry::beginRequest(_slot, _http.contentLength())) {
      _code = 200;
      _contentType = F("application/octet-stream");
      _contentLength = _http.contentLength();
      _sendContentLength = true;
      _chunked = false;
      char md5[33];
      for (uint8_t index = 0; index < sizeof(_http.artifact().imageMd5); index++)
        snprintf(md5 + index * 2, sizeof(md5) - index * 2, "%02x", _http.artifact().imageMd5[index]);
      addHeader(F("x-MD5"), md5);
    }
  }

  bool _sourceValid() const override { return _code == 200 && _contentLength > 0; }

  size_t _fillBuffer(uint8_t* buffer, size_t maxLength) override {
    // ESPAsyncWebServer may ask an abstract response to fill a zero-capacity
    // packet when its safe allocator is temporarily constrained. That is
    // backpressure, not a flash read failure; ask the server to retry.
    if (maxLength == 0) return RESPONSE_TRY_AGAIN;
    const size_t count = _http.read(buffer, maxLength);
    if (count == 0 && !_http.complete()) {
      if (_http.failed()) {
        LegacyPullTelemetry::failRead();
        return 0;
      }
      return RESPONSE_TRY_AGAIN;
    }
    LegacyPullTelemetry::addBytes(_slot, count, _http.complete());
    return count;
  }

private:
  FirmwareHttpSource _http;
  uint8_t _slot;
};

class LegacyPullHost {
public:
  // Combined length stays within Steve's gen1 FleetUpdateOffer v1 envelope,
  // even though gen0 AutoUpdateOffer is the bootstrap transport here.
  static constexpr const char* SSID = "TubesOTA";
  static constexpr const char* PASSWORD = "tubes123";
  static constexpr uint32_t REQUEST_TIMEOUT_MS = 360000;
  static constexpr uint32_t STREAM_IDLE_TIMEOUT_MS = 20000;
  static constexpr uint32_t ASSOCIATED_REQUEST_TIMEOUT_MS = 20000;
  static constexpr uint32_t FINAL_RESPONSE_DRAIN_MS = 3000;
  // Once every download that actually started has completed, leave one short
  // admission window for a second woken receiver. A station that associated but
  // never requested the image must not pin the host for the full six minutes.
  static constexpr uint32_t SECOND_RECEIVER_GRACE_MS = 60000;

  LegacyPullHost() : _source(makeTarget()) {}

  void setEnrolledMac(const uint8_t mac[6]) {
    memcpy(_enrolledMac, mac, sizeof(_enrolledMac));
    _hasEnrollment = true;
  }

  void setConcurrentCapacity(uint8_t capacity) {
    _concurrentCapacity = capacity > 1 ? 2 : 1;
  }

  void setup() {
    auto serve = [this](AsyncWebServerRequest* request, bool modern) {
      if (!_prepared || !_started) {
        request->send(503, F("text/plain"), F("migration host unavailable"));
        return;
      }
      uint8_t stationMac[6] = {0};
      if (!findRequestStation(request->client()->remoteIP(), stationMac)) {
        request->send(403, F("text/plain"), F("migration receiver not admitted"));
        return;
      }
      if (modern && !authorizeModernRequest(request, stationMac)) {
        request->send(403, F("text/plain"), F("modern peer request rejected"));
        Serial.println(F("TUBE_PULL_HTTP rejected_modern_identity"));
        return;
      }
      const int slot = admitRequestStation(stationMac);
      if (slot < 0) {
        request->send(403, F("text/plain"), F("migration receiver not admitted"));
        Serial.println(F("TUBE_PULL_HTTP rejected_non_enrolled_station"));
        return;
      }
      auto* response = new LegacyFirmwareResponse(_source, uint8_t(slot));
      if (!response->_sourceValid()) {
        delete response;
        request->send(503, F("text/plain"), F("running image unavailable"));
        return;
      }
      response->addHeader(F("Cache-Control"), F("no-store"));
      request->send(response);
      Serial.println(F("TUBE_PULL_HTTP headers_sent"));
    };
    server.on(F("/firmware.bin"), HTTP_GET,
        [serve](AsyncWebServerRequest* request) { serve(request, false); });
    server.on(F("/tubes/firmware.bin"), HTTP_GET,
        [serve](AsyncWebServerRequest* request) { serve(request, true); });
  }

  void setModernTurn(uint32_t nonce, uint16_t release,
      uint8_t hardwareFamily, uint8_t firmwareVariant) {
    _modernTurn = ModernPeerRequestIdentity();
    _modernTurn.nonce = nonce;
    _modernTurn.release = release;
    _modernTurn.hardwareFamily = hardwareFamily;
    _modernTurn.firmwareVariant = firmwareVariant;
    if (!makeModernPropagationSessionSSID(
            _sessionSSID, sizeof(_sessionSSID), nonce))
      strlcpy(_sessionSSID, SSID, sizeof(_sessionSSID));
  }

  void clearModernTurn() {
    _modernTurn = ModernPeerRequestIdentity();
    strlcpy(_sessionSSID, SSID, sizeof(_sessionSSID));
  }

  bool prepare() {
    if (_prepared) return true;
    if (!_source.inspect(_artifact) || _artifact.imageLengthBytes == 0) {
      Serial.println(F("TUBE_PULL_PREPARE failed"));
      return false;
    }
    _prepared = true;
    LegacyPullTelemetry::reset(_artifact.imageLengthBytes);
    Serial.printf("TUBE_PULL_PREPARE bytes=%lu md5=%02x%02x%02x%02x sha256=%02x%02x%02x%02x\n",
        static_cast<unsigned long>(_artifact.imageLengthBytes),
        _artifact.imageMd5[0], _artifact.imageMd5[1],
        _artifact.imageMd5[2], _artifact.imageMd5[3],
        _artifact.imageSha256[0], _artifact.imageSha256[1],
        _artifact.imageSha256[2], _artifact.imageSha256[3]);
    return true;
  }

  bool start(uint32_t now) {
    if (_started || !_prepared) return false;
    _storedAPSSID = String(apSSID);
    _storedAPPass = String(apPass);
    _storedAPBehavior = apBehavior;
    _storedAPChannel = apChannel;
    _configurationOverridden = true;
    strlcpy(apSSID, _sessionSSID, sizeof(apSSID));
    strlcpy(apPass, PASSWORD, sizeof(apPass));
    apBehavior = AP_BEHAVIOR_ALWAYS;
    apChannel = WLED_ESPNOW_WIFI_CHANNEL;
    WLED::instance().initAP(false);
    wifi_config_t apConfig = {};
    if (esp_wifi_get_config(WIFI_IF_AP, &apConfig) != ESP_OK) {
      Serial.println(F("TUBE_PULL_ERROR ap_config_read"));
      stop();
      return false;
    }
    apConfig.ap.max_connection = _concurrentCapacity;
    if (esp_wifi_set_config(WIFI_IF_AP, &apConfig) != ESP_OK) {
      Serial.println(F("TUBE_PULL_ERROR ap_config_write"));
      stop();
      return false;
    }
    dnsServer.stop();
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    const bool dnsReady = dnsServer.start(53, "*", WiFi.softAPIP());
    server.begin();
    _startedAt = now;
    _started = true;
    _restoreRequested = false;
    const bool radioReady = espnowBroadcast.startAPCarrier(apChannel);
    if (!apActive || WiFi.softAPIP() != IPAddress(4, 3, 2, 1) || !dnsReady || !radioReady) {
      Serial.printf("TUBE_PULL_HOST not_ready ap=%u ip=%s dns=%u radio=%u\n",
          apActive, WiFi.softAPIP().toString().c_str(), dnsReady, radioReady);
      stop();
      return false;
    }
    Serial.printf("TUBE_PULL_HOST ready ssid=%s ip=%s\n", _sessionSSID,
        WiFi.softAPIP().toString().c_str());
    return true;
  }

  bool shouldRestore(uint32_t now) const {
    if (!_started) return false;
    LegacyPullHostLifecycle lifecycle;
    lifecycle.startedAt = _startedAt;
    lifecycle.stationSeenAt = LegacyPullTelemetry::stationSeenAt();
    lifecycle.lastProgressAt = LegacyPullTelemetry::lastProgressAt();
    lifecycle.completedAt = LegacyPullTelemetry::completedAt();
    lifecycle.restoreRequested = _restoreRequested;
    lifecycle.readFailed = LegacyPullTelemetry::readFailed();
    lifecycle.stationSeen = LegacyPullTelemetry::stationSeen();
    lifecycle.requestSeen = LegacyPullTelemetry::requestSeen();
    lifecycle.incompleteRequest = LegacyPullTelemetry::hasIncompleteRequest();
    lifecycle.bodyComplete = bodyComplete();
    lifecycle.allLifetimeSlotsUsed = LegacyPullTelemetry::completedCount() >= 2;
    return legacyPullHostRestoreReason(lifecycle, now, REQUEST_TIMEOUT_MS,
        STREAM_IDLE_TIMEOUT_MS, ASSOCIATED_REQUEST_TIMEOUT_MS,
        FINAL_RESPONSE_DRAIN_MS, SECOND_RECEIVER_GRACE_MS)
        != LegacyPullHostKeepServing;
  }

  void requestRestore() { _restoreRequested = true; }

  void observe() {
    if (!_started) return;
    wifi_sta_list_t stations = {};
    if (esp_wifi_ap_get_sta_list(&stations) != ESP_OK || stations.num == 0) return;
    if (!LegacyPullTelemetry::stationSeen()) {
      LegacyPullTelemetry::stationSeen() = true;
      LegacyPullTelemetry::stationSeenAt() = millis();
    }
#if defined(TUBES_DIG2GO_DYNAMIC_ENROLLMENT)
    for (int index = 0; index < stations.num && index < 2; index++) {
      LegacyPullTelemetry::admit(stations.sta[index].mac);
      if (!_hasEnrollment) setEnrolledMac(stations.sta[index].mac);
    }
#endif
    if (_lastStationCount != stations.num) {
      _lastStationCount = stations.num;
      Serial.printf("TUBE_PULL_WIFI stations=%u admitted=%u\n", stations.num,
          LegacyPullTelemetry::admittedCount());
    }
  }

  bool bodyComplete() const {
    return LegacyPullTelemetry::completedAt() != 0
        && LegacyPullTelemetry::allRequestedComplete();
  }

  void stop() {
    if (!_started && !_configurationOverridden) return;
    dnsServer.stop();
    WiFi.softAPdisconnect(true);
    apActive = false;
    strlcpy(apSSID, _storedAPSSID.c_str(), sizeof(apSSID));
    strlcpy(apPass, _storedAPPass.c_str(), sizeof(apPass));
    apBehavior = _storedAPBehavior;
    apChannel = _storedAPChannel;
    _started = false;
    _prepared = false;
    _configurationOverridden = false;
    Serial.printf("TUBE_PULL_HOST stopped bytes=%lu expected=%lu complete=%u\n",
        static_cast<unsigned long>(LegacyPullTelemetry::servedBytes()),
        static_cast<unsigned long>(LegacyPullTelemetry::expectedBytes()), bodyComplete());
  }

  bool started() const { return _started; }
  bool stationSeen() const { return LegacyPullTelemetry::stationSeen(); }
  bool requestSeen() const { return LegacyPullTelemetry::requestSeen(); }
  bool capacityReached() const { return LegacyPullTelemetry::admittedCount() >= 2; }
  const char* sessionSSID() const { return _sessionSSID; }
  const char* sessionPassword() const { return PASSWORD; }
  bool hasEnrollment() const { return _hasEnrollment; }
  bool copyEnrolledMac(uint8_t mac[6]) const {
    if (!_hasEnrollment) return false;
    memcpy(mac, _enrolledMac, sizeof(_enrolledMac));
    return true;
  }
  const FirmwareImageArtifact& artifact() const { return _artifact; }

private:
  bool enrolledMacMatches(const uint8_t mac[6]) const {
    return _hasEnrollment && memcmp(_enrolledMac, mac, sizeof(_enrolledMac)) == 0;
  }

  bool onlyEnrolledStationConnected() const {
    wifi_sta_list_t stations = {};
    if (esp_wifi_ap_get_sta_list(&stations) != ESP_OK) return false;
    for (int index = 0; index < stations.num; index++)
      if (enrolledMacMatches(stations.sta[index].mac)) return true;
    return false;
  }

  bool findRequestStation(const IPAddress& remoteIp, uint8_t stationMac[6]) const {
    wifi_sta_list_t stations = {};
    esp_netif_sta_list_t netifStations = {};
    if (esp_wifi_ap_get_sta_list(&stations) != ESP_OK
        || stations.num == 0 || stations.num > 2
        || esp_netif_get_sta_list(&stations, &netifStations) != ESP_OK
        || netifStations.num == 0 || netifStations.num > 2)
      return false;
    for (int index = 0; index < netifStations.num; index++) {
      const uint32_t stationIp = netifStations.sta[index].ip.addr;
      if (stationIp != static_cast<uint32_t>(remoteIp)) continue;
      memcpy(stationMac, netifStations.sta[index].mac, 6);
      return true;
    }
    return false;
  }

  int admitRequestStation(const uint8_t stationMac[6]) {
#if !defined(TUBES_DIG2GO_DYNAMIC_ENROLLMENT)
    if (!enrolledMacMatches(stationMac)) return -1;
#endif
    if (!_hasEnrollment) {
        setEnrolledMac(stationMac);
        Serial.printf("TUBE_PULL_WIFI dynamically_enrolled=%02x:%02x:%02x:%02x:%02x:%02x\n",
            _enrolledMac[0], _enrolledMac[1], _enrolledMac[2],
            _enrolledMac[3], _enrolledMac[4], _enrolledMac[5]);
    }
    LegacyPullTelemetry::stationSeen() = true;
    return LegacyPullTelemetry::admit(stationMac);
  }

  static bool parseUnsignedParam(AsyncWebServerRequest* request, const char* name,
      int base, uint32_t maximum, uint32_t& value) {
    if (!request->hasParam(name)) return false;
    const String text = request->getParam(name)->value();
    if (!text.length()) return false;
    char* end = nullptr;
    const unsigned long parsed = strtoul(text.c_str(), &end, base);
    if (!end || *end != '\0' || parsed > maximum) return false;
    value = uint32_t(parsed);
    return true;
  }

  bool authorizeModernRequest(AsyncWebServerRequest* request,
      const uint8_t stationMac[6]) const {
    uint8_t seen = 0;
    for (size_t index = 0; index < request->params(); index++) {
      AsyncWebParameter* parameter = request->getParam(index);
      const String name = parameter->name();
      uint8_t bit = 0;
      if (name == "nonce") bit = 1 << 0;
      else if (name == "release") bit = 1 << 1;
      else if (name == "family") bit = 1 << 2;
      else if (name == "variant") bit = 1 << 3;
      else if (name == "mac") bit = 1 << 4;
      else return false;
      if (seen & bit) return false;
      seen |= bit;
    }
    if (seen != 0x1F) return false;
    ModernPeerRequestIdentity candidate;
    uint32_t parsed = 0;
    if (!parseUnsignedParam(request, "nonce", 16, UINT32_MAX, candidate.nonce)
        || !parseUnsignedParam(request, "release", 10, UINT16_MAX, parsed))
      return false;
    candidate.release = uint16_t(parsed);
    if (!parseUnsignedParam(request, "family", 10, UINT8_MAX, parsed)) return false;
    candidate.hardwareFamily = uint8_t(parsed);
    if (!parseUnsignedParam(request, "variant", 10, UINT8_MAX, parsed)) return false;
    candidate.firmwareVariant = uint8_t(parsed);
    if (!request->hasParam("mac")
        || !parseModernPeerMac(request->getParam("mac")->value().c_str(), candidate.mac))
      return false;
    return authorizeModernPeerRequest(candidate, _modernTurn, stationMac);
  }
  static FirmwareTargetContract makeTarget() {
    FirmwareTargetContract target;
    target.hardwareFamily = TubeHardwareDig2Go;
    target.chipFamily = FirmwareChipEsp32;
    return target;
  }

  RunningFirmwareImageSource _source;
  FirmwareImageArtifact _artifact;
  String _storedAPSSID;
  String _storedAPPass;
  byte _storedAPBehavior = AP_BEHAVIOR_BOOT_NO_CONN;
  byte _storedAPChannel = 6;
  bool _prepared = false;
  bool _started = false;
  bool _configurationOverridden = false;
  bool _restoreRequested = false;
  uint8_t _enrolledMac[6] = {0};
  bool _hasEnrollment = false;
  bool _foreignStationLogged = false;
  uint8_t _lastStationCount = 0;
  uint8_t _concurrentCapacity = 1;
  uint32_t _startedAt = 0;
  char _sessionSSID[25] = "TubesOTA";
  ModernPeerRequestIdentity _modernTurn;
};

#endif

#pragma once

#include "wled.h"
#include "fastled_compat.h"

#include "util.h"
#include "options.h"

// #define USERADIO

#include "FX.h"

#include "virtual_strip.h"
#include "led_strip.h"
#include "master.h"

#include "controller.h"
#include "debug.h"
#include "dig2go_push_source_adapter.h"
#include "legacy_pull_host.h"
#include "legacy_pull_rendezvous.h"
#include "modern_propagation_lease_storage.h"

#ifndef PIXEL_COUNTS
#define PIXEL_COUNTS DEFAULT_LED_COUNT
#endif

#ifndef DATA_PINS
#define DATA_PINS DEFAULT_LED_PIN
#endif

#ifndef LED_TYPES
#define LED_TYPES DEFAULT_LED_TYPE
#endif

#ifndef DEFAULT_LED_COLOR_ORDER
#define DEFAULT_LED_COLOR_ORDER COL_ORDER_GRB
#endif

#define MASTER_PIN 25
#define LEGACY_PIN 32  // DigUno Q4


class TubesUsermod : public Usermod
#if TUBES_ENABLE_DIG2GO_PUSH_BRIDGE
    , public tubes_p2p::Dig2GoPushBridgeHooks
#endif
{
  private:
    PatternController controller = PatternController();
    DebugController debug = DebugController(controller);
    Master master = Master(controller);
    bool isLegacy = false;
    bool checkedLedSegments = false;
#if defined(TUBES_DIG2GO_LEGACY_PULL_HOST)
    LegacyPullHost legacyPullHost;
    LegacyPullRendezvous legacyPullRendezvous;
    bool legacyPullOfferSent = false;
    bool legacyPullWakeAccepted = false;
    bool legacyPullNeedsRestore = false;
    bool legacyPullRestoreStarted = false;
    bool legacyPullBodyServed = false;
    bool legacyPullHealthVerified = false;
    bool legacyPullNoReceiver = false;
    bool legacyHostRetired = false;
    uint32_t legacyPullHealthNonce = 0;
    uint32_t legacyPullNextHealthRequest = 0;
    uint32_t legacyPullHealthDeadline = 0;
    uint32_t legacyPullFleetNonce = 0;
    bool modernPropagationTurn = false;
    bool modernPropagationLeaseCleared = false;
    uint32_t modernPropagationNonce = 0;
    uint32_t modernPropagationStartAt = 0;
    bool modernPropagationWaitForSourceQuiet = false;
    uint32_t modernPropagationSourceNonce = 0;
    uint32_t modernPropagationBatonUntil = 0;
    uint32_t modernPropagationNextBatonAt = 0;
    bool legacyMigrationBootCandidate = false;
    bool currentReleaseMarkerWritten = false;
    static constexpr uint32_t LEGACY_BOOTSTRAP_BATON_WINDOW_MS = 60000;
    static constexpr uint32_t LEGACY_BOOTSTRAP_SOURCE_QUIET_MS = 5000;
    static constexpr uint32_t LEGACY_BOOTSTRAP_BATON_GRACE_MS = 15000;
#endif
#if TUBES_ENABLE_DIG2GO_PUSH_BRIDGE
    tubes_p2p::Dig2GoPushSourceAdapter dig2GoSourceAdapter;
    tubes_p2p::Dig2GoPushBridgeRuntime dig2GoPushBridge{*this};
    uint8_t dig2GoEnrolledMac[6] = {0};
    uint32_t dig2GoHealthNonce = 0;
    bool dig2GoJoinStarted = false;
    bool dig2GoRestoreStarted = false;
    bool dig2GoHealthRequested = false;
    bool dig2GoIsPrime = false;
#if defined(TUBES_DIG2GO_PUSH_AUTO_TRIGGER)
    tubes_p2p::Dig2GoAutoTrigger dig2GoAutoTrigger;
    bool dig2GoAutoAttempted = false;
#endif

    static TubesUsermod*& dig2GoBridgeInstance() {
      static TubesUsermod* instance = nullptr;
      return instance;
    }

    static void armDig2GoBridge(const uint8_t targetMac[6], uint32_t timeoutMs) {
      if (dig2GoBridgeInstance())
        dig2GoBridgeInstance()->armDig2GoBridgeInternal(targetMac, timeoutMs);
    }

    static void observeDig2GoReport(const DeviceReportMessage& report) {
      if (dig2GoBridgeInstance())
        dig2GoBridgeInstance()->observeDig2GoReportInternal(report);
    }

    static bool acceptDig2GoPropagation(const FleetUpdateOffer& offer) {
      return dig2GoBridgeInstance()
          && dig2GoBridgeInstance()->acceptDig2GoPropagationInternal(offer);
    }

    bool acceptDig2GoPropagationInternal(const FleetUpdateOffer& offer) {
#if defined(TUBES_DIG2GO_LEGACY_PULL_HOST)
      if (!(offer.flags & FleetUpdatePropagate)
          || offer.tubesVersion != RELEASE_VERSION
          || legacyPullHost.started())
        return false;
      if (offer.serverPort != 0) {
        if (modernPropagationWaitForSourceQuiet
            && modernPropagationSourceNonce == offer.nonce) {
          modernPropagationStartAt = millis() + LEGACY_BOOTSTRAP_SOURCE_QUIET_MS;
          return true;
        }
        if (!isFreshLegacyBootstrapBaton(
                offer, RELEASE_VERSION, millis(), LEGACY_BOOTSTRAP_BATON_WINDOW_MS,
                legacyMigrationBootCandidate)
            || !legacyPullCanAcceptExplicitTurn(modernPropagationTurn))
          return false;
        currentReleaseMarkerWritten = writeCurrentReleaseMarker(RELEASE_VERSION);
        legacyMigrationBootCandidate = false;
        initializePeerPropagationTurn();
        modernPropagationTurn = true;
        modernPropagationWaitForSourceQuiet = true;
        modernPropagationSourceNonce = offer.nonce;
        modernPropagationNonce = esp_random();
        if (modernPropagationNonce == 0) modernPropagationNonce = 1;
        modernPropagationStartAt = millis() + LEGACY_BOOTSTRAP_SOURCE_QUIET_MS;
        Serial.printf("FLEET_PROPAGATION legacy_baton source=%08lX offer=%08lX\n",
            static_cast<unsigned long>(offer.nonce),
            static_cast<unsigned long>(modernPropagationNonce));
        return true;
      }
      if (!legacyPullCanAcceptExplicitTurn(modernPropagationTurn))
        return false;
      initializePeerPropagationTurn();
      modernPropagationTurn = true;
      modernPropagationWaitForSourceQuiet = false;
      modernPropagationSourceNonce = 0;
      modernPropagationNonce = esp_random();
      if (modernPropagationNonce == 0) modernPropagationNonce = 1;
      modernPropagationStartAt = millis() + 1000;
      Serial.printf("FLEET_PROPAGATION commanded source=%08lX offer=%08lX\n",
          static_cast<unsigned long>(offer.nonce),
          static_cast<unsigned long>(modernPropagationNonce));
      return true;
#else
      (void)offer;
      return false;
#endif
    }

#if defined(TUBES_DIG2GO_LEGACY_PULL_HOST)
    void initializePeerPropagationTurn() {
      legacyPullOfferSent = false;
      legacyPullWakeAccepted = false;
      legacyPullNeedsRestore = false;
      legacyPullRestoreStarted = false;
      legacyPullBodyServed = false;
      legacyPullHealthVerified = false;
      legacyPullNoReceiver = false;
      legacyHostRetired = false;
      legacyPullHealthNonce = 0;
      legacyPullNextHealthRequest = 0;
      legacyPullHealthDeadline = 0;
      legacyPullFleetNonce = 0;
      modernPropagationTurn = false;
      modernPropagationLeaseCleared = false;
      modernPropagationNonce = 0;
      modernPropagationStartAt = 0;
      modernPropagationWaitForSourceQuiet = false;
      modernPropagationSourceNonce = 0;
      modernPropagationBatonUntil = 0;
      modernPropagationNextBatonAt = 0;
      legacyPullHost.clearModernTurn();
    }

    void finishPeerPropagationTurn() {
      // Preserve offerSent/hostRetired so PRIME and test-only boot gates cannot
      // immediately start another host. A fresh explicit command is the only
      // operation that reinitializes those admission latches.
      legacyPullWakeAccepted = false;
      legacyPullNeedsRestore = false;
      legacyPullRestoreStarted = false;
      legacyPullBodyServed = false;
      legacyPullHealthVerified = false;
      legacyPullNoReceiver = false;
      legacyPullHealthNonce = 0;
      legacyPullNextHealthRequest = 0;
      legacyPullHealthDeadline = 0;
      legacyPullFleetNonce = 0;
      modernPropagationTurn = false;
      modernPropagationLeaseCleared = false;
      modernPropagationNonce = 0;
      modernPropagationStartAt = 0;
      modernPropagationWaitForSourceQuiet = false;
      modernPropagationSourceNonce = 0;
      modernPropagationBatonUntil = 0;
      modernPropagationNextBatonAt = 0;
      legacyPullHost.clearModernTurn();
    }
#endif

    void armDig2GoBridgeInternal(const uint8_t targetMac[6], uint32_t timeoutMs) {
      if (memcmp(targetMac, dig2GoEnrolledMac, sizeof(dig2GoEnrolledMac)) != 0) {
        Serial.println(F("TUBE_PUSH_ERROR mac_not_enrolled"));
        return;
      }
      dig2GoJoinStarted = false;
      dig2GoRestoreStarted = false;
      dig2GoHealthRequested = false;
      dig2GoHealthNonce = 0;
      if (!dig2GoPushBridge.arm(targetMac, timeoutMs)) {
        Serial.println(F("TUBE_PUSH_ERROR unavailable"));
        return;
      }
      controller.setDig2GoBridgeOverlay(Ready);
      Serial.println(F("TUBE_PUSH armed"));
    }

    void observeDig2GoReportInternal(const DeviceReportMessage& report) {
      const bool standardOutput = (report.ledCount == 112 || report.ledCount == 150)
          && report.busCount == 1 && report.ledPin == 16 && report.ledType == 22;
      const bool newBoot = report.uptimeSeconds <= 300;
#if defined(TUBES_DIG2GO_LEGACY_PULL_HOST)
      if (legacyPullBodyServed && !legacyPullHealthVerified
          && report.nonce == legacyPullHealthNonce) {
        const bool exactTarget = memcmp(report.mac, dig2GoEnrolledMac, sizeof(report.mac)) == 0;
        const bool healthy = exactTarget
            && report.tubesVersion == RELEASE_VERSION
            && report.hardwareFamily == TubeHardwareDig2Go
            && report.firmwareVariant == TubeVariantStandard
            && report.releaseHash == WLED_BUILD_DESCRIPTION.hash
            && (report.meshFlags & DeviceReportMeshStarted)
            && standardOutput && newBoot;
        if (healthy) {
          legacyPullHealthVerified = true;
          controller.setDig2GoBridgeOverlay(Received);
          Serial.printf("TUBE_PULL_VERIFY healthy mac=%02x%02x%02x%02x%02x%02x release=%u hash=%08lx uptime=%lu\n",
              report.mac[0], report.mac[1], report.mac[2], report.mac[3], report.mac[4], report.mac[5],
              report.tubesVersion, static_cast<unsigned long>(report.releaseHash),
              static_cast<unsigned long>(report.uptimeSeconds));
        } else {
          Serial.printf("TUBE_PULL_VERIFY rejected mac=%u release=%u family=%u variant=%u hash=%u mesh=%u output=%u fresh=%u\n",
              exactTarget, report.tubesVersion == RELEASE_VERSION,
              report.hardwareFamily == TubeHardwareDig2Go,
              report.firmwareVariant == TubeVariantStandard,
              report.releaseHash == WLED_BUILD_DESCRIPTION.hash,
              !!(report.meshFlags & DeviceReportMeshStarted), standardOutput, newBoot);
        }
        return;
      }
#endif
      if (report.nonce != dig2GoHealthNonce
          || report.tubesVersion != RELEASE_VERSION
          || report.hardwareFamily != TubeHardwareDig2Go
          || report.firmwareVariant != TubeVariantStandard
          || report.releaseHash != WLED_BUILD_DESCRIPTION.hash
          || !(report.meshFlags & DeviceReportMeshStarted)
          || !standardOutput || !newBoot)
        return;
      if (dig2GoPushBridge.observeFreshV15Health(report.mac, millis())) {
        controller.setDig2GoBridgeOverlay(Complete);
        Serial.println(F("TUBE_PUSH healthy"));
      }
    }

    static bool parseEnrolledMac(uint8_t mac[6]) {
#if defined(TUBES_DIG2GO_DYNAMIC_ENROLLMENT)
      memset(mac, 0, 6);
      return true;
#else
      return parseDeviceReportMac(TUBES_DIG2GO_PUSH_ENROLLED_MAC, mac);
#endif
    }

#if defined(TUBES_DIG2GO_PUSH_PRIME_MAC)
    static bool isPrimeDevice() {
      uint8_t expected[6] = {0};
      uint8_t local[6] = {0};
      if (!parseDeviceReportMac(TUBES_DIG2GO_PUSH_PRIME_MAC, expected)) return false;
      Network.localMAC(local);
      return memcmp(expected, local, sizeof(local)) == 0;
    }
#endif

    uint32_t now() const override { return millis(); }

    bool sendLegacyV15Selection(const uint8_t targetMac[6]) override {
      if (memcmp(targetMac, dig2GoEnrolledMac, sizeof(dig2GoEnrolledMac)) != 0)
        return false;
      // Legacy receivers do not acknowledge the V15 selection offer. Repeat
      // it for a bounded window before suspending ESP-NOW so a single lost
      // broadcast cannot prevent the receiver from opening WLED-UPDATE.
      for (uint8_t attempt = 0;
          attempt < tubes_p2p::DIG2GO_SELECTION_BROADCAST_ATTEMPTS;
          attempt++) {
        controller.sendLegacyV15UpdateSelection();
        delay(tubes_p2p::DIG2GO_SELECTION_BROADCAST_INTERVAL_MS);
      }
      return true;
    }

    bool pauseTubesRadio() override {
      if (!controller.stopMeshRadioForDig2Go()) return false;
      const uint32_t deadline = millis() + 2000;
      while (!controller.meshRadioStoppedForDig2Go()
          && static_cast<int32_t>(deadline - millis()) > 0)
        delay(1);
      if (controller.meshRadioStoppedForDig2Go()) return true;
      controller.restoreMeshRadioAfterDig2Go();
      return false;
    }

    bool beginExclusiveWledJoin() override {
      if (!dig2GoJoinStarted) {
        dig2GoJoinStarted = WLED::instance().beginTemporaryStaLease(
            tubes_p2p::DIG2GO_UPDATE_SSID,
            tubes_p2p::DIG2GO_UPDATE_PASSWORD);
      }
      return dig2GoJoinStarted;
    }

    bool joinOwnerExclusive() const override { return dig2GoJoinStarted; }

    bool updateAccessPointConnected() const override {
      return WiFi.status() == WL_CONNECTED
          && WiFi.SSID() == tubes_p2p::DIG2GO_UPDATE_SSID
          && WiFi.gatewayIP() == IPAddress(4, 3, 2, 1);
    }

    bool probeUpdateAccessPointReachability() override {
      return dig2GoSourceAdapter.probeReachability();
    }

    tubes_p2p::Dig2GoSourceAdapterResult inspectSelectedTarget(
        const tubes_p2p::Dig2GoTargetAdmission& admission,
        tubes_p2p::LegacyDig2GoEvidence& evidence) override {
      return dig2GoSourceAdapter.inspectTarget(admission, evidence);
    }

    tubes_p2p::FirmwarePostResult uploadActiveImage() override {
#if defined(TUBES_DIG2GO_STEP3_DIAGNOSTIC) || defined(TUBES_DIG2GO_READINESS_DELAY_TEST) || defined(TUBES_DIG2GO_INSPECTION_ONLY_TEST)
      return tubes_p2p::FirmwarePostHttpRejected;
#else
      FirmwareTargetContract target;
      target.hardwareFamily = TubeHardwareDig2Go;
      target.chipFamily = FirmwareChipEsp32;
      const tubes_p2p::FirmwarePostResult result = dig2GoSourceAdapter.uploadRunningImage(target);
      Serial.printf("TUBE_PUSH_HTTP result=%u\n", static_cast<unsigned>(result));
      return result;
#endif
    }

    bool restoreTubesRadio() override {
      if (!dig2GoRestoreStarted) {
        if (!WLED::instance().endTemporaryStaLease())
          return false;
        if (!controller.restoreMeshRadioAfterDig2Go())
          return false;
        dig2GoRestoreStarted = true;
        return false;
      }
      return controller.meshRadioStartedAfterDig2Go();
    }

    void updateDig2GoPushBridge() {
      const tubes_p2p::PushBridgeState before = dig2GoPushBridge.state();
      dig2GoPushBridge.update();
      const tubes_p2p::PushBridgeState state = dig2GoPushBridge.state();
      if (state == tubes_p2p::PushBridgeAwaitingHealth && !dig2GoHealthRequested) {
        dig2GoHealthRequested = true;
        dig2GoHealthNonce = controller.requestDig2GoHealthReport(dig2GoEnrolledMac);
      }
      if (state == tubes_p2p::PushBridgeFailed && before != tubes_p2p::PushBridgeFailed) {
        controller.setDig2GoBridgeOverlay(Failed);
        Serial.println(F("TUBE_PUSH failed"));
      } else if (state == tubes_p2p::PushBridgeHealthy && before != tubes_p2p::PushBridgeHealthy) {
        controller.setDig2GoBridgeOverlay(Complete);
        Serial.println(F("TUBE_PUSH diagnostic passed; upload disabled"));
      } else if (state == tubes_p2p::PushBridgeUploading || state == tubes_p2p::PushBridgeRestoringMesh
          || state == tubes_p2p::PushBridgeAwaitingHealth) {
        controller.setDig2GoBridgeOverlay(Received);
      }
    }
#endif

    void drawDig2GoConnectionDiagnostic() {
#if defined(TUBES_DIG2GO_LEGACY_PULL_HOST)
      const bool diagnosticHost = dig2GoIsPrime
#if defined(TUBES_DIG2GO_LEGACY_BOOT_FALLBACK_TEST)
          || !dig2GoIsPrime
#endif
          || modernPropagationTurn
          ;
      if (diagnosticHost && legacyPullOfferSent && !legacyHostRetired) {
        const bool terminal = legacyPullNeedsRestore;
        const auto stageColor = [terminal](bool passed) {
          return passed ? CRGB::Green : (terminal ? CRGB::Red : CRGB::Blue);
        };
        CRGB finalStage = stageColor(legacyPullHealthVerified);
        if (legacyPullBodyServed && !legacyPullHealthVerified)
          finalStage = CRGB::Yellow;
        const CRGB stages[5] = {
          stageColor(legacyPullWakeAccepted),
          stageColor(legacyPullHost.stationSeen()),
          stageColor(legacyPullHost.requestSeen()),
          stageColor(legacyPullHost.bodyComplete()),
          finalStage
        };
        for (uint8_t pair = 0; pair < 5; pair++) {
          strip.setPixelColor(pair * 2, stages[pair]);
          strip.setPixelColor(pair * 2 + 1, stages[pair]);
        }
        return;
      }
#elif defined(TUBES_DIG2GO_INSPECTION_ONLY_TEST)
      const auto state = dig2GoPushBridge.state();
      if (state != tubes_p2p::PushBridgeHealthy && state != tubes_p2p::PushBridgeFailed) return;
      CRGB result = CRGB::Green;
      switch (dig2GoPushBridge.inspectionResult()) {
        case tubes_p2p::Dig2GoSourceAdapterAccepted: result = CRGB::Green; break;
        case tubes_p2p::Dig2GoSourceAdapterHttpFailed: result = CRGB(255, 96, 0); break;
        case tubes_p2p::Dig2GoSourceAdapterResponseTooLarge: result = CRGB::Blue; break;
        case tubes_p2p::Dig2GoSourceAdapterJsonInvalid: result = CRGB::Blue; break;
        case tubes_p2p::Dig2GoSourceAdapterIdentityRejected: result = CRGB::Purple; break;
        case tubes_p2p::Dig2GoSourceAdapterConfigurationRejected: result = CRGB::Yellow; break;
      }
      for (uint8_t pixel = 0; pixel < 10; pixel++) strip.setPixelColor(pixel, result);
#elif defined(TUBES_DIG2GO_STEP3_DIAGNOSTIC)
      const auto state = dig2GoPushBridge.state();
      if (state != tubes_p2p::PushBridgeHealthy && state != tubes_p2p::PushBridgeFailed) return;
      const CRGB join = dig2GoPushBridge.joinPassed() ? CRGB::Green : CRGB::Red;
      const CRGB http = dig2GoPushBridge.httpPassed() ? CRGB::Green : CRGB::Red;
      for (uint8_t pixel = 0; pixel < 5; pixel++) strip.setPixelColor(pixel, join);
      for (uint8_t pixel = 5; pixel < 10; pixel++) strip.setPixelColor(pixel, http);
#endif
    }

    void randomize() {
      randomSeed(esp_random());
      random16_set_seed(random(0, 65535));
      random16_add_entropy(esp_random());
    }

    void recoverLedBussesIfNeeded() {
      if (strip.getLengthTotal() > 0 || BusManager::getNumBusses() > 0 || !busConfigs.empty()) return;

      constexpr unsigned defDataTypes[] = {LED_TYPES};
      constexpr unsigned defDataPins[] = {DATA_PINS};
      constexpr unsigned defCounts[] = {PIXEL_COUNTS};
      constexpr unsigned defNumTypes = sizeof(defDataTypes) / sizeof(defDataTypes[0]);
      constexpr unsigned defNumPins = sizeof(defDataPins) / sizeof(defDataPins[0]);
      constexpr unsigned defNumCounts = sizeof(defCounts) / sizeof(defCounts[0]);

      unsigned pinsIndex = 0;
      unsigned start = 0;
      for (unsigned i = 0; i < WLED_MAX_BUSSES; i++) {
        uint8_t pins[OUTPUT_MAX_PINS] = {255, 255, 255, 255, 255};
        unsigned dataType = defDataTypes[(i < defNumTypes) ? i : defNumTypes - 1];
        unsigned busPins = Bus::getNumberOfPins(dataType);
        if (pinsIndex + busPins > defNumPins) break;

        for (unsigned j = 0; j < busPins && j < OUTPUT_MAX_PINS; j++) {
          pins[j] = defDataPins[pinsIndex + j];
        }
        pinsIndex += busPins;

        unsigned count = defCounts[(i < defNumCounts) ? i : defNumCounts - 1];
        if (Bus::isPWM(dataType) || Bus::isOnOff(dataType)) count = 1;

        busConfigs.emplace_back(dataType, pins, start, count, DEFAULT_LED_COLOR_ORDER, false, 0, RGBW_MODE_MANUAL_ONLY, 0, LED_MILLIAMPS_DEFAULT, ABL_MILLIAMPS_DEFAULT, 0);
        start += count;
      }

      if (!busConfigs.empty()) {
        // A legacy config may leave a zero-length segment running an effect
        // such as Flow. WLED services that segment before it consumes
        // doInitBusses later in the same loop, and some native effects divide
        // by their derived zero zone length. Keep the placeholder inert for
        // that single loop; finalizeInit/fixInvalidSegments restores the real
        // bus-backed segment immediately afterward.
        strip.getMainSegment().setMode(FX_MODE_STATIC);
        doInitBusses = true;
        Serial.println(F("Tubes: recovered default LED bus config"));
      }
    }

    void recoverLedSegmentsIfNeeded() {
      if (checkedLedSegments || doInitBusses || strip.getLengthTotal() == 0 || BusManager::getNumBusses() == 0) return;

      bool needsSegments = strip.getSegmentsNum() == 0;
      if (!needsSegments) {
        const Segment& seg = strip.getMainSegment();
        // WLED may retain its one-pixel placeholder after the LED bus is restored.
        needsSegments = seg.length() == 0
          || seg.start >= strip.getLengthTotal()
          || (strip.getSegmentsNum() == 1 && seg.start == 0 && seg.stop == 1 && strip.getLengthTotal() > 1);
      }

      if (needsSegments) {
        strip.makeAutoSegments(true);
        Serial.println(F("Tubes: recovered LED segment config"));
      }
      checkedLedSegments = true;
    }

  public:
    void setup() {
      randomize();

      recoverLedBussesIfNeeded();
      // Start timing
      globalTimer.setup();
      controller.setup();
#if defined(TUBES_DIG2GO_LEGACY_PULL_HOST)
      legacyPullHost.setup();
      currentReleaseMarkerWritten = hasCurrentReleaseMarker(RELEASE_VERSION);
      legacyMigrationBootCandidate = !currentReleaseMarkerWritten
          && esp_reset_reason() == ESP_RST_SW;
      if (!currentReleaseMarkerWritten && !legacyMigrationBootCandidate)
        currentReleaseMarkerWritten = writeCurrentReleaseMarker(RELEASE_VERSION);
      Serial.printf("FLEET_PROPAGATION bootstrap_candidate=%u marker=%u reset=%u\n",
          legacyMigrationBootCandidate, currentReleaseMarkerWritten,
          static_cast<unsigned>(esp_reset_reason()));
      ModernPropagationLeaseRecord modernLease;
      if (claimStoredModernPropagationLease(modernLease, RELEASE_VERSION)) {
        currentReleaseMarkerWritten = writeCurrentReleaseMarker(RELEASE_VERSION)
            || currentReleaseMarkerWritten;
        legacyMigrationBootCandidate = false;
        modernPropagationTurn = true;
        modernPropagationNonce = esp_random();
        if (modernPropagationNonce == 0) modernPropagationNonce = 1;
        modernPropagationStartAt = millis() + 5000;
        Serial.printf("FLEET_PROPAGATION claimed release=%u source=%08lX offer=%08lX\n",
            modernLease.tubesVersion,
            static_cast<unsigned long>(modernLease.sourceNonce),
            static_cast<unsigned long>(modernPropagationNonce));
      }
#endif
#if TUBES_ENABLE_DIG2GO_PUSH_BRIDGE
      dig2GoBridgeInstance() = this;
      if (!parseEnrolledMac(dig2GoEnrolledMac)) {
        memset(dig2GoEnrolledMac, 0, sizeof(dig2GoEnrolledMac));
        Serial.println(F("TUBE_PUSH disabled: invalid enrolled MAC"));
      }
#if defined(TUBES_DIG2GO_LEGACY_PULL_HOST)
#if !defined(TUBES_DIG2GO_DYNAMIC_ENROLLMENT)
      else {
        legacyPullHost.setEnrolledMac(dig2GoEnrolledMac);
      }
#endif
#endif
      controller.setDig2GoBridgeCallbacks(
          armDig2GoBridge, observeDig2GoReport, acceptDig2GoPropagation);
#if defined(TUBES_DIG2GO_PUSH_AUTO_TRIGGER)
      dig2GoAutoTrigger.booted(millis());
      dig2GoIsPrime = isPrimeDevice();
#if defined(TUBES_DIG2GO_LEGACY_BOOT_FALLBACK_TEST)
      Serial.println(dig2GoIsPrime
          ? F("TUBE_PUSH_AUTO armed: PRIME legacy host at 15s")
          : F("TUBE_PUSH_AUTO test-only boot fallback host at 15s"));
#else
      Serial.println(dig2GoIsPrime
          ? F("TUBE_PUSH_AUTO armed: PRIME waiting for mesh-ready boot window")
          : F("TUBE_PUSH_AUTO disabled: this device is not PRIME"));
#endif
#endif
#endif

      if (!controller.isHomeLightRole()) {
        if (PinManager::isPinOk(MASTER_PIN)) {
          pinMode(MASTER_PIN, INPUT_PULLUP);
          if(PinManager::isPinOk(LEGACY_PIN)) {
            pinMode(LEGACY_PIN, INPUT_PULLUP);
          }
          isLegacy = (digitalRead(MASTER_PIN) == LOW);
        }

        // Override some behaviors on Tubes that render the mesh patterns.
        bootPreset = 0;  // Try to prevent initial playlists from starting
        transitionDelay = 8000;   // Fade them for a long time
        strip.setTransition(transitionDelay);
        strip.setTargetFps(60);
        strip.setCCT(100);
      }

      if (controller.isMasterRole()) {
        master.setup();
      }
      debug.setup();
    }

    void loop()
    {
      EVERY_N_MILLISECONDS(10000) {
        randomize();
      }

      globalTimer.update();
      recoverLedSegmentsIfNeeded();

      if (controller.isMasterRole()) {
        master.update();
      }
      controller.update();
#if defined(TUBES_DIG2GO_LEGACY_PULL_HOST)
      if (!currentReleaseMarkerWritten
          && millis() > LEGACY_BOOTSTRAP_BATON_WINDOW_MS) {
        currentReleaseMarkerWritten = writeCurrentReleaseMarker(RELEASE_VERSION);
        legacyMigrationBootCandidate = false;
        Serial.printf("FLEET_PROPAGATION marker_written=%u\n",
            currentReleaseMarkerWritten);
      }
      const uint32_t legacyHostStartMs = modernPropagationTurn
          ? modernPropagationStartAt : 15000;
      const bool legacyBootEligible = dig2GoIsPrime
#if defined(TUBES_DIG2GO_LEGACY_BOOT_FALLBACK_TEST)
          || !dig2GoIsPrime
#endif
          ;
      const bool legacyHostEligible = legacyPullAutomaticHostEligible(
          legacyBootEligible, modernPropagationTurn, legacyPullOfferSent,
          legacyHostRetired);
      if (legacyHostEligible
          && millis() >= legacyHostStartMs && controller.meshRadioStartedAfterDig2Go()
          && !controller.deviceUpdateInProgress()) {
        legacyPullOfferSent = true;
        if (!legacyPullHost.prepare()) {
          controller.setDig2GoBridgeOverlay(Failed);
          Serial.println(F("TUBE_PULL failed: running image unavailable"));
          if (modernPropagationTurn) {
            clearModernPropagationLease();
            finishPeerPropagationTurn();
          }
        } else {
          // One explicit field turn can contain both deployed legacy clients
          // and current FleetUpdateOffer receivers. Serialize both lifetime
          // slots because a legacy client treats momentary backpressure as EOF.
          legacyPullHost.setConcurrentCapacity(1);
          if (modernPropagationTurn) {
            legacyPullHost.setModernTurn(modernPropagationNonce, RELEASE_VERSION,
                TUBES_HARDWARE_FAMILY, TUBES_FIRMWARE_VARIANT);
          } else {
            legacyPullHost.clearModernTurn();
          }
          if (!legacyPullHost.start(millis())) {
            controller.setDig2GoBridgeOverlay(Failed);
            Serial.println(F("TUBE_PULL failed: host start"));
            legacyPullNeedsRestore = true;
          } else {
            legacyPullRendezvous.begin(millis());
            Serial.println(F("TUBE_PULL_RENDEZVOUS started"));
          }
        }
      }
      legacyPullHost.observe();
      switch (legacyPullRendezvous.update(millis(), legacyPullHost.capacityReached())) {
        case LegacyPullRendezvousSendWake: {
          if (modernPropagationTurn) {
            FleetUpdateOffer offer;
            const uint8_t serverAddress[4] = {4, 3, 2, 1};
            const bool madeOffer = makeModernPropagationOffer(
                offer, RELEASE_VERSION, modernPropagationNonce, serverAddress,
                80, 1000, legacyPullHost.sessionSSID(),
                legacyPullHost.sessionPassword());
            legacyPullWakeAccepted = (madeOffer
                && controller.sendFleetPullUpdateOffer(offer))
                || legacyPullWakeAccepted;
          }
          // The deployed wake is additive during a modern turn. Old Dig2Gos
          // understand only this command; current Dig2Gos ignore it because
          // the offered release is not newer and consume FleetUpdateOffer.
          AutoUpdateOffer legacyOffer;
          legacyOffer.version = RELEASE_VERSION;
          strlcpy(legacyOffer.ssid, legacyPullHost.sessionSSID(), sizeof(legacyOffer.ssid));
          strlcpy(legacyOffer.password, legacyPullHost.sessionPassword(), sizeof(legacyOffer.password));
          legacyOffer.host = IPAddress(4, 3, 2, 1);
          legacyPullWakeAccepted = controller.sendLegacyPullUpdateOffer(legacyOffer)
              || legacyPullWakeAccepted;
#if !defined(TUBES_DIG2GO_DYNAMIC_ENROLLMENT)
          if (legacyPullFleetNonce == 0) {
            legacyPullFleetNonce = esp_random();
            if (legacyPullFleetNonce == 0) legacyPullFleetNonce = 1;
          }
          FleetUpdateOffer fleet;
          fleet.flags = FleetUpdateForce;
          fleet.tubesVersion = RELEASE_VERSION;
          fleet.nonce = legacyPullFleetNonce;
          fleet.serverAddress[0] = 4;
          fleet.serverAddress[1] = 3;
          fleet.serverAddress[2] = 2;
          fleet.serverAddress[3] = 1;
          fleet.serverPort = 80;
          // The legacy wake remains a one-hop broadcast, but current receivers
          // get Steve's existing targeted offer. A wildcard here lets the host
          // consume its own offer and abandon radio ownership mid-transfer.
          fleet.targetDeviceId = TUBES_DIG2GO_PUSH_ENROLLED_DEVICE_ID;
          setFleetUpdateCredentials(fleet, legacyPullHost.sessionSSID(),
              legacyPullHost.sessionPassword());
          // The exact-target diagnostic uses Steve's modern receiver path too.
          // Bind its HTTP request to the same identity contract as a propagated
          // peer without changing the deployed legacy /firmware.bin endpoint.
          legacyPullHost.setModernTurn(legacyPullFleetNonce, RELEASE_VERSION,
              TUBES_HARDWARE_FAMILY, TUBES_FIRMWARE_VARIANT);
          legacyPullWakeAccepted = controller.sendFleetPullUpdateOffer(fleet)
              || legacyPullWakeAccepted;
#endif
          if (legacyPullRendezvous.wakeAttempts() == 1
              || legacyPullRendezvous.wakeAttempts() % 10 == 0)
            Serial.printf("TUBE_PULL_WAKE attempts=%u radio_accepted=%u\n",
                legacyPullRendezvous.wakeAttempts(), legacyPullWakeAccepted);
          break;
        }
        case LegacyPullRendezvousStationArrived:
          Serial.printf("TUBE_PULL_RENDEZVOUS station attempts=%u\n",
              legacyPullRendezvous.wakeAttempts());
          break;
        case LegacyPullRendezvousTimedOut:
          Serial.printf("TUBE_PULL_RENDEZVOUS timeout attempts=%u\n",
              legacyPullRendezvous.wakeAttempts());
          legacyPullHost.requestRestore();
          legacyPullNoReceiver = !legacyPullHost.stationSeen();
          break;
        default:
          break;
      }
      if (legacyPullHost.shouldRestore(millis())) {
        legacyPullBodyServed = legacyPullHost.bodyComplete();
        legacyPullRendezvous.cancel();
#if defined(TUBES_DIG2GO_DYNAMIC_ENROLLMENT)
        legacyPullHost.copyEnrolledMac(dig2GoEnrolledMac);
#endif
        legacyPullHost.stop();
        legacyPullNeedsRestore = true;
        controller.setDig2GoBridgeOverlay(legacyPullBodyServed ? Received
            : (legacyPullNoReceiver ? Idle : Failed));
      }
      if (legacyPullNeedsRestore && !legacyPullRestoreStarted) {
        legacyPullRestoreStarted = controller.restoreMeshRadioAfterDig2Go();
        if (legacyPullRestoreStarted)
          Serial.println(F("TUBE_PULL_RESTORE radio_requested"));
      }
      if (legacyPullRestoreStarted && controller.meshRadioStartedAfterDig2Go()) {
        if (legacyPullNoReceiver && !legacyPullBodyServed && !legacyHostRetired) {
          legacyHostRetired = true;
          controller.setDig2GoBridgeOverlay(Idle);
          Serial.println(F("TUBE_PULL chain_complete_no_receiver"));
        }
#if defined(TUBES_DIG2GO_DYNAMIC_ENROLLMENT)
        // A complete body is the predecessor's terminal success condition.
        // Do not wait for a reboot report or second acknowledgement: the child
        // continues independently from its pre-reboot lease / first-boot turn.
        if (legacyPullBodyServed && !legacyHostRetired) {
          legacyHostRetired = true;
          controller.setDig2GoBridgeOverlay(Idle);
          Serial.println(F("TUBE_PULL predecessor_recovered transfer_complete_no_ack"));
        }
#else
        if (legacyPullHealthDeadline == 0) {
          legacyPullHealthDeadline = millis() + 90000;
          legacyPullNextHealthRequest = millis();
          Serial.println(F("TUBE_PULL_RESTORE mesh_started"));
        }
        if (legacyPullBodyServed && !legacyPullHealthVerified
            && static_cast<int32_t>(millis() - legacyPullNextHealthRequest) >= 0
            && static_cast<int32_t>(legacyPullHealthDeadline - millis()) > 0) {
          legacyPullHealthNonce = controller.requestDig2GoHealthReport(dig2GoEnrolledMac);
          legacyPullNextHealthRequest = millis() + 5000;
          Serial.printf("TUBE_PULL_VERIFY requested nonce=%08lx\n",
              static_cast<unsigned long>(legacyPullHealthNonce));
        }
        if (legacyPullBodyServed && !legacyPullHealthVerified
            && static_cast<int32_t>(millis() - legacyPullHealthDeadline) >= 0) {
          legacyPullBodyServed = false;
          controller.setDig2GoBridgeOverlay(Failed);
          Serial.println(F("TUBE_PULL_VERIFY timeout"));
        }
#endif
      }
      // A legacy client cannot persist propagation intent before installing
      // this image. Once the AP is gone and ESP-NOW is restored, repeat the
      // same existing offer briefly so freshly rebooted children can take the
      // baton. This is radio-only; the predecessor does not wait for an ACK.
      if (modernPropagationTurn && legacyPullBodyServed
          && legacyPullRestoreStarted && controller.meshRadioStartedAfterDig2Go()) {
        if (modernPropagationBatonUntil == 0) {
          modernPropagationBatonUntil = millis() + LEGACY_BOOTSTRAP_BATON_GRACE_MS;
          modernPropagationNextBatonAt = millis();
          Serial.println(F("FLEET_PROPAGATION baton_grace_started"));
        }
        if (static_cast<int32_t>(modernPropagationBatonUntil - millis()) > 0
            && static_cast<int32_t>(millis() - modernPropagationNextBatonAt) >= 0) {
          FleetUpdateOffer baton;
          const uint8_t serverAddress[4] = {4, 3, 2, 1};
          if (makeModernPropagationOffer(
                  baton, RELEASE_VERSION, modernPropagationNonce, serverAddress,
                  80, 1000, legacyPullHost.sessionSSID(),
                  legacyPullHost.sessionPassword()))
            controller.sendFleetPullUpdateOffer(baton);
          modernPropagationNextBatonAt = millis() + 1000;
        }
      }
      const bool modernBatonGraceComplete = modernPropagationBatonUntil == 0
          || static_cast<int32_t>(millis() - modernPropagationBatonUntil) >= 0;
      if (modernPropagationTurn && legacyPullRestoreStarted
          && controller.meshRadioStartedAfterDig2Go()
          && !modernPropagationLeaseCleared && modernBatonGraceComplete) {
        clearModernPropagationLease();
        modernPropagationLeaseCleared = true;
        Serial.println(F("FLEET_PROPAGATION lease_cleared"));
      }
      if (legacyPullPropagationTurnFinished(modernPropagationTurn,
          legacyPullRestoreStarted, controller.meshRadioStartedAfterDig2Go(),
          legacyHostRetired, legacyPullNeedsRestore, legacyPullBodyServed)
          && modernBatonGraceComplete) {
        Serial.println(F("FLEET_PROPAGATION turn_reset"));
        finishPeerPropagationTurn();
      }
#endif
#if TUBES_ENABLE_DIG2GO_PUSH_BRIDGE
#if defined(TUBES_DIG2GO_PUSH_AUTO_TRIGGER) && !defined(TUBES_DIG2GO_LEGACY_PULL_HOST)
      if (dig2GoIsPrime && dig2GoAutoTrigger.maybeStart(
          millis(), controller.meshRadioStartedAfterDig2Go(), dig2GoAutoAttempted)) {
        Serial.println(F("TUBE_PUSH_AUTO one-shot: starting exact enrolled target"));
        armDig2GoBridgeInternal(dig2GoEnrolledMac, 120000);
      }
#endif
      updateDig2GoPushBridge();
#endif
      debug.update();

      // Draw after everything else is done
      controller.led_strip.update();
    }

    void readFromJsonState(JsonObject& root) override {
      controller.readJsonOperations(root);
    }

    void addToJsonInfo(JsonObject& root) override {
      controller.addV3JsonInfo(root);
    }

    void handleOverlayDraw() {
      // WiFi mode leaves the WLED frame untouched; Tubes mode renders the mesh.
      if (!controller.shouldRenderTubes())
        return;

      // AI: below section was generated by an AI
      // Preserve both render inputs so diagnostics can explain the final composited frame.
      debug.captureRenderInputs();
      // AI: end

      // Draw effects layers over whatever WLED is doing.
      controller.handleOverlayDraw();
      debug.handleOverlayDraw();
      if (controller.isMasterRole()) {
        master.handleOverlayDraw();
      }

      // When AP mode is on, make sure it's obvious
      // Blink when there's a connected client
      if (apActive) {
        strip.setPixelColor(0, CRGB::Purple);
        if (millis() % 4000 > 1000 && WiFi.softAPgetStationNum()) {
          strip.setPixelColor(0, CRGB::Black);
        }
        strip.setPixelColor(1, CRGB::Black);
      }

      // AI: below section was generated by an AI
      controller.handleIdentifyOverlayDraw();
      // AI: end

      // AI: below section was generated by an AI
      debug.observeRenderedOutput();
      // AI: end
      drawDig2GoConnectionDiagnostic();
    }

    bool handleButton(uint8_t b) {
      if (controller.isHomeLightRole())
        return false;

      // Special code for handling the "power save" button
      if (b == 100) { // Press button 0 for WLED_LONG_POWER_SAVE ms
        controller.togglePowerSave();
        return true;
      }
      if (b == 101) { // Short press button 0 (piggybacks with default)
        controller.cancelOverrides();
        return true;
      }
      if (b == 102) { // Double-click button 0
        if (controller.isPropagationSelecting()) {
          if (controller.startSelectedPropagation())
            controller.acknowledge();
        } else if (controller.isSelecting()) {
          controller.acknowledge();
          if (controller.isSelected())
            controller.deselect();
          else
            controller.select();
        } else {
          controller.acknowledge();
          controller.request_new_bpm();
        }
        return true;
      }

      return false;
    }

    uint16_t getId() override { return USERMOD_ID_TUBES; }
};

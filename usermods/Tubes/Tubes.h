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
#include "s3_field_api.h"

#include "controller.h"
#include "debug.h"
#include "dig2go_peer_config.h"
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


class TubesUsermod : public Usermod {
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
    bool legacyPullNoReceiver = false;
    bool legacyHostRetired = false;
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
    uint32_t currentReleaseMarkerNextAttemptAt = 0;
    static constexpr uint32_t LEGACY_BOOTSTRAP_BATON_WINDOW_MS = 60000;
    static constexpr uint32_t CURRENT_RELEASE_MARKER_RETRY_MS = 60000;
    static constexpr uint32_t LEGACY_BOOTSTRAP_SOURCE_QUIET_MS = 5000;
    static constexpr uint32_t LEGACY_BOOTSTRAP_BATON_GRACE_MS = 15000;
#endif
#if TUBES_ENABLE_DIG2GO_PEER_PROPAGATION
    static TubesUsermod*& dig2GoPeerPropagationInstance() {
      static TubesUsermod* instance = nullptr;
      return instance;
    }

    static bool acceptDig2GoPropagation(const FleetUpdateOffer& offer) {
      return dig2GoPeerPropagationInstance()
          && dig2GoPeerPropagationInstance()->acceptDig2GoPropagationInternal(offer);
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
      legacyPullNoReceiver = false;
      legacyHostRetired = false;
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
      legacyPullNoReceiver = false;
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

#endif

    void drawDig2GoConnectionDiagnostic() {
#if defined(TUBES_DIG2GO_LEGACY_PULL_HOST)
      if (modernPropagationTurn && legacyPullOfferSent && !legacyHostRetired) {
        const bool terminal = legacyPullNeedsRestore;
        const auto stageColor = [terminal](bool passed) {
          return passed ? CRGB::Green : (terminal ? CRGB::Red : CRGB::Blue);
        };
        const CRGB stages[5] = {
          stageColor(legacyPullWakeAccepted),
          stageColor(legacyPullHost.stationSeen()),
          stageColor(legacyPullHost.requestSeen()),
          stageColor(legacyPullHost.bodyComplete()),
          legacyPullBodyServed ? CRGB::Yellow : stageColor(false)
        };
        for (uint8_t pair = 0; pair < 5; pair++) {
          strip.setPixelColor(pair * 2, stages[pair]);
          strip.setPixelColor(pair * 2 + 1, stages[pair]);
        }
        return;
      }
#endif
    }

    void randomize() {
      randomSeed(esp_random());
      random16_set_seed(random(0, 65535));
      random16_add_entropy(esp_random());
    }

    void recoverLedBussesIfNeeded() {
#ifdef TUBES_NULL_OUTPUT
      BusManager::removeAll();
      busConfigs.clear();
      uint8_t noPins[OUTPUT_MAX_PINS] = {255, 255, 255, 255, 255};
      busConfigs.emplace_back(TYPE_TUBES_NULL, noPins, 0, PIXEL_COUNTS, COL_ORDER_RGB);
      doInitBusses = true;
      return;
#endif
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
#ifdef TUBES_NULL_OUTPUT
      if (!needsSegments) {
        const Segment& seg = strip.getMainSegment();
        needsSegments = seg.length() != strip.getLengthTotal() || seg.start >= strip.getLengthTotal();
      }
#else
      if (!needsSegments) {
        const Segment& seg = strip.getMainSegment();
        // WLED may retain its one-pixel placeholder after the LED bus is restored.
        needsSegments = seg.length() == 0
          || seg.start >= strip.getLengthTotal()
          || (strip.getSegmentsNum() == 1 && seg.start == 0 && seg.stop == 1 && strip.getLengthTotal() > 1);
      }
#endif

      if (needsSegments) {
        strip.makeAutoSegments(true);
        Serial.println(F("Tubes: recovered LED segment config"));
      }
      checkedLedSegments = true;
    }

  public:
    // AI: below section was generated by an AI
    // Supplies the board UI with a fixed-size, read-only snapshot of Tubes state.
    void readS3FieldStatus(TubesS3FieldStatus &status) {
      status.isMaster = controller.isMasterRole();
      status.isFollowing = controller.node.isFollowing();
      status.radioReady = espnowBroadcast.isStarted();
      status.powerSave = controller.power_save;
      status.canForceNext = controller.can_force_next();
      status.role = static_cast<uint8_t>(controller.role);
      status.radioChannel = WiFi.channel();
      status.patternId = controller.current_state.pattern_id;
      status.paletteId = controller.current_state.palette_id;
      status.bpm = controller.current_state.bpm >> 8;
      status.beatFrame = controller.current_state.beat_frame;
      status.beat = (controller.current_state.beat_frame >> 8) % 16;
      status.currentPatternPhrase = controller.current_state.beat_frame >> 12;
      status.nextPatternPhrase = controller.next_state.pattern_phrase;
      status.localNodeId = controller.node.header.id;
      status.uplinkId = controller.node.header.uplinkId;
      status.tubesVersion = RELEASE_VERSION;
      status.currentSyncMode = controller.current_state.pattern_sync_id;
      status.nextPatternId = controller.next_state.pattern_id;
      status.nextSyncMode = controller.next_state.pattern_sync_id;
      status.currentPalettePhrase = controller.current_state.palette_phrase;
      status.nextPalettePhrase = controller.next_state.palette_phrase;
      status.nextPaletteId = controller.next_state.palette_id;
      status.tempoListening = controller.s3TempoListening();
      status.beatOverlayChoice = controller.s3BeatOverlayChoice(false);
      status.nextBeatOverlayChoice = controller.s3BeatOverlayChoice(true);
      const uint32_t now = millis();
      status.peerCount = controller.node.peerTelemetry.freshCount(now, 60000);
      fillS3ChannelStatus(status.beatChannel, BeatChannel, now);
      fillS3ChannelStatus(status.patternChannel, PatternChannel, now);
      fillS3ChannelStatus(status.paletteChannel, PaletteChannel, now);
      extractModeName(status.patternId, JSON_mode_names, status.patternName,
                      sizeof(status.patternName));
      extractModeName(status.paletteId, JSON_palette_names, status.paletteName,
                      sizeof(status.paletteName));
      const uint16_t length = strip.getLengthTotal();
      for (size_t i = 0; i < TUBES_S3_PREVIEW_PIXELS; i++) {
        const uint16_t pixel = length == 0 ? 0 : static_cast<uint16_t>((i * length) / TUBES_S3_PREVIEW_PIXELS);
        status.preview[i] = length == 0 ? 0 : strip.getPixelColor(pixel);
      }
    }

    void fillS3ChannelStatus(TubesS3ChannelStatus &status, uint8_t channel, uint32_t now) {
      status = TubesS3ChannelStatus{};
      status.localChannelId = controller.localChannelId(channel);
      const ChannelWinner &winner = controller.channelWinners.get(channel, now);
      status.active = winner.active;
      if (!winner.active) return;
      status.ownerChannelId = winner.authority.channelId;
      status.ownerControlId = winner.authority.controlId;
      status.sourceSession = winner.sourceSession;
      status.sequence = winner.sequence;
      status.leaseRemainingMs = static_cast<int32_t>(winner.expiresAtMs - now) > 0
          ? winner.expiresAtMs - now : 0;
    }

    bool readS3Peer(size_t index, TubesS3PeerStatus &peer) const {
      const PeerTelemetryEntry *entry = controller.node.peerTelemetry.entry(index);
      if (entry == nullptr) return false;
      peer.nodeId = entry->nodeId;
      peer.uplinkId = entry->uplinkId;
      peer.lastSeenMs = entry->lastSeenMs;
      peer.samples = entry->samples;
      peer.latestRssi = entry->latestRssi;
      peer.protocolGeneration = entry->protocolGeneration;
      peer.tubesVersion = entry->tubesVersion;
      peer.rssiKnown = entry->rssiKnown;
      return true;
    }

    bool s3ForceNext() { return controller.force_next_if_authoritative(); }
    bool s3SelectPattern(uint8_t patternId) {
      return controller.selectS3Pattern(patternId);
    }
    bool s3ScheduleGradient(const uint32_t colors[3]) {
      return controller.scheduleS3Gradient(colors);
    }
    bool s3SetTempoListening(bool enabled) {
      return controller.setS3TempoListening(enabled);
    }
    bool s3TapDownbeat() { return controller.tapS3Downbeat(); }
    bool s3SelectBeatOverlay(uint8_t choice) {
      return controller.selectS3BeatOverlay(choice);
    }
    bool s3BroadcastFleetOffer(const FleetUpdateOffer &offer) {
      return controller.broadcastFleetUpdateOffer(offer);
    }
    bool s3RequestDeviceReport(const uint8_t mac[6], uint32_t nonce) {
      return controller.requestDeviceReport(mac, nonce);
    }
    // AI: end

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
      currentReleaseMarkerNextAttemptAt = millis() + CURRENT_RELEASE_MARKER_RETRY_MS;
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
#if TUBES_ENABLE_DIG2GO_PEER_PROPAGATION
      dig2GoPeerPropagationInstance() = this;
      controller.setDig2GoPropagationCallback(acceptDig2GoPropagation);
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
#ifndef TUBES_S3_FIELD_OS
        master.setup();
#endif
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
#ifndef TUBES_S3_FIELD_OS
        master.update();
#endif
      }
      controller.update();
#if defined(TUBES_DIG2GO_LEGACY_PULL_HOST)
      if (!currentReleaseMarkerWritten
          && static_cast<int32_t>(millis() - currentReleaseMarkerNextAttemptAt) >= 0) {
        currentReleaseMarkerWritten = writeCurrentReleaseMarker(RELEASE_VERSION);
        legacyMigrationBootCandidate = false;
        currentReleaseMarkerNextAttemptAt = millis() + CURRENT_RELEASE_MARKER_RETRY_MS;
        Serial.printf("FLEET_PROPAGATION marker_written=%u\n",
            currentReleaseMarkerWritten);
      }
      const uint32_t legacyHostStartMs = modernPropagationStartAt;
      const bool legacyHostEligible = legacyPullAutomaticHostEligible(
          false, modernPropagationTurn, legacyPullOfferSent,
          legacyHostRetired);
      if (legacyHostEligible
          && millis() >= legacyHostStartMs && controller.meshRadioStartedAfterDig2Go()
          && !controller.deviceUpdateInProgress()) {
        legacyPullOfferSent = true;
        if (!legacyPullHost.prepare()) {
          controller.setDig2GoPeerPropagationOverlay(Failed);
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
            controller.setDig2GoPeerPropagationOverlay(Failed);
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
        legacyPullHost.stop();
        legacyPullNeedsRestore = true;
        controller.setDig2GoPeerPropagationOverlay(legacyPullBodyServed ? Received
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
          controller.setDig2GoPeerPropagationOverlay(Idle);
          Serial.println(F("TUBE_PULL chain_complete_no_receiver"));
        }
        // A complete body is the predecessor's terminal success condition.
        // Do not wait for a reboot report or second acknowledgement: the child
        // continues independently from its pre-reboot lease / first-boot turn.
        if (legacyPullBodyServed && !legacyHostRetired) {
          legacyHostRetired = true;
          controller.setDig2GoPeerPropagationOverlay(Idle);
          Serial.println(F("TUBE_PULL predecessor_recovered transfer_complete_no_ack"));
        }
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
#ifndef TUBES_S3_FIELD_OS
        master.handleOverlayDraw();
#endif
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
        if (controller.isSelecting()) {
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

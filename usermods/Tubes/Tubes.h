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

#ifndef PIXEL_COUNTS
#define PIXEL_COUNTS DEFAULT_LED_COUNT
#endif

#ifdef TUBES_NULL_OUTPUT
static_assert(PIXEL_COUNTS > 0, "Tubes logical framebuffer requires PIXEL_COUNTS > 0");
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

#ifdef TUBES_NULL_OUTPUT
    bool hasExpectedLogicalOutput() const {
      if (BusManager::getNumBusses() != 1) return false;
      const Bus *bus = BusManager::getBus(0);
      return bus != nullptr && bus->isOk() &&
             bus->getType() == TYPE_VIRTUAL_FRAMEBUFFER_RGB &&
             bus->getStart() == 0 && bus->getLength() == PIXEL_COUNTS;
    }

    bool hasExpectedLogicalOutputConfig() const {
      return busConfigs.size() == 1 &&
             busConfigs[0].type == TYPE_VIRTUAL_FRAMEBUFFER_RGB &&
             busConfigs[0].start == 0 && busConfigs[0].count == PIXEL_COUNTS;
    }
#endif

    void randomize() {
      randomSeed(esp_random());
      random16_set_seed(random(0, 65535));
      random16_add_entropy(esp_random());
    }

    void recoverLedBussesIfNeeded() {
#ifdef TUBES_NULL_OUTPUT
      // The S3 adapter owns one logical output. Preserve existing state, but
      // fail closed instead of replacing persisted or unrelated bus config.
      if (BusManager::getNumBusses() > 0) {
        if (!hasExpectedLogicalOutput()) {
          errorFlag = ERR_DENIED;
          Serial.println(F("Tubes: rejected incompatible S3 output bus"));
        }
        return;
      }
      if (!busConfigs.empty()) {
        if (!hasExpectedLogicalOutputConfig()) {
          errorFlag = ERR_DENIED;
          Serial.println(F("Tubes: rejected incompatible S3 output config"));
        }
        return;
      }

      uint8_t noPins[OUTPUT_MAX_PINS] = {255, 255, 255, 255, 255};
      busConfigs.emplace_back(TYPE_VIRTUAL_FRAMEBUFFER_RGB, noPins, 0, PIXEL_COUNTS, COL_ORDER_RGB);
      doInitBusses = true;
      Serial.println(F("Tubes: created logical framebuffer output"));
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
        doInitBusses = true;
        Serial.println(F("Tubes: recovered default LED bus config"));
      }
    }

    void recoverLedSegmentsIfNeeded() {
      if (checkedLedSegments || doInitBusses || strip.getLengthTotal() == 0 || BusManager::getNumBusses() == 0) return;

      bool needsSegments = strip.getSegmentsNum() == 0;
      if (!needsSegments) {
        const Segment& seg = strip.getMainSegment();
        needsSegments = seg.length() == 0 || seg.start >= strip.getLengthTotal();
      }

      if (needsSegments) {
        strip.makeAutoSegments(true);
        Serial.println(F("Tubes: recovered LED segment config"));
      }
      checkedLedSegments = true;
    }

  public:
    void copyReadOnlySnapshot(TubesReadOnlySnapshot& snapshot) const {
      controller.copyReadOnlySnapshot(snapshot);
    }

#ifdef TUBES_READ_ONLY_FIELD_SHELL
    bool s3ForcePrevious() {
      controller.force_previous_pattern();
      return true;
    }

    bool s3ForceNext() {
      controller.force_next_pattern();
      return true;
    }

    bool setS3BroadcastEnabled(bool enabled) {
      controller.setS3BroadcastEnabled(enabled);
      return true;
    }

    bool isS3BroadcastEnabled() const {
      return controller.isS3BroadcastEnabled();
    }
#endif

    void setup() {
      randomize();

      recoverLedBussesIfNeeded();
      // Start timing
      globalTimer.setup();
      controller.setup();

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
      debug.update();

      // Draw after everything else is done
      controller.led_strip.update();
    }

    void readFromJsonState(JsonObject& root) override {
      controller.readJsonOperations(root);
    }

    void handleOverlayDraw() {
      // WiFi mode leaves the WLED frame untouched; Tubes mode renders the mesh.
      if (!controller.shouldRenderTubes())
        return;

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
        controller.acknowledge();
        if (controller.isSelecting()) {
          if (controller.isSelected())
            controller.deselect();
          else
            controller.select();
        } else {
          controller.request_new_bpm();
        }
        return true;
      }

      return false;
    }

    uint16_t getId() override { return USERMOD_ID_TUBES; }
};

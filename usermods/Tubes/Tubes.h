#pragma once

#include "wled.h"

#include "util.h"
#include "options.h"

// #define USERADIO

#include "FX.h"

#include "virtual_strip.h"
#include "led_strip.h"
#include "master.h"

#include "controller.h"
#include "debug.h"


#define MASTER_PIN 25
#define LEGACY_PIN 32  // DigUno Q4


class TubesUsermod : public Usermod {
  private:
    PatternController controller = PatternController();
    DebugController debug = DebugController(controller);
    Master master = Master(controller);
    bool isLegacy = false;

    void randomize() {
      randomSeed(esp_random());
      random16_set_seed(random(0, 65535));
      random16_add_entropy(esp_random());
    }

  public:
    void setup() {

      if (pinManager.isPinOk(MASTER_PIN)) {
        pinMode(MASTER_PIN, INPUT_PULLUP);
        if(pinManager.isPinOk(LEGACY_PIN)) {
          pinMode(LEGACY_PIN, INPUT_PULLUP);
        }
        if (digitalRead(MASTER_PIN) == LOW) {
        }
        isLegacy = (digitalRead(MASTER_PIN) == LOW);
      }
      randomize();

      // Override some behaviors on all Tubes
      bootPreset = 0;  // Try to prevent initial playlists from starting
      fadeTransition = true;  // Fade palette transitions
      transitionDelay = 8000;   // Fade them for a long time
      strip.setTargetFps(60);
      strip.setCCT(100);

      // Start timing
      globalTimer.setup();
      controller.setup();
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

      if (controller.isMasterRole()) {
        master.update();
      }
      controller.update();
      debug.update();

      // Draw after everything else is done
      controller.led_strip.update();
    }

    void handleOverlayDraw() {
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
      // Button 0 behaviors:
      // - Short press (101): Toggle LED strip on/off
      // - Double press (102): Toggle between lamp mode and rave mode
      // - Long press (100): Activate WLED WiFi AP mode

      if (b == 100) { // Long press button 0 (1-5 seconds)
        // Activate WiFi AP mode for configuration
        WLED::instance().initAP(true);
        controller.acknowledge();
        Serial.println("Button: Activating WiFi AP mode");
        return true;
      }

      if (b == 101) { // Short press button 0
        // Toggle on/off is handled by WLED core (toggleOnOff)
        // Just cancel any manual overrides
        controller.cancelOverrides();
        Serial.println("Button: Toggle on/off");
        return true;
      }

      if (b == 102) { // Double-click button 0
        // Toggle between lamp mode (warm ambient) and rave mode (pattern cycling)
        controller.acknowledge();
        controller.toggleLampMode();

        if (controller.isLampMode()) {
          Serial.println("Button: Switched to LAMP mode (warm ambient)");
        } else {
          // When switching back to rave mode, pick a new pattern
          controller.set_next_pattern(0);
          controller.force_next_pattern();
          Serial.println("Button: Switched to RAVE mode (pattern cycling)");
        }
        return true;
      }

      return false;
    }

};

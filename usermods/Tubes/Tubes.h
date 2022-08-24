#pragma once

#include "wled.h"

#include "util.h"
#include "options.h"

// #define USERADIO

#include "FX.h"

#include "beats.h"
#include "virtual_strip.h"
#include "led_strip.h"
#include "master.h"

#include "controller.h"
#include "debug.h"


#define MASTER_PIN 25
#define LEGACY_PIN 32  // DigUno Q4


class TubesUsermod : public Usermod {
  private:
    BeatController beats;
    PatternController controller = PatternController(MAX_REAL_LEDS, &beats);
    DebugController debug = DebugController(&controller);
    Master *master = nullptr;
    bool isLegacy = false;

    void randomize() {
      randomSeed(esp_random());
      random16_set_seed(random(0, 65535));
      random16_add_entropy(esp_random());
    }

  public:
    void setup() {
      pinMode(MASTER_PIN, INPUT_PULLUP);
      pinMode(LEGACY_PIN, INPUT_PULLUP);
      if (digitalRead(MASTER_PIN) == LOW) {
      }
      isLegacy = (digitalRead(MASTER_PIN) == LOW);
      randomize();

      // Override some behaviors on all Tubes
      bootPreset = 0;  // Try to prevent initial playlists from starting
      fadeTransition = true;  // Fade palette transitions
      transitionDelay = 8000;   // Fade them for a long time
      strip.setTargetFps(60);
      strip.setCCT(100);

      // Start timing
      globalTimer.setup();
      beats.setup();
      controller.setup();
      if (controller.isMaster()) {
        master = new Master(&controller);
        master->setup();
      }
      debug.setup();
    }

    void loop()
    {
      EVERY_N_MILLISECONDS(10000) {
        randomize();
      }

      if (master) 
        master->update();
      beats.update();
      controller.update();
      debug.update();

      // Draw after everything else is done
      controller.led_strip->update();
    }

    void handleOverlayDraw()
    {
      // Perform a cross-fade between current WLED mode and the external buffer
      uint8_t fade; // amount that Tubes overwrites WLED, 0-255
      switch (this->controller.options.fader) {
        case AUTO:
        default:
          fade = sin8(millis() / 40);
          break;
        case LEFT:
          fade = 255;
          break;
        case RIGHT:
          fade = 0;
          break;
        case MIDDLE:
          fade = 127;
          break;
      }

      if (fade > 0) {
        uint16_t length = strip.getLengthTotal();
        for (int i = 0; i < length; i++) {
          CRGB color1 = strip.getPixelColor(i);
          CRGB color2 = controller.led_strip->getPixelColor(i);

          uint8_t r = blend8(color1.r, color2.r, fade);
          uint8_t g = blend8(color1.g, color2.g, fade);
          uint8_t b = blend8(color1.b, color2.b, fade);

          strip.setPixelColor(i, CRGB(r,g,b));
        }
      }

      // Draw effects layers over whatever WLED is doing.
      this->controller.handleOverlayDraw();
      this->debug.handleOverlayDraw();
      if (master) 
        master->handleOverlayDraw();

      // When AP mode is on, make sure it's obvious
      // Blink when there's a connected client
      if (apActive) {
        strip.setPixelColor(0, CRGB::Purple);
        if (millis() % 10000 > 1000 && WiFi.softAPgetStationNum()) {
          strip.setPixelColor(0, CRGB::Black);
        }
        strip.setPixelColor(1, CRGB::Black);
      }
    }

    bool handleButton(uint8_t b) {
      // Special code for handling the "power save" button
      if (b == 100) {
        this->controller.togglePowerSave();
        return true;
      }
      if (b == 101) {
        this->controller.cancelOverrides();
        return true;
      }

      return false;
    }

};

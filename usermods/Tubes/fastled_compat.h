#pragma once

#include "wled.h"

typedef uint16_t accum88;
typedef int16_t saccum78;

#define TUBES_CONCAT_IMPL(a, b) a##b
#define TUBES_CONCAT(a, b) TUBES_CONCAT_IMPL(a, b)
#define TUBES_EVERY_N_MILLISECONDS(var, ms) \
  static uint32_t var = 0; \
  if (millis() - var >= static_cast<uint32_t>(ms) && ((var = millis()), true))

#ifndef EVERY_N_MILLISECONDS
#define EVERY_N_MILLISECONDS(ms) TUBES_EVERY_N_MILLISECONDS(TUBES_CONCAT(tubes_every_n_ms_, __LINE__), ms)
#endif

#ifndef EVERY_N_MILLIS
#define EVERY_N_MILLIS(ms) EVERY_N_MILLISECONDS(ms)
#endif

inline uint8_t random8() { return hw_random8(); }
inline uint8_t random8(uint8_t upper) { return hw_random8(upper); }
inline uint8_t random8(uint8_t lower, uint8_t upper) { return hw_random8(lower, upper); }
inline uint16_t random16() { return hw_random16(); }
inline uint16_t random16(uint16_t upper) { return hw_random16(upper); }
inline int16_t random16(int16_t lower, int16_t upper) { return hw_random16(lower, upper); }
inline void random16_set_seed(uint16_t) {}
inline void random16_add_entropy(uint16_t) {}

inline uint8_t sin8(uint8_t theta) { return sin8_t(theta); }
inline int16_t sin16(uint16_t theta) { return sin16_t(theta); }
inline int16_t cos16(uint16_t theta) { return cos16_t(theta); }
inline uint8_t ease8InOutApprox(uint8_t i) { return ease8InOutQuad(i); }
inline uint8_t beatsin8(uint16_t bpm, uint8_t low = 0, uint8_t high = 255, uint32_t timebase = 0, uint8_t phase = 0) {
  return beatsin8_t(bpm, low, high, timebase, phase);
}
inline uint16_t beatsin16(uint16_t bpm, uint16_t low = 0, uint16_t high = 65535, uint32_t timebase = 0, uint16_t phase = 0) {
  return beatsin16_t(bpm, low, high, timebase, phase);
}

inline uint8_t blend8(uint8_t a, uint8_t b, uint8_t amountOfB) {
  return lerp8by8(a, b, amountOfB);
}

inline void nscale8x3(uint8_t& r, uint8_t& g, uint8_t& b, uint8_t scale) {
  r = scale8(r, scale);
  g = scale8(g, scale);
  b = scale8(b, scale);
}

inline void nscale8(CRGB* leds, uint16_t numLeds, uint8_t scale) {
  for (uint16_t i = 0; i < numLeds; i++) leds[i].nscale8(scale);
}

inline void fadeToBlackBy(CRGB* leds, uint16_t numLeds, uint8_t fadeBy) {
  nscale8(leds, numLeds, 255 - fadeBy);
}

inline void fill_solid(CRGB* leds, uint16_t numLeds, const CRGB& color) {
  fill_solid_RGB(leds, numLeds, color);
}

inline void fill_rainbow(CRGB* leds, uint16_t numLeds, uint8_t initialHue, uint8_t deltaHue = 5) {
  uint8_t hue = initialHue;
  for (uint16_t i = 0; i < numLeds; i++, hue += deltaHue) leds[i] = CHSV(hue, 255, 255);
}

inline void nblend(CRGB& existing, const CRGB& overlay, uint8_t amountOfOverlay) {
  existing.r = blend8(existing.r, overlay.r, amountOfOverlay);
  existing.g = blend8(existing.g, overlay.g, amountOfOverlay);
  existing.b = blend8(existing.b, overlay.b, amountOfOverlay);
}

inline CRGB operator|(CRGB lhs, const CRGB& rhs) {
  lhs |= rhs;
  return lhs;
}

inline CRGB operator&(CRGB lhs, const CRGB& rhs) {
  lhs &= rhs;
  return lhs;
}

static const TProgmemRGBPalette16 TubesPartyColors_p PROGMEM = {
  0x9B00D5, 0xBD00B8, 0xDA0092, 0xF3005C,
  0xF45500, 0xDC8F00, 0xD5B400, 0xD5D500,
  0xD59B00, 0xEF6600, 0xF90044, 0xE10086,
  0xC400B0, 0xA300CF, 0x7600E8, 0x0032FC
};

#define PartyColors_p TubesPartyColors_p

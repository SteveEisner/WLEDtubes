#pragma once

#include <EEPROM.h>
#include "wled.h"
#include "FX.h"
#include "updater.h"
#include "sound.h"

#include "beats.h"

#include "pattern.h"
#include "effects.h"
#include "led_strip.h"
#include "global_state.h"
#include "node.h"
#include "deferred_bpm_broadcast.h"
#include "device_report_protocol.h"

#define EEPSIZE 2560

const static uint8_t DEFAULT_MASTER_BRIGHTNESS = 200;
const static uint8_t DEFAULT_TUBE_BRIGHTNESS = 120;
const static uint8_t DEFAULT_TANK_BRIGHTNESS = 240;
#define DEFAULT_WLED_FX FX_MODE_RAINBOW_CYCLE

#define STATUS_UPDATE_PERIOD 2000
#define OPTIONS_BROADCAST_PERIOD 500
#define OPTIONS_BROADCAST_RETRIES 8

static_assert(GRADIENT_PALETTE_COUNT <= UINT8_MAX, "Tubes palette IDs must fit in one byte");
static constexpr uint8_t gGradientPaletteCount = GRADIENT_PALETTE_COUNT;

#define MIN_COLOR_CHANGE_PHRASES 4
#define MAX_COLOR_CHANGE_PHRASES 10

#define ROLE_EEPROM_LOCATION 2559
#define BOOT_OPTIONS_EEPROM_LOCATION 2551

// #define IDENTIFY_STUCK_PATTERNS
// #define IDENTIFY_STUCK_PALETTES

typedef struct {
  bool debugging;
  uint8_t brightness;

  uint8_t reserved[12];
} ControllerOptions;

typedef struct {
  TubeState current;
  TubeState next;
} TubeStates;

typedef enum ControllerRole : uint8_t {
  UnknownRole = 0,
  DefaultRole = 10,         // Standard non-master role
  CampRole = 50,            // Turn on in non-power-saving mode
  InstallationRole = 100,   // Disable power-saving mode completely
  SmallArtRole = 120,       // < 1/2 the pixels, scale the art
  HomeLightRole = 150,      // Join the mesh while WLED retains permanent control of the LEDs
  LegacyRole = 190,         // LEGACY: 1/2 the pixels, no "power saving" necessary, no scaling
  MasterRole = 200          // Controls all the others
} ControllerRole;

typedef struct BootOptions {
  unsigned int default_power_save:2;
} BootOptions;

#define BOOT_OPTION_POWER_SAVE_DEFAULT 0
#define BOOT_OPTION_POWER_SAVE_OFF 1
#define BOOT_OPTION_POWER_SAVE_ON 2

typedef struct {
  char key;
  uint8_t arg;
} Action;

enum TubeScope : uint8_t {
  LocalScope = 0,
  MeshScope = 1,
  SelectedScope = 2,
};

enum TubeOperationCode : uint8_t {
  DebugOperation,
  RebootOperation,
  PowerSaveOperation,
  BrightnessOperation,
  AccessPointOperation,
  DisconnectWifiOperation,
  ForgetWifiOperation,
  BpmOperation,
  StartPhraseOperation,
  NextOperation,
  PatternOperation,
  SyncModeOperation,
  PaletteOperation,
  EffectOperation,
  EffectChanceOperation,
  NodeIdOperation,
  UpdateOperation,
  UpdateOfferOperation,
  SelectOperation,
  GlitterOperation,
  FlashOperation,
  RoleOperation,
  CancelOverrideOperation,
  SoundOverlayOperation,
  TubesModeOperation,
  HelpOperation,
};

// The tag leaves six bits for commands and keeps the destination in its high bits.
struct TubeOperation {
  uint16_t argument;
  uint8_t tag;
};

static TubeOperationCode tubeOperationCode(const TubeOperation& operation) {
  return TubeOperationCode(operation.tag & 0x3F);
}

static TubeScope tubeOperationScope(const TubeOperation& operation) {
  return TubeScope(operation.tag >> 6);
}

struct TubeCommandDefinition {
  char command;
  uint8_t tag;
};

#define TUBE_COMMAND(command, operation, scope) \
  {command, uint8_t(uint8_t(operation) | (uint8_t(scope) << 6))}

static const TubeCommandDefinition tubeCommandDefinitions[] PROGMEM = {
  TUBE_COMMAND('d', DebugOperation, MeshScope),
  TUBE_COMMAND('~', RebootOperation, LocalScope),
  TUBE_COMMAND('_', PowerSaveOperation, LocalScope),
  TUBE_COMMAND('-', BrightnessOperation, MeshScope),
  TUBE_COMMAND('+', BrightnessOperation, MeshScope),
  TUBE_COMMAND('l', BrightnessOperation, MeshScope),
  TUBE_COMMAND('a', AccessPointOperation, LocalScope),
  TUBE_COMMAND('q', DisconnectWifiOperation, LocalScope),
  TUBE_COMMAND('b', BpmOperation, MeshScope),
  TUBE_COMMAND('s', StartPhraseOperation, MeshScope),
  TUBE_COMMAND('n', NextOperation, MeshScope),
  TUBE_COMMAND('p', PatternOperation, MeshScope),
  TUBE_COMMAND('m', SyncModeOperation, MeshScope),
  TUBE_COMMAND('c', PaletteOperation, MeshScope),
  TUBE_COMMAND('e', EffectOperation, MeshScope),
  TUBE_COMMAND('%', EffectChanceOperation, MeshScope),
  TUBE_COMMAND('i', NodeIdOperation, LocalScope),
  TUBE_COMMAND('u', UpdateOperation, LocalScope),
  TUBE_COMMAND('U', UpdateOperation, SelectedScope),
  TUBE_COMMAND('V', UpdateOfferOperation, MeshScope),
  TUBE_COMMAND('*', SelectOperation, MeshScope),
  TUBE_COMMAND('(', SelectOperation, MeshScope),
  TUBE_COMMAND(')', SelectOperation, MeshScope),
  TUBE_COMMAND('@', PowerSaveOperation, MeshScope),
  TUBE_COMMAND('P', PowerSaveOperation, MeshScope),
  TUBE_COMMAND('G', GlitterOperation, MeshScope),
  TUBE_COMMAND('A', AccessPointOperation, MeshScope),
  TUBE_COMMAND('W', ForgetWifiOperation, MeshScope),
  TUBE_COMMAND('X', RebootOperation, SelectedScope),
  TUBE_COMMAND('F', FlashOperation, MeshScope),
  TUBE_COMMAND('r', RoleOperation, LocalScope),
  TUBE_COMMAND('R', RoleOperation, SelectedScope),
  TUBE_COMMAND('M', CancelOverrideOperation, MeshScope),
  TUBE_COMMAND('O', SoundOverlayOperation, MeshScope),
  TUBE_COMMAND('t', TubesModeOperation, LocalScope),
  TUBE_COMMAND('?', HelpOperation, LocalScope),
};

#undef TUBE_COMMAND

static_assert(HelpOperation < 64, "Tube operations must fit below the scope bits");
static_assert(sizeof(TubeOperation) <= 4, "Tube operations must remain compact");

#define NUM_VSTRIPS 3

#define DEBOUNCE_TIME 40

class TubesButton {
  public:
    TubesTimer debounceTimer;
    uint8_t pin;
    bool lastPressed = false;

  void setup(uint8_t p) {
    pin = p;
    pinMode(pin, INPUT_PULLUP);
    debounceTimer.start(0);
  }

  bool pressed() {
    if (digitalRead(pin) == HIGH) {
      return !debounceTimer.ended();
    }

    debounceTimer.start(DEBOUNCE_TIME);
    return true;
  }

  bool triggered() {
    // Triggers BOTH low->high AND high->low
    bool p = pressed();
    bool lp = lastPressed;
    lastPressed = p;
    return p != lp;
  }
};

class PatternController : public MessageReceiver {
  public:
    struct WledDisplayState {
      bool valid = false;
      uint8_t brightness = 0;
      uint8_t mode = 0;
      uint8_t palette = 0;
      uint8_t speed = 0;
      uint8_t intensity = 0;
      uint8_t preset = 0;
    };

    const static int FRAMES_PER_SECOND = 60;  // how often we animate, in frames per second
    const static int REFRESH_PERIOD = 1000 / FRAMES_PER_SECOND;  // how often we animate, in milliseconds

    VirtualStrip *vstrips[NUM_VSTRIPS];
    uint8_t next_vstrip = 0;
    bool canOverride = false;
    uint8_t optionsBroadcastsPending = 0;
    uint8_t paletteOverride = 0;
    uint8_t patternOverride = 0;
    uint16_t wled_fader = 0;
    ControllerRole role;
    bool power_save = false;  // Default to power save mode OFF but 3 sec press turns it on
    uint8_t flashColor = 0;

    AutoUpdater updater = AutoUpdater();
    Sounder sound = Sounder();

    void copyReadOnlySnapshot(TubesReadOnlySnapshot& snapshot) const {
      snapshot = TubesReadOnlySnapshot{};
      snapshot.hardwareFamily = TUBES_HARDWARE_FAMILY;
      snapshot.reportProtocolVersion = DEVICE_REPORT_PROTOCOL_VERSION;
      snapshot.compatibilityClass = TubeCompatibilityUnknown;
      snapshot.tubesRelease = RELEASE_VERSION;
      strlcpy(snapshot.wledVersion, versionString, sizeof(snapshot.wledVersion));
      snapshot.controllerRole = role;
      if (node.status == LightNode::NODE_STATUS_STARTED)
        snapshot.meshFlags |= DeviceReportMeshStarted;
      if (node.isFollowing())
        snapshot.meshFlags |= DeviceReportMeshFollowing;
      if (isMasterRole())
        snapshot.meshFlags |= DeviceReportMasterBehavior;
      snapshot.nodeId = node.header.id;
      snapshot.uplinkId = node.header.uplinkId;
    }

    TubesTimer graphicsTimer;
    TubesTimer updateTimer;
    TubesTimer optionsBroadcastTimer;
    TubesTimer paletteOverrideTimer;
    TubesTimer patternOverrideTimer;
    TubesTimer flashTimer;
    TubesTimer selectTimer;
    TubesTimer tubesModeTimer;

#ifdef USELCD
    Lcd *lcd;
#endif
    LEDs led_strip;
    BeatController beats;
    Effects effects;
    LightNode node;

    ControllerOptions options;
    WledDisplayState wledDisplayBeforeTubes;
    char key_buffer[20] = {0};
    int8_t pendingTubesMode = -1;
    DeferredBpmBroadcast deferredBpmBroadcast;

    Energy energy=Chill;
    TubeState current_state;
    TubeState next_state;
    bool hasLoadedPattern = false;

    // When a pattern is boring, spice it up a bit with more effects
    bool isBoring = false;

  PatternController() : node(this) {
#ifdef USELCD
    lcd = new Lcd();
#endif

    for (auto i=0; i < NUM_VSTRIPS; i++) {
      vstrips[i] = new VirtualStrip();
    }
  }

  bool isMasterRole() const {
#if defined(GOLDEN) || defined(CHRISTMAS) || defined(RUBY) || defined(MAUVE) || defined(MASTER)
    return true;
#endif
    return role >= MasterRole;
  }

  bool isHomeLightRole() const {
    return role == HomeLightRole;
  }

  bool shouldRenderTubes() const {
#ifdef HOMELIGHT
    if (isHomeLightRole())
      return espnowBroadcast.isEnabled();
#endif
    return !isHomeLightRole();
  }

  void captureWledDisplay() {
    Segment& segment = strip.getMainSegment();
    wledDisplayBeforeTubes.valid = true;
    wledDisplayBeforeTubes.brightness = bri;
    wledDisplayBeforeTubes.mode = segment.mode;
    wledDisplayBeforeTubes.palette = segment.palette;
    wledDisplayBeforeTubes.speed = segment.speed;
    wledDisplayBeforeTubes.intensity = segment.intensity;
    wledDisplayBeforeTubes.preset = currentPreset;
  }

  void restoreWledDisplay() {
    if (!wledDisplayBeforeTubes.valid)
      return;

    Segment& segment = strip.getMainSegment();
    bri = wledDisplayBeforeTubes.brightness;
    segment.speed = wledDisplayBeforeTubes.speed;
    segment.intensity = wledDisplayBeforeTubes.intensity;
    segment.setMode(wledDisplayBeforeTubes.mode);
    segment.setPalette(wledDisplayBeforeTubes.palette);
    stateChanged = true;
    stateUpdated(CALL_MODE_DIRECT_CHANGE);
    currentPreset = wledDisplayBeforeTubes.preset;
    wledDisplayBeforeTubes.valid = false;
  }

  void setTubesMode(bool enabled) {
#ifndef HOMELIGHT
    (void)enabled;
    Serial.println(F("nope"));
    return;
#else
    if (!isHomeLightRole()) {
      Serial.println(F("nope"));
      return;
    }

    if (enabled) {
      if (espnowBroadcast.isEnabled())
        return;

      Serial.println(F("Tubes mode on."));
      captureWledDisplay();
      WiFi.softAPdisconnect(true);
      apActive = false;
      if (!espnowBroadcast.setEnabled(true)) {
        Serial.println(F("Unable to start Tubes mode."));
        restoreWledDisplay();
        return;
      }

      load_options(options, true);
      update_background();
      return;
    }

    if (!espnowBroadcast.isEnabled())
      return;

    Serial.println(F("Tubes mode off; reconnecting WiFi."));
    espnowBroadcast.setEnabled(false);
    restoreWledDisplay();
    forceReconnect = true;
#endif
  }

  void setup()
  {
    EEPROM.begin(EEPSIZE);
#ifdef HOMELIGHT
    // A HOMELIGHT build owns its role, regardless of the device's previous use.
    role = HomeLightRole;
#else
    uint8_t storedRole = EEPROM.read(ROLE_EEPROM_LOCATION);
    role = (ControllerRole)storedRole;
    if (role == 255) {
      role = UnknownRole;
    }
#if defined(MASTER)
    role = MasterRole;
#endif
#endif
    Serial.printf("Role = %d\n", role);

    auto b = EEPROM.read(BOOT_OPTIONS_EEPROM_LOCATION);
    Serial.printf("EEPROM read: %d\n", b);
    EEPROM.end();

    BootOptions* boot = (BootOptions*)&b;
    switch (boot->default_power_save) {
      case BOOT_OPTION_POWER_SAVE_OFF:
        power_save = 0;
        break;
      case BOOT_OPTION_POWER_SAVE_ON:
        power_save = 1;
        break;
      default:
        power_save = false;
        break;
    }

    if (!isHomeLightRole()) {
      if (role <= CampRole)
        BusManager::setMilliampsMax(min(ABL_MILLIAMPS_DEFAULT,700));  // Really limit for batteries
      else if (role <= InstallationRole)
        BusManager::setMilliampsMax(1000);
      else
        BusManager::setMilliampsMax(1400);
    }


    beats.setup();
    node.setWledNetworkOwnership(isHomeLightRole());
    node.setup();

    if (role >= MasterRole) {
      node.reset(3850 + role); // MASTER ID
      options.brightness = DEFAULT_MASTER_BRIGHTNESS;
    } else if (role >= LegacyRole) {
        options.brightness = DEFAULT_TUBE_BRIGHTNESS;
    } else if (role == InstallationRole) {
        options.brightness = DEFAULT_TANK_BRIGHTNESS;
    } else {
        options.brightness = DEFAULT_TUBE_BRIGHTNESS;
    }
#if defined(GOLDEN) || defined(CHRISTMAS) || defined(RUBY) || defined(MAUVE) || defined(MASTER)
    node.reset(0xFFF);
#endif
    options.debugging = false;
    load_options(options, true);

#ifdef USELCD
    lcd->setup();
#endif
    set_next_pattern(0);
    set_next_palette(0);
    set_next_effect(0);
    next_state.pattern_phrase = 0;
    next_state.palette_phrase = 0;
    next_state.effect_phrase = 0;
    set_wled_palette(0); // Default palette
    set_wled_pattern(0, 128, 128); // Default pattern

    sound.setup();

    updateTimer.start(STATUS_UPDATE_PERIOD); // Ready to send an update as soon as we're able to
    server.rewrite("/tube", "/json");
    Serial.println("Controller: ok");
  }

  void do_pattern_changes() {
    uint16_t phrase = current_state.beat_frame >> 12;
    bool changed = false;

    if (phrase >= next_state.pattern_phrase) {
#ifdef IDENTIFY_STUCK_PATTERNS
      Serial.println("Time to change pattern");
#endif
      load_pattern(next_state);
      next_state.pattern_phrase = phrase + set_next_pattern(phrase);

      // Don't change pattern and others at the same time
      while (next_state.pattern_phrase == next_state.palette_phrase || next_state.pattern_phrase == next_state.effect_phrase) {
        next_state.pattern_phrase += random8(1,3);
      }
      changed = true;
    }
    if (phrase >= next_state.palette_phrase) {
#ifdef IDENTIFY_STUCK_PATTERNS
      Serial.println("Time to change palette");
#endif
      load_palette(next_state);
      next_state.palette_phrase = phrase + set_next_palette(phrase);

      // Don't change palette and others at the same time
      while (next_state.palette_phrase == next_state.pattern_phrase || next_state.palette_phrase == next_state.effect_phrase) {
        next_state.palette_phrase += random8(1,3);
      }
      changed = true;
    }
    if (phrase >= next_state.effect_phrase) {
#ifdef IDENTIFY_STUCK_PATTERNS
      Serial.println("Time to change effect");
#endif
      load_effect(next_state);
      next_state.effect_phrase = phrase + set_next_effect(phrase);

      // Don't change palette and others at the same time
      while (next_state.effect_phrase == next_state.pattern_phrase || next_state.effect_phrase == next_state.palette_phrase) {
        next_state.effect_phrase += random8(1,3);
      }
      changed = true;
    }

    if (changed) {
      next_state.print();
      Serial.println();
    }
  }

  void cancelOverrides() {
    if (isHomeLightRole())
      return;

    // Release the WLED overrides and take over control of the strip again.
    paletteOverrideTimer.stop();
    patternOverrideTimer.stop();
  }

  void enterSelectMode() {
    selectTimer.start(20000);
  }

  bool isSelecting() const {
    return !selectTimer.ended();
  }

  bool isSelected() const {
    return updater.status == Ready;
  }

  void select(bool selected = true) {
    if (isHomeLightRole())
      return;

    if (selected)
      updater.ready();
    else {
      updater.stop();
      WiFi.softAPdisconnect(true);
    }
  }

  void deselect() {
    select(false);
  }

  void set_palette_override(uint8_t value) {
    if (isHomeLightRole() || !canOverride)
      return;
    if (value == paletteOverride)
      return;

    paletteOverride = value;
    if (value) {
      Serial.println("WLED has control of palette.");
      paletteOverrideTimer.start(300000); // 5 minutes of manual control
    } else {
      Serial.println("Turning off WLED control of palette.");
      paletteOverrideTimer.stop();
      set_wled_palette(current_state.palette_id);
    }
  }

  void set_pattern_override(uint8_t value, uint8_t auto_mode) {
    if (isHomeLightRole() || !canOverride)
      return;
    if (value == DEFAULT_WLED_FX && !patternOverride)
      return;
    if (value == patternOverride)
      return;

    patternOverride = value;
    if (value) {
      Serial.println("WLED has control of patterns.");
      patternOverrideTimer.start(300000); // 5 minutes of manual control
      transitionDelay = 500;  // Short transitions
    } else {
      Serial.println("Turning off WLED control of patterns.");
      patternOverrideTimer.stop();
      transitionDelay = 8000; // Back to long transitions

      uint8_t param = modeParameter(auto_mode);
      set_wled_pattern(auto_mode, param, param);
    }
  }

  void update()
  {
    read_keys();

    // Network mode changes wait until an HTTP response has left the async handler.
    if (pendingTubesMode >= 0 && tubesModeTimer.ended()) {
      bool enabled = pendingTubesMode;
      pendingTubesMode = -1;
      setTubesMode(enabled);
    }

    beats.update();

    // Update the mesh
    node.update();

    // Update sound meter
    sound.update();

    // Update patterns to the beat
    update_beat();

    // A locally selected BPM becomes public only when that local clock starts a phrase.
    send_deferred_bpm();

    Segment& segment = strip.getMainSegment();

    // You can only go into manual control after enabling the wifi
    if (apActive && updater.status != Ready)
      canOverride = true;

    // Detect manual overrides & update the current state to match.
    if (canOverride) {
      if (paletteOverride && (paletteOverrideTimer.ended() || !apActive)) {
        set_palette_override(0);
      } else if (segment.palette != current_state.palette_id) {
        set_palette_override(segment.palette);
      }

      uint8_t wled_mode = gPatterns[current_state.pattern_id].wled_fx_id;
      if (wled_mode < 10)
        wled_mode = DEFAULT_WLED_FX;
      if (patternOverride && (patternOverrideTimer.ended() || !apActive)) {
        set_pattern_override(0, wled_mode);
      } else if (segment.mode != wled_mode) {
        set_pattern_override(segment.mode, wled_mode);
      }
    }

    do_pattern_changes();

    if (graphicsTimer.every(REFRESH_PERIOD)) {
      updateGraphics();
    }

    // Update current status
    if (updateTimer.every(STATUS_UPDATE_PERIOD)) {
      // Transmit less often when following
      if (!node.isFollowing() || random(0, 4) == 0) {
        send_update();
      }
    }

    if (optionsBroadcastsPending) {
      if (optionsBroadcastTimer.every(OPTIONS_BROADCAST_PERIOD)) {
        broadcast_options();
        optionsBroadcastsPending--;
      }
    }

    updater.update();

#ifdef USELCD
    if (lcd->active) {
      lcd->size(1);
      lcd->write(0,56, current_state.beat_frame);
      lcd->write(80,56, x_axis);
      lcd->write(100,56, y_axis);
      lcd->show();

      lcd->update();
    }
#endif
  }

  void handleOverlayDraw() {
    // In manual mode WLED is always active
    if (patternOverride) {
      wled_fader = 0xFFFF;
    }

    uint16_t length = strip.getLengthTotal();

    // Crossfade between the custom pattern engine and WLED
    uint8_t fader = wled_fader >> 8;
    if (fader < 255) {
      // Perform a cross-fade between current WLED mode and the external buffer
      for (int i = 0; i < length; i++) {
        CRGB c = getBlendedPixelColor(i);
        if (fader > 0) {
          CRGB color2 = strip.getPixelColor(i);
          uint8_t r = blend8(c.r, color2.r, fader);
          uint8_t g = blend8(c.g, color2.g, fader);
          uint8_t b = blend8(c.b, color2.b, fader);
#ifdef RUBY
          // Simple average brightness for a "luminosity" measure
          uint8_t brightness = (uint16_t)(r + g + b) / 3;

          // Check if it's near white (all channels fairly similar and somewhat bright)
          // You can tweak thresholds to taste.
          bool isNearWhite = (abs(r - g) < 20 && abs(g - b) < 20 && (r + g + b) > 200);

          // Force everything into a shade of red:
          uint8_t redLevel = brightness;
          uint8_t greenLevel = 0;
          uint8_t blueLevel  = 0;

          // If it’s near white, add a little G/B so it’s not pure red.
          if(isNearWhite) {
            greenLevel = brightness / 2;
            blueLevel  = brightness / 2;
          }

          c = CRGB(redLevel, greenLevel, blueLevel);
#else
          c = CRGB(r,g,b);
#endif
        }
        strip.setPixelColor(i, c);
      }
    }

    // Power Save mode: reduce number of displayed pixels
    // Only affects non-powered poles
    if (power_save && role < InstallationRole) {
      // Screen door effect to save power
      for (int i = 0; i < length; i++) {
        if (i % 2) {
            strip.setPixelColor(i, CRGB::Black);
        }
      }
    }

    sound.handleOverlayDraw();

    // Draw effects layers over whatever WLED is doing.
    // But not in manual (WLED) mode
    if (!patternOverride) {
      effects.draw(&strip);
    }

    // Make the art half-size if it has a small number of pixels
    if (role >= MasterRole || role == SmallArtRole) {
      int p = 0;
      for (int i = 0; i < length; i++) {
        CRGB c = strip.getPixelColor(i++); // i advances by 2
        CRGB c2 = strip.getPixelColor(i);
        nblend(c, c2, 128);
        if (role >= MasterRole) {
          nblend(c, CRGB::Black, 128);
        }
        strip.setPixelColor(p++, c);
      }
    }

    if (flashColor) {
      if (flashTimer.ended())
        flashColor = 0;
      else {
        if (millis() % 4000 < 2000) {
          auto chsv = CHSV(flashColor, 255, 255);
          for (int i = 0; i < length; i++) {
            strip.setPixelColor(i, CRGB(chsv));
          }
        }
      }
    }

    updater.handleOverlayDraw();
  }

  void restart_phrase() {
    beats.start_phrase();
    update_beat();
    send_update();
  }

  void set_phrase_position(uint8_t pos) {
    beats.sync(beats.bpm, (beats.frac & -0xFFF) + (pos<<8));
    update_beat();
    send_update();
  }

  void set_tapped_bpm(accum88 bpm, uint8_t pos=15) {
    // By default, restarts at 15th beat - because this is the end of a tap
    apply_bpm(bpm, pos);
    send_update();
  }

  void apply_bpm(accum88 bpm, uint8_t pos=0) {
    beats.sync(bpm, (beats.frac & -0xFFF) + (pos<<8));
    update_beat();
  }

  void request_new_bpm(accum88 new_bpm = 0) {
    // 0 = toggle 120 to 125
    if (new_bpm == 0)
      new_bpm = current_state.bpm>>8 >= 123 ? 120<<8 : 125<<8;

    // The controlling device responds now, while the mesh stays on its current clock
    // until this device reaches the phrase boundary used for the legacy declaration.
    apply_bpm(new_bpm);
    deferredBpmBroadcast.schedule(new_bpm, current_state.beat_frame);
  }

  void send_deferred_bpm() {
    accum88 bpm;
    if (deferredBpmBroadcast.takeAtPhraseBoundary(current_state.beat_frame, bpm))
      broadcast_bpm(bpm);
  }

  void update_beat() {
    current_state.bpm = next_state.bpm = beats.bpm;
    current_state.beat_frame = particle_beat_frame = beats.frac;  // (particle_beat_frame is a hack)
    if (current_state.bpm>>8 <= 118) // Hip hop / ghettofunk
      energy = MediumEnergy;
    else if (current_state.bpm>>8 >= 125) // House & breaks
      energy = HighEnergy;
    else if (current_state.bpm>>8 > 120) // Tech house
      energy = MediumEnergy;
    else
      energy = Chill; // Deep house
  }

  void send_update() {
    Serial.print("     ");
    current_state.print();
    Serial.print(F(" "));

    uint16_t phrase = current_state.beat_frame >> 12;
    Serial.print(F("    "));
    Serial.print(next_state.pattern_phrase - phrase);
    Serial.print(F("P "));
    Serial.print(next_state.palette_phrase - phrase);
    Serial.print(F("C "));
    Serial.print(next_state.effect_phrase - phrase);
    Serial.print(F("E: "));
    next_state.print();
    Serial.print(F(" "));
    Serial.println();

    broadcast_state();
  }

  void background_changed() {
    update_background();
    current_state.print();
    Serial.println();
  }

  void load_options(ControllerOptions &options, bool init=false) {
    if (!shouldRenderTubes())
      return;

    if (init && !turnOnAtBoot && bri == 0) {
      return;
    }

    // Power-saving devices retain their WLED brightness
    if (!init && power_save) {
      return;
    }

    if (init || !power_save) {
      bri = options.brightness;
      briOld = options.brightness;
      briLast = options.brightness;
      briT = options.brightness;
      strip.setBrightness(options.brightness);
    }
  }

  void load_pattern(TubeState &tube_state) {
    if (hasLoadedPattern
        && current_state.pattern_id == tube_state.pattern_id
        && current_state.pattern_sync_id == tube_state.pattern_sync_id)
      return;

    current_state.pattern_phrase = tube_state.pattern_phrase;
    current_state.pattern_id = tube_state.pattern_id % gPatternCount;
    current_state.pattern_sync_id = tube_state.pattern_sync_id;
    hasLoadedPattern = true;
    isBoring = gPatterns[current_state.pattern_id].control.energy == Boring;

    Serial.print(F("Change pattern "));
    background_changed();
  }

  bool isShowingWled() const {
    return current_state.pattern_id >= numInternalPatterns;
  }

  uint8_t modeParameter(uint8_t mode) {
    switch (energy) {
      case Boring:
        // Spice things up a bit
        return 128;

      case Chill:
        return 90;

      case HighEnergy:
        return 140;

      default:
      case MediumEnergy:
        return 128;
    }
  }

  // For now, can't crossfade between internal and WLED patterns
  // If currently running an WLED pattern, only select from internal patterns.
  uint8_t get_valid_next_pattern() {
    if (isShowingWled())
      return random8(0, numInternalPatterns);
    return random8(0, gPatternCount);
  }

  // Choose the pattern to display at the next pattern cycle
  // Return the number of phrases until the next pattern cycle
  uint16_t set_next_pattern(uint16_t phrase) {
    uint8_t pattern_id;
    PatternDef def;

#ifdef IDENTIFY_STUCK_PATTERNS
    Serial.println("Changing next pattern");
#endif
    // Try 10 times to find a pattern that fits the current "energy"
    for (int i = 0; i < 10; i++) {
      pattern_id = get_valid_next_pattern();
      def = gPatterns[pattern_id];
      if (def.control.energy <= energy)
        break;
    }
#ifdef IDENTIFY_STUCK_PATTERNS
    Serial.printf("Next pattern will be %d\n", pattern_id);
#endif

    next_state.pattern_id = pattern_id;
    next_state.pattern_sync_id = randomSyncMode();

    switch (def.control.duration) {
      case ExtraShortDuration: return random8(2, 6);
      case ShortDuration: return random8(5,15);
      case MediumDuration: return random8(15,25);
      case LongDuration: return random8(20,40);
      case ExtraLongDuration: return random8(25, 60);
    }
    return 5;
  }

  void load_palette(TubeState &tube_state) {
    if (current_state.palette_id == tube_state.palette_id)
      return;

    current_state.palette_phrase = tube_state.palette_phrase;
    current_state.palette_id = tube_state.palette_id % gGradientPaletteCount;
    set_wled_palette(current_state.palette_id);
  }

  // Choose the palette to display at the next palette cycle
  // Return the number of phrases until the next palette cycle
  uint16_t set_next_palette(uint16_t phrase) {
#if defined(GOLDEN)
    uint r = random8(0, 4);
    uint colors[4] = {18, 58, 71, 111};
    next_state.palette_id = colors[r];
#elif defined(CHRISTMAS)   // 81, 107 are too bright
    uint r = random8(0, 26);
    uint colors[26] = {/*gold:*/18, 58, 71, 111,
                      /*yes:*/25, 34, 61, 63, 81, 112,
                      /*yesx2:*/25, 34, 61, 63, 81, 112,
                      /*best yes:*/25, 34, 34, 61, 63, 81, 112,
                      /*maybe:*/81, 28, 107};
    next_state.palette_id = colors[r];
#elif defined(RUBY)   // 81, 107 are too bright
    uint r = random8(0, 20);
    uint colors[20] = {/*gold:*/,
                      /*yes:*/21,
                      /*best yes:*/,
                      /*maybe:*/33, 35, 44, 81, 93, 107;
    next_state.palette_id = colors[r];
#elif defined(MAUVE)
    uint r = random8(0, 10);
    // Absolute WLED palette IDs (0..12 are built-ins, gradients start at 13)
    uint colors[10] = {
      20, 21, 33, 39, 40, 108, 109, 114, 120, 82
    };
    next_state.palette_id = colors[r];
#else
    // Don't select the built-in palettes
    next_state.palette_id = random8(6, gGradientPaletteCount);
#endif

    auto phrases = random8(MIN_COLOR_CHANGE_PHRASES, MAX_COLOR_CHANGE_PHRASES);

    // Change color more often in boring patterns
    if (isBoring) {
      phrases /= 2;
    }
    return phrases;
  }

  void load_effect(TubeState &tube_state) {
    if (current_state.effect_params.effect == tube_state.effect_params.effect &&
        current_state.effect_params.pen == tube_state.effect_params.pen &&
        current_state.effect_params.chance == tube_state.effect_params.chance)
      return;

    _load_effect(tube_state.effect_params);
  }

  void _load_effect(EffectParameters params) {
    current_state.effect_params = params;

    Serial.print(F("Change effect "));
    current_state.print();
    Serial.println();

    effects.load(current_state.effect_params);
  }

  // Choose the effect to display at the next effect cycle
  // Return the number of phrases until the next effect cycle
  uint16_t set_next_effect(uint16_t phrase) {
    uint8_t effect_num = random8(gEffectCount);

    // Pick a random effect to add; boring patterns get better chance at having an effect.
    EffectDef def = gEffects[effect_num];
    if (def.control.energy > energy) {
      def = gEffects[0];
    }

    next_state.effect_params = def.params;

    switch (def.control.duration) {
      case ExtraShortDuration: return random(1,3);
      case ShortDuration: return random(2,4);
      case MediumDuration: return random(4,7);
      case LongDuration: return random(8, 11);
      case ExtraLongDuration: return random(10,15);
    }
    return 1;
  }

  void update_background() {
    Background background;
    background.animate = gPatterns[current_state.pattern_id].backgroundFn;
    background.wled_fx_id = gPatterns[current_state.pattern_id].wled_fx_id;
    background.palette_id = current_state.palette_id;
    background.sync = (SyncMode)current_state.pattern_sync_id;

    // Use one of the virtual strips to render the patterns.
    // A WLED-based pattern exists on the virtual strip, but causes
    // it to do nothing since WLED merging happens in handleOverlayDraw.
    // Reuse virtual strips to prevent heap fragmentation
    for (uint8_t i = 0; i < NUM_VSTRIPS; i++) {
      vstrips[i]->fadeOut();
    }
    vstrips[next_vstrip]->load(background);
    next_vstrip = (next_vstrip + 1) % NUM_VSTRIPS;

    uint8_t param = modeParameter(background.wled_fx_id);
    set_wled_pattern(background.wled_fx_id, param, param);
    set_wled_palette(background.palette_id);
  }

  bool isUnderWledControl() const {
    return !shouldRenderTubes() || paletteOverride || patternOverride;
  }

  void set_wled_palette(uint8_t palette_id) {
    if (!shouldRenderTubes())
      return;

    if (paletteOverride)
      palette_id = paletteOverride;

    Segment& seg = strip.getMainSegment();
    seg.setPalette(palette_id);

    stateChanged = true;
    stateUpdated(CALL_MODE_DIRECT_CHANGE);
  }

  void set_wled_pattern(uint8_t pattern_id, uint8_t speed, uint8_t intensity) {
    if (!shouldRenderTubes())
      return;

    if (patternOverride)
      pattern_id = patternOverride;
    else if (pattern_id == 0)
      pattern_id = DEFAULT_WLED_FX; // Never set it to solid

    Segment& seg = strip.getMainSegment();
    seg.speed = speed;
    seg.intensity = intensity;
    seg.setMode(pattern_id);

    stateChanged = true;
    stateUpdated(CALL_MODE_DIRECT_CHANGE);
  }

  void setBrightness(uint8_t brightness, bool share = true) {
    Serial.printf("brightness: %d\n", brightness);

    options.brightness = brightness;
    load_options(options);

    if (share)
      queue_options_broadcast();
  }

  void setDebugging(bool debugging, bool share = true) {
    Serial.printf("debugging: %d\n", debugging);

    options.debugging = debugging;
    load_options(options);

    if (share)
      queue_options_broadcast();
  }

  void togglePowerSave() {
    setPowerSave(!power_save);
  }

  void setPowerSave(bool ps) {
    power_save = ps;
    Serial.printf("power_save: %d\n", power_save);

    // Remember this setting on the next boot
    EEPROM.begin(2560);
    auto b = EEPROM.read(BOOT_OPTIONS_EEPROM_LOCATION);
    BootOptions* boot = (BootOptions*)&b;
    if (power_save)
      boot->default_power_save = BOOT_OPTION_POWER_SAVE_ON;
    else
      boot->default_power_save = BOOT_OPTION_POWER_SAVE_OFF;
    EEPROM.write(BOOT_OPTIONS_EEPROM_LOCATION, b); // Reset all boot options
    Serial.printf("wrote: %d\n", b);
    EEPROM.end();
  }

  void setRole(ControllerRole r) {
    role = r;
    Serial.printf("Role = %d", role);
    EEPROM.begin(EEPSIZE);
    EEPROM.write(ROLE_EEPROM_LOCATION, role);
    EEPROM.write(BOOT_OPTIONS_EEPROM_LOCATION, 0); // Reset all boot options
    EEPROM.end();
    delay(10);
    doReboot = true;
  }

  SyncMode randomSyncMode() {
    uint8_t r = random8(128);

    // For boring patterns, up the chance of a sync mode
    if (isBoring)
      r -= 20;

    if (r < 30)
      return SinDrift;
    if (r < 50)
      return Pulse;
    if (r < 70)
      return Swing;
    if (r < 80)
      return SwingDrift;
    return All;
  }

  void updateGraphics() {
    static BeatFrame_24_8 lastFrame = 0;
    BeatFrame_24_8 beat_frame = current_state.beat_frame;

    uint8_t beat_pulse = 0;
    for (int i = 0; i < 8; i++) {
      if ( (beat_frame >> (5+i)) != (lastFrame >> (5+i)))
        beat_pulse |= 1<<i;
    }
    lastFrame = beat_frame;

    wled_fader = 0;

    VirtualStrip *first_strip = NULL;
    for (uint8_t i=0; i < NUM_VSTRIPS; i++) {
      VirtualStrip *vstrip = vstrips[i];
      if (vstrip->fade == Dead)
        continue;

      // Remember the first strip
      if (first_strip == NULL)
        first_strip = vstrip;

      // Remember the strip that's actually WLED
      if (vstrip->isWled())
        wled_fader = vstrip->fader;

      vstrip->update(beat_frame, beat_pulse);
    }

    effects.update(first_strip, beat_frame, (BeatPulse)beat_pulse);
  }

  CRGB getBlendedPixelColor(int32_t pos) const {
    // Calculate the color of the pixel at position i by blending the colors of the virtual strips
    CRGB color = CRGB::Black;

    bool first_strip = true;
    for (uint8_t i=0; i < NUM_VSTRIPS; i++) {
      VirtualStrip *vstrip = vstrips[i];

      // Don't bother blending a fully faded strip, or the WLED strip itself
      if (vstrip->fade == Dead || vstrip->isWled())
        continue;

      auto br = vstrip->brightness;
      // TODO: code intended to use scale8(options.brightness, vstrip->brightness);
      // but that was never implemented - should review later to see if we want
      // options.brightness to be a factor in the brightness of the strip

      // Fetch the color from the strip and dim it according to the brightness
      CRGB c = vstrip->getPixelColor(pos);
      nscale8x3(c.r, c.g, c.b, br);
      nscale8x3(c.r, c.g, c.b, vstrip->fader>>8);

      if (first_strip) {
        color = c;
        first_strip = false;
      } else {
        color |= c;
      }
    }

    return color;
  }

  virtual void acknowledge() {
    addFlash(CRGB::Green);
  }

  bool decodeOperation(char command, uint16_t argument, TubeOperation& operation) const {
    for (const TubeCommandDefinition& definition : tubeCommandDefinitions) {
      if (definition.command != command)
        continue;

      if (command == '*' || command == '(')
        argument = 1;
      else if (command == ')')
        argument = 0;
      operation = {argument, definition.tag};
      return true;
    }
    return false;
  }

  void broadcastAction(char key, uint16_t argument) {
    Action action = {.key = key, .arg = uint8_t(argument)};
    broadcast_action(action);
  }

  void printHelp() const {
    Serial.println(F("b###.# - set bpm\ns - start phrase\n\np### - patterns\nm### - sync mode\nc### - colors\ne### - effects\nn - force next\n\ni### - set ID\nd - toggle debugging\nl### - brightness"));
    Serial.println(F("@ - set power saving mode\nU - begin auto-update\nP - toggle all power saves\nO - toggle all sound overlays\n==== wifi ====\na - turn on access point\nq - turn off access point\nt0/1 - Tubes mode off/on"));
    Serial.println(F("==== global actions ====\n* - enter select mode (double-click to Ready)\nA - turn on access point (Ready to update)\nW - forget WiFi client\nX - restart\nV### - auto-upgrade to version\nz############ - probe a device by MAC\nM - cancel manual pattern override"));
  }

  bool executeOperation(const TubeOperation& operation) {
    uint16_t argument = operation.argument;
    TubeScope scope = tubeOperationScope(operation);
    bool share = scope != LocalScope;

    const TubeOperationCode operationCode = tubeOperationCode(operation);
#ifdef TUBES_READ_ONLY_FIELD_SHELL
    // The field shell is an inventory/control display, never an update or role authority.
    if (operationCode == RebootOperation || operationCode == UpdateOperation
        || operationCode == UpdateOfferOperation || operationCode == SelectOperation
        || operationCode == RoleOperation) {
      Serial.println(F("Tubes: operation denied by read-only field-shell capability"));
      return false;
    }
#endif

    switch (operationCode) {
      case DebugOperation:
        setDebugging(argument, share);
        return true;
      case RebootOperation:
        if (!share)
          doReboot = true;
        else
          broadcastAction('X', 0);
        return true;
      case PowerSaveOperation:
        if (!share)
          setPowerSave(argument);
        else
          broadcastAction('@', argument);
        return true;
      case BrightnessOperation:
        if (argument < 5 || argument > 255)
          break;
        setBrightness(argument, share);
        return true;
      case AccessPointOperation:
        if (!share) {
          if (!isHomeLightRole()) {
            Serial.println(F("Turning on WiFi access point."));
            WLED::instance().initAP(true);
          }
        } else {
          broadcastAction('A', 0);
        }
        return true;
      case DisconnectWifiOperation:
        if (!isHomeLightRole()) {
          Serial.println(F("Turning off WiFi access point."));
          WiFi.disconnect(true);
        }
        return true;
      case ForgetWifiOperation:
        if (!share) {
          if (!isHomeLightRole()) {
            Serial.println(F("Clearing WiFi connection."));
            strcpy(multiWiFi[0].clientSSID, "");
            strcpy(multiWiFi[0].clientPass, "");
            WiFi.disconnect(false, true);
          }
        } else {
          broadcastAction('W', 0);
        }
        return true;
      case BpmOperation:
        if (argument < 60 * 256)
          break;
        if (share)
          request_new_bpm(argument);
        else
          set_tapped_bpm(argument, 0);
        return true;
      case StartPhraseOperation:
        beats.start_phrase();
        update_beat();
        if (share)
          send_update();
        return true;
      case NextOperation:
        force_next(share);
        return true;
      case PatternOperation:
        next_state.pattern_phrase = 0;
        next_state.pattern_id = argument;
        next_state.pattern_sync_id = All;
        if (share) broadcast_state();
        return true;
      case SyncModeOperation:
        next_state.pattern_phrase = 0;
        next_state.pattern_id = current_state.pattern_id;
        next_state.pattern_sync_id = argument;
        if (share) broadcast_state();
        return true;
      case PaletteOperation:
        next_state.palette_phrase = 0;
        next_state.palette_id = argument;
        if (share) broadcast_state();
        return true;
      case EffectOperation:
        next_state.effect_phrase = 0;
        next_state.effect_params = gEffects[argument % gEffectCount].params;
        if (share) broadcast_state();
        return true;
      case EffectChanceOperation:
        next_state.effect_phrase = 0;
        next_state.effect_params = current_state.effect_params;
        next_state.effect_params.chance = argument;
        if (share) broadcast_state();
        return true;
      case NodeIdOperation:
        Serial.printf("Reset! ID -> %03X\n", argument);
        node.reset(argument);
        return true;
      case UpdateOperation:
        if (!share) {
          if (!isHomeLightRole()) updater.start();
        } else {
          broadcastAction('U', 0);
        }
        return true;
      case UpdateOfferOperation:
        if (!share) {
          if (updater.current_version.version < argument)
            select();
        } else {
          broadcastAction('V', argument);
        }
        return true;
      case SelectOperation:
        if (!share) {
          if (argument) {
            Serial.println(F("enter select mode"));
            enterSelectMode();
          } else {
            Serial.println(F("exit select mode"));
            deselect();
          }
        } else {
          broadcastAction(argument ? '*' : ')', 0);
        }
        return true;
      case GlitterOperation:
        if (!share) {
          Serial.println(F("glitter!"));
          for (uint8_t i = 0; i < 10; i++) addGlitter();
        } else {
          broadcastAction('G', 0);
        }
        return true;
      case FlashOperation:
        if (!share) {
          Serial.println(F("flash!"));
          flashTimer.start(20000);
          flashColor = argument;
        } else {
          broadcastAction('F', argument);
        }
        return true;
      case RoleOperation:
        if (!share)
          setRole(ControllerRole(argument));
        else
          broadcastAction('R', argument);
        return true;
      case CancelOverrideOperation:
        if (!share) {
          Serial.println(F("cancel manual mode"));
          cancelOverrides();
        } else {
          broadcastAction('M', 0);
        }
        return true;
      case SoundOverlayOperation:
        if (!share)
          sound.overlay = argument;
        else
          broadcastAction('O', argument);
        return true;
      case TubesModeOperation:
        if (scope != LocalScope || argument > 1)
          break;
        pendingTubesMode = argument;
        tubesModeTimer.start(250);
        return true;
      case HelpOperation:
        printHelp();
        return true;
    }

    Serial.println(F("nope"));
    return false;
  }

  void readJsonOperations(JsonObject& root) {
    JsonObject json = root[F("tube")];
    if (json.isNull())
      return;

    if (root.containsKey(F("pin")))
      checkSettingsPIN(root[F("pin")].as<const char*>());
    if ((!correctPIN && strlen(settingsPIN)) || (json[F("v")] | 1) != 1)
      return;

    int scopeOverride = json[F("to")] | -1;
    if (scopeOverride < -1 || scopeOverride > SelectedScope)
      return;

    // Each sparse serial-style key becomes an operation and is consumed immediately.
    for (JsonPair item : json) {
      const char* key = item.key().c_str();
      if (!strcmp_P(key, PSTR("to")) || !strcmp_P(key, PSTR("v")))
        continue;
      if (!key[0] || key[1] || (!item.value().is<long>() && !item.value().is<bool>()))
        continue;
      long value = item.value().as<long>();
      if (value < 0 || value > UINT16_MAX)
        continue;

      TubeOperation operation;
      if (!decodeOperation(key[0], value, operation))
        continue;
      if (scopeOverride >= 0)
        operation.tag = (operation.tag & 0x3F) | (uint8_t(scopeOverride) << 6);
      executeOperation(operation);
    }
  }

  void read_keys() {
    if (!Serial.available())
      return;

    char c = Serial.read();
    char *k = key_buffer;
    uint8_t max = sizeof(key_buffer);
    for (uint8_t i = 0; *k && i < max - 1; i++)
      k++;
    if (c == '\n') {
      keyboard_command(key_buffer);
      key_buffer[0] = 0;
    } else {
      *k++ = c;
      *k = 0;
    }
  }

  accum88 parse_number(const char *s) const {
    uint16_t whole = 0;
    uint16_t fraction = 0;

    while (*s == ' ') s++;
    while (*s >= '0' && *s <= '9')
      whole = whole * 10 + (*s++ - '0');
    whole <<= 8;

    if (*s == '.') {
      uint16_t divisor = 1;
      while (*++s >= '0' && *s <= '9') {
        fraction = fraction * 10 + (*s - '0');
        divisor *= 10;
      }
      fraction = (fraction << 8) / divisor;
    }
    return whole + fraction;
  }

  void keyboard_command(char *command) {
    char key = command[0];
    if (!key)
      return;

    if (key == DEVICE_REPORT_ACTION_KEY) {
      requestDeviceReport(command + 1);
      return;
    }

    accum88 parsed = parse_number(command + 1);
    uint16_t argument = parsed >> 8;
    Serial.printf("[command=%c arg=%04x]\n", key, parsed);

    if (key == 'b')
      argument = parsed;
    else if (key == 'i')
      argument = parsed >> 4;
    else if (key == 'd')
      argument = !options.debugging;
    else if (key == '_')
      argument = !power_save;
    else if (key == 'P')
      argument = !power_save;
    else if (key == 'O')
      argument = !sound.overlay;
    else if (key == '-' || key == '+') {
      uint8_t brightness = options.brightness;
      char *position = command;
      while (*position++ == key)
        brightness += key == '+' ? 5 : -5;
      argument = brightness + (key == '+' ? 5 : -5);
    } else if (key == 't' && parsed != 0 && parsed != 256) {
      Serial.println(F("nope"));
      return;
    }

    TubeOperation operation;
    if (!decodeOperation(key, argument, operation)) {
      Serial.println(F("dunno?"));
      return;
    }
    executeOperation(operation);
  }

  void force_next(bool share = true) {
    uint16_t phrase = current_state.beat_frame >> 12;
    uint16_t next_phrase = min(next_state.pattern_phrase, min(next_state.palette_phrase, next_state.effect_phrase)) - phrase;
    next_state.pattern_phrase -= next_phrase;
    next_state.palette_phrase -= next_phrase;
    next_state.effect_phrase -= next_phrase;
    if (share)
      broadcast_state();
  }

  void broadcast_action(Action& action) {
    if (!node.isFollowing()) {
      onAction(&action);
    }
    node.sendCommand(COMMAND_ACTION, &action, sizeof(Action));
  }

  void broadcast_info(NodeInfo *info) {
    node.sendCommand(COMMAND_INFO, &info, sizeof(NodeInfo));
  }

  void broadcast_state() {
    // Publishing this device's state would reveal the new BPM before its phrase boundary.
    if (deferredBpmBroadcast.active())
      return;
    node.sendCommand(COMMAND_STATE, &current_state, sizeof(TubeStates));
  }

  void broadcast_options() {
    node.sendCommand(COMMAND_OPTIONS, &options, sizeof(options));
  }

  void queue_options_broadcast() {
    broadcast_options();
    optionsBroadcastsPending = OPTIONS_BROADCAST_RETRIES;
    optionsBroadcastTimer.start(OPTIONS_BROADCAST_PERIOD);
  }

  void broadcast_autoupdate() {
    node.sendCommand(COMMAND_UPGRADE, &updater.current_version, sizeof(updater.current_version));
  }

  void broadcast_bpm(accum88 bpm) {
    // Hacked in feature: request a new BPM
    node.sendCommand(COMMAND_BEATS, &bpm, sizeof(bpm));
  }

  void requestDeviceReport(const char* macText) {
    DeviceReportMessage request;
    if (!parseDeviceReportMac(macText, request.mac)) {
      Serial.println(F("TUBE_PROBE_ERROR invalid_mac"));
      return;
    }

    request.nonce = esp_random();
    Serial.printf(
      "TUBE_PROBE nonce=%08lX mac=%02x%02x%02x%02x%02x%02x\n",
      (unsigned long)request.nonce,
      request.mac[0], request.mac[1], request.mac[2],
      request.mac[3], request.mac[4], request.mac[5]
    );
    node.sendCommand(COMMAND_ACTION, &request, sizeof(request));
  }

  void broadcastDeviceReport(uint32_t nonce, const uint8_t mac[6]) {
    DeviceReportMessage report;
    report.kind = DeviceReportReply;
    report.hardwareFamily = TUBES_HARDWARE_FAMILY;
    report.tubesVersion = RELEASE_VERSION;
    report.nonce = nonce;
    memcpy(report.mac, mac, sizeof(report.mac));
    report.ledCount = strip.getLengthTotal();
    report.busCount = min((int)BusManager::getNumBusses(), 255);
    report.firmwareVariant = TUBES_FIRMWARE_VARIANT;
    report.controllerRole = role;
    if (node.status == LightNode::NODE_STATUS_STARTED)
      report.meshFlags |= DeviceReportMeshStarted;
    if (node.isFollowing())
      report.meshFlags |= DeviceReportMeshFollowing;
    if (isMasterRole())
      report.meshFlags |= DeviceReportMasterBehavior;
    report.nodeId = node.header.id;
    report.uplinkId = node.header.uplinkId;
    report.releaseHash = WLED_BUILD_DESCRIPTION.hash;
    report.uptimeSeconds = millis() / 1000;

    Bus* firstBus = BusManager::getBus(0);
    if (firstBus) {
      uint8_t pins[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
      if (firstBus->getPins(pins) > 0)
        report.ledPin = pins[0];
      report.ledType = firstBus->getType() & 0x7F;
    }
    node.sendCommand(COMMAND_ACTION, &report, sizeof(report));
  }

  void printDeviceReport(const DeviceReportMessage& report) const {
    Serial.printf(
      "TUBE_REPORT nonce=%08lX mac=%02x%02x%02x%02x%02x%02x family=%u variant=%u tubes=%u release=%08lX leds=%u buses=%u pin=%u type=%u role=%u mesh=%u node=%u uplink=%u uptime=%lu\n",
      (unsigned long)report.nonce,
      report.mac[0], report.mac[1], report.mac[2],
      report.mac[3], report.mac[4], report.mac[5],
      report.hardwareFamily,
      report.firmwareVariant,
      report.tubesVersion,
      (unsigned long)report.releaseHash,
      report.ledCount,
      report.busCount,
      report.ledPin,
      report.ledType,
      report.controllerRole,
      report.meshFlags,
      report.nodeId,
      report.uplinkId,
      (unsigned long)report.uptimeSeconds
    );
  }

  void onDeviceReportMessage(const DeviceReportMessage& message) {
    if (!isDeviceReportMessage(message))
      return;

    if (message.kind == DeviceReportReply) {
      printDeviceReport(message);
      return;
    }

    uint8_t deviceMac[6];
    Network.localMAC(deviceMac);
    if (deviceReportTargetsMac(message, deviceMac))
      broadcastDeviceReport(message.nonce, deviceMac);
  }

  virtual bool onCommand(CommandId command, void *data) override {
    switch (command) {
      case COMMAND_INFO:
        Serial.printf("   \"%s\"\n",
          ((NodeInfo*)data)->message
        );
        return true;

      case COMMAND_OPTIONS:
        memcpy(&options, data, sizeof(options));
        load_options(options);
        Serial.printf("[debug=%d  bri=%d]",
          options.debugging,
          options.brightness
        );
        return true;

      case COMMAND_STATE: {
        auto update_data = (TubeStates*)data;

        TubeState state;
        memcpy(&state, &update_data->current, sizeof(TubeState));
        memcpy(&next_state, &update_data->next, sizeof(TubeState));
        state.print();
        next_state.print();

        // Catch up to this state
        load_pattern(state);
        load_palette(state);
        load_effect(state);
        // Visual state still follows the root, but its old clock must not undo a local
        // BPM change that is waiting for this device's phrase boundary.
        if (!deferredBpmBroadcast.active())
          beats.sync(state.bpm, state.beat_frame);
        return true;
      }

      case COMMAND_UPGRADE:
#ifdef TUBES_READ_ONLY_FIELD_SHELL
        Serial.println(F("Tubes: upgrade offer denied by read-only field-shell capability"));
        return false;
#else
        // HOMELIGHT must relay upgrade offers without installing Tubes firmware.
        if (!isHomeLightRole())
          updater.start((AutoUpdateOffer*)data);
        return true;
#endif

      case COMMAND_ACTION:
        if (((Action*)data)->key == DEVICE_REPORT_ACTION_KEY)
          onDeviceReportMessage(*(DeviceReportMessage*)data);
        else
          onAction((Action*)data);
        return true;

      case COMMAND_BEATS:
        // A declared BPM supersedes any local change that has not reached its boundary.
        deferredBpmBroadcast.cancel();
        apply_bpm(*(accum88*)data);
        return true;
    }

    Serial.printf("UNKNOWN COMMAND %02X", command);
    return false;
  }

  void onAction(Action* action) {
    TubeOperation operation;
    if (!decodeOperation(action->key, action->arg, operation))
      return;
    if (tubeOperationScope(operation) == SelectedScope && !isSelected())
      return;

    operation.tag &= 0x3F;
    executeOperation(operation);
  }

#define WIZMOTE_BUTTON_ON          1
#define WIZMOTE_BUTTON_OFF         2
#define WIZMOTE_BUTTON_NIGHT       3
#define WIZMOTE_BUTTON_ONE         16
#define WIZMOTE_BUTTON_TWO         17
#define WIZMOTE_BUTTON_THREE       18
#define WIZMOTE_BUTTON_FOUR        19
#define WIZMOTE_BUTTON_BRIGHT_UP   9
#define WIZMOTE_BUTTON_BRIGHT_DOWN 8

  void force_next_pattern() {
    next_state.pattern_phrase = current_state.beat_frame >> 12;
    if (next_state.palette_phrase == next_state.pattern_phrase)
      next_state.palette_phrase += random8(0, 5);
    force_next();
  }

  void force_next_effect() {
    next_state.effect_phrase = current_state.beat_frame >> 12;
    force_next();
  }

  virtual bool onButton(uint8_t button_id) override {
    bool isMaster = !this->node.isFollowing();

    switch (button_id) {
      case WIZMOTE_BUTTON_ON:
        if (!isHomeLightRole())
          WLED::instance().initAP(true);
        setDebugging(true);
        acknowledge();
        return true;

      case WIZMOTE_BUTTON_OFF:
        if (!isHomeLightRole()) {
          WiFi.softAPdisconnect(true);
          apActive = false;
          WiFi.disconnect(false, true);
#if WLED_WATCHDOG_TIMEOUT > 0
          WLED::instance().enableWatchdog();
#endif
          apBehavior = AP_BEHAVIOR_BUTTON_ONLY;
        }
        setDebugging(false);
        acknowledge();
        return true;

      case WIZMOTE_BUTTON_ONE:
        // Make it interesting - switch to a good pattern and sync mode
        // Only the master will respond to this
        if (!isMaster)
          return false;

        Serial.println("WizMote preset 1: de-sync");

        set_next_pattern(0);
        while (next_state.pattern_sync_id == All)
          set_next_pattern(0);

        this->force_next_pattern();
        return true;

      case WIZMOTE_BUTTON_TWO:
        // Apply an interesting effect & sync layer
        // Only the master will respond to this
        if (!isMaster)
          return false;

        Serial.println("WizMote preset 2: add an effect");

        set_next_effect(0);
        while (next_state.effect_params.effect == None)
          set_next_effect(0);

        this->force_next_effect();
        return true;

      case WIZMOTE_BUTTON_THREE:
        // Turn on flames.  Also up the tempo to 125
        // Only the master will respond to this
        if (!isMaster)
          return false;

        // Switch to house mode
        set_tapped_bpm(125<<8);

        Serial.println("WizMote preset 3: flames!");
        next_state.pattern_id = 63; // Fire
        next_state.pattern_sync_id = SyncMode::All;
        this->force_next_pattern();
        return true;

      case WIZMOTE_BUTTON_FOUR:
        // Make it an interesting combo
        // Only the master will respond to this
        if (!isMaster)
          return false;

        // 38: Noise 3
        Serial.println("WizMote preset 4: interesting pattern");

        set_next_pattern(0);
        next_state.pattern_id = 38; // overwrite with: Noise 3

        this->force_next_pattern();
        return true;

      case WIZMOTE_BUTTON_BRIGHT_UP:
        // Brighten (ignored if in power save mode)
        Serial.println("WizMote: brightness up");
        if (options.brightness <= 230)
          setBrightness(options.brightness + 25);
        return true;

      case WIZMOTE_BUTTON_BRIGHT_DOWN:
        // Dim (ignored if in power save mode)
        Serial.println("WizMote: brightness down");

        if (options.brightness >= 25)
          setBrightness(options.brightness - 25);
        return true;

      case WIZMOTE_BUTTON_NIGHT:
        // Chill mode
        // Only the master will respond to this
        if (!isMaster)
          return false;

        Serial.println("WizMote: chill");

        // Switch to deep house mode
        set_tapped_bpm(120<<8);

        this->force_next();
        return true;

      default:
        Serial.printf("TubesButton %d master=%d\n", button_id, isMaster);
        return false;
    }
  }


};

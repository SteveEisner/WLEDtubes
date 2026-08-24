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
#include "v3_runtime.h"

#define EEPSIZE 2560

const static uint8_t DEFAULT_MASTER_BRIGHTNESS = 200;
const static uint8_t DEFAULT_TUBE_BRIGHTNESS = 120;
const static uint8_t DEFAULT_TANK_BRIGHTNESS = 240;
const static uint16_t DEFAULT_TUBE_CURRENT_LIMIT = 500;
#define DEFAULT_WLED_FX FX_MODE_RAINBOW_CYCLE

#define STATUS_UPDATE_PERIOD 2000
#define V3_HEARTBEAT_PERIOD 10000
#define V3_PRESENCE_PERIOD 10000

static_assert(GRADIENT_PALETTE_COUNT <= UINT8_MAX, "Tubes palette IDs must fit in one byte");
static constexpr uint8_t gGradientPaletteCount = GRADIENT_PALETTE_COUNT;
static const char tubesV3PaletteName[] PROGMEM = "Tubes v3";

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

static_assert(sizeof(TubeStates) == V3_PROJECTION_TRAILER_OFFSET,
              "V2 TubeStates must end where the gen1 projection marker begins");

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
  BeatChannelIdOperation,
  PatternChannelIdOperation,
  PaletteChannelIdOperation,
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
  TUBE_COMMAND('F', FlashOperation, SelectedScope),
  TUBE_COMMAND('r', RoleOperation, LocalScope),
  TUBE_COMMAND('R', RoleOperation, SelectedScope),
  TUBE_COMMAND('M', CancelOverrideOperation, MeshScope),
  TUBE_COMMAND('O', SoundOverlayOperation, MeshScope),
  TUBE_COMMAND('t', TubesModeOperation, LocalScope),
  TUBE_COMMAND('B', BeatChannelIdOperation, LocalScope),
  TUBE_COMMAND('K', PatternChannelIdOperation, LocalScope),
  TUBE_COMMAND('C', PaletteChannelIdOperation, LocalScope),
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
    uint8_t paletteOverride = 0;
    uint8_t patternOverride = 0;
    uint8_t activePatternRenderMask = 0;
    uint16_t wled_fader = 0;
    ControllerRole role;
    bool power_save = false;  // Default to power save mode OFF but 3 sec press turns it on
    uint8_t flashColor = 0;

    AutoUpdater updater = AutoUpdater();
    Sounder sound = Sounder();

    TubesTimer graphicsTimer;
    TubesTimer updateTimer;
    TubesTimer paletteOverrideTimer;
    TubesTimer patternOverrideTimer;
    TubesTimer flashTimer;
    TubesTimer selectTimer;
    TubesTimer tubesModeTimer;
    TubesTimer v3HeartbeatTimer;
    TubesTimer v3PresenceTimer;
    TubesTimer v3DebugExpiryTimer;
    TubesTimer v3PositionTimer;
    TubesTimer v3PaletteRepeatTimer;
    TubesTimer deviceReportTimer;
    TubesTimer targetedUpdateTimer;
    TubesTimer identifyTimer;
    TubesTimer startupBrightnessTimer;

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
    V3ProtocolRuntime v3;

    ChannelWinnerTable channelWinners;
    TubesChannelPayload channelSnapshots[3];
    DeviceId channelIds[3] = {0, 0, 0};
    uint16_t channelSequences[3] = {0, 0, 0};
    uint16_t controlSequence = 0;
    uint32_t channelSession = 1;

    uint16_t v3DebugOverlayMask = V3DebugBeat | V3DebugNode | V3DebugLeader;
    uint8_t v3DebugVerbosity = 0;
    bool v3DebugExpiryActive = false;
    uint8_t v3RuntimePaletteId = 0;
    CRGBPalette16 v3RuntimePalette;
    bool v3RuntimePaletteActive = false;
    bool v3PalettePending = false;
    V3PaletteSchedulePayload v3PendingPaletteSchedule;
    V3Palette16Definition v3PendingPaletteDefinition;
    V3Palette16Definition v3LocalPaletteDefinition;
    V3PaletteSchedulePayload v3LocalPaletteSchedule;
    uint8_t v3PaletteRepeatsPending = 0;
    uint8_t lastPublishedBeat = UINT8_MAX;
    V3PositionAdvertisementPayload v3LocalPosition;
    bool hasV3LocalPosition = false;
    V3PositionFramePayload v3ConfiguredPositionFrame;
    bool hasV3ConfiguredPositionFrame = false;
    DeviceReportMessage pendingDeviceReport;
    uint32_t lastDeviceReportProbeNonce = 0;
    bool deviceReportPending = false;
    bool targetedUpdatePending = false;
    bool identifyActive = false;
    uint8_t startupBrightness = 0;
    bool startupBrightnessRamping = false;

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

  // Reserve one runtime-only WLED palette slot; updating it never writes LittleFS.
  void registerV3RuntimePalette() {
    removeUsermodPalettes(tubesV3PaletteName);
    if (usermodPalettes.size() >= WLED_MAX_USERMOD_PALETTES) {
      Serial.println(F("Tubes v3: no runtime palette slot available"));
      v3RuntimePaletteId = 0;
      return;
    }

    uint8_t paletteIndex = usermodPalettes.size();
    v3RuntimePalette = CRGBPalette16(CRGB::Black);
    usermodPalettes.push_back({v3RuntimePalette, tubesV3PaletteName, 0, nullptr});
    v3RuntimePaletteId = WLED_USERMOD_PALETTE_ID_BASE - paletteIndex;
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
        BusManager::setMilliampsMax(min<uint16_t>(ABL_MILLIAMPS_DEFAULT, DEFAULT_TUBE_CURRENT_LIMIT));
      else if (role <= InstallationRole)
        BusManager::setMilliampsMax(1000);
      else
        BusManager::setMilliampsMax(1400);
      // AI: below section was generated by an AI
      // WLED initialized ABL before the usermod changed the role-specific cap.
      // Reinitialize it so a saved disabled configuration cannot bypass this limit.
      BusManager::initializeABL();
      // AI: end
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
    registerV3RuntimePalette();
    v3.setup(node.header.id, esp_random());
    channelSession = esp_random();
    if (!channelSession)
      channelSession = 1;
    for (uint8_t index = 0; index < 3; index++)
      channelIds[index] = node.header.id;
    v3HeartbeatTimer.start(V3_HEARTBEAT_PERIOD);
    ProtocolLocalValue localValue = protocolLocalValueFromId(node.header.id);
    v3PresenceTimer.start((localValue * 37U) % V3_PRESENCE_PERIOD);
    v3PositionTimer.start((localValue * 53U) % 1000U);
    options.debugging = false;
    startBrightnessRamp();

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
    bool publishPattern = false;
    bool publishPalette = false;
    bool publishEffect = false;

    if (phrase >= next_state.pattern_phrase) {
#ifdef IDENTIFY_STUCK_PATTERNS
      Serial.println("Time to change pattern");
#endif
      load_pattern(next_state);
      bool canSchedule = channelWinners.localMayRequest(
          PatternChannel,
          localChannelId(PatternChannel),
          node.header.id,
          millis()
      );
      if (canSchedule) {
        next_state.pattern_phrase = phrase + set_next_pattern(phrase);
        publishPattern = true;
        // Keep independently scheduled visual changes on distinct phrases.
        while (next_state.pattern_phrase == next_state.palette_phrase || next_state.pattern_phrase == next_state.effect_phrase)
          next_state.pattern_phrase += random8(1,3);
      } else {
        next_state.pattern_phrase = UINT16_MAX;
      }
      changed = true;
    }
    if (phrase >= next_state.palette_phrase) {
#ifdef IDENTIFY_STUCK_PATTERNS
      Serial.println("Time to change palette");
#endif
      load_palette(next_state);
      bool canSchedule = channelWinners.localMayRequest(
          PaletteChannel,
          localChannelId(PaletteChannel),
          node.header.id,
          millis()
      );
      if (canSchedule && !v3RuntimePaletteActive) {
        next_state.palette_phrase = phrase + set_next_palette(phrase);
        publishPalette = true;
        while (next_state.palette_phrase == next_state.pattern_phrase || next_state.palette_phrase == next_state.effect_phrase)
          next_state.palette_phrase += random8(1,3);
      } else {
        next_state.palette_phrase = UINT16_MAX;
      }
      changed = true;
    }
    if (phrase >= next_state.effect_phrase) {
#ifdef IDENTIFY_STUCK_PATTERNS
      Serial.println("Time to change effect");
#endif
      load_effect(next_state);
      bool canSchedule = channelWinners.localMayRequest(
          PatternChannel,
          localChannelId(PatternChannel),
          node.header.id,
          millis()
      );
      if (canSchedule) {
        next_state.effect_phrase = phrase + set_next_effect(phrase);
        publishEffect = true;
        while (next_state.effect_phrase == next_state.pattern_phrase || next_state.effect_phrase == next_state.palette_phrase)
          next_state.effect_phrase += random8(1,3);
      } else {
        next_state.effect_phrase = UINT16_MAX;
      }
      changed = true;
    }

    if (changed && node.serialTraceEnabled()) {
      next_state.print();
      Serial.println();
    }
    if (publishPattern || publishEffect)
      publishApplicationChannel(PatternChannel);
    if (publishPalette)
      publishApplicationChannel(PaletteChannel);
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

  // AI: below section was generated by an AI
  // Includes temporary Device-ID selection without putting the device into update mode.
  bool isOperationSelected() const {
    return isSelected() || identifyActive;
  }
  // AI: end

  void select(bool selected = true) {
    if (isHomeLightRole())
      return;

    if (selected)
      updater.ready();
    else
      updater.stop();
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
      if (node.serialTraceEnabled())
        Serial.println("WLED has control of palette.");
      paletteOverrideTimer.start(300000); // 5 minutes of manual control
    } else {
      if (node.serialTraceEnabled())
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
      if (node.serialTraceEnabled())
        Serial.println("WLED has control of patterns.");
      patternOverrideTimer.start(300000); // 5 minutes of manual control
      transitionDelay = 500;  // Short transitions
    } else {
      if (node.serialTraceEnabled())
        Serial.println("Turning off WLED control of patterns.");
      patternOverrideTimer.stop();
      transitionDelay = 8000; // Back to long transitions

      uint8_t param = modeParameter(auto_mode);
      const PatternRenderOptions& renderOptions = gPatterns[current_state.pattern_id].renderOptions;
      set_wled_pattern(auto_mode, param, param, renderOptions);
    }
  }

  // AI: below section was generated by an AI
  uint16_t currentPhrase() const {
    return current_state.beat_frame >> 12;
  }

  uint16_t activeDebugOverlayMask() const {
    if (!options.debugging)
      return 0;
    return v3DebugOverlayMask;
  }

  uint8_t activePositionConfidence() const {
    return hasV3LocalPosition ? v3LocalPosition.confidence : 0;
  }

  void addV3JsonInfo(JsonObject& root) const {
    JsonObject info = root.createNestedObject(F("tubesV3"));
    info[F("mode")] = 0;
    info[F("accepted")] = v3.counters.accepted;
    info[F("rejected")] = v3.counters.rejected;
    info[F("stale")] = v3.counters.stale;
    info[F("unknown")] = v3.counters.unknown;
    info[F("legacyPackets")] = v3.counters.legacyPackets;
  }

  bool localOwnsV3Topic(uint8_t topic) const {
    const V3AuthorityState& authority = v3.authorities.get(topic);
    return authority.active
        && authority.authorityId == node.header.id
        && authority.authoritySession == v3.session();
  }

  // Send a claim when ownership is new or its repeat interval has elapsed.
  bool sendV3Claim(uint8_t topic, bool force = false) {
    V3TopicPayload claim;
    if (!v3.prepareClaim(topic, millis(), claim, force))
      return localOwnsV3Topic(topic);
    return node.sendV3Topic(topic, claim);
  }

  // Publish one authority-owned body after the local claim establishes its ordering session.
  bool sendV3Body(
      uint8_t topic,
      V3MessageKind kind,
      const void* body,
      uint8_t bodyLength,
      uint16_t leaseDeciseconds = V3_DEFAULT_TOPIC_LEASE_DECISECONDS,
      uint8_t quality = 0,
      uint8_t flags = 0,
      bool forceClaim = false,
      uint16_t* sequence = nullptr
  ) {
    if (topic != V3TopicPresence
        && !(topic == V3TopicPosition && kind == V3Advertisement)
        && !sendV3Claim(topic, forceClaim))
      return false;

    V3TopicPayload payload;
    if (!v3.prepareMessage(
          topic,
          kind,
          body,
          bodyLength,
          leaseDeciseconds,
          millis(),
          payload,
          quality,
          flags))
      return false;
    if (sequence)
      *sequence = payload.envelope.sequence;
    return node.sendV3Topic(topic, payload);
  }

  void publishV3Presence() {
    V3PresencePayload presence;
    presence.protocolMode = 0;
    presence.topicMask = (1U << (V3TopicControl + 1)) - 1;
    presence.paletteFormatMask = (1U << V3PaletteLegacyId)
        | (1U << V3Palette16)
        | (1U << V3PaletteGradientStops);
    presence.paletteCacheEntries = 2;
    presence.bootSession = v3.session();
    sendV3Body(V3TopicPresence, V3State, &presence, sizeof(presence));
  }

  void publishV3Beat(bool forceClaim = false) {
    V3BeatPayload beat;
    beat.bpm = beats.bpm;
    beat.beatFrame = beats.frac;
    beat.measuredAtTimebase = strip.timebase + millis();
    beat.sourceType = 4; // Fixed/local clock until a sensor source identifies itself.
    sendV3Body(V3TopicBeat, V3State, &beat, sizeof(beat), 40, 128, 0, forceClaim);
  }

  void publishV3Pattern(bool forceClaim = false) {
    V3PatternPayload pattern;
    pattern.current.effectivePhrase = current_state.pattern_phrase;
    pattern.current.patternId = current_state.pattern_id;
    pattern.current.syncMode = current_state.pattern_sync_id;
    pattern.current.legacyPatternId = current_state.pattern_id;
    pattern.current.legacySyncMode = current_state.pattern_sync_id;
    pattern.next.effectivePhrase = next_state.pattern_phrase;
    pattern.next.patternId = next_state.pattern_id;
    pattern.next.syncMode = next_state.pattern_sync_id;
    pattern.next.legacyPatternId = next_state.pattern_id;
    pattern.next.legacySyncMode = next_state.pattern_sync_id;
    sendV3Body(V3TopicPattern, V3State, &pattern, sizeof(pattern),
               V3_DEFAULT_TOPIC_LEASE_DECISECONDS, 0, 0, forceClaim);
  }

  void publishV3LegacyPalette(bool forceClaim = false) {
    V3PaletteSchedulePayload schedule;
    schedule.effectivePhrase = next_state.palette_phrase;
    schedule.transitionMs = transitionDelay;
    schedule.legacyPaletteId = next_state.palette_id;
    schedule.paletteFormat = V3PaletteLegacyId;
    sendV3Body(V3TopicPalette, V3Schedule, &schedule, sizeof(schedule),
               V3_DEFAULT_TOPIC_LEASE_DECISECONDS, 0, 0, forceClaim);
  }

  void publishV3Effect(bool forceClaim = false) {
    V3EffectPayload effect;
    effect.current.effectivePhrase = current_state.effect_phrase;
    effect.current.effect = current_state.effect_params.effect;
    effect.current.pen = current_state.effect_params.pen;
    effect.current.beat = current_state.effect_params.beat;
    effect.current.chance = current_state.effect_params.chance;
    effect.next.effectivePhrase = next_state.effect_phrase;
    effect.next.effect = next_state.effect_params.effect;
    effect.next.pen = next_state.effect_params.pen;
    effect.next.beat = next_state.effect_params.beat;
    effect.next.chance = next_state.effect_params.chance;
    sendV3Body(V3TopicEffect, V3State, &effect, sizeof(effect),
               V3_DEFAULT_TOPIC_LEASE_DECISECONDS, 0, 0, forceClaim);
  }

  void publishV3Debug(bool forceClaim = false) {
    V3DebugPayload debug;
    debug.enabled = options.debugging;
    debug.overlayMask = v3DebugOverlayMask;
    debug.verbosity = v3DebugVerbosity;
    sendV3Body(V3TopicDebug, V3State, &debug, sizeof(debug),
               V3_DEFAULT_TOPIC_LEASE_DECISECONDS, 0, 0, forceClaim);
  }

  void publishV3VisualSnapshot(bool claimMissingAuthorities) {
    bool rootCanSeed = !node.isFollowing() && claimMissingAuthorities;
    if (localOwnsV3Topic(V3TopicBeat) || (!v3.authorities.get(V3TopicBeat).active && rootCanSeed))
      publishV3Beat(rootCanSeed);
    if (localOwnsV3Topic(V3TopicPattern) || (!v3.authorities.get(V3TopicPattern).active && rootCanSeed))
      publishV3Pattern(rootCanSeed);
    if ((localOwnsV3Topic(V3TopicPalette) || (!v3.authorities.get(V3TopicPalette).active && rootCanSeed))
        && !v3PalettePending && !v3RuntimePaletteActive)
      publishV3LegacyPalette(rootCanSeed);
    if (localOwnsV3Topic(V3TopicEffect) || (!v3.authorities.get(V3TopicEffect).active && rootCanSeed))
      publishV3Effect(rootCanSeed);
    if (localOwnsV3Topic(V3TopicDebug) || (!v3.authorities.get(V3TopicDebug).active && rootCanSeed))
      publishV3Debug(rootCanSeed);
  }

  // Project a literal color master's palette into the runtime cache and schedule it atomically.
  bool publishV3LiteralPalette(
      const V3Palette16Definition& definition,
      uint8_t legacyPaletteId,
      uint16_t effectivePhrase,
      uint16_t transitionMs,
      bool forceClaim = true,
      bool queueRepeats = true
  ) {
    if (legacyPaletteId >= gGradientPaletteCount)
      return false;
    if (!sendV3Claim(V3TopicPalette, forceClaim))
      return false;

    uint16_t definitionSequence = 0;
    if (!sendV3Body(
          V3TopicPalette,
          V3Definition,
          &definition,
          sizeof(definition),
          V3_DEFAULT_TOPIC_LEASE_DECISECONDS,
          0,
          0,
          false,
          &definitionSequence))
      return false;

    V3PaletteSchedulePayload schedule;
    schedule.definitionSequence = definitionSequence;
    schedule.effectivePhrase = effectivePhrase;
    schedule.transitionMs = transitionMs;
    schedule.legacyPaletteId = legacyPaletteId;
    schedule.paletteFormat = V3Palette16;
    if (!sendV3Body(V3TopicPalette, V3Schedule, &schedule, sizeof(schedule)))
      return false;

    v3PendingPaletteDefinition = definition;
    v3PendingPaletteSchedule = schedule;
    v3PalettePending = true;
    v3LocalPaletteDefinition = definition;
    v3LocalPaletteSchedule = schedule;
    if (queueRepeats) {
      v3PaletteRepeatsPending = 2;
      v3PaletteRepeatTimer.start(400);
    }
    return true;
  }

  void activatePendingV3Palette() {
    if (!v3PalettePending || int16_t(currentPhrase() - v3PendingPaletteSchedule.effectivePhrase) < 0)
      return;

    next_state.palette_phrase = v3PendingPaletteSchedule.effectivePhrase;
    next_state.palette_id = v3PendingPaletteSchedule.legacyPaletteId;
    transitionDelay = v3PendingPaletteSchedule.transitionMs;
    strip.setTransition(transitionDelay);
    if (v3PendingPaletteSchedule.paletteFormat == V3Palette16
        || v3PendingPaletteSchedule.paletteFormat == V3PaletteGradientStops) {
      for (uint8_t index = 0; index < 16; index++) {
        v3RuntimePalette[index] = CRGB(
            v3PendingPaletteDefinition.rgb[index][0],
            v3PendingPaletteDefinition.rgb[index][1],
            v3PendingPaletteDefinition.rgb[index][2]
        );
      }
      if (v3RuntimePaletteId) {
        uint8_t paletteIndex = WLED_USERMOD_PALETTE_ID_BASE - v3RuntimePaletteId;
        if (paletteIndex < usermodPalettes.size())
          usermodPalettes[paletteIndex].palette = v3RuntimePalette;
      }
      v3RuntimePaletteActive = true;
      current_state.palette_phrase = v3PendingPaletteSchedule.effectivePhrase;
      current_state.palette_id = v3PendingPaletteSchedule.legacyPaletteId;
      update_background();
    } else {
      v3RuntimePaletteActive = false;
      load_palette(next_state);
    }
    v3PalettePending = false;
    v3PaletteRepeatsPending = 0;
  }

  // AI: below section was generated by an AI
  static uint8_t channelIndex(uint8_t channel) {
    return channel - BeatChannel;
  }

  DeviceId localChannelId(uint8_t channel) const {
    return channelIds[channelIndex(channel)];
  }

  TubesChannelPayload makeChannelPayload(
      uint8_t channel,
      TubesChannelMessageKind kind,
      const void* body,
      uint8_t bodyLength
  ) {
    TubesChannelPayload payload;
    payload.envelope.messageKind = kind;
    payload.envelope.channelId = isApplicationChannel(channel)
        ? localChannelId(channel)
        : node.header.id;
    payload.envelope.sourceControlId = node.header.id;
    payload.envelope.sourceSession = channelSession;
    payload.envelope.sequence = isApplicationChannel(channel)
        ? ++channelSequences[channelIndex(channel)]
        : ++controlSequence;
    payload.envelope.leaseDeciseconds = kind == ControlBeacon
        ? 0
        : TUBES_CHANNEL_LEASE_DECISECONDS;
    writeChannelBody(payload, body, bodyLength);
    return payload;
  }

  BeatChannelState currentBeatChannelState() const {
    BeatChannelState state;
    state.bpm = current_state.bpm;
    state.beatFrame = current_state.beat_frame;
    return state;
  }

  PatternChannelState currentPatternChannelState() const {
    PatternChannelState state;
    state.current.patternPhrase = current_state.pattern_phrase;
    state.current.patternId = current_state.pattern_id;
    state.current.syncMode = current_state.pattern_sync_id;
    state.current.effectPhrase = current_state.effect_phrase;
    state.current.effect = current_state.effect_params.effect;
    state.current.pen = current_state.effect_params.pen;
    state.current.beat = current_state.effect_params.beat;
    state.current.chance = current_state.effect_params.chance;
    state.next.patternPhrase = next_state.pattern_phrase;
    state.next.patternId = next_state.pattern_id;
    state.next.syncMode = next_state.pattern_sync_id;
    state.next.effectPhrase = next_state.effect_phrase;
    state.next.effect = next_state.effect_params.effect;
    state.next.pen = next_state.effect_params.pen;
    state.next.beat = next_state.effect_params.beat;
    state.next.chance = next_state.effect_params.chance;
    return state;
  }

  PaletteChannelState currentPaletteChannelState() const {
    PaletteChannelState state;
    state.current.palettePhrase = current_state.palette_phrase;
    state.current.paletteId = current_state.palette_id;
    state.next.palettePhrase = next_state.palette_phrase;
    state.next.paletteId = next_state.palette_id;
    return state;
  }

  bool publishApplicationChannel(uint8_t channel) {
    uint32_t nowMs = millis();
    if (!channelWinners.localMayRequest(channel, localChannelId(channel), node.header.id, nowMs))
      return false;

    TubesChannelPayload payload;
    if (channel == BeatChannel) {
      BeatChannelState state = currentBeatChannelState();
      payload = makeChannelPayload(channel, ChannelRequest, &state, sizeof(state));
    } else if (channel == PatternChannel) {
      PatternChannelState state = currentPatternChannelState();
      payload = makeChannelPayload(channel, ChannelRequest, &state, sizeof(state));
    } else {
      PaletteChannelState state = currentPaletteChannelState();
      payload = makeChannelPayload(channel, ChannelRequest, &state, sizeof(state));
    }

    if (node.isFollowing())
      return node.sendV3Channel(channel, payload);

    if (!channelWinners.acceptRequest(channel, payload.envelope, nowMs))
      return false;
    payload.envelope.messageKind = ChannelDeclaration;
    channelSnapshots[channelIndex(channel)] = payload;
    return node.sendV3Channel(channel, payload);
  }

  void publishControlBeacon() {
    ControlBeaconBody beacon;
    beacon.bootSession = channelSession;
    TubesChannelPayload payload = makeChannelPayload(
        ControlChannel,
        ControlBeacon,
        &beacon,
        sizeof(beacon)
    );
    node.sendV3Channel(ControlChannel, payload);
  }

  void publishV2Projection() {
    if (node.isFollowing())
      return;
    TubeStates projection = {current_state, next_state};
    V3ProjectionTrailer trailer;
    trailer.controlId = node.header.id;
    trailer.controlSession = uint16_t(channelSession);
    node.sendCommand(COMMAND_STATE, &projection, sizeof(projection), &trailer);
  }

  void updateV3Channels() {
    if (v3HeartbeatTimer.every(V3_HEARTBEAT_PERIOD))
      publishControlBeacon();

    uint8_t beat = beats.frac >> 8;
    if (beat != lastPublishedBeat) {
      lastPublishedBeat = beat;
      publishApplicationChannel(BeatChannel);
    }
  }
  // AI: end

  void updateV3Protocol() {
    updateV3Channels();
  }

  void update()
  {
    read_keys();

    // AI: below section was generated by an AI
    updateStartupBrightness();
    // AI: end

    // Network mode changes wait until an HTTP response has left the async handler.
    if (pendingTubesMode >= 0 && tubesModeTimer.ended()) {
      bool enabled = pendingTubesMode;
      pendingTubesMode = -1;
      setTubesMode(enabled);
    }

    beats.update();

    // Update the mesh
    node.update();

    if (deviceReportPending && deviceReportTimer.ended()) {
      deviceReportPending = false;
      uint8_t deviceMac[6];
      Network.localMAC(deviceMac);
      broadcastDeviceReport(pendingDeviceReport.nonce, deviceMac);
    }

    if (targetedUpdatePending && targetedUpdateTimer.ended()) {
      targetedUpdatePending = false;
      select();
    }

    // AI: below section was generated by an AI
    if (identifyActive && identifyTimer.ended()) {
      identifyActive = false;
      if (node.serialTraceEnabled())
        Serial.println(F("TUBE_IDENTIFY expired"));
    }
    // AI: end

    // Update sound meter
    sound.update();

    // Update patterns to the beat
    update_beat();

    // Publish channel requests and declarations from the freshly updated beat frame.
    updateV3Protocol();

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
      broadcast_state();
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

  // AI: below section was generated by an AI
  // Flashes the full pole magenta briefly while preserving its normal render between pulses.
  void handleIdentifyOverlayDraw() const {
    if (!identifyActive || millis() % 1000 >= 250)
      return;

    uint16_t length = strip.getLengthTotal();
    for (uint16_t position = 0; position < length; position++)
      strip.setPixelColor(position, CRGB::Magenta);
  }
  // AI: end

  void restart_phrase() {
    beats.start_phrase();
    update_beat();
    publishApplicationChannel(BeatChannel);
  }

  void set_phrase_position(uint8_t pos) {
    beats.sync(beats.bpm, (beats.frac & -0xFFF) + (pos<<8));
    update_beat();
    publishApplicationChannel(BeatChannel);
  }

  void set_tapped_bpm(accum88 bpm, uint8_t pos=15) {
    // By default, restarts at 15th beat - because this is the end of a tap
    apply_bpm(bpm, pos);
    publishApplicationChannel(BeatChannel);
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
    publishApplicationChannel(BeatChannel);
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
    if (node.serialTraceEnabled()) {
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
    }

    broadcast_state();
  }

  void background_changed() {
    update_background();
    if (node.serialTraceEnabled()) {
      current_state.print();
      Serial.println();
    }
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

  // AI: below section was generated by an AI
  // Starts Tubes at low load and raises brightness gradually after ABL is active.
  void startBrightnessRamp() {
    if (role > CampRole) {
      load_options(options, true);
      return;
    }

    if (!turnOnAtBoot && bri == 0) {
      load_options(options, true);
      return;
    }

    startupBrightness = min<uint8_t>(options.brightness, 5);
    ControllerOptions startupOptions = options;
    startupOptions.brightness = startupBrightness;
    load_options(startupOptions, true);
    startupBrightnessRamping = startupBrightness != options.brightness;
    if (startupBrightnessRamping)
      startupBrightnessTimer.start(100);
  }

  // Advances the startup ramp without blocking rendering or network processing.
  void updateStartupBrightness() {
    if (!startupBrightnessRamping || !startupBrightnessTimer.ended())
      return;

    uint8_t target = options.brightness;
    if (startupBrightness < target)
      startupBrightness = min<uint16_t>(uint16_t(startupBrightness) + 10, target);
    else if (startupBrightness > target)
      startupBrightness = max<int16_t>(int16_t(startupBrightness) - 10, target);

    ControllerOptions rampOptions = options;
    rampOptions.brightness = startupBrightness;
    load_options(rampOptions);
    startupBrightnessRamping = startupBrightness != target;
    if (startupBrightnessRamping)
      startupBrightnessTimer.start(100);
  }
  // AI: end

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

    if (node.serialTraceEnabled())
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
    // WLED 16 virtual layers snapshot their palette, so replace the layer when
    // a palette-only transition becomes effective.
    update_background();
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

    if (node.serialTraceEnabled()) {
      Serial.print(F("Change effect "));
      current_state.print();
      Serial.println();
    }

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
    const CRGBPalette16* runtimePalette = v3RuntimePaletteActive ? &v3RuntimePalette : nullptr;
    vstrips[next_vstrip]->load(background, DEFAULT_FADE_SPEED, runtimePalette);
    next_vstrip = (next_vstrip + 1) % NUM_VSTRIPS;

    const PatternRenderOptions& renderOptions = gPatterns[current_state.pattern_id].renderOptions;
    uint8_t param = modeParameter(background.wled_fx_id);
    set_wled_pattern(background.wled_fx_id, param, param, renderOptions);
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
    else if (v3RuntimePaletteActive && v3RuntimePaletteId)
      palette_id = v3RuntimePaletteId;

    Segment& seg = strip.getMainSegment();
    seg.setPalette(palette_id);

    stateChanged = true;
    stateUpdated(CALL_MODE_DIRECT_CHANGE);
  }

  // Select a WLED effect and apply only the shared checkbox controls owned by its Tubes mapping.
  void set_wled_pattern(uint8_t pattern_id, uint8_t speed, uint8_t intensity,
                        const PatternRenderOptions& renderOptions = PatternRenderOptions()) {
    if (!shouldRenderTubes())
      return;

    bool applyRenderOptions = !patternOverride;
    if (patternOverride)
      pattern_id = patternOverride;
    else if (pattern_id == 0)
      pattern_id = DEFAULT_WLED_FX; // Never set it to solid

    Segment& seg = strip.getMainSegment();
    seg.speed = speed;
    seg.intensity = intensity;
    seg.setMode(pattern_id);

    // AI: below section was generated by an AI
    uint8_t nextRenderMask = applyRenderOptions ? renderOptions.checkMask : 0;
    uint8_t changedRenderMask = activePatternRenderMask | nextRenderMask;
    if (changedRenderMask & PatternRenderCheck1)
      seg.check1 = (nextRenderMask & renderOptions.checkValues & PatternRenderCheck1) != 0;
    if (changedRenderMask & PatternRenderCheck2)
      seg.check2 = (nextRenderMask & renderOptions.checkValues & PatternRenderCheck2) != 0;
    if (changedRenderMask & PatternRenderCheck3)
      seg.check3 = (nextRenderMask & renderOptions.checkValues & PatternRenderCheck3) != 0;
    activePatternRenderMask = nextRenderMask;
    // AI: end

    stateChanged = true;
    stateUpdated(CALL_MODE_DIRECT_CHANGE);
  }

  void setBrightness(uint8_t brightness, bool share = true) {
    if (node.serialTraceEnabled())
      Serial.printf("brightness: %d\n", brightness);

    options.brightness = brightness;
    load_options(options);

    if (share)
      broadcast_options();
  }

  void setDebugging(bool debugging, bool share = true) {
    if (node.serialTraceEnabled())
      Serial.printf("debugging: %d\n", debugging);

    options.debugging = debugging;
    load_options(options);

    if (share)
      publishV3Debug(true);
  }

  void togglePowerSave() {
    setPowerSave(!power_save);
  }

  void setPowerSave(bool ps) {
    power_save = ps;
    if (node.serialTraceEnabled())
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
    if (node.serialTraceEnabled())
      Serial.printf("wrote: %d\n", b);
    EEPROM.end();
  }

  void setRole(ControllerRole r) {
    role = r;
    if (node.serialTraceEnabled())
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
    bool hasWledLayer = false;
    bool hasCustomLayer = false;

    VirtualStrip *first_strip = NULL;
    for (uint8_t i=0; i < NUM_VSTRIPS; i++) {
      VirtualStrip *vstrip = vstrips[i];
      if (vstrip->fade == Dead)
        continue;

      vstrip->update(beat_frame, beat_pulse);
      if (vstrip->fade == Dead)
        continue;

      // Remember the first strip
      if (first_strip == NULL)
        first_strip = vstrip;

      // AI: below section was generated by an AI
      // WLED owns one live pixel buffer, so concurrent WLED virtual layers are
      // coverage weights for that same buffer. Combine their weights instead
      // of letting the last layer fade the live image against black.
      if (vstrip->isWled()) {
        hasWledLayer = true;
        uint32_t combinedWledFader = uint32_t(wled_fader) + vstrip->fader;
        wled_fader = min(combinedWledFader, uint32_t(UINT16_MAX));
      } else {
        hasCustomLayer = true;
      }
      // AI: end
    }

    // AI: below section was generated by an AI
    // Multiple WLED layers all describe the one live WLED buffer. When no
    // custom layer competes with it, WLED's own transition must stay undimmed.
    if (hasWledLayer && !hasCustomLayer)
      wled_fader = UINT16_MAX;
    // AI: end

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
    Serial.println(F("b###.# - set bpm\ns - start phrase\n\np### - patterns\nm### - sync mode\nc### - colors\ne### - effects\nn - force next\n\ni### - set Control ID\nB/K/C### - set Beat/Pattern/Palette Channel ID\nd - toggle debugging\nl### - brightness"));
    Serial.println(F("@ - set power saving mode\nU - begin auto-update\nP - toggle all power saves\nO - toggle all sound overlays\n==== wifi ====\na - turn on access point\nq - turn off access point\nt0/1 - Tubes mode off/on"));
    Serial.println(F("==== global actions ====\n* - enter select mode (double-click to Ready)\nA - turn on access point (Ready to update)\nW - forget WiFi client\nX - restart\nV### - auto-upgrade to version\nz - report all visible devices\nz############ - probe a device by MAC\nyhhhh - select one hexadecimal Device ID for update\n(hhhh/)hhhh - select/unselect a Device ID\nF### - flash selected devices\nM - cancel manual pattern override"));
  }

  bool executeOperation(const TubeOperation& operation) {
    uint16_t argument = operation.argument;
    TubeScope scope = tubeOperationScope(operation);
    bool share = scope != LocalScope;

    switch (tubeOperationCode(operation)) {
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
        v3RuntimePaletteActive = false;
        v3PalettePending = false;
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
        if (!protocolLocalValueFromId(argument))
          break;
        Serial.printf("Reset! ID -> %04X\n",
            makeDeviceId(CURRENT_PROTOCOL_GENERATION, protocolLocalValueFromId(argument)));
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
      case BeatChannelIdOperation:
      case PatternChannelIdOperation:
      case PaletteChannelIdOperation: {
        ProtocolLocalValue localValue = protocolLocalValueFromId(argument);
        if (scope != LocalScope || !localValue)
          break;
        uint8_t channel = BeatChannel + tubeOperationCode(operation) - BeatChannelIdOperation;
        DeviceId channelId = makeDeviceId(CURRENT_PROTOCOL_GENERATION, localValue);
        channelIds[channelIndex(channel)] = channelId;
        channelWinners.clear(channel);
        publishApplicationChannel(channel);
        Serial.printf("Channel %02X ID -> %04X\n", channel, channelId);
        return true;
      }
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

    readJsonV3Palette(json);
    readJsonV3PositionFrame(json);
    readJsonV3Position(json);
    readJsonV3Debug(json);

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

  // AI: below section was generated by an AI
  void readJsonV3Palette(JsonObject& json) {
    JsonArray colors = json[F("palette16")];
    if (colors.isNull())
      return;
    if (colors.size() != 16)
      return;

    V3Palette16Definition definition;
    uint8_t index = 0;
    for (JsonVariant colorValue : colors) {
      if (!colorValue.is<uint32_t>())
        return;
      uint32_t color = colorValue.as<uint32_t>();
      if (color > 0xFFFFFF)
        return;
      definition.rgb[index][0] = color >> 16;
      definition.rgb[index][1] = color >> 8;
      definition.rgb[index][2] = color;
      index++;
    }

    uint32_t legacyPalette = json[F("legacyPalette")] | current_state.palette_id;
    uint32_t effectivePhrase = json[F("palettePhrase")] | uint32_t(currentPhrase() + 1);
    uint32_t transitionMs = json[F("paletteTransition")] | uint32_t(transitionDelay);
    if (legacyPalette >= gGradientPaletteCount
        || effectivePhrase > UINT16_MAX
        || int16_t(uint16_t(effectivePhrase) - currentPhrase()) < 1
        || transitionMs > 60000)
      return;

    publishV3LiteralPalette(
        definition,
        legacyPalette,
        effectivePhrase,
        transitionMs
    );
  }

  void readJsonV3PositionFrame(JsonObject& json) {
    JsonObject frameJson = json[F("positionFrame")];
    if (frameJson.isNull())
      return;

    uint32_t frameNamespace = frameJson[F("namespace")] | 0U;
    uint32_t epoch = frameJson[F("epoch")] | 0U;
    uint32_t originId = frameJson[F("origin")] | uint32_t(node.header.id);
    uint32_t axisId = frameJson[F("axis")] | 0U;
    uint32_t positiveYId = frameJson[F("positiveY")] | 0U;
    if (!frameNamespace || epoch > UINT16_MAX || originId > UINT16_MAX
        || axisId > UINT16_MAX || positiveYId > UINT16_MAX)
      return;

    V3PositionFramePayload frame;
    frame.frameNamespace = frameNamespace;
    frame.frameEpoch = uint16_t(epoch);
    frame.originId = DeviceId(originId);
    frame.axisId = DeviceId(axisId);
    frame.positiveYId = DeviceId(positiveYId);
    if (sendV3Body(V3TopicPosition, V3State, &frame, sizeof(frame),
                   V3_DEFAULT_TOPIC_LEASE_DECISECONDS, 0, 0, true)) {
      v3.positionFrame = frame;
      v3.hasPositionFrame = true;
      v3ConfiguredPositionFrame = frame;
      hasV3ConfiguredPositionFrame = true;
    }
  }

  void readJsonV3Position(JsonObject& json) {
    JsonObject position = json[F("position")];
    if (position.isNull())
      return;

    long xQ8_8 = position[F("xQ8_8")] | LONG_MIN;
    long yQ8_8 = position[F("yQ8_8")] | LONG_MIN;
    uint32_t confidence = position[F("confidence")] | 0U;
    long txCalibration = position[F("txCalibrationDbm")] | 0L;
    bool anchor = position[F("anchor")] | false;
    bool settled = position[F("settled")] | anchor;
    if (xQ8_8 < INT16_MIN || xQ8_8 > INT16_MAX
        || yQ8_8 < INT16_MIN || yQ8_8 > INT16_MAX
        || confidence > UINT8_MAX
        || txCalibration < INT8_MIN || txCalibration > INT8_MAX)
      return;
    if (!v3.hasPositionFrame)
      return;

    v3LocalPosition = V3PositionAdvertisementPayload();
    v3LocalPosition.bootSession = v3.session();
    v3LocalPosition.frameNamespace = v3.positionFrame.frameNamespace;
    v3LocalPosition.frameEpoch = v3.positionFrame.frameEpoch;
    v3LocalPosition.xQ8_8 = int16_t(xQ8_8);
    v3LocalPosition.yQ8_8 = int16_t(yQ8_8);
    v3LocalPosition.leaderViaId = node.header.uplinkId;
    v3LocalPosition.txCalibrationDbm = int8_t(txCalibration);
    v3LocalPosition.confidence = uint8_t(confidence);
    v3LocalPosition.flags = V3PositionValid;
    if (anchor) v3LocalPosition.flags |= V3PositionAnchor;
    if (settled) v3LocalPosition.flags |= V3PositionSettled;
    hasV3LocalPosition = true;
    sendV3Body(
        V3TopicPosition,
        V3Advertisement,
        &v3LocalPosition,
        sizeof(v3LocalPosition),
        0,
        confidence
    );
  }

  void readJsonV3Debug(JsonObject& json) {
    bool changed = false;
    if (json.containsKey(F("debugMask"))) {
      uint32_t mask = json[F("debugMask")].as<uint32_t>();
      if (mask > UINT16_MAX)
        return;
      v3DebugOverlayMask = mask;
      changed = true;
    }
    if (json.containsKey(F("debugVerbosity"))) {
      uint32_t verbosity = json[F("debugVerbosity")].as<uint32_t>();
      if (verbosity > UINT8_MAX)
        return;
      v3DebugVerbosity = verbosity;
      changed = true;
    }
    if (changed)
      publishV3Debug(true);
  }
  // AI: end

  void read_keys() {
    if (!Serial.available())
      return;

    node.renewSerialTrace();
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
    if (key == DEVICE_UPDATE_SERIAL_KEY) {
      requestDeviceUpdate(command + 1);
      return;
    }
    if ((key == '(' || key == ')') && strnlen(command + 1, 5) == 4) {
      requestDeviceIdentify(command + 1, key == '(');
      return;
    }

    accum88 parsed = parse_number(command + 1);
    uint16_t argument = parsed >> 8;
    Serial.printf("[command=%c arg=%04x]\n", key, parsed);

    if (key == 'b')
      argument = parsed;
    else if (key == 'i' || key == 'B' || key == 'K' || key == 'C')
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

  bool sendV3ControlCommand(CommandId command, const void* data, uint8_t length) {
    if (length > TUBES_CHANNEL_BODY_SIZE - 2)
      return false;
    ControlChannelBody control;
    control.command = command;
    control.commandLength = length;
    if (length > 0)
      memcpy(control.commandData, data, length);
    TubesChannelMessageKind kind = node.isFollowing()
        ? ChannelRequest
        : ChannelDeclaration;
    TubesChannelPayload payload = makeChannelPayload(
        ControlChannel,
        kind,
        &control,
        length + 2
    );
    bool sent = node.sendV3Channel(ControlChannel, payload);
    if (!node.isFollowing())
      sendLegacyCommand(command, data, length);
    return sent;
  }

  void sendLegacyCommand(CommandId command, const void* data, uint8_t length) {
    V3ProjectionTrailer trailer;
    trailer.controlId = node.header.id;
    trailer.controlSession = uint16_t(channelSession);
    node.sendCommand(command, data, length, &trailer);
  }

  void broadcast_action(Action& action) {
    if (!node.isFollowing()) {
      onAction(&action);
    }
    sendV3ControlCommand(COMMAND_ACTION, &action, sizeof(Action));
  }

  void broadcast_info(NodeInfo *info) {
    sendV3ControlCommand(COMMAND_INFO, info, sizeof(NodeInfo));
  }

  void broadcast_state() {
    // Publishing this device's state would reveal the new BPM before its phrase boundary.
    if (deferredBpmBroadcast.active())
      return;
    publishApplicationChannel(BeatChannel);
    publishApplicationChannel(PatternChannel);
    publishApplicationChannel(PaletteChannel);
    publishV2Projection();
  }

  void broadcast_options() {
    sendV3ControlCommand(COMMAND_OPTIONS, &options, sizeof(options));
  }

  void broadcast_autoupdate() {
    // The deployed update offer is larger than the gen1 Control body, so only the
    // Control master emits this legacy-only command. It does not carry visual state.
    if (!node.isFollowing())
      node.sendCommand(COMMAND_UPGRADE, &updater.current_version, sizeof(updater.current_version));
  }

  void broadcast_bpm(accum88 bpm) {
    (void)bpm;
    publishApplicationChannel(BeatChannel);
  }

  void requestDeviceReport(const char* macText) {
    DeviceReportMessage request;
    bool wildcard = !macText || !*macText;
    if (!wildcard && !parseDeviceReportMac(macText, request.mac)) {
      Serial.println(F("TUBE_PROBE_ERROR invalid_mac"));
      return;
    }

    request.nonce = esp_random();
    Serial.printf(
      "TUBE_PROBE nonce=%08lX mac=%s\n",
      (unsigned long)request.nonce,
      wildcard ? "*" : macText
    );
    onDeviceReportMessage(request);
    sendV3ControlCommand(COMMAND_ACTION, &request, sizeof(request));
  }

  void requestDeviceUpdate(const char* idText) {
    DeviceReportMessage request;
    request.kind = DeviceUpdateSelect;
    if (!parseDeviceReportId(idText, request.nodeId)) {
      Serial.println(F("TUBE_UPDATE_ERROR invalid_id"));
      return;
    }
    request.nonce = esp_random();
    Serial.printf(
      "TUBE_UPDATE_TARGET nonce=%08lX node=0x%04X\n",
      (unsigned long)request.nonce,
      request.nodeId
    );
    onDeviceReportMessage(request);
    sendV3ControlCommand(COMMAND_ACTION, &request, sizeof(request));
  }

  // AI: below section was generated by an AI
  // Sets a temporary visual marker on one full Device ID without entering update mode.
  void requestDeviceIdentify(const char* idText, bool enabled) {
    DeviceReportMessage request;
    if (!parseDeviceReportId(idText, request.nodeId)) {
      Serial.println(F("TUBE_IDENTIFY_ERROR invalid_id"));
      return;
    }
    request.kind = enabled ? DeviceIdentifyOn : DeviceIdentifyOff;
    request.nonce = esp_random();
    Serial.printf(
      "TUBE_IDENTIFY_TARGET nonce=%08lX node=0x%04X state=%s\n",
      (unsigned long)request.nonce,
      request.nodeId,
      request.kind == DeviceIdentifyOn ? "on" : "off"
    );
    onDeviceReportMessage(request);
    sendV3ControlCommand(COMMAND_ACTION, &request, sizeof(request));
  }
  // AI: end

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
    printDeviceReport(report);
    sendV3ControlCommand(COMMAND_ACTION, &report, sizeof(report));
  }

  void printDeviceReport(const DeviceReportMessage& report) const {
    Serial.printf(
      "TUBE_REPORT nonce=%08lX mac=%02x%02x%02x%02x%02x%02x family=%u variant=%u tubes=%u release=%08lX leds=%u buses=%u pin=%u type=%u role=%u mesh=%u node=0x%04X uplink=0x%04X uptime=%lu\n",
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

    if (message.kind == DeviceUpdateSelect) {
      if (deviceReportTargetsNode(message, node.header.id) && !isHomeLightRole()) {
        uint8_t deviceMac[6];
        Network.localMAC(deviceMac);
        broadcastDeviceReport(message.nonce, deviceMac);
        targetedUpdatePending = true;
        targetedUpdateTimer.start(250);
      }
      return;
    }

    // AI: below section was generated by an AI
    if (message.kind == DeviceIdentifyOn || message.kind == DeviceIdentifyOff) {
      if (deviceReportTargetsNode(message, node.header.id) && !isHomeLightRole()) {
        if (message.kind == DeviceIdentifyOn) {
          identifyActive = true;
          identifyTimer.start(300000);
          Serial.println(F("TUBE_IDENTIFY on duration=300s"));
        } else {
          identifyActive = false;
          identifyTimer.stop();
          Serial.println(F("TUBE_IDENTIFY off"));
        }
      }
      return;
    }
    // AI: end

    if (message.nonce == lastDeviceReportProbeNonce)
      return;
    lastDeviceReportProbeNonce = message.nonce;

    uint8_t deviceMac[6];
    Network.localMAC(deviceMac);
    if (!deviceReportTargetsMac(message, deviceMac))
      return;

    pendingDeviceReport = message;
    deviceReportPending = true;
    uint16_t delayMs = deviceReportMacIsWildcard(message.mac)
        ? 50 + ((uint32_t(protocolLocalValueFromId(node.header.id)) * 37U) % 1200U)
        : 20;
    deviceReportTimer.start(delayMs);
  }

  // AI: below section was generated by an AI
  bool isValidRemoteOperation(const TubeOperation& operation) const {
    if (tubeOperationScope(operation) == LocalScope)
      return false;
    switch (tubeOperationCode(operation)) {
      case DebugOperation:
      case PowerSaveOperation:
      case SelectOperation:
      case SoundOverlayOperation:
        return operation.argument <= 1;
      case BrightnessOperation:
        return operation.argument >= 5 && operation.argument <= UINT8_MAX;
      case BpmOperation:
        return operation.argument >= (60U << 8) && operation.argument <= (300U << 8);
      case PatternOperation:
        return operation.argument < gPatternCount;
      case SyncModeOperation:
        return operation.argument <= SwingDrift;
      case PaletteOperation:
        return operation.argument < gGradientPaletteCount;
      case EffectOperation:
        return operation.argument < gEffectCount;
      case EffectChanceOperation:
      case FlashOperation:
        return operation.argument <= UINT8_MAX;
      case RoleOperation:
        return operation.argument == DefaultRole
            || operation.argument == CampRole
            || operation.argument == InstallationRole
            || operation.argument == SmallArtRole
            || operation.argument == HomeLightRole
            || operation.argument == LegacyRole
            || operation.argument == MasterRole;
      case RebootOperation:
      case AccessPointOperation:
      case DisconnectWifiOperation:
      case ForgetWifiOperation:
      case StartPhraseOperation:
      case NextOperation:
      case UpdateOperation:
      case UpdateOfferOperation:
      case GlitterOperation:
      case CancelOverrideOperation:
        return true;
      case NodeIdOperation:
      case TubesModeOperation:
      case HelpOperation:
        return false;
    }
    return false;
  }

  bool isValidActionPayload(const uint8_t* data, uint8_t logicalLength, bool exactLength) const {
    if (!data || logicalLength < sizeof(Action))
      return false;
    if (data[0] == DEVICE_REPORT_ACTION_KEY) {
      if (logicalLength < sizeof(DeviceReportMessage)
          || (exactLength && logicalLength != sizeof(DeviceReportMessage)))
        return false;
      DeviceReportMessage report;
      memcpy(&report, data, sizeof(report));
      return isDeviceReportMessage(report);
    }
    if (exactLength && logicalLength != sizeof(Action))
      return false;
    Action action;
    memcpy(&action, data, sizeof(action));
    TubeOperation operation;
    return decodeOperation(action.key, action.arg, operation)
        && isValidRemoteOperation(operation);
  }

  bool isValidLegacyState(const TubeState& state) const {
    if (state.bpm < (40U << 8) || state.bpm > (300U << 8))
      return false;
    if (state.pattern_id >= gPatternCount || state.pattern_sync_id > SwingDrift)
      return false;
    if (state.palette_id >= gGradientPaletteCount)
      return false;
    if (state.effect_params.effect > Flash || state.effect_params.pen > Flicker)
      return false;
    return isValidBeatPulse(state.effect_params.beat);
  }

  bool isValidBeatPulse(uint8_t beatPulse) const {
    return beatPulse == Continuous
        || beatPulse == Eighth
        || beatPulse == Quarter
        || beatPulse == Half
        || beatPulse == Beat
        || beatPulse == TwoBeats
        || beatPulse == Measure
        || beatPulse == TwoMeasures
        || beatPulse == Phrase;
  }

  bool applyLegacyStates(const TubeStates& update) {
    if (!isValidLegacyState(update.current) || !isValidLegacyState(update.next))
      return false;

    TubeState state = update.current;
    next_state = update.next;
    if (node.serialTraceEnabled()) {
      state.print();
      next_state.print();
    }

    // Apply the complete v2 snapshot only after every field has passed ingress validation.
    v3RuntimePaletteActive = false;
    load_pattern(state);
    load_palette(state);
    load_effect(state);
    if (!deferredBpmBroadcast.active())
      beats.sync(state.bpm, state.beat_frame);
    return true;
  }

  virtual bool isValidV2Command(CommandId command, const uint8_t* data) const override {
    switch (command) {
      case COMMAND_STATE: {
        TubeStates update;
        memcpy(&update, data, sizeof(update));
        return isValidLegacyState(update.current) && isValidLegacyState(update.next);
      }
      case COMMAND_BEATS: {
        accum88 bpm;
        memcpy(&bpm, data, sizeof(bpm));
        return bpm >= (40U << 8) && bpm <= (300U << 8);
      }
      case COMMAND_INFO:
      case COMMAND_UPGRADE:
        return true;
      case COMMAND_OPTIONS:
        return data[0] <= 1;
      case COMMAND_ACTION:
        return isValidActionPayload(data, MESSAGE_DATA_SIZE, false);
      default:
        return false;
    }
  }

  virtual bool onCommand(CommandId command, void *data) override {
    return applyCommand(command, data);
  }

  virtual bool onV2Message(const NodeMessage& message) override {
    V3ProjectionTrailer trailer;
    if (readV3ProjectionTrailer(message, trailer))
      return true;

    // A gen1 Control owner never accepts gen0 visual or Control authority. The
    // legacy-only upgrade offer remains usable because it carries no rendered state.
    if (message.command == COMMAND_UPGRADE)
      return applyCommand(message.command, const_cast<uint8_t*>(message.data));
    return false;
  }

  bool applyCommand(CommandId command, void *data) {
    switch (command) {
      case COMMAND_INFO:
        Serial.printf("   \"%.*s\"\n",
          int(sizeof(((NodeInfo*)data)->message)),
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
        TubeStates update;
        memcpy(&update, data, sizeof(update));
        return applyLegacyStates(update);
      }

      case COMMAND_UPGRADE:
        // HOMELIGHT must relay upgrade offers without installing Tubes firmware.
        if (!isHomeLightRole())
          updater.start((AutoUpdateOffer*)data);
        return true;

      case COMMAND_ACTION:
        if (((Action*)data)->key == DEVICE_REPORT_ACTION_KEY)
          onDeviceReportMessage(*(DeviceReportMessage*)data);
        else
          onAction((Action*)data);
        return true;

      case COMMAND_BEATS:
        if (*(accum88*)data < (40U << 8) || *(accum88*)data > (300U << 8))
          return false;
        // A declared BPM supersedes any local change that has not reached its boundary.
        deferredBpmBroadcast.cancel();
        apply_bpm(*(accum88*)data);
        return true;
    }

    Serial.printf("UNKNOWN COMMAND %02X", command);
    return false;
  }

  virtual void onV2PacketObserved(const NodeMessage& message) override {
    (void)message;
    v3.observeLegacyPacket();
  }

  // AI: below section was generated by an AI
  bool isValidPatternChannelState(const PatternChannelState& state) const {
    auto valid = [this](const PatternChannelEntry& entry) {
      return entry.patternId < gPatternCount
          && entry.syncMode <= SwingDrift
          && entry.effect <= Flash
          && entry.pen <= Flicker
          && isValidBeatPulse(entry.beat);
    };
    return valid(state.current) && valid(state.next);
  }

  virtual bool isValidV3Channel(
      uint8_t channel,
      const TubesChannelPayload& payload,
      const MeshNodeHeader& transportSender,
      MessageRecipients recipients
  ) const override {
    (void)transportSender;
    if (!isValidTubesChannelPayload(channel, payload))
      return false;
    if (payload.envelope.messageKind == ControlBeacon)
      return channel == ControlChannel && recipients == RECIPIENTS_NEIGHBORS;
    if (payload.envelope.messageKind == ChannelRequest && recipients != RECIPIENTS_ROOT)
      return false;
    if (payload.envelope.messageKind == ChannelDeclaration && recipients != RECIPIENTS_ALL)
      return false;

    if (channel == BeatChannel) {
      BeatChannelState state;
      return readChannelBody(payload, state)
          && state.bpm >= (40U << 8)
          && state.bpm <= (300U << 8);
    }
    if (channel == PatternChannel) {
      PatternChannelState state;
      return readChannelBody(payload, state) && isValidPatternChannelState(state);
    }
    if (channel == PaletteChannel) {
      PaletteChannelState state;
      return readChannelBody(payload, state)
          && state.current.paletteId < gGradientPaletteCount
          && state.next.paletteId < gGradientPaletteCount;
    }
    if (channel == ControlChannel) {
      ControlChannelBody control;
      memset(&control, 0, sizeof(control));
      memcpy(&control, payload.body, payload.envelope.bodyLength);
      if (control.command == COMMAND_STATE || control.command == COMMAND_BEATS)
        return false;
      return isValidV2Command(control.command, control.commandData);
    }
    return false;
  }

  bool applyApplicationChannel(uint8_t channel, const TubesChannelPayload& payload) {
    if (channel == BeatChannel) {
      BeatChannelState state;
      if (!readChannelBody(payload, state)) return false;
      deferredBpmBroadcast.cancel();
      beats.sync(state.bpm, state.beatFrame);
      return true;
    }
    if (channel == PatternChannel) {
      PatternChannelState state;
      if (!readChannelBody(payload, state)) return false;
      TubeState current = current_state;
      current.pattern_phrase = state.current.patternPhrase;
      current.pattern_id = state.current.patternId;
      current.pattern_sync_id = state.current.syncMode;
      current.effect_phrase = state.current.effectPhrase;
      current.effect_params = EffectParameters(
          EffectMode(state.current.effect),
          PenMode(state.current.pen),
          BeatPulse(state.current.beat),
          state.current.chance
      );
      next_state.pattern_phrase = state.next.patternPhrase;
      next_state.pattern_id = state.next.patternId;
      next_state.pattern_sync_id = state.next.syncMode;
      next_state.effect_phrase = state.next.effectPhrase;
      next_state.effect_params = EffectParameters(
          EffectMode(state.next.effect),
          PenMode(state.next.pen),
          BeatPulse(state.next.beat),
          state.next.chance
      );
      load_pattern(current);
      load_effect(current);
      return true;
    }
    if (channel == PaletteChannel) {
      PaletteChannelState state;
      if (!readChannelBody(payload, state)) return false;
      TubeState current = current_state;
      current.palette_phrase = state.current.palettePhrase;
      current.palette_id = state.current.paletteId;
      next_state.palette_phrase = state.next.palettePhrase;
      next_state.palette_id = state.next.paletteId;
      v3RuntimePaletteActive = false;
      load_palette(current);
      return true;
    }
    return false;
  }

  virtual bool onV3Channel(
      uint8_t channel,
      const TubesChannelPayload& payload,
      const MeshNodeHeader& transportSender,
      MessageRecipients recipients
  ) override {
    (void)transportSender;
    (void)recipients;
    if (payload.envelope.messageKind == ControlBeacon)
      return true;

    if (channel == ControlChannel) {
      ControlChannelBody control;
      memset(&control, 0, sizeof(control));
      memcpy(&control, payload.body, payload.envelope.bodyLength);
      bool applied = applyCommand(control.command, control.commandData);
      if (applied && !node.isFollowing()
          && payload.envelope.messageKind == ChannelRequest)
        sendLegacyCommand(control.command, control.commandData, control.commandLength);
      return applied;
    }

    bool accepted = payload.envelope.messageKind == ChannelRequest
        ? channelWinners.acceptRequest(channel, payload.envelope, millis())
        : channelWinners.acceptDeclaration(channel, payload.envelope, millis());
    if (!accepted)
      return false;
    channelSnapshots[channelIndex(channel)] = payload;
    bool applied = applyApplicationChannel(channel, payload);
    if (applied && !node.isFollowing())
      publishV2Projection();
    return applied;
  }
  // AI: end

  // Prove topic ranges at ingress; only applying nodes require referenced cache entries,
  // because an upward relay intentionally does not apply or cache Root requests.
  bool isValidV3TopicSemantics(
      uint8_t topic,
      const V3TopicPayload& payload,
      const MeshNodeHeader& transportSender,
      MessageRecipients recipients,
      uint32_t nowMs,
      bool requireCachedReferences
  ) const {
    if (topic == V3TopicPresence && payload.envelope.messageKind != V3State)
      return false;
    if (payload.envelope.messageKind == V3Claim
        || payload.envelope.messageKind == V3Release
        || payload.envelope.messageKind == V3Heartbeat)
      return true;

    switch (topic) {
      case V3TopicPresence: {
        V3PresencePayload presence;
        return readV3Body(payload, presence)
            && recipients == RECIPIENTS_NEIGHBORS
            && presence.minimumOuterVersion <= presence.maximumOuterVersion
            && presence.maximumOuterVersion >= TUBES_PROTOCOL_V3
            && presence.protocolMode == 0
            && presence.maximumBodyLength <= TUBES_V3_BODY_SIZE
            && presence.bootSession != 0;
      }
      case V3TopicBeat: {
        V3BeatPayload beat;
        if (!readV3Body(payload, beat) || beat.bpm < (40U << 8) || beat.bpm > (300U << 8))
          return false;
        int32_t elapsedMs = int32_t(strip.timebase + nowMs - beat.measuredAtTimebase);
        return elapsedMs >= -1000 && elapsedMs <= 60000;
      }
      case V3TopicPattern: {
        V3PatternPayload pattern;
        return readV3Body(payload, pattern)
            && pattern.current.patternId < gPatternCount
            && pattern.next.patternId < gPatternCount
            && pattern.current.syncMode <= SwingDrift
            && pattern.next.syncMode <= SwingDrift
            && pattern.current.legacyPatternId < gPatternCount
            && pattern.next.legacyPatternId < gPatternCount
            && pattern.current.legacySyncMode <= SwingDrift
            && pattern.next.legacySyncMode <= SwingDrift
            && int16_t(pattern.current.effectivePhrase - currentPhrase()) <= 0
            && int16_t(pattern.next.effectivePhrase - currentPhrase()) >= -1
            && int16_t(pattern.next.effectivePhrase - currentPhrase()) <= 1024;
      }
      case V3TopicPalette:
        if (payload.envelope.messageKind == V3Definition)
          if (payload.envelope.bodyLength == sizeof(V3Palette16Definition))
            return true;
          else if (payload.envelope.bodyLength == sizeof(V3GradientPaletteDefinition)) {
            V3GradientPaletteDefinition gradient;
            V3Palette16Definition expanded;
            return readV3Body(payload, gradient) && expandV3Gradient(gradient, expanded);
          } else
            return false;
        if (payload.envelope.messageKind == V3Schedule) {
          V3PaletteSchedulePayload schedule;
          if (!readV3Body(payload, schedule)
              || schedule.legacyPaletteId >= gGradientPaletteCount
              || (schedule.paletteFormat != V3PaletteLegacyId
                  && schedule.paletteFormat != V3Palette16
                  && schedule.paletteFormat != V3PaletteGradientStops)
              || schedule.transitionMs > 60000
              || int16_t(schedule.effectivePhrase - currentPhrase()) < -1
              || int16_t(schedule.effectivePhrase - currentPhrase()) > 1024)
            return false;
          return schedule.paletteFormat == V3PaletteLegacyId
              || !requireCachedReferences
              || v3.palettes.find(
                  payload.envelope.authorityId,
                  payload.envelope.authoritySession,
                  schedule.definitionSequence) != nullptr;
        }
        return false;
      case V3TopicEffect: {
        V3EffectPayload effect;
        return readV3Body(payload, effect)
            && effect.current.effect <= Flash
            && effect.next.effect <= Flash
            && effect.current.pen <= Flicker
            && effect.next.pen <= Flicker
            && isValidBeatPulse(effect.current.beat)
            && isValidBeatPulse(effect.next.beat)
            && int16_t(effect.current.effectivePhrase - currentPhrase()) <= 0
            && int16_t(effect.next.effectivePhrase - currentPhrase()) >= -1
            && int16_t(effect.next.effectivePhrase - currentPhrase()) <= 1024;
      }
      case V3TopicDebug: {
        V3DebugPayload debug;
        return readV3Body(payload, debug) && debug.enabled <= 1;
      }
      case V3TopicPosition:
        if (payload.envelope.messageKind == V3State) {
          V3PositionFramePayload frame;
          return readV3Body(payload, frame)
              && frame.frameNamespace != 0
              && frame.originId != 0
              && frame.flags == 0
              && frame.reserved[0] == 0
              && frame.reserved[1] == 0
              && frame.reserved[2] == 0;
        }
        if (payload.envelope.messageKind == V3Advertisement) {
          V3PositionAdvertisementPayload advertisement;
          return recipients == RECIPIENTS_NEIGHBORS
              && payload.envelope.authorityId == transportSender.id
              && readV3Body(payload, advertisement)
              && advertisement.bootSession == payload.envelope.authoritySession
              && advertisement.frameNamespace != 0
              && v3.hasPositionFrame
              && advertisement.frameNamespace == v3.positionFrame.frameNamespace
              && advertisement.frameEpoch == v3.positionFrame.frameEpoch
              && !(advertisement.flags & ~(V3PositionValid | V3PositionAnchor | V3PositionSettled | V3PositionAmbiguous));
        }
        return false;
      case V3TopicControl: {
        V3ControlPayload control;
        memset(&control, 0, sizeof(control));
        memcpy(&control, payload.body, payload.envelope.bodyLength);
        switch (control.command) {
          case COMMAND_OPTIONS:
            return control.commandLength == sizeof(ControllerOptions)
                && control.commandData[0] <= 1;
          case COMMAND_INFO: return control.commandLength == sizeof(NodeInfo);
          case COMMAND_ACTION:
            return isValidActionPayload(control.commandData, control.commandLength, true);
          default: return false;
        }
      }
    }
    return false;
  }

  virtual bool isValidV3Topic(
      uint8_t topic,
      const V3TopicPayload& payload,
      const MeshNodeHeader& transportSender,
      MessageRecipients recipients,
      int8_t rssi
  ) const override {
    (void)rssi;
    return isValidV3TopicSemantics(topic, payload, transportSender, recipients, millis(), false);
  }

  virtual bool onV3Topic(
      uint8_t topic,
      const V3TopicPayload& payload,
      const MeshNodeHeader& transportSender,
      MessageRecipients recipients,
      int8_t rssi
  ) override {
    uint32_t nowMs = millis();
    if (!isKnownV3Topic(topic)) {
      v3.counters.unknown++;
      return false;
    }
    if (!isValidV3TopicSemantics(topic, payload, transportSender, recipients, nowMs, true))
      return false;

    if (topic == V3TopicPresence) {
      V3PresencePayload presence;
      if (!readV3Body(payload, presence))
        return false;
      return v3.acceptTopic(topic, payload, nowMs);
    }

    if (!v3.acceptTopic(topic, payload, nowMs))
      return false;
    if (payload.envelope.messageKind == V3Claim
        || payload.envelope.messageKind == V3Release
        || payload.envelope.messageKind == V3Heartbeat)
      return true;

    switch (topic) {
      case V3TopicBeat: {
        V3BeatPayload beat;
        if (!readV3Body(payload, beat)
            || beat.bpm < (40U << 8)
            || beat.bpm > (300U << 8))
          return false;
        uint32_t localTimebase = strip.timebase + nowMs;
        int32_t elapsedMs = int32_t(localTimebase - beat.measuredAtTimebase);
        if (elapsedMs < -1000 || elapsedMs > 60000)
          return false;
        uint32_t projectedFrame = beat.beatFrame;
        if (elapsedMs > 0)
          projectedFrame += (uint64_t(uint32_t(elapsedMs)) * beat.bpm) / 60000U;
        deferredBpmBroadcast.cancel();
        beats.sync(beat.bpm, projectedFrame);
        update_beat();
        return true;
      }

      case V3TopicPattern: {
        V3PatternPayload pattern;
        if (!readV3Body(payload, pattern)
            || pattern.current.patternId >= gPatternCount
            || pattern.next.patternId >= gPatternCount
            || pattern.current.syncMode > SwingDrift
            || pattern.next.syncMode > SwingDrift
            || pattern.current.legacyPatternId >= gPatternCount
            || pattern.next.legacyPatternId >= gPatternCount)
          return false;
        TubeState state = current_state;
        state.pattern_phrase = pattern.current.effectivePhrase;
        state.pattern_id = pattern.current.patternId;
        state.pattern_sync_id = pattern.current.syncMode;
        load_pattern(state);
        next_state.pattern_phrase = pattern.next.effectivePhrase;
        next_state.pattern_id = pattern.next.patternId;
        next_state.pattern_sync_id = pattern.next.syncMode;
        return true;
      }

      case V3TopicPalette:
        if (payload.envelope.messageKind == V3Definition) {
          V3Palette16Definition definition;
          if (payload.envelope.bodyLength == sizeof(V3GradientPaletteDefinition)) {
            V3GradientPaletteDefinition gradient;
            if (!readV3Body(payload, gradient) || !expandV3Gradient(gradient, definition))
              return false;
          } else if (!readV3Body(payload, definition)) {
            return false;
          }
          v3.palettes.store(payload.envelope, definition);
          return true;
        }
        if (payload.envelope.messageKind == V3Schedule) {
          V3PaletteSchedulePayload schedule;
          if (!readV3Body(payload, schedule)
              || schedule.legacyPaletteId >= gGradientPaletteCount
              || (schedule.paletteFormat != V3PaletteLegacyId
                  && schedule.paletteFormat != V3Palette16
                  && schedule.paletteFormat != V3PaletteGradientStops))
            return false;
          if (int16_t(schedule.effectivePhrase - currentPhrase()) < -1)
            return false;
          if (schedule.paletteFormat == V3Palette16 || schedule.paletteFormat == V3PaletteGradientStops) {
            const V3Palette16Definition* definition = v3.palettes.find(
                payload.envelope.authorityId,
                payload.envelope.authoritySession,
                schedule.definitionSequence
            );
            if (!definition)
              return false;
            v3PendingPaletteDefinition = *definition;
          }
          v3PendingPaletteSchedule = schedule;
          v3PalettePending = true;
          return true;
        }
        return false;

      case V3TopicEffect: {
        V3EffectPayload effect;
        if (!readV3Body(payload, effect)
            || effect.current.effect > Flash
            || effect.next.effect > Flash
            || effect.current.pen > Flicker
            || effect.next.pen > Flicker)
          return false;
        TubeState state = current_state;
        state.effect_phrase = effect.current.effectivePhrase;
        state.effect_params = EffectParameters(
            EffectMode(effect.current.effect),
            PenMode(effect.current.pen),
            BeatPulse(effect.current.beat),
            effect.current.chance
        );
        if (!isValidLegacyState(state))
          return false;
        load_effect(state);
        next_state.effect_phrase = effect.next.effectivePhrase;
        next_state.effect_params = EffectParameters(
            EffectMode(effect.next.effect),
            PenMode(effect.next.pen),
            BeatPulse(effect.next.beat),
            effect.next.chance
        );
        return isValidLegacyState(next_state);
      }

      case V3TopicDebug: {
        V3DebugPayload debug;
        if (!readV3Body(payload, debug) || debug.enabled > 1)
          return false;
        options.debugging = debug.enabled;
        v3DebugOverlayMask = debug.overlayMask;
        v3DebugVerbosity = debug.verbosity;
        if (debug.expiresAfterSeconds)
          v3DebugExpiryTimer.start(uint32_t(debug.expiresAfterSeconds) * 1000U);
        else
          v3DebugExpiryTimer.stop();
        v3DebugExpiryActive = debug.expiresAfterSeconds != 0;
        return true;
      }

      case V3TopicPosition:
        if (payload.envelope.messageKind == V3State) {
          V3PositionFramePayload frame;
          if (!readV3Body(payload, frame) || frame.reserved[0] || frame.reserved[1] || frame.reserved[2])
            return false;
          v3.positionFrame = frame;
          v3.hasPositionFrame = true;
          return true;
        }
        if (payload.envelope.messageKind == V3Advertisement && recipients == RECIPIENTS_NEIGHBORS) {
          V3PositionAdvertisementPayload advertisement;
          if (!readV3Body(payload, advertisement)
              || advertisement.flags & ~(V3PositionValid | V3PositionAnchor | V3PositionSettled | V3PositionAmbiguous))
            return false;
          v3.positionPeers.observe(transportSender.id, rssi, nowMs, advertisement);
          return true;
        }
        return false;

      case V3TopicControl: {
        V3ControlPayload control;
        memset(&control, 0, sizeof(control));
        memcpy(&control, payload.body, payload.envelope.bodyLength);
        if (control.command == COMMAND_STATE || control.command == COMMAND_UPGRADE)
          return false;
        if (control.command == COMMAND_OPTIONS) {
          ControllerOptions receivedOptions;
          memcpy(&receivedOptions, control.commandData, sizeof(receivedOptions));
          options.brightness = receivedOptions.brightness;
          load_options(options);
          return true;
        }
        return applyCommand(control.command, control.commandData);
      }
    }
    return false;
  }
  // AI: end

  void onAction(Action* action) {
    TubeOperation operation;
    if (!decodeOperation(action->key, action->arg, operation))
      return;
    if (tubeOperationScope(operation) == SelectedScope && !isOperationSelected())
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

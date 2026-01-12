# WLEDTubes Development Plan

## Project Vision

Transform WLEDTubes into a polished, production-ready mesh LED system that supports multiple hardware platforms, offers an intuitive control experience, and provides an extensive library of synchronized patterns.

---

## Project Goals

### Phase 1: Foundation
- [ ] Establish stable WLED upstream sync workflow
- [ ] Create Tubes usermod structure in `usermods/Tubes/`
- [ ] Define build environments for all target hardware
- [ ] Set up CI/CD for automated builds

### Phase 2: Core Features
- [ ] Implement ESP-NOW mesh networking
- [ ] Build BPM synchronization system
- [ ] Create 3-layer effect composition engine
- [ ] Develop leader/follower coordination

### Phase 3: User Experience
- [ ] Design custom Tubes web UI
- [ ] Implement remote control support
- [ ] Expand pattern library
- [ ] Add configuration presets for common setups

### Phase 4: Production
- [ ] Hardware-specific optimizations
- [ ] Power management refinements
- [ ] Documentation and guides
- [ ] Release packaging

---

## Hardware Support

### Target Platforms

| Platform | Status | Notes |
|----------|--------|-------|
| ESP32-S3 | Planned | Primary target - PSRAM, USB-OTG, more GPIO |
| ESP32-C3 | Planned | Budget option - RISC-V, low power |
| QuinLED Dig2Go | Planned | Popular LED controller board |
| Custom PCB | Planned | Purpose-built Tubes hardware |
| Generic ESP32 | Supported | Development/testing |

### ESP32-S3 Configuration
```ini
[env:esp32s3_tubes]
board = esp32-s3-devkitc-1
board_build.mcu = esp32s3
board_build.f_cpu = 240000000L
board_build.flash_mode = qio
board_build.psram_type = opi
build_flags =
    ${env.build_flags}
    -D USERMOD_TUBES
    -D BOARD_HAS_PSRAM
    -D ARDUINO_USB_MODE=1
    -D ARDUINO_USB_CDC_ON_BOOT=1
```

### ESP32-C3 Configuration
```ini
[env:esp32c3_tubes]
board = esp32-c3-devkitm-1
board_build.mcu = esp32c3
build_flags =
    ${env.build_flags}
    -D USERMOD_TUBES
    -D WLED_DISABLE_INFRARED  ; No RMT available for IR
```

### QuinLED Dig2Go Configuration
```ini
[env:quinled_dig2go_tubes]
board = esp32dev
build_flags =
    ${env.build_flags}
    -D USERMOD_TUBES
    -D QUINLED_DIG2GO
    -D DATA_PIN=16
    -D BTNPIN=0
```

### Custom PCB Requirements
- ESP32-S3-WROOM-1 module (8MB flash, 8MB PSRAM)
- USB-C for power and programming
- Level shifter for 5V LED data (SN74LV1T34)
- Single button + status LED
- Optional: LiPo charging circuit
- Optional: Microphone for audio reactive

---

## Feature Roadmap

### Remote Control

The Tubes Master Remote is a dedicated ESP32-based controller that commands the entire mesh network via ESP-NOW. It provides tactile control for live events where phone/web interfaces are impractical.

---

#### Hardware Design

**Core Components**
| Component | Part | Purpose |
|-----------|------|---------|
| MCU | ESP32-S3-DevKitC or ESP32-C3 | Main processor, ESP-NOW |
| Buttons | 6-8 tactile switches | Primary input |
| Status LEDs | WS2812B strip (8-16 LEDs) | Visual feedback |
| Power | 18650 LiPo + TP4056 | Portable operation |
| Antenna | External 2.4GHz (optional) | Extended range |

**Button Layout**
```
┌─────────────────────────────────┐
│  [POWER]              [BRIGHT+] │
│                                 │
│  [PATTERN-] [PATTERN+]          │
│                       [BRIGHT-] │
│  [PALETTE-] [PALETTE+]          │
│                                 │
│  [MODE]               [SYNC]    │
│                                 │
│  ░░░░░░░░░░░░░░░░  (LED strip)  │
└─────────────────────────────────┘
```

**GPIO Pin Assignment (ESP32-S3)**
```cpp
#define BTN_POWER       GPIO_NUM_1
#define BTN_PATTERN_UP  GPIO_NUM_2
#define BTN_PATTERN_DN  GPIO_NUM_3
#define BTN_PALETTE_UP  GPIO_NUM_4
#define BTN_PALETTE_DN  GPIO_NUM_5
#define BTN_BRIGHT_UP   GPIO_NUM_6
#define BTN_BRIGHT_DN   GPIO_NUM_7
#define BTN_MODE        GPIO_NUM_8
#define BTN_SYNC        GPIO_NUM_9
#define LED_DATA_PIN    GPIO_NUM_10
#define BATTERY_ADC     GPIO_NUM_11
```

**Enclosure Options**
- 3D printed handheld case
- Hammond 1593 series project box
- Waterproof IP65 enclosure for outdoor use

---

#### Button Functions

| Button | Short Press | Long Press (1s) | Double Press |
|--------|-------------|-----------------|--------------|
| POWER | Toggle all tubes on/off | Enter sleep mode | Force AP mode on master |
| PATTERN+ | Next pattern | Jump to favorites | Enter pattern select mode |
| PATTERN- | Previous pattern | Random pattern | - |
| PALETTE+ | Next palette | Jump to favorites | Enter palette select mode |
| PALETTE- | Previous palette | Random palette | - |
| BRIGHT+ | Increase brightness 10% | Max brightness | - |
| BRIGHT- | Decrease brightness 10% | Min brightness (5%) | - |
| MODE | Cycle power mode | Enter config mode | - |
| SYNC | Force mesh resync | Tap tempo (BPM) | Reset BPM to 120 |

**Power Modes (via MODE button)**
1. **Party** - Full brightness, all effects enabled
2. **Ambient** - 50% brightness, calm patterns only
3. **Low Power** - 25% brightness, minimal effects
4. **Blackout** - LEDs off, mesh still active

---

#### Status LED Feedback

The 8-16 LED strip on the remote provides visual feedback:

```
LED Layout: [0][1][2][3][4][5][6][7]

Pattern Display:
- LEDs 0-3: Current pattern index (binary or color-coded)
- LEDs 4-5: Current palette preview
- LED 6: Power mode indicator
- LED 7: Mesh/battery status

Color Codes:
- Green pulse: Command sent successfully
- Red flash: Error or no mesh connection
- Blue breathe: Scanning for nodes
- Yellow: Low battery warning
- White flash: Button press acknowledgment
```

**Brightness Indicator**
When adjusting brightness, all LEDs show the current level:
```
25%:  ██░░░░░░
50%:  ████░░░░
75%:  ██████░░
100%: ████████
```

---

#### ESP-NOW Protocol

**Message Types**

| Command | ID | Payload | Description |
|---------|-----|---------|-------------|
| MASTER_STATE | 0x20 | 12 bytes | Full state broadcast |
| MASTER_ACTION | 0x30 | 4 bytes | Immediate command |
| MASTER_PING | 0x40 | 2 bytes | Mesh discovery |
| MASTER_CONFIG | 0x50 | 32 bytes | Configuration push |

**MASTER_STATE Payload (12 bytes)**
```cpp
struct MasterState {
    uint8_t  command;      // 0x20
    uint8_t  flags;        // Bit flags for on/off, sync, etc.
    uint8_t  pattern;      // Current pattern index (0-255)
    uint8_t  palette;      // Current palette index (0-255)
    uint8_t  brightness;   // 0-255
    uint8_t  power_mode;   // 0=party, 1=ambient, 2=low, 3=blackout
    uint16_t bpm;          // BPM in 8.8 fixed point
    uint32_t beat_millis;  // Timestamp for beat sync
};
```

**MASTER_ACTION Payload (4 bytes)**
```cpp
struct MasterAction {
    uint8_t command;       // 0x30
    uint8_t action;        // Action type (see below)
    uint8_t value;         // Action parameter
    uint8_t reserved;
};

// Action types
#define ACTION_POWER_ON      0x01
#define ACTION_POWER_OFF     0x02
#define ACTION_POWER_TOGGLE  0x03
#define ACTION_PATTERN_NEXT  0x10
#define ACTION_PATTERN_PREV  0x11
#define ACTION_PATTERN_SET   0x12  // value = pattern index
#define ACTION_PALETTE_NEXT  0x20
#define ACTION_PALETTE_PREV  0x21
#define ACTION_PALETTE_SET   0x22  // value = palette index
#define ACTION_BRIGHT_UP     0x30
#define ACTION_BRIGHT_DOWN   0x31
#define ACTION_BRIGHT_SET    0x32  // value = brightness
#define ACTION_SYNC_NOW      0x40
#define ACTION_BPM_TAP       0x50
#define ACTION_BPM_SET       0x51  // value = BPM
```

**Broadcast Strategy**
- State broadcast every 2 seconds (keepalive)
- Action commands sent 3x with 10ms spacing (reliability)
- Ping/discovery on startup and every 30 seconds

---

#### Mesh Management

**Node Discovery**
On startup, master broadcasts MASTER_PING and listens for responses:
```cpp
struct NodeInfo {
    uint8_t mac[6];
    uint8_t role;          // tube, installation, etc.
    uint8_t signal;        // RSSI
    uint16_t led_count;
    uint8_t firmware_ver;
};
```

**Node Grouping**
Master can assign tubes to groups for selective control:
- Group 0: All nodes (default)
- Group 1-7: User-defined groups
- Hold MODE + PATTERN to assign selected pattern to group

**Signal Strength Display**
Long-press SYNC to show mesh health on status LEDs:
```
Strong (>-50dBm):  ████████ Green
Good (-50 to -70): ██████░░ Yellow
Weak (-70 to -85): ████░░░░ Orange
Poor (<-85dBm):    ██░░░░░░ Red
```

---

#### Firmware Structure

**File: `usermods/Tubes/master_remote.h`**
```cpp
#pragma once

#include "wled.h"
#include "mesh_protocol.h"

class TubesMasterRemote : public Usermod {
private:
    // Button state
    struct Button {
        uint8_t pin;
        bool pressed;
        uint32_t pressTime;
        uint32_t lastDebounce;
    };
    Button buttons[9];

    // Current state
    MasterState state;

    // Mesh info
    std::vector<NodeInfo> nodes;
    uint32_t lastPing;
    uint32_t lastBroadcast;

    // Status LEDs
    CRGB statusLeds[16];

    // Methods
    void initButtons();
    void scanButtons();
    void handleButton(uint8_t btn, bool longPress, bool doublePress);
    void broadcastState();
    void sendAction(uint8_t action, uint8_t value = 0);
    void updateStatusLeds();
    void discoverNodes();

public:
    void setup() override;
    void loop() override;
    void connected() override;
    bool handleButton(uint8_t b) override;
    void addToConfig(JsonObject& root) override;
    bool readFromConfig(JsonObject& root) override;
    uint16_t getId() override { return USERMOD_ID_TUBES_MASTER; }
};
```

**Button Debouncing & Detection**
```cpp
#define DEBOUNCE_MS      50
#define LONG_PRESS_MS    1000
#define DOUBLE_PRESS_MS  300

void TubesMasterRemote::scanButtons() {
    uint32_t now = millis();

    for (int i = 0; i < 9; i++) {
        bool reading = digitalRead(buttons[i].pin) == LOW;

        if (reading != buttons[i].pressed) {
            if (now - buttons[i].lastDebounce > DEBOUNCE_MS) {
                buttons[i].lastDebounce = now;

                if (reading) {
                    // Button pressed
                    buttons[i].pressTime = now;
                    buttons[i].pressed = true;
                } else {
                    // Button released
                    uint32_t duration = now - buttons[i].pressTime;
                    bool longPress = duration > LONG_PRESS_MS;
                    bool doublePress = (now - lastRelease[i]) < DOUBLE_PRESS_MS;

                    handleButton(i, longPress, doublePress);
                    buttons[i].pressed = false;
                    lastRelease[i] = now;
                }
            }
        }
    }
}
```

---

#### Build Configuration

**PlatformIO Environment**
```ini
[env:tubes_master_remote]
board = esp32-s3-devkitc-1
board_build.mcu = esp32s3
build_flags =
    -D USERMOD_TUBES_MASTER_REMOTE
    -D TUBES_MASTER_MODE
    -D WLED_DISABLE_MQTT
    -D WLED_DISABLE_ALEXA
    -D WLED_DISABLE_BLYNK
    -D WLED_DISABLE_INFRARED
    -D WLED_DISABLE_ADALIGHT
    -D STATUSLED_PIN=10
    -D STATUSLED_COUNT=8
    -D BTN_ACTIVE_LOW=1
lib_deps =
    fastled/FastLED
    ; Minimal deps for remote
```

---

#### Implementation Tasks

**Phase 1: Basic Control**
- [ ] Set up button GPIO with interrupts
- [ ] Implement debounce and long-press detection
- [ ] Create MasterState struct and broadcast
- [ ] Test single-button pattern/palette cycling
- [ ] Add status LED feedback for button presses

**Phase 2: Full Button Matrix**
- [ ] Wire all 8-9 buttons
- [ ] Implement all short-press functions
- [ ] Add long-press behaviors
- [ ] Double-press detection for power and favorites

**Phase 3: Mesh Integration**
- [ ] Node discovery via MASTER_PING
- [ ] Track connected nodes and signal strength
- [ ] Implement group assignment
- [ ] Add mesh health display mode

**Phase 4: Polish**
- [ ] Battery monitoring and low-power warning
- [ ] Sleep mode for extended battery life
- [ ] Status LED animations (startup, sync, error)
- [ ] Persistent storage for favorites and groups

**Phase 5: Enclosure**
- [ ] Design 3D printable case
- [ ] Add lanyard attachment point
- [ ] Optional: Weatherproofing for outdoor use

---

#### Future Enhancements

**Phone App Bridge**
- Master remote can expose BLE GATT service
- Phone app connects to remote, remote relays to mesh
- Enables phone control without WiFi on tubes

**Tap Tempo with Visual Feedback**
- SYNC button acts as tap tempo
- Status LEDs flash on detected beat
- 4 taps to set BPM, displayed on LEDs

**Preset System**
- Long-press PATTERN to save current state as preset
- Double-press to cycle through saved presets
- 8 preset slots stored in NVS

---

### Pattern Library Expansion

#### Current Patterns (57)
Categorized as: Solid, Gradient, Chase, Pulse, Noise, WLED FX

#### New Pattern Categories

**Beat-Reactive Patterns**
- [ ] Strobe sync (flash on beat)
- [ ] Bass pulse (expand from center on kick)
- [ ] Hi-hat sparkle (glitter on hi-hat hits)
- [ ] Phrase drop (big effect on phrase boundary)

**Geometric Patterns**
- [ ] Rotating segments
- [ ] Expanding rings
- [ ] Zigzag chase
- [ ] Helix/DNA spiral

**Ambient Patterns**
- [ ] Fire with sparks
- [ ] Ocean waves
- [ ] Northern lights
- [ ] Starfield

**Coordinated Multi-Tube Patterns**
- [ ] Sequential cascade (pattern travels across tubes)
- [ ] Mirror mode (adjacent tubes reflect)
- [ ] Position-aware gradients
- [ ] Leader spotlight

#### Pattern Definition Format
```cpp
// pattern.h entry
{
    .id = PATTERN_BASS_PULSE,
    .name = "Bass Pulse",
    .category = CAT_BEAT_REACTIVE,
    .wled_mode = FX_MODE_STATIC,
    .custom_render = render_bass_pulse,
    .speed_factor = 1.0,
    .palette_mode = PALETTE_FULL,
}
```

---

### Tubes Web UI

#### Goals
- Streamlined interface optimized for Tubes use cases
- Mobile-first responsive design
- Real-time mesh status visualization
- Easy BPM and pattern control

#### UI Components

**Dashboard**
- [ ] Mesh network visualization (node tree)
- [ ] Global pattern/palette selector
- [ ] BPM display with tap tempo
- [ ] Master brightness slider
- [ ] Power mode toggle (party/ambient/off)

**Node Management**
- [ ] List all discovered nodes
- [ ] Assign roles (master, follower, installation)
- [ ] Per-node brightness override
- [ ] Node firmware version display
- [ ] Signal strength indicators

**Pattern Browser**
- [ ] Grid view with animated previews
- [ ] Category filters
- [ ] Favorites system
- [ ] Pattern queue/playlist

**Settings**
- [ ] WiFi configuration
- [ ] LED strip settings (length, type, color order)
- [ ] Power limits
- [ ] Mesh network settings
- [ ] Firmware update

#### Technical Approach
- Build on WLED's existing web infrastructure
- Use Preact for reactive components
- Custom CSS theme for Tubes branding
- WebSocket for real-time updates
- Store UI in LittleFS

---

## Implementation Tasks

### Usermod Structure

```
usermods/Tubes/
├── Tubes.h              # Main usermod entry point
├── controller.h         # State machine, pattern management
├── node.h               # ESP-NOW mesh networking
├── beats.h              # BPM synchronization
├── pattern.h            # Pattern definitions
├── particle.h           # Particle physics engine
├── virtual_strip.h      # 3-layer animation system
├── effects.h            # Effect recipes
├── palettes.h           # Custom color palettes
├── master.h             # Remote control support
├── mesh_protocol.h      # Message definitions
├── config.h             # Compile-time options
├── debug.h              # Serial diagnostics
└── web/                 # Custom UI assets
    ├── tubes.htm
    ├── tubes.js
    └── tubes.css
```

### Build Configuration

#### platformio_tubes.ini
```ini
; Tubes-specific build environments
; Include via: extra_configs = platformio_tubes.ini

[tubes_common]
build_flags =
    -D USERMOD_TUBES
    -D WLED_DISABLE_MQTT
    -D WLED_DISABLE_ALEXA
    -D WLED_DISABLE_BLYNK
    -D WLED_DISABLE_HUESYNC
    -D WLED_DISABLE_LOXONE
lib_deps =
    ${env.lib_deps}

[env:esp32s3_tubes]
extends = env:esp32s3dev_8MB
build_flags =
    ${env:esp32s3dev_8MB.build_flags}
    ${tubes_common.build_flags}

[env:esp32c3_tubes]
extends = env:esp32c3dev
build_flags =
    ${env:esp32c3dev.build_flags}
    ${tubes_common.build_flags}
```

---

## Milestones

### M1: Development Environment
- Upstream WLED builds successfully
- Basic Tubes usermod compiles
- Test hardware flashed and running

### M2: Mesh Networking
- Two devices communicate via ESP-NOW
- Leader election works
- State sync between nodes

### M3: Pattern System
- 20+ patterns working
- BPM sync functional
- Particle effects rendering

### M4: Web UI
- Custom Tubes dashboard
- Mesh status display
- Pattern/palette selection

### M5: Remote Control
- Physical master controller
- Full mesh control from master
- Battery status reporting

### M6: Production Ready
- All target hardware supported
- Documentation complete
- Release binaries available

---

## Technical Notes

### Memory Budget (ESP32)
- Total RAM: 320KB
- WLED core: ~80KB
- Tubes usermod: ~20KB
- LED buffer (300 LEDs): ~1KB
- Particle buffer (80 particles): ~2KB
- Available: ~200KB for WiFi/networking

### ESP-NOW Constraints
- Max payload: 250 bytes
- Max peers: 20 (encrypted) / 6 (unencrypted with encryption enabled)
- Range: ~200m line of sight
- Latency: <10ms typical

### Power Calculations
- WS2812B: 60mA per LED at full white
- 144 LED tube at 50% brightness: ~4.3A @ 5V
- Battery tube target: 700mA max draw

---

## Open Questions

1. Should Tubes UI replace or supplement default WLED UI?
2. Best approach for phone app - BLE direct or WiFi bridge?
3. How to handle firmware updates across mesh?
4. Optimal mesh topology for 50+ nodes?

---

*Plan created: January 2026*
*Last updated: January 2026*

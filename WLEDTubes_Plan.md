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
| QuinLED Dig2Go | Supported | Popular LED controller, 1 button (GPIO0) |
| Waveshare ESP32-S3-Audio | **Recommended** | Multiple buttons, dual mic, RGB LEDs, $16-25 |
| ESP32-S3-BOX-Lite | Planned | 3 buttons, dual mic, LCD, Espressif official |
| Adafruit Sparkle Motion | Planned | WLED-native, 4 outputs, 100W USB-C PD, $25 |
| SMLIGHT SLWF-03 | Planned | Pre-flashed WLED, sensor button, IR, compact |
| ATHOM Sound Reactive | Planned | I2S mic, 16A relay, 5-48V, 1 button |
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

## Multi-Button Board Configurations

### Waveshare ESP32-S3-Audio Board (Recommended)

**Purchase:** [Waveshare Store](https://www.waveshare.com/esp32-s3-audio-board.htm) ($15.99-$24.99) | [Amazon](https://www.amazon.com/ESP32-S3-Development-Supports-Dual-MIC-Interaction/dp/B0FP5QYZM9) ($23.99)

**Features:**
- ESP32-S3 with Wi-Fi 4 + Bluetooth 5 LE
- Dual microphone array (ES7210 ADC)
- ES8311 audio DAC + speaker header
- **Multiple reserved buttons** for custom functions
- 7x onboard RGB LEDs (programmable)
- USB-C power/programming
- Optional Li-ion battery with charging circuit
- MicroSD card slot
- LCD/camera expansion headers

**GPIO Assignments (TBD - needs verification):**
| Function | GPIO | Notes |
|----------|------|-------|
| Button 1 | TBD | User button |
| Button 2 | TBD | User button |
| Button 3 | TBD | User button (if available) |
| RGB LEDs | TBD | 7x WS2812-compatible |
| I2S MCLK | TBD | Audio codec |
| I2S BCLK | TBD | Audio codec |
| I2S WS | TBD | Audio codec |
| I2S DIN | TBD | Mic input |
| LED Data | TBD | External LED strip |

```ini
[env:waveshare_s3_audio_tubes]
board = esp32-s3-devkitc-1
board_build.mcu = esp32s3
board_build.f_cpu = 240000000L
board_build.arduino.memory_type = qio_opi
build_flags =
    ${common.build_flags}
    ${esp32s3.build_flags}
    -D USERMOD_TUBES
    -D BOARD_HAS_PSRAM
    -D WLED_USE_PSRAM
    -D ARDUINO_USB_MODE=1
    -D ARDUINO_USB_CDC_ON_BOOT=1
    ; Button configuration (verify GPIOs)
    -D BTNPIN=0
    -D TUBES_BTN_COUNT=3
    ; Audio reactive
    -D USERMOD_AUDIOREACTIVE
    -D I2S_MIC=1
    ; Onboard status LEDs
    -D TUBES_STATUS_LEDS=7
lib_deps =
    ${esp32s3.lib_deps}
    https://github.com/kosme/arduinoFFT#v2.0.2
```

---

### ESP32-S3-BOX-Lite

**Purchase:** [Amazon](https://www.amazon.com/Espressif-ESP32-S3-BOX-Lite-Development-Board/dp/B0B888N9F2)

**Features:**
- ESP32-S3-WROOM-1-N16R8 (16MB flash, 8MB PSRAM)
- **3 independent buttons**
- Dual microphones
- 2.4" LCD touchscreen (ILI9341)
- Speaker output
- Two Pmod-compatible expansion interfaces
- USB-C

**GPIO Assignments:**
| Function | GPIO | Notes |
|----------|------|-------|
| Button 1 (Boot) | GPIO0 | Boot/user button |
| Button 2 | GPIO1 | Mute button |
| Button 3 | GPIO2 | User button |
| I2S Mic | GPIO41/42 | Dual MEMS microphones |
| Speaker | GPIO45/46 | I2S DAC output |
| LCD SPI | Various | ILI9341 display |
| LED Data | GPIO39 | Suggested for external strip |

```ini
[env:esp32s3_box_lite_tubes]
board = esp32s3box
board_build.mcu = esp32s3
board_build.arduino.memory_type = qio_opi
board_upload.flash_size = 16MB
build_flags =
    ${common.build_flags}
    ${esp32s3.build_flags}
    -D USERMOD_TUBES
    -D BOARD_HAS_PSRAM
    -D WLED_USE_PSRAM
    -D ARDUINO_USB_MODE=1
    -D ARDUINO_USB_CDC_ON_BOOT=1
    ; 3-button configuration
    -D BTNPIN=0
    -D TUBES_BTN_MODE=1
    -D TUBES_BTN_WIFI=2
    -D TUBES_BTN_COUNT=3
    ; External LED output
    -D LEDPIN=39
    ; Audio reactive with dual mics
    -D USERMOD_AUDIOREACTIVE
    -D I2S_MIC=1
    -D I2S_MIC_CHANNEL=3
lib_deps =
    ${esp32s3.lib_deps}
    https://github.com/kosme/arduinoFFT#v2.0.2
```

---

### Adafruit Sparkle Motion

**Purchase:** [Adafruit](https://www.adafruit.com/product/6065) ($24.95)

**Features:**
- ESP32-S2 based (not S3)
- **WLED/xLights native support**
- 100W USB-C PD (5V/12V/20V selectable)
- 4 level-shifted LED outputs
- I2S digital microphone
- IR receiver
- User button + reset button
- Onboard NeoPixel + status LED
- Stemma QT (I2C) expansion
- 5A fuse protection

**GPIO Assignments:**
| Function | GPIO | Notes |
|----------|------|-------|
| User Button | GPIO0 | Boot button |
| LED Out 1 | GPIO16 | Level-shifted 5V |
| LED Out 2 | GPIO17 | Level-shifted 5V |
| LED Out 3 | GPIO18 | Level-shifted 5V |
| LED Out 4 | GPIO19 | Level-shifted 5V |
| I2S Mic | GPIO36 | PDM microphone |
| IR Receiver | GPIO38 | 38kHz IR |
| Onboard NeoPixel | GPIO33 | Status LED |
| Stemma QT | GPIO8/9 | I2C SDA/SCL |

```ini
[env:adafruit_sparkle_motion_tubes]
board = adafruit_feather_esp32s2
board_build.mcu = esp32s2
build_flags =
    ${common.build_flags}
    ${esp32s2.build_flags}
    -D USERMOD_TUBES
    -D ARDUINO_USB_MODE=1
    -D ARDUINO_USB_CDC_ON_BOOT=1
    ; Multi-output configuration
    -D DATA_PINS=16,17,18,19
    -D PIXEL_COUNTS="150,150,150,150"
    -D NUM_STRIPS=4
    ; Single button (+ reset)
    -D BTNPIN=0
    -D TUBES_BTN_COUNT=1
    ; Audio reactive
    -D USERMOD_AUDIOREACTIVE
    -D I2S_MIC=1
    -D I2S_SDPIN=36
    ; IR support
    -D IRPIN=38
    ; Onboard status LED
    -D TUBES_STATUS_PIN=33
lib_deps =
    ${esp32s2.lib_deps}
    https://github.com/kosme/arduinoFFT#v2.0.2
    IRremoteESP8266
```

---

### SMLIGHT SLWF-03

**Purchase:** [SMLIGHT Store](https://smlight.tech/product/slwf-03)

**Features:**
- ESP32-WROOM-32E
- Pre-flashed with WLED
- Digital microphone ICS-43434
- **Sensor/touch button** with LED feedback
- IR receiver
- Compact form factor
- 5-24V input

**GPIO Assignments:**
| Function | GPIO | Notes |
|----------|------|-------|
| Sensor Button | GPIO0 | Touch-sensitive with LED |
| Button LED | GPIO2 | Red feedback LED |
| LED Data | GPIO16 | External strip |
| I2S Mic | GPIO32/33 | ICS-43434 |
| IR Receiver | GPIO39 | 38kHz IR |

```ini
[env:smlight_slwf03_tubes]
board = esp32dev
platform = espressif32@5.3.0
build_flags =
    ${tubes.build_flags}
    -D WLED_RELEASE_NAME=SMLIGHT_SLWF03_TUBES
    ; Single sensor button
    -D BTNPIN=0
    -D TUBES_BTN_COUNT=1
    -D TUBES_BTN_LED=2
    ; LED output
    -D LEDPIN=16
    ; Audio reactive
    -D USERMOD_AUDIOREACTIVE
    -D I2S_MIC=1
    ; IR support
    -D IRPIN=39
lib_deps =
    ${tubes.lib_deps}
```

---

### ATHOM WLED Sound Reactive Controller

**Purchase:** [ATHOM Store](https://www.athom.tech/blank-1/wled-esp32-music-addressable-led-strip-controller)

**Features:**
- ESP32-WROOM-32E
- I2S PDM digital microphone
- Level shifter built-in
- 16A relay for power control
- IR receiver (optional)
- 1 button (GPIO0)
- 2 LED channels (expandable to 4)
- 5V-24V or 5V-48V versions

**GPIO Assignments:**
| Function | GPIO | Notes |
|----------|------|-------|
| Button | GPIO0 | Boot/user button |
| LED Data 1 | GPIO16 | Primary output |
| LED Data 2 | GPIO3 | Secondary output |
| I2S Mic | GPIO32/33 | PDM microphone |
| IR Receiver | GPIO39 | Optional |
| Relay | GPIO12 | 16A power relay |

```ini
[env:athom_sound_reactive_tubes]
board = esp32dev
platform = espressif32@5.3.0
build_flags =
    ${tubes.build_flags}
    -D WLED_RELEASE_NAME=ATHOM_SOUND_TUBES
    ; Single button
    -D BTNPIN=0
    -D TUBES_BTN_COUNT=1
    ; Dual LED output
    -D DATA_PINS=16,3
    -D NUM_STRIPS=2
    -D DEFAULT_LED_COUNT=150
    ; Audio reactive
    -D USERMOD_AUDIOREACTIVE
    -D I2S_MIC=1
    ; Relay control
    -D RLYPIN=12
    ; IR (if equipped)
    -D IRPIN=39
lib_deps =
    ${tubes.lib_deps}
```

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
*Last updated: January 11, 2026*

---

## Development Log

### January 11, 2026

**Firmware Flashing**
- Successfully flashed `esp32_quinled_dig2go_tubes` firmware to QuinLED Dig2Go board via USB
- Device: ESP32-D0WD-V3 (revision v3.1) at `/dev/cu.usbserial-2120`
- Build stats: RAM 26.4%, Flash 92.5%
- Serial output confirmed successful boot and ESP-NOW initialization
- LED strip connected to GPIO 16 was non-functional (hardware issue, to be replaced)

**Clawdbot Gateway**
- Diagnosed gateway connection issues - LaunchAgent was disabled
- Re-enabled and bootstrapped `com.clawdbot.gateway` service
- Gateway now running on `ws://127.0.0.1:18789`
- All providers connected: Discord (@GregBot), Telegram (@GregOrbBot), iMessage

---

## Upcoming Tasks

### Multi-Button Hardware Interface

#### Task 1: Hardware-Compatible Button Logic for ESP32

Design a flexible button input system that works across different ESP32 boards with varying button configurations.

**Requirements**
- [ ] Support 1-4 physical buttons depending on hardware
- [ ] Handle boards with different GPIO assignments (QuinLED, Athom, custom)
- [ ] Debouncing with configurable timing
- [ ] Support for short press, long press, and double press
- [ ] Active-low and active-high button configurations
- [ ] Optional internal pull-up/pull-down resistor configuration

**Hardware Compatibility Matrix**
| Board | Button Count | GPIOs | Notes |
|-------|--------------|-------|-------|
| QuinLED Dig2Go | 1 | GPIO0 | Boot button, active-low |
| Waveshare ESP32-S3-Audio | 3+ | TBD | **Recommended** - multiple user buttons |
| ESP32-S3-BOX-Lite | 3 | GPIO0, 1, 2 | Boot + mute + user buttons |
| Adafruit Sparkle Motion | 2 | GPIO0 + reset | User + reset, ESP32-S2 |
| SMLIGHT SLWF-03 | 1 | GPIO0 | Sensor button with LED feedback |
| ATHOM Sound Reactive | 1 | GPIO0 | Boot button, has relay |
| Athom ESP32-C3 | 1 | GPIO9 | Active-low |
| ESP32-S3 DevKit | 1 | GPIO0 | Boot button |
| Custom Tubes PCB | 3-4 | TBD | Dedicated button GPIO |

**Proposed Button Structure**
```cpp
struct ButtonConfig {
    int8_t pin;              // GPIO pin (-1 = disabled)
    bool activeLow;          // true = pressed when LOW
    bool internalPullup;     // use internal pull-up resistor
    uint16_t debounceMs;     // debounce time (default 50ms)
    uint16_t longPressMs;    // long press threshold (default 1000ms)
    uint16_t doublePressMs;  // double press window (default 300ms)
};

struct ButtonState {
    bool pressed;
    bool wasPressed;
    uint32_t pressStartTime;
    uint32_t lastReleaseTime;
    uint8_t pressCount;      // for double-press detection
};
```

**Implementation Sketch**
```cpp
class TubesButtonHandler {
private:
    ButtonConfig configs[4];
    ButtonState states[4];
    uint8_t buttonCount;

public:
    void begin(uint8_t count, const ButtonConfig* cfgs);
    void update();  // call in loop()

    // Event callbacks
    void onShortPress(uint8_t btn, void (*callback)());
    void onLongPress(uint8_t btn, void (*callback)());
    void onDoublePress(uint8_t btn, void (*callback)());
};
```

---

#### Task 2: Multi-Button State Machine Logic

Define button mappings for common Tubes operations with graceful fallback for single-button boards.

**State Modes**
| Mode | Description |
|------|-------------|
| OFF | LEDs off, low power, mesh still listening |
| LAMP | Warm white ambient light, no patterns |
| RAVE | Full pattern mode, beat-synced effects |
| CONFIG | WiFi AP mode for configuration |

**Multi-Button Layout (3-4 buttons)**
```
┌─────────────────────────────────┐
│  [POWER]    [MODE]    [WIFI]    │
│                                 │
│           [PATTERN]             │
└─────────────────────────────────┘
```

**Button Action Matrix**
| Button | Short Press | Long Press (1s) | Double Press |
|--------|-------------|-----------------|--------------|
| POWER | Toggle ON/OFF | Enter deep sleep | Force reboot |
| MODE | Cycle: LAMP → RAVE | - | Return to LAMP |
| WIFI | Show WiFi status (LED flash) | Start AP mode | Reset WiFi credentials |
| PATTERN | Next pattern | Previous pattern | Random pattern |

**Single-Button Fallback (Boot button only)**
| Action | Trigger |
|--------|---------|
| Toggle ON/OFF | Short press |
| Next pattern | Double press |
| Cycle mode (LAMP/RAVE) | Triple press |
| WiFi AP mode | Long press (3s) |
| Factory reset | Very long press (10s) |

**State Machine Implementation**
```cpp
enum class TubesMode : uint8_t {
    OFF = 0,
    LAMP = 1,
    RAVE = 2,
    CONFIG = 3
};

class TubesStateMachine {
private:
    TubesMode currentMode = TubesMode::LAMP;
    TubesMode previousMode = TubesMode::LAMP;
    bool wifiAPActive = false;

public:
    void togglePower();
    void cycleMode();
    void setMode(TubesMode mode);
    TubesMode getMode() const { return currentMode; }

    // Mode-specific behaviors
    void enterLampMode();   // warm white, ~2700K, 30% brightness
    void enterRaveMode();   // restore last pattern, full sync
    void enterConfigMode(); // start AP, status LED pattern
    void enterOffMode();    // fade out, minimal power
};

// Example lamp mode settings
void TubesStateMachine::enterLampMode() {
    // Set to warm white
    strip.setColor(0, 255, 180, 80);  // Warm white RGB
    strip.setBrightness(76);           // ~30%

    // Disable pattern effects
    effectCurrent = FX_MODE_STATIC;

    // Keep mesh active for remote control
    meshEnabled = true;
}

// Example mode cycling
void TubesStateMachine::cycleMode() {
    switch (currentMode) {
        case TubesMode::OFF:
            setMode(TubesMode::LAMP);
            break;
        case TubesMode::LAMP:
            setMode(TubesMode::RAVE);
            break;
        case TubesMode::RAVE:
            setMode(TubesMode::LAMP);
            break;
        case TubesMode::CONFIG:
            setMode(previousMode);
            break;
    }
}
```

**LED Feedback for Button Actions**
```cpp
// Visual feedback patterns
void flashConfirm() {
    // Quick green flash - action accepted
    strip.setPixelColor(0, 0, 255, 0);
    strip.show();
    delay(100);
    strip.setPixelColor(0, 0, 0, 0);
    strip.show();
}

void flashModeChange(TubesMode mode) {
    // Color indicates new mode
    uint32_t color;
    switch (mode) {
        case TubesMode::OFF:  color = 0x000000; break;  // Off
        case TubesMode::LAMP: color = 0xFFB040; break;  // Warm
        case TubesMode::RAVE: color = 0xFF00FF; break;  // Magenta
        case TubesMode::CONFIG: color = 0x0000FF; break; // Blue
    }
    // Pulse first 3 LEDs with mode color
    for (int i = 0; i < 3; i++) {
        strip.setPixelColor(i, color);
    }
    strip.show();
    delay(500);
}

void flashWiFiStatus() {
    // Show connection status
    if (WiFi.status() == WL_CONNECTED) {
        // Green pulses = connected, count = signal strength
        int bars = map(WiFi.RSSI(), -90, -30, 1, 5);
        for (int i = 0; i < bars; i++) {
            flashConfirm();
            delay(200);
        }
    } else {
        // Red flash = not connected
        strip.setPixelColor(0, 255, 0, 0);
        strip.show();
        delay(500);
    }
}
```

**Integration with Existing WLED Button Handler**
```cpp
// In Tubes usermod, override button handling
bool TubesUsermod::handleButton(uint8_t b) {
    // Let TubesButtonHandler process the event
    ButtonEvent event = buttonHandler.getEvent(b);

    switch (event) {
        case ButtonEvent::SHORT_PRESS:
            if (b == BTN_POWER) stateMachine.togglePower();
            else if (b == BTN_MODE) stateMachine.cycleMode();
            else if (b == BTN_PATTERN) nextPattern();
            break;

        case ButtonEvent::LONG_PRESS:
            if (b == BTN_WIFI) startAPMode();
            else if (b == BTN_PATTERN) previousPattern();
            break;

        case ButtonEvent::DOUBLE_PRESS:
            if (b == BTN_PATTERN) randomPattern();
            break;
    }

    return true;  // event handled
}
```

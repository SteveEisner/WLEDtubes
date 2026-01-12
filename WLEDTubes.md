# WLEDTubes - Research & Architecture Documentation

## Table of Contents
1. [What is WLED?](#what-is-wled)
2. [WLED Architecture](#wled-architecture)
3. [PlatformIO Configuration](#platformio-configuration)
4. [What is WLEDTubes?](#what-is-wledtubes)
5. [WLEDTubes Architecture](#wledtubes-architecture)
6. [Key Modifications from Standard WLED](#key-modifications-from-standard-wled)
7. [File Structure Reference](#file-structure-reference)

---

## What is WLED?

WLED is an open-source firmware for ESP32 and ESP8266 microcontrollers that provides a full-featured web server to control addressable LEDs. Created by Aircoookie (Christian Schwinne) and first released in 2018, it has grown into one of the most popular LED control systems available.

### Supported LED Types
- **NeoPixel/WS2812B**: Most common addressable RGB LEDs
- **WS2811**: Lower-cost addressable LEDs
- **SK6812**: RGBW variant with dedicated white channel
- **APA102/SK9822**: SPI-based high-speed LEDs (clock + data)
- **WS2801**: SPI-based LEDs
- **PWM white strips**: Non-addressable single-color strips

### Core Capabilities
- **100+ built-in effects** via WS2812FX library integration
- **50+ color palettes** with FastLED noise effects
- **Multi-output support**: Up to 3 strips on ESP8266, up to 10 on ESP32
- **Segment system**: Different effects/colors on different LED sections
- **250 user presets**: Save and recall configurations
- **OTA updates**: Over-the-air firmware updates via HTTP or ArduinoOTA

---

## WLED Architecture

### Software Stack

```
┌─────────────────────────────────────────────────────┐
│                   Web Interface                      │
│              (HTML/JS served from SPIFFS)            │
├─────────────────────────────────────────────────────┤
│                  Control Layer                       │
│   JSON API │ HTTP API │ MQTT │ E1.31 │ Art-Net     │
├─────────────────────────────────────────────────────┤
│                  Effects Engine                      │
│          WS2812FX + FastLED + Custom FX             │
├─────────────────────────────────────────────────────┤
│                 Segment Manager                      │
│        (Multiple independent LED sections)           │
├─────────────────────────────────────────────────────┤
│                   LED Drivers                        │
│           NeoPixelBus │ FastLED │ PWM               │
├─────────────────────────────────────────────────────┤
│              Hardware Abstraction                    │
│              ESP32 │ ESP8266 │ GPIO                 │
└─────────────────────────────────────────────────────┘
```

### Key Source Files (wled00/)

| File | Purpose |
|------|---------|
| `wled.h` / `wled.cpp` | Main application entry, global state, main loop |
| `FX.h` / `FX.cpp` | Effects engine with 100+ modes |
| `FX_fcn.cpp` | Effect helper functions and utilities |
| `palettes.h` | Color palette definitions |
| `wled_server.cpp` | Web server and API endpoints |
| `json.cpp` | JSON API handling |
| `button.cpp` | Physical button input handling |
| `led.cpp` | LED output management |
| `cfg.cpp` | Configuration management |
| `wled_eeprom.cpp` | EEPROM storage for settings |
| `bus_manager.h` | Multi-bus LED output abstraction |

### Control Interfaces

WLED supports multiple control methods:
- **Web UI**: Browser-based interface at device IP
- **JSON API**: RESTful endpoints at `/json/*`
- **MQTT**: Home automation integration
- **E1.31/Art-Net/DDP**: Professional lighting protocols
- **Infrared**: IR remote control support
- **Alexa/Google Home**: Voice control integration
- **Blynk**: Mobile app control

### Network Modes
- **Access Point (AP)**: Creates its own WiFi network (WLED-AP)
- **Station Mode**: Connects to existing WiFi
- **Failsafe AP**: Falls back to AP if station connection fails

---

## PlatformIO Configuration

PlatformIO is the build system used for WLED and WLEDTubes. Configuration is split across multiple `.ini` files.

### Core Configuration Files

#### `platformio.ini` (Main)
```ini
[env]
platform = espressif32@5.3.0   ; ESP-IDF version
framework = arduino             ; Arduino compatibility layer
lib_deps =                      ; Shared dependencies
    fastled/FastLED @ ^3.7.3
    makuna/NeoPixelBus @ ^2.8.0
    ESPAsyncWebServer
    ArduinoJson @ ^7.0.0
```

#### `platformio_tubes.ini` (Tubes-specific)
Defines build environments for WLEDTubes hardware variants.

### Build Environments

| Environment | Board | Features |
|-------------|-------|----------|
| `esp32dev` | Generic ESP32 | Full features |
| `esp32_quinled_dig2go_tubes` | QuinLED Dig2Go | SPI LED support |
| `esp32-s3-matrix-m1_tubes` | ESP32-S3 | PSRAM, 16MB flash |
| `esp32-c3-athom_tubes` | Athom ESP32-C3 | Smart bulb form factor |

### Common Build Flags

```ini
build_flags =
    -D USERMOD_TUBES           ; Enable Tubes usermod
    -D WLED_DISABLE_MQTT       ; Disable unused features
    -D WLED_DISABLE_ALEXA
    -D WLED_DISABLE_BLYNK
    -D WLED_WATCHDOG_TIMEOUT=0
```

### Dependencies Used by WLEDTubes

| Library | Version | Purpose |
|---------|---------|---------|
| FastLED | ^3.7.3 | LED control, color math |
| NeoPixelBus | ^2.8.0 | Alternative LED driver |
| ESPAsyncWebServer | custom fork | Async HTTP server |
| IRremoteESP8266 | ^2.8.6 | IR remote support (optional) |
| arduinoFFT | v2.0.2 | Audio reactive features (optional) |

---

## What is WLEDTubes?

WLEDTubes is a sophisticated fork of WLED that transforms single-device LED control into a **coordinated, mesh-networked light tube system**. It was designed for synchronized light installations at events, camps, and art installations.

### Key Innovations

1. **ESP-NOW Mesh Network**: Devices sync without WiFi infrastructure
2. **BPM-Based Animation**: All patterns sync to musical beats
3. **Automatic Coordination**: Devices self-organize into leader/follower hierarchy
4. **Curated Pattern Library**: 57 patterns optimized for visual coherence
5. **3-Layer Effects**: Background + overlay + particle effects
6. **Role-Based Behavior**: Different modes for battery, mains, installations

### Use Cases
- Burning Man-style light installations
- Dance party synchronized lighting
- Art installations spanning large areas
- Outdoor events without WiFi infrastructure

---

## WLEDTubes Architecture

### System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    WLEDTubes Device                          │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────────┐   │
│  │   ESP-NOW   │   │  Controller │   │   Virtual       │   │
│  │   Mesh      │◄─►│  (State     │◄─►│   Strips        │   │
│  │   Network   │   │   Machine)  │   │   (3 layers)    │   │
│  └─────────────┘   └─────────────┘   └─────────────────┘   │
│         │                │                    │             │
│         ▼                ▼                    ▼             │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────────┐   │
│  │   Node      │   │   Beat      │   │   Particle      │   │
│  │   Manager   │   │   Controller│   │   Engine        │   │
│  └─────────────┘   └─────────────┘   └─────────────────┘   │
│                          │                                   │
│                          ▼                                   │
│  ┌──────────────────────────────────────────────────────┐   │
│  │              WLED Core (LED output, Web UI)           │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### Tubes Usermod Components

Located in `/usermods/Tubes/`:

| File | Lines | Purpose |
|------|-------|---------|
| `Tubes.h` | 130 | Main entry point, WLED usermod integration |
| `controller.h` | 400+ | Core state machine, pattern/palette management |
| `node.h` | 400+ | ESP-NOW mesh networking, leader election |
| `pattern.h` | 327 | 57 curated pattern definitions |
| `particle.h` | 253 | Particle physics engine (glitter, sparks, etc.) |
| `virtual_strip.h` | 197 | 3-layer animation system with crossfading |
| `effects.h` | 158 | 24 effect recipes combining particles |
| `master.h` | 192 | Physical master remote control |
| `beats.h` | 74 | BPM synchronization (8.8 fixed-point) |
| `palettes.h` | 2413 | 100+ custom vibrant color palettes |
| `debug.h` | 145 | Serial diagnostics and visual debugging |
| `options.h` | - | Configuration options |
| `global_state.h` | 54 | Shared state structure |
| `sound.h` | - | Audio reactive features |

### Mesh Network Protocol

WLEDTubes uses **ESP-NOW** for peer-to-peer communication without requiring WiFi infrastructure.

#### Node Hierarchy
```
Master Node (highest ID)
    │
    ├── Follower Node A
    │       └── Relay to distant nodes
    ├── Follower Node B
    └── Follower Node C
```

#### Message Types

| Command ID | Name | Purpose |
|------------|------|---------|
| 0x20 | STATE | Sync pattern, palette, effect |
| 0x10 | OPTIONS | Configuration broadcast |
| 0x30 | ACTION | Immediate commands |
| 0x40 | INFO | Status/debug information |
| 0x50 | BEATS | BPM synchronization |
| 0xE0 | UPGRADE | OTA firmware updates |

#### Timing Constants
- `STATUS_UPDATE_PERIOD`: 2000ms (regular broadcasts)
- `UPLINK_TIMEOUT`: 20000ms (leader lost detection)
- `REBROADCAST_TIME`: 30000ms (relay window)

### Beat Synchronization

All animations are driven by BPM (beats per minute):

```cpp
// 8.8 fixed-point BPM representation
// 256 "fracs" per beat for sub-beat precision
// Default: 120 BPM
// Minimum: 40 BPM (rejected if lower)
```

Pattern changes occur at "phrase" boundaries (16 beats = 1 phrase).

### 3-Layer Effect Composition

```
Layer 1: Background Pattern
         (rainbow, noise, WLED FX, custom patterns)
              │
              ▼
Layer 2: Virtual Strip Overlay
         (crossfading between 3 strips)
              │
              ▼
Layer 3: Particle Effects
         (glitter, sparks, beatbox, bubbles)
              │
              ▼
        Final LED Output
```

### Particle System

Up to 80 simultaneous particles with physics simulation:

| Type | Duration | Description |
|------|----------|-------------|
| Glitter | 128ms | Stationary twinkle points |
| Spark | 64ms | Moving points with velocity/gravity |
| Beatbox | 256ms | Expanding box on beat |
| Bubble | 1024ms | Expanding pop effect |
| Flash | 256ms | Full-screen brightness pulse |
| Drop | 360ms | Falling particle with gravity |

### Device Roles

Stored in EEPROM, controls behavior:

| Role | Value | Brightness | Power | Use Case |
|------|-------|------------|-------|----------|
| Unknown | 0 | - | Save on | Default |
| Default | 10 | 120/255 | 700mA | Battery tubes |
| Camp | 50 | 120/255 | 700mA | Mains, always on |
| Installation | 100 | 240/255 | 1000mA | Fixed "tank" |
| SmallArt | 120 | Half | - | Half-brightness |
| Legacy | 190 | - | - | 2019 hardware |
| Master | 200 | 200/255 | - | Controls mesh |

---

## Key Modifications from Standard WLED

### Feature Comparison

| Aspect | Standard WLED | WLEDTubes |
|--------|--------------|-----------|
| **Control** | Web UI, mobile app, MQTT | Web UI + ESP-NOW mesh |
| **Sync** | Manual preset selection | Automatic BPM-based sync |
| **Patterns** | 100+ effects | Curated 57 (WLED + custom) |
| **Network** | WiFi required | Works without WiFi |
| **Scaling** | Single device | 75+ devices meshed |
| **Power** | Fixed brightness | Adaptive per role |
| **Effects** | Single per segment | 3-layer composition |
| **Palettes** | Standard set | 100+ vibrant custom |
| **Animation** | Speed parameter | BPM-driven beat sync |

### Disabled WLED Features

To reduce memory footprint, WLEDTubes disables:
- MQTT integration
- Blynk support
- Alexa voice control
- Loxone integration
- Hue sync
- WebSockets
- Adalight serial
- Cronixie display

### Modified Core Files

| File | Modification |
|------|--------------|
| `wled.h` | Feature flags, Tubes integration |
| `wled.cpp` | Main loop hooks for Tubes |
| `button.cpp` | Custom: long-press power save, double-click BPM |
| `FX.h/FX.cpp` | Hooks for pattern system |
| `palettes.h` | 100+ additional vibrant palettes |
| `espnow_broadcast.cpp` | Extended ESP-NOW for mesh |

### Button Behavior Changes

| Action | Standard WLED | WLEDTubes |
|--------|--------------|-----------|
| Short press | Toggle on/off | Toggle on/off |
| Long press (600ms) | - | Toggle power save mode |
| Double-click | - | Request BPM adjustment |
| 6-second hold | - | Activate WiFi AP |

---

## File Structure Reference

### WLEDTubes Repository Layout

```
~/Desktop/WLEDtubes/
├── platformio.ini              # Main PIO config
├── platformio_tubes.ini        # Tubes-specific environments
├── wled00/                     # WLED core source
│   ├── wled.h                  # Main header
│   ├── wled.cpp                # Entry point
│   ├── FX.h / FX.cpp           # Effects engine
│   ├── FX_fcn.cpp              # Effect utilities
│   ├── palettes.h              # Color palettes
│   ├── button.cpp              # Button handling
│   ├── json.cpp                # JSON API
│   ├── wled_server.cpp         # Web server
│   ├── cfg.cpp                 # Configuration
│   ├── led.cpp                 # LED management
│   ├── bus_manager.h           # Multi-output support
│   ├── espnow_broadcast.cpp    # ESP-NOW mesh
│   └── data/                   # Web UI assets
├── usermods/
│   └── Tubes/                  # Tubes usermod (18 files)
│       ├── Tubes.h             # Entry point
│       ├── controller.h        # State machine
│       ├── node.h              # Mesh network
│       ├── beats.h             # BPM sync
│       ├── pattern.h           # Pattern definitions
│       ├── particle.h          # Particle engine
│       ├── virtual_strip.h     # Animation layers
│       ├── effects.h           # Effect recipes
│       ├── palettes.h          # Custom palettes
│       ├── master.h            # Remote control
│       ├── debug.h             # Diagnostics
│       └── ...                 # Other utilities
├── tools/                      # Build scripts
├── pio-scripts/                # PIO automation
└── build_output/               # Compiled binaries
```

### Build Commands

```bash
# Build for ESP32 dev board
pio run -e esp32dev

# Build Tubes variant
pio run -e esp32_tubes

# Upload to connected device
pio run -e esp32_tubes --target upload

# Monitor serial output
pio device monitor --baud 115200
```

---

## Resources

- **WLED Official**: https://kno.wled.ge/
- **WLED GitHub**: https://github.com/wled/WLED
- **ESP-NOW Documentation**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html
- **FastLED**: https://fastled.io/
- **PlatformIO**: https://platformio.org/

---

*Document generated: January 2026*
*Based on analysis of WLEDTubes fork at ~/Desktop/WLEDtubes*

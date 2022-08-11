#pragma once

#include "wled.h"
#include "FX.h"
#include "updater.h"

#include "beats.h"

#include "pattern.h"
#include "palettes.h"
#include "effects.h"
#include "global_state.h"
#include "node.h"

const static uint8_t DEFAULT_MASTER_BRIGHTNESS = 200;
#define STATUS_UPDATE_PERIOD 4000

#define MIN_COLOR_CHANGE_PHRASES 2  // 4
#define MAX_COLOR_CHANGE_PHRASES 4  // 40


typedef struct {
  bool debugging;
  uint8_t brightness;
  Fader fader; // temp
} ControllerOptions;

typedef struct {
  TubeState current;
  TubeState next;
} TubeStates;

#define NUM_VSTRIPS 3

#define DEBOUNCE_TIME 40

class Button {
  public:
    Timer debounceTimer;
    uint8_t pin;
    bool lastPressed = false;

  void setup(uint8_t pin) {
    this->pin = pin;
    pinMode(pin, INPUT_PULLUP);
    this->debounceTimer.start(0);
  }

  bool pressed() {
    if (digitalRead(this->pin) == HIGH) {
      return !this->debounceTimer.ended();
    }

    this->debounceTimer.start(DEBOUNCE_TIME);
    return true;
  }

  bool triggered() {
    // Triggers BOTH low->high AND high->low
    bool p = this->pressed();
    bool lp = this->lastPressed;
    this->lastPressed = p;
    return p != lp;
  }
};

class PatternController : public MessageReceiver {
  public:
    const static int FRAMES_PER_SECOND = 300;  // how often we animate, in frames per second
    const static int REFRESH_PERIOD = 1000 / FRAMES_PER_SECOND;  // how often we animate, in milliseconds

    uint8_t num_leds;
    VirtualStrip *vstrips[NUM_VSTRIPS];
    uint8_t next_vstrip = 0;
    bool isMaster = false;
    
    Timer graphicsTimer;
    Timer updateTimer;

#ifdef USELCD
    Lcd *lcd;
#endif
    LEDs *led_strip;
    BeatController *beats;
    Effects *effects;
    LightNode *node;

    ControllerOptions options;
    char key_buffer[20] = {0};

    Energy energy=LowEnergy;
    TubeState current_state;
    TubeState next_state;

  PatternController(uint8_t num_leds, BeatController *beats) {
    this->num_leds = num_leds;
#ifdef USELCD
    this->lcd = new Lcd();
#endif
    this->led_strip = new LEDs(num_leds);
    this->beats = beats;
    this->effects = new Effects();
    this->node = new LightNode(this);
    // this->mesh = new BLEMeshNode(this);

    for (uint8_t i=0; i < NUM_VSTRIPS; i++) {
#ifdef DOUBLED
      this->vstrips[i] = new VirtualStrip(num_leds * 2 + 1);
#else
      this->vstrips[i] = new VirtualStrip(num_leds);
#endif
    }
  }
  
  void setup(bool isMaster)
  {
    this->node->setup();
    this->isMaster = isMaster;
    this->options.debugging = false;
    this->options.brightness = DEFAULT_MASTER_BRIGHTNESS;
    this->options.fader = AUTO;

#ifdef USELCD
    this->lcd->setup();
#endif
    this->set_next_pattern(0);
    this->set_next_palette(0);
    this->set_next_effect(0);
    this->next_state.pattern_phrase = 0;
    this->next_state.palette_phrase = 0;
    this->next_state.effect_phrase = 0;

    this->updateTimer.start(STATUS_UPDATE_PERIOD); // Ready to send an update as soon as we're able to
    Serial.println("Patterns: ok");

    WS2812FX::load_pattern(FX_MODE_EXTERNAL);
  }

  void do_pattern_changes() {
    uint16_t phrase = this->current_state.beat_frame >> 12;

    if (phrase >= this->next_state.pattern_phrase) {
      this->load_pattern(this->next_state);
      this->next_state.pattern_phrase = phrase + this->set_next_pattern(phrase);
    }
    if (phrase >= this->next_state.palette_phrase) {
      this->load_palette(this->next_state);
      this->next_state.palette_phrase = phrase + this->set_next_palette(phrase);
    }
    if (phrase >= this->next_state.effect_phrase) {
      this->load_effect(this->next_state);
      this->next_state.effect_phrase = phrase + this->set_next_effect(phrase);
    }
  }

  void update()
  {
    this->read_keys();
    
    // Update the mesh
    this->node->update();

    // Update patterns to the beat
    this->update_beat();

    // Detect manual overrides & update the current state to match.
    Segment& segment = WS2812FX::get_strip()->getMainSegment();
    if (segment.palette != this->current_state.palette_id) {
      Serial.printf("Palette override = %d\n",segment.palette);
      this->next_state.palette_phrase = 0;
      this->next_state.palette_id = segment.palette;
      this->broadcast_state();
    }
    // if (segment.mode != FX_MODE_EXTERNAL) {
    //   Serial.printf("Pattern override = %d\n",segment.mode);
    // }

    do_pattern_changes();

    if (this->graphicsTimer.every(REFRESH_PERIOD)) {
      this->updateGraphics();
    }

    // Update current status
    if (this->updateTimer.every(STATUS_UPDATE_PERIOD)) {
      // Transmit less often when following
      if (!this->node->is_following() || random(0, 5) == 0) {
        this->send_update();
      }

   }

#ifdef USELCD
    if (this->lcd->active) {
      this->lcd->size(1);
      this->lcd->write(0,56, this->current_state.beat_frame);
      this->lcd->write(80,56, this->x_axis);
      this->lcd->write(100,56, this->y_axis);
      this->lcd->show();

      this->lcd->update();
    }
#endif
  }

  void restart_phrase() {
    this->beats->start_phrase();
    this->update_beat();
    this->send_update();
  }

  void set_phrase_position(uint8_t pos) {
    this->beats->sync(this->beats->bpm, (this->beats->frac & -0xFFF) + (pos<<8));
    this->update_beat();
    this->send_update();
  }
  
  void set_tapped_bpm(accum88 bpm, uint8_t pos=15) {
    // By default, restarts at 15th beat - because this is the end of a tap
    this->beats->sync(bpm, (this->beats->frac & -0xFFF) + (pos<<8));
    this->update_beat();
    this->send_update();
  }

  void update_beat() {
    this->current_state.bpm = this->next_state.bpm = this->beats->bpm;
    this->current_state.beat_frame = particle_beat_frame = this->beats->frac;  // (particle_beat_frame is a hack)
    if (this->current_state.bpm >= 125>>8)
      this->energy = HighEnergy;
    else if (this->current_state.bpm > 120>>8)
      this->energy = MediumEnergy;
    else
      this->energy = LowEnergy;
  }
  
  void send_update() {
    Serial.print("     ");
    this->current_state.print();
    Serial.print(F(" "));

    uint16_t phrase = this->current_state.beat_frame >> 12;
    Serial.print(F("    "));
    Serial.print(this->next_state.pattern_phrase - phrase);
    Serial.print(F("P "));
    Serial.print(this->next_state.palette_phrase - phrase);
    Serial.print(F("C "));
    Serial.print(this->next_state.effect_phrase - phrase);
    Serial.print(F("E: "));
    this->next_state.print();
    Serial.print(F(" "));
    Serial.println();    

    this->broadcast_state();
  }

  void background_changed() {
    this->update_background();
    this->current_state.print();
    Serial.println();
  }

  void load_pattern(TubeState &tube_state) {
    if (this->current_state.pattern_id == tube_state.pattern_id 
        && this->current_state.pattern_sync_id == tube_state.pattern_sync_id)
      return;

    this->current_state.pattern_phrase = tube_state.pattern_phrase;
    this->current_state.pattern_id = tube_state.pattern_id % gPatternCount;
    this->current_state.pattern_sync_id = tube_state.pattern_sync_id;

    Serial.print(F("Change pattern "));
    this->background_changed();
  }

  // Choose the pattern to display at the next pattern cycle
  // Return the number of phrases until the next pattern cycle
  uint16_t set_next_pattern(uint16_t phrase) {
    uint8_t pattern_id = random8(gPatternCount);
    PatternDef def = gPatterns[pattern_id];
    if (def.control.energy > this->energy) {
      pattern_id = 0;
      def = gPatterns[0];
    }

    this->next_state.pattern_id = pattern_id;
    this->next_state.pattern_sync_id = this->randomSyncMode();

    switch (def.control.duration) {
      case ShortDuration: return random8(5,15);
      case MediumDuration: return random8(15,25);
      case LongDuration: return random8(35,45);
      case ExtraLongDuration: return random8(70, 100);
    }
    return 5;
  }

  void load_palette(TubeState &tube_state) {
    if (this->current_state.palette_id == tube_state.palette_id)
      return;

    this->current_state.palette_phrase = tube_state.palette_phrase;
    this->_load_palette(tube_state.palette_id);
  }

  void _load_palette(uint8_t palette_id) {
    this->current_state.palette_id = palette_id;
    
    Serial.print(F("Change palette"));
    this->background_changed();
  }

  // Choose the palette to display at the next palette cycle
  // Return the number of phrases until the next palette cycle
  uint16_t set_next_palette(uint16_t phrase) {
    this->next_state.palette_id = random8(gGradientPaletteCount);
    return random8(MIN_COLOR_CHANGE_PHRASES, MAX_COLOR_CHANGE_PHRASES);
  }

  void load_effect(TubeState &tube_state) {
    if (this->current_state.effect_params.effect == tube_state.effect_params.effect && 
        this->current_state.effect_params.pen == tube_state.effect_params.pen && 
        this->current_state.effect_params.chance == tube_state.effect_params.chance)
      return;

    this->_load_effect(tube_state.effect_params);
  }

  void _load_effect(EffectParameters params) {
    this->current_state.effect_params = params;
  
    Serial.print(F("Change effect "));
    this->current_state.print();
    Serial.println();
    
    this->effects->load(this->current_state.effect_params);
  }

  // Choose the effect to display at the next effect cycle
  // Return the number of phrases until the next effect cycle
  uint16_t set_next_effect(uint16_t phrase) {
    uint8_t effect_num = random8(gEffectCount);

    EffectDef def = gEffects[effect_num];
    if (def.control.energy > this->energy)
      def = gEffects[0];

    this->next_state.effect_params = def.params;

    switch (def.control.duration) {
      case ShortDuration: return 3;
      case MediumDuration: return 6;
      case LongDuration: return 10;
      case ExtraLongDuration: return 20;
    }
    return 1;
  }

  void update_background() {
    Background background;
    background.animate = gPatterns[this->current_state.pattern_id].backgroundFn;
    background.palette_id = this->current_state.palette_id;
    background.sync = (SyncMode)this->current_state.pattern_sync_id;

    // re-use virtual strips to prevent heap fragmentation
    for (uint8_t i = 0; i < NUM_VSTRIPS; i++) {
      this->vstrips[i]->fadeOut();
    }
    this->vstrips[this->next_vstrip]->load(background);
    this->next_vstrip = (this->next_vstrip + 1) % NUM_VSTRIPS; 
  }

  void setBrightness(uint8_t brightness) {
    Serial.print(F("brightness "));
    Serial.println(brightness);

    this->options.brightness = brightness;
    this->broadcast_options();
  }

  void setDebugging(bool debugging) {
    Serial.print(F("debugging "));
    Serial.println(debugging);

    this->options.debugging = debugging;
    this->broadcast_options();
  }
  
  SyncMode randomSyncMode() {
  #ifdef TESTING_PATTERNS
    return All;
  #endif
    uint8_t r = random8(128);
    if (r < 40)
      return SinDrift;
    if (r < 65)
      return Pulse;
    if (r < 72)
      return Swing;
    if (r < 84)
      return SwingDrift;
    return All;
  }

  void updateGraphics() {
    static BeatFrame_24_8 lastFrame = 0;
    BeatFrame_24_8 beat_frame = this->current_state.beat_frame;

    uint8_t beat_pulse = 0;
    for (int i = 0; i < 8; i++) {
      if ( (beat_frame >> (5+i)) != (lastFrame >> (5+i)))
        beat_pulse |= 1<<i;
    }
    lastFrame = beat_frame;

    VirtualStrip *first_strip = NULL;
    for (uint8_t i=0; i < NUM_VSTRIPS; i++) {
      VirtualStrip *vstrip = this->vstrips[i];
      if (vstrip->fade == Dead)
        continue;

      // Remember the first strip
      if (first_strip == NULL)
        first_strip = vstrip;
     
      vstrip->update(beat_frame, beat_pulse);
      vstrip->blend(this->led_strip->leds, this->led_strip->num_leds, this->options.brightness, vstrip == first_strip);
    }

    this->effects->update(first_strip, beat_frame, (BeatPulse)beat_pulse);
  }

  virtual void acknowledge() {
    addFlash(CRGB::Green);
  }

  void read_keys() {
    if (!Serial.available())
      return;
      
    char c = Serial.read();
    char *k = this->key_buffer;
    uint8_t max = sizeof(this->key_buffer);
    for (uint8_t i=0; *k && (i < max-1); i++) {
      k++;
    }
    if (c == 10) {
      this->keyboard_command(this->key_buffer);
      this->key_buffer[0] = 0;
    } else {
      *k++ = c;
      *k = 0;    
    }
  }

  accum88 parse_number(char *s) {
    uint16_t n=0, d=0;
    
    while (*s == ' ')
      s++;
    while (*s) {
      if (*s < '0' || *s > '9')
        break;
      n = n*10 + (*s++ - '0');
    }
    n = n << 8;
    
    if (*s == '.') {
      uint16_t div = 1;
      s++;
      while (*s) {
        if (*s < '0' || *s > '9')
          break;
        d = d*10 + (*s++ - '0');
        div *= 10;
      }
      d = (d << 8) / div;
    }
    return n+d;
  }

  void keyboard_command(char *command) {
    // If not the lead, send it to the lead.
    uint8_t b;
    accum88 arg = this->parse_number(command+1);
    
    switch (command[0]) {
      case 'd':
        this->setDebugging(!this->options.debugging);
        break;
      
      case '-':
        b = this->options.brightness;
        while (*command++ == '-')
          b -= 5;
        this->setBrightness(b - 5);
        break;
      case '+':
        b = this->options.brightness;
        while (*command++ == '+')
          b += 5;
        this->setBrightness(b + 5);
        return;
      case 'l':
        if (arg < 5*256) {
          Serial.println(F("nope"));
          return;
        }
        this->setBrightness(arg >> 8);
        return;

      case 'b':
        if (arg < 60*256) {
          Serial.println(F("nope"));
          return;
        }
        this->beats->set_bpm(arg);
        this->update_beat();
        this->send_update();
        return;

      case 's':
        this->beats->start_phrase();
        this->update_beat();
        this->send_update();
        return;

      case 'n':
        this->force_next();
        return;

      case 'p':
        this->next_state.pattern_phrase = 0;
        this->next_state.pattern_id = arg >> 8;
        this->next_state.pattern_sync_id = All;
        this->broadcast_state();
        return;        

      case '[':
        switch (this->options.fader) {
          case LEFT:
            this->options.fader = AUTO;
            break;

          case RIGHT:
            this->options.fader = MIDDLE;
            break;

          case MIDDLE:
          case AUTO:
          default:
            this->options.fader = LEFT;
            break;
        }
        this->broadcast_options();
        return;

      case ']':
        switch (this->options.fader) {
          case RIGHT:
            this->options.fader = AUTO;
            break;

          case LEFT:
            this->options.fader = MIDDLE;
            break;

          case MIDDLE:
          case AUTO:
          default:
            this->options.fader = RIGHT;
            break;
        }
        this->broadcast_options();
        return;

      case 'm':
        this->next_state.pattern_phrase = 0;
        this->next_state.pattern_id = this->current_state.pattern_id;
        this->next_state.pattern_sync_id = arg >> 8;
        this->broadcast_state();
        return;
        
      case 'c':
        this->next_state.palette_phrase = 0;
        this->next_state.palette_id = arg >> 8;
        this->broadcast_state();
        return;
        
      case 'e':
        this->next_state.effect_phrase = 0;
        this->next_state.effect_params = gEffects[(arg >> 8) % gEffectCount].params;
        this->broadcast_state();
        return;

      case '%':
        this->next_state.effect_phrase = 0;
        this->next_state.effect_params = this->current_state.effect_params;
        this->next_state.effect_params.chance = arg;
        this->broadcast_state();
        return;

      case 'g':
        for (int i=0; i< 10; i++)
          addGlitter();
        return;

      case '?':
        Serial.println(F("b###.# - set bpm"));
        Serial.println(F("s - start phrase"));
        Serial.println();
        Serial.println(F("p### - patterns"));
        Serial.println(F("m### - sync mode"));
        Serial.println(F("c### - colors"));
        Serial.println(F("e### - effects"));
        Serial.println();
        Serial.println(F("i### - set ID"));
        Serial.println(F("d - toggle debugging"));
        Serial.println(F("l### - brightness"));
        return;

      case 'U':
        WifiUpdater().web_update();
        return;

      case 0:
        // Empty command
        return;

      default:
        Serial.println("dunno?");
        return;
    }
  }

  void force_next() {
    uint16_t phrase = this->current_state.beat_frame >> 12;
    uint16_t next_phrase = min(this->next_state.pattern_phrase, min(this->next_state.palette_phrase, this->next_state.effect_phrase)) - phrase;
    this->next_state.pattern_phrase -= next_phrase;
    this->next_state.palette_phrase -= next_phrase;
    this->next_state.effect_phrase -= next_phrase;
    this->broadcast_state();
  }

  void broadcast_state() {
    this->node->sendCommand(COMMAND_UPDATE, &this->current_state, sizeof(TubeStates));
  }

  void broadcast_options() {
    this->node->sendCommand(COMMAND_OPTIONS, &this->options, sizeof(this->options));
  }

  virtual void onCommand(CommandId command, void *data) {
    switch (command) {
      case COMMAND_RESET:
        Serial.println(F("reset"));
        return;
  
      case COMMAND_BRIGHTNESS: {
        uint8_t *bright = (uint8_t *)data;
        this->setBrightness(*bright);
        Serial.println();
        return;
      }
  
      case COMMAND_OPTIONS:
        Serial.println(F("options"));
        memcpy(&this->options, data, sizeof(this->options));
        return;

      case COMMAND_UPDATE: {
        Serial.print(F(" update "));

        auto update_data = (TubeStates*)data;

        TubeState state;
        memcpy(&state, &update_data->current, sizeof(TubeState));
        memcpy(&this->next_state, &update_data->next, sizeof(TubeState));
        state.print();
        this->next_state.print();
        Serial.println();
  
        // Catch up to this state
        this->load_pattern(state);
        this->load_palette(state);
        this->load_effect(state);
        this->beats->sync(state.bpm, state.beat_frame);
        return;
      }
    }
  
    Serial.printf("UNKNOWN COMMAND %02X", command);
  }

};

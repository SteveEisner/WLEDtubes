#pragma once
#include <time.h>
#include "options.h"
#include "effects.h"

#define CURRENT_NODE_VERSION 2

#pragma pack(push,4) // set packing for consist transport across network
// ideally this would have been pack 1, so we're actually wasting a
// number of bytes across the network, but we've already shipped...

#define WLED_MESSAGE_MAX_SIZE 250

typedef enum CommandId: uint8_t {
    COMMAND_OPTIONS  = 0x10,
    COMMAND_STATE    = 0x20,
    COMMAND_ACTION   = 0x30,
    COMMAND_INFO     = 0x40,
    COMMAND_BEATS    = 0x50,
    COMMAND_KEYBOARD = 0x60,
    COMMAND_UPGRADE  = 0xE0
} CommandId;

typedef struct {
  char key;
  uint8_t arg;
} Action;



typedef enum{
    RECIPIENTS_ALL=0,  // Send to all neighbors; non-followers will ignore
    RECIPIENTS_ROOT=1, // Send to root for rebroadcasting downward, all will see
    RECIPIENTS_INFO=2, // Send to all neighbors "FYI"; none will ignore
} MessageRecipients;

typedef uint16_t MeshId;

typedef struct {
    MeshId id = 0;
    MeshId uplinkId = 0;
    uint8_t version = CURRENT_NODE_VERSION;
} MeshNodeHeader;

#define MESSAGE_DATA_SIZE 64
typedef struct {
    MeshNodeHeader header;
    MessageRecipients recipients;
    uint32_t timebase;
    CommandId command:8;
    uint8_t data[MESSAGE_DATA_SIZE] = {0};
} NodeMessage;

#pragma pack(pop)

typedef struct wizmote_message {
    uint8_t program;      // 0x91 for ON button, 0x81 for all others
    uint8_t seq[4];       // Incremetal sequence number 32 bit unsigned integer LSB first
    uint8_t byte5 = 32;   // Unknown
    uint8_t button;       // Identifies which button is being pressed
    uint8_t byte8 = 1;    // Unknown, but always 0x01
    uint8_t byte9 = 100;  // Unnkown, but always 0x64

    uint8_t byte10;  // Unknown, maybe checksum
    uint8_t byte11;  // Unknown, maybe checksum
    uint8_t byte12;  // Unknown, maybe checksum
    uint8_t byte13;  // Unknown, maybe checksum
} wizmote_message;



#pragma pack(push,1) // set packing for consist transport across network

// This macro is for backwards compatibility with the v2 protocol based on size of messages
#define WLED_MESSAGE_COMPATIBILITY_ASSERT(msg) static_assert(sizeof(msg) != sizeof(NodeMessage) && sizeof(msg) != sizeof(wizmote_message), # msg " message should not be the same size as NodeMessage")


struct WLED_Header {

    enum class ID : uint16_t {
        Keyboard = 0,
        RTC,
        GPS,
        WellknownPeer,
        LogMessage,
        Background,
        Effect,
        BPM,
        MAX_ID
    };

    static_assert(static_cast<uint16_t>(WLED_Header::ID::MAX_ID) <= 0x3FFF, "Too large of message id" );

    uint32_t header_ver = WLEDHeaderValue();
    uint16_t id:12;
    uint16_t sig:4;
    uint8_t  unused = 0;  // perhaps for nonce or checksum  - makes uint32_t aligned
    uint8_t  len;

    WLED_Header( ID i, uint8_t s = 0) : id(static_cast<uint16_t>(i) & 0x07ff), sig(s & 0x07) {}

    constexpr bool isHeaderValid() {
        return WLEDHeaderValue() == (header_ver & 0x7f7f7f7f);
    }

    constexpr uint32_t version() {
        return header_ver & 0x80808080;
    }

    constexpr bool isCurrentVersion() {
        return version() == 0x00000000;
    }

private:
    constexpr uint32_t WLEDHeaderValue() {
        return *((uint32_t*)"WLED");
    }

};

static_assert(sizeof(WLED_Header) == 2*sizeof(uint32_t), "WLED_Header is not 32bit aligned");

struct WLED_RTC : WLED_Header {
    WLED_RTC() : WLED_Header(WLED_Header::ID::RTC) {}
    timeval  tv;
};
WLED_MESSAGE_COMPATIBILITY_ASSERT(WLED_RTC);

struct WLED_Keyboard : WLED_Header {
    WLED_Keyboard() : WLED_Header(WLED_Header::ID::Keyboard) {}
    char keys[20] = {0};
};
WLED_MESSAGE_COMPATIBILITY_ASSERT(WLED_Keyboard);


struct WLED_GPS : WLED_Header {
    WLED_GPS() : WLED_Header(WLED_Header::ID::GPS) {}
    float lat;
    float lng;
};
WLED_MESSAGE_COMPATIBILITY_ASSERT(WLED_GPS);


struct WLED_WellknownPeer : WLED_Header {
    WLED_WellknownPeer() : WLED_Header(WLED_Header::ID::WellknownPeer) {}

    enum class Role: uint8_t {
        None,
        Master,
        LogReceiver,
        GPS,
    };

    uint8_t macaddr[6];
    uint8_t role;
    uint8_t unused;
};
WLED_MESSAGE_COMPATIBILITY_ASSERT(WLED_WellknownPeer);


struct WLED_LogMessage : WLED_Header {
    WLED_LogMessage() : WLED_Header(WLED_Header::ID::LogMessage) {}

    time_t tv; // TODO should we include time?
    char msg[0];
};
WLED_MESSAGE_COMPATIBILITY_ASSERT(WLED_LogMessage);


struct WLED_Background : WLED_Header {
    WLED_Background() : WLED_Header(WLED_Header::ID::Background) {}

    uint8_t paletteId;
    uint8_t patternId;
};
WLED_MESSAGE_COMPATIBILITY_ASSERT(WLED_Background);

struct WLED_Effect : WLED_Header {
    WLED_Effect() : WLED_Header(WLED_Header::ID::Effect) {}
    EffectDef effect;
};
WLED_MESSAGE_COMPATIBILITY_ASSERT(WLED_Effect);

struct WLED_BPM : WLED_Header {
    WLED_BPM() : WLED_Header(WLED_Header::ID::BPM) {}
    accum88 bpm;
};
WLED_MESSAGE_COMPATIBILITY_ASSERT(WLED_BPM);

struct WLED_Message {
    union U {
        WLED_Header header;
        WLED_RTC rtc;
        WLED_Keyboard keyboard;
        WLED_GPS gps;
        WLED_WellknownPeer peer;
        WLED_LogMessage log;
        WLED_Background background;
        WLED_Effect effect;
        WLED_BPM bpm;
    };

    U u;
};
static_assert(sizeof(WLED_Message) < WLED_MESSAGE_MAX_SIZE, "WLED_Message too large");

struct WLED_SignedMessage : WLED_Message{
    uint32_t hmac[8];
};

static_assert(sizeof(WLED_SignedMessage) < WLED_MESSAGE_MAX_SIZE, "WLED_SignedMessage too large");

#pragma pop(pack)


#pragma once

#include <Arduino.h>
#include "global_state.h"
#include "espnow_broadcast.h"

// #define NODE_DEBUGGING
// #define RELAY_DEBUGGING
#define TESTING_NODE_ID 0

#include "protocol.h"

typedef struct {
    uint8_t status;
    char message[40];
} NodeInfo;

#pragma once

const char *command_name(CommandId command) {
    switch (command) {
        case COMMAND_STATE:
            return "UPDATE";
        case COMMAND_OPTIONS:
            return "OPTIONS";
        case COMMAND_ACTION:
            return "ACTION";
        case COMMAND_INFO:
            return "INFO";
        case COMMAND_BEATS:
            return "BEATS";
        default:
            return "?COMMAND?";
    }
}

class MessageReceiver {
  public:
    virtual bool onCommand(CommandId command, void *data) = 0;
    virtual bool onButton(uint8_t button_id) = 0;
};

class LightNode {
  public:
    static LightNode* instance;

    MessageReceiver *receiver;
    MeshNodeHeader header;

    typedef enum{
        NODE_STATUS_QUIET=0,
        NODE_STATUS_RECEIVING,
        NODE_STATUS_STARTED,
        NODE_STATUS_MAX,        
    } NodeStatus;
    NodeStatus status = NODE_STATUS_QUIET;

    PGM_P status_code() const {
        switch (status) {
        case NODE_STATUS_QUIET:
            return PSTR(" (quiet)");
        case NODE_STATUS_RECEIVING:
            return PSTR(" (receiving)");
        case NODE_STATUS_STARTED:
            return PSTR(" (started)");
        default:
            return PSTR("??");
        }
    }

    char node_name[20];

    LightNode(MessageReceiver *r) : receiver(r) {
        instance = this;
    }

  protected:

    const uint32_t STATUS_TIMEOUT_BASE =  3000;    // Base time to wait to send broadcasts
    const uint32_t UPLINK_TIMEOUT      = 20000;    // Time at which uplink is presumed lost
    const uint32_t REBROADCAST_TIME    = 30000;    // Time at which followers are presumed re-uplinked

    Timer statusTimer;      // Use this timer to initialize and check wifi status
    Timer uplinkTimer;      // When this timer ends, assume uplink is lost.
    Timer rebroadcastTimer; // Until this timer ends, re-broadcast messages from uplink

    void onMeshChange() {
        sprintf(node_name,
            "Tube %03X:%03X",
            header.id,
            header.uplinkId
        );

        configureAP();
    }

    void configureAP() {
#ifdef DEFAULT_WIFI
        strcpy(clientSSID, DEFAULT_WIFI);
        strcpy(clientPass, DEFAULT_WIFI_PASSWORD);
#else
        // Don't connect to any networks.
        strcpy(clientSSID, "");
        strcpy(clientPass, "");
#endif

        // By default, we don't want these visible.
        apBehavior = AP_BEHAVIOR_BUTTON_ONLY; // Must press button for 6 seconds to get AP
    }

    void onPeerPing(const MeshNodeHeader& node) {
        // When receiving a message, if the IDs match, it's a conflict
        // Reset to create a new ID.
        if (node.id == header.id) {
            Serial.println("Detected an ID conflict.");
            reset();
        }

        // If the message arrives from a higher ID, switch into follower mode
        if (node.id > header.uplinkId && node.id > header.id) {
#ifdef RELAY_DEBUGGING
          // When debugging relay, pretend not to see any nodes above 0x800
          if (node->id < 0x800)
#endif
            follow(&node);
        }

        // If the message arrived from our uplink, track that we're still linked.
        if (node.id == header.uplinkId) {
            uplinkTimer.start(UPLINK_TIMEOUT);
        }

        // If a message indicates that another node is following this one, or
        // should be (it's not following anything, but this node's ID is higher)
        // enter or continue re-broadcasting mode.
        if (node.uplinkId == header.id
            || (node.uplinkId == 0 && node.id < header.id)) {
            Serial.printf("        %03X/%03X is following me", node.id, node.uplinkId);
            rebroadcastTimer.start(REBROADCAST_TIME);
        }
    }

    void printMessage(const NodeMessage* message, signed int rssi) const {
        Serial.printf("%03X/%03X %s",
            message->header.id,
            message->header.uplinkId,
            command_name(message->command)
        );
        if (message->recipients == RECIPIENTS_ROOT)
            Serial.printf(":ROOT");
        if (rssi)
            Serial.printf(" %ddB ", rssi);
    }

    void onPeerData(const uint8_t* address, const NodeMessage* message, uint8_t len, signed int rssi, bool broadcast) {
        // Track that another node exists, updating this node's understanding of the mesh.
        onPeerPing(message->header);

        bool ignore = false;
        switch (message->recipients) {
            case RECIPIENTS_ALL:
                // Ignore this message if not from the uplink
                ignore = (message->header.id != header.uplinkId);
                break;

            case RECIPIENTS_ROOT:
                // Ignore this message if not from one of this node's downlinks
                ignore = (message->header.uplinkId != header.id);
                break;

            case RECIPIENTS_INFO:
                ignore = false;
                break;

            default:
                // ignore this!
                ignore = true;
                break;
        }

        if (ignore) {
#ifdef NODE_DEBUGGING
            Serial.print("  -- ignored ");
            printMessage(message, rssi);
            Serial.println();
#endif
            return;
        }

        // Execute the received command
        if (message->recipients != RECIPIENTS_ROOT || !isFollowing()) {
            Serial.print("  >> ");
            printMessage(message, rssi);
            Serial.print(" ");

            // Adjust the timebase to match uplink
            // But only if it's drifting, else animations get jittery
            uint32_t new_timebase = message->timebase - millis() + 3; // Factor for network delay
            int32_t diff = new_timebase - strip.timebase;
            if (diff < -10 || diff > 10)
                strip.timebase = new_timebase;

            // Execute the command
            auto valid = receiver->onCommand(
                message->command,
                const_cast<uint8_t*>(message->data)
            );
            Serial.println();

            if (!valid)
                return;
        }

        // Re-broadcast the message if appropriate
        if (!rebroadcastTimer.ended() && message->recipients != RECIPIENTS_INFO) {
            static NodeMessage msg;
            memcpy(&msg, &message, len);
            msg.header = header;
            if (!isFollowing()) {
                msg.recipients = RECIPIENTS_ALL;
            }
#ifdef NODE_DEBUGGING
            Serial.println("rebroadcast");
#endif
            broadcastMessage(&msg, true);
        }
    }

    void broadcastMessage(NodeMessage *message, bool is_rebroadcast=false) {
        // Don't broadcast anything if this node isn't active.
        if (status != NODE_STATUS_STARTED) {
            if (status == NODE_STATUS_RECEIVING && statusTimer.ended()) {
                status = NODE_STATUS_STARTED;
                statusTimer.stop();
                Serial.printf("LightNode %s\n", status_code());
            } else {
                Serial.printf("broadcastMessage() - not started - %s\n", status_code());
                return;
            }
        }
        message->timebase = strip.timebase + millis();
        
#ifdef NODE_DEBUGGING
        Serial.print("  <<< ");
        printMessage(message, 0);
        Serial.println();
#endif

        __attribute__((unused)) auto success = espnowBroadcast.send((const uint8_t*)message, sizeof(*message));
#ifdef NODE_DEBUGGING
        if (!success) {
            Serial.println("espnowBroadcast.send() failed!");
        } else {
            Serial.println("successful broadcast");
        }
#endif

    }

  public:

    void sendCommand(CommandId command, void *data, uint8_t len) {
        // if (!ESP_NOW.isStarted()) {
        //     Serial.println("SendCommand ESP Not Started!");
        //     return;
        // }
        if (len > MESSAGE_DATA_SIZE) {
            Serial.printf("Message is too big: %d vs %d\n",
                len, MESSAGE_DATA_SIZE);
            return;
        }

        NodeMessage message;
        message.header = header;
        if (command == COMMAND_INFO) {
            message.recipients = RECIPIENTS_INFO;
        } else if (command == COMMAND_STATE) {
            message.recipients = RECIPIENTS_ALL;
        } else if (isFollowing()) {
            // Follower nodes must request that the root re-sends this message
            message.recipients = RECIPIENTS_ROOT;
        } else {
            message.recipients = RECIPIENTS_ALL;
        }
        message.command = command;
        memcpy(&message.data, data, len);
#ifdef NODE_DEBUGGING
        Serial.println("sendCommand");
#endif        
        broadcastMessage(&message);
    }

    void setup() {
#ifdef NODE_DEBUGGING
        reset(TESTING_NODE_ID);
#else
        reset();
#endif


#ifdef NODE_DEBUGGING
        delay(2000);
#endif

        espnowBroadcast.registerCallback(onEspNowMessage);

        espnowBroadcast.registerFilter(onEspNowFilter);


        Serial.println("setup: ok");
    }

    void update() {

        //process any wifi events to turn on/off ESPNode
        updateESPNowState();

        // Check the last time we heard from the uplink node
        if (isFollowing() && uplinkTimer.ended()) {
            follow(NULL);
        }

    }

    void reset(MeshId id = 0) {
        if (id == 0) {
            id = random(256, 4000);  // Leave room at bottom and top of 12 bits
        }
        header.id = id;
        follow(NULL);
    }

    void follow(const MeshNodeHeader* node) {
        if (node == NULL) {
            if (header.uplinkId != 0) {
                Serial.println("Uplink lost");
            }

            // Unfollow: following zero means you have no uplink
            header.uplinkId = 0;
            onMeshChange();
            return;
        }

        // Already following? ignore
        if (header.uplinkId == node->id)
            return;

        // Follow
        Serial.printf("Following %03X:%03X\n",
            node->id,
            node->uplinkId
        );
        header.uplinkId = node->id;
        onMeshChange();
    }

    bool isFollowing() const {
        return header.uplinkId != 0;
    }

protected:

    void updateESPNowState() {
        auto state = espnowBroadcast.getState();
        static auto prev = espnowBroadcast.STOPPED;
        switch(state) {
            case ESPNOWBroadcast::STOPPED:
                if (NODE_STATUS_QUIET != status) {
                    Serial.printf("updateESPNowState() - %d node_status:%s\n", state, status_code());
                    status = NODE_STATUS_QUIET;
                    rebroadcastTimer.stop();
                    Serial.printf("LightNode %s\n", status_code());
                }
                break;
            case ESPNOWBroadcast::STARTING: {}
                if ( state != prev ) {
                    Serial.printf("updateESPNowState() - %d node_status:%s\n", state, status_code());
                }
                break;
            case ESPNOWBroadcast::STARTED:
                if (NODE_STATUS_QUIET == status) {
                    Serial.printf("updateESPNowState() - %d node_status:%s\n", state, status_code());
                    status = NODE_STATUS_RECEIVING;
                    statusTimer.start(STATUS_TIMEOUT_BASE - header.id / 2);
                    Serial.printf("LightNode %s\n", status_code());
                }
                break;
            default:
                break;
        }
        prev = state;
    }

    void onWizmote(const uint8_t* address, const wizmote_message* data, uint8_t len) {
        static uint32_t last_seq = 0;
        uint32_t cur_seq = data->seq[0] | (data->seq[1] << 8) | (data->seq[2] << 16) | (data->seq[3] << 24);
        if (cur_seq == last_seq)
            return;
        last_seq = cur_seq;

        receiver->onButton(data->button);
    }

    static void onEspNowMessage(const uint8_t *address, const uint8_t *msg, uint8_t len, int8_t rssi) {
        // basic length and field checking has been done in onEspNowFilter
        if (msg) {
            if(len == sizeof(NodeMessage)) {
                instance->onPeerData(address, (const NodeMessage*)msg, len, rssi, true);
            } else if (len == sizeof(wizmote_message)) {
                instance->onWizmote(address, (const wizmote_message*)msg, len);
            } else {
                auto wled = (const WLED_Message*)msg;
                switch(static_cast<WLED_Header::ID>(wled->u.header.id)) {
                    case WLED_Header::ID::Keyboard: {
                        Action action = { 'K', 0};
                        instance->receiver->onCommand(
                            COMMAND_KEYBOARD,
                            (void*)(wled->u.keyboard.keys)
                        );
                        }
                        break;
                    case WLED_Header::ID::Effect: {
                        Action action = { 'G', 0};
                        instance->receiver->onCommand(
                            COMMAND_ACTION,
                            &action
                        );
                        }
                        break;
                    case WLED_Header::ID::BPM:
                        instance->receiver->onCommand(
                            COMMAND_BEATS,
                            (void*)&(wled->u.bpm.bpm)
                        );
                        break;
                    default:
                        break;
                }
            }
        }
    }

    static bool onEspNowFilter(const uint8_t *address, const uint8_t *msg, uint8_t len, int8_t rssi) {
        if (len == sizeof(NodeMessage)) {
            return ((const NodeMessage*)msg)->header.version == instance->header.version;
        } else if (len == sizeof(wizmote_message)) {
            auto wizmote = (const wizmote_message*)msg;
            return !( wizmote->byte8 != 1 || wizmote->byte9 != 100 || wizmote->byte5 != 32);
        } else {
            auto wled = (const WLED_Message*)msg;
            if  (!wled->u.header.isHeaderValid()) {
                return false;
            }

            if (!wled->u.header.isCurrentVersion()){
                return false;
            }

            if (wled->u.header.len > len) {
                return false;
            }

            if (wled->u.header.sig != 0) {
                // currently signing is not supported
                return false;
            }

            if (wled->u.header.id >= static_cast<uint16_t>(WLED_Header::ID::MAX_ID)) {
                return false;
            }

            switch(static_cast<WLED_Header::ID>(wled->u.header.id)) {
                case WLED_Header::ID::RTC: {
                    static auto rtc = false;
                    if (!rtc) {
                        settimeofday(&(wled->u.rtc.tv), nullptr);
                        rtc = true;
                    } else {
                        adjtime(&(wled->u.rtc.tv), nullptr);
                    }
                    return false;
                    }
                    break;
                default:
                    return true;
            }

        }
        return false;
    }
};

LightNode* LightNode::instance = nullptr;


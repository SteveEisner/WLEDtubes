#pragma once

#include <Arduino.h>
#if defined ESP32
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <freertos/ringbuf.h>

#elif defined ESP8266
#include <ESP8266WiFi.h>
#define WIFI_MODE_STA WIFI_STA 
#else
#error "Unsupported platform"
#endif //ESP32
#include "global_state.h"

#ifdef USERMOD_WLED_TUBES_OLED
#undef BLACK
#undef WHITE
#include "SSD1306Wire.h"
SSD1306Wire display(0x3c, 21, 22);   ins for Dig2Go
#endif

//#define NODE_DEBUGGING
// #define RELAY_DEBUGGING
#define TESTING_NODE_ID 0

#define CURRENT_NODE_VERSION 2

#pragma pack(push,4) // set packing for consist transport across network
// ideally this would have been pack 1, so we're actually wasting a
// number of bytes across the network, but we've already shipped...

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
    CommandId command;
    byte data[MESSAGE_DATA_SIZE] = {0};
} NodeMessage;
#pragma pack(pop)

typedef struct {
    uint8_t mac[6];
    uint8_t len;
    NodeMessage msg;
} QueuedNodeMessage;

typedef struct {
    uint8_t status;
    char message[40];
} NodeInfo;

#define BROADCAST_ADDR_ARRAY_INITIALIZER {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

#define ESPNOW_WIFI_CHANNEL 0

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
    virtual bool onCommand(CommandId command, void *data) {
      // Abstract: subclasses must define
      return false;
    }  
    virtual bool onButton(uint8_t button_id) {
      // Abstract: subclasses must define
      return false;
    }  
};

class LightNode {
  public:
    static LightNode* instance;

    MessageReceiver *receiver;
    MeshNodeHeader header;

    typedef enum{
        NODE_STATUS_QUIET=0,
        NODE_STATUS_STARTING,
        NODE_STATUS_RECEIVING,
        NODE_STATUS_STARTED,
        NODE_STATUS_MAX,        
    } NodeStatus;
    NodeStatus status = NODE_STATUS_QUIET;

    PGM_P status_code() {
        switch (status) {
        case NODE_STATUS_QUIET:
            return PSTR(" (quiet)");
        case NODE_STATUS_STARTING:
            return PSTR(" (starting)");
        case NODE_STATUS_RECEIVING:
            return PSTR(" (receiving)");
        case NODE_STATUS_STARTED:
            return PSTR("");
        default:
            return PSTR("??");
        }
    }

    char node_name[20];

    LightNode(MessageReceiver *r) : receiver(r) {
        instance = this;
        _wifiEvents = xRingbufferCreateNoSplit(sizeof(WiFiEvent_t), 4);
        _networkRxMessages = xRingbufferCreateNoSplit(sizeof(QueuedNodeMessage), MAX_QUEUED_NETWORK_MESSAGES);
    }

  protected:
    const size_t MAX_QUEUED_NETWORK_MESSAGES = 20; // number of messages to queue from WiFi Task to our core thread

    const uint32_t STATUS_CHECK_RATE =   200;    // Time at which we should check wifi status again
    const uint32_t UPLINK_TIMEOUT    = 20000;    // Time at which uplink is presumed lost
    const uint32_t REBROADCAST_TIME  = 30000;    // Time at which followers are presumed re-uplinked

    Timer statusTimer;      // Use this timer to initialize and check wifi status
    Timer uplinkTimer;      // When this timer ends, assume uplink is lost.
    Timer rebroadcastTimer; // Until this timer ends, re-broadcast messages from uplink



    RingbufHandle_t _wifiEvents;
    RingbufHandle_t _networkRxMessages;

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

    void setupESPNow() {
        // To enable ESPNow, we need to be in WIFI_STA mode
        if ( !WiFi.mode(WIFI_STA) ) {
            Serial.println("WiFi.mode() failed");
        }
        // and not have the WiFi connect
        // Calling discount with tigger an async Wifi Event
        if ( !WiFi.disconnect(false, true) ) {
            Serial.println("WiFi.disconnect() failed");
        }
    }

    void startESPNow() {
        statusTimer.stop();
        status = NODE_STATUS_STARTING;

        Serial.println("starting ESPNow");

        if ( WiFi.mode(WIFI_STA) ) {
            if ( WiFi.status() == WL_DISCONNECTED ) {
                if (esp_wifi_start() == ESP_OK) {
                    if (esp_now_init() == ESP_OK) {
                        if (esp_now_register_recv_cb(onESPNowRxCallback) == ESP_OK) {
                            static esp_now_peer_info_t peer = {
                                BROADCAST_ADDR_ARRAY_INITIALIZER,
                                {0},
                                ESPNOW_WIFI_CHANNEL,
                                WIFI_IF_STA,
                                false,
                                NULL
                                };

                            if (esp_now_add_peer(&peer) == ESP_OK) {
                                // Initialization timer: wait for a bit before trying to broadcast.
                                // If this node's ID is high, it's more likely to be the leader, so wait less.
                                status = NODE_STATUS_RECEIVING;
                                statusTimer.start(3000 - header.id / 2);
                                Serial.println("ESPNow receiving");
                                rebroadcastTimer.stop();
                                return;

                            } else {
#ifdef NODE_DEBUGGING
                                Serial.println("esp_now_add_peer failed");
#endif
                            }
                        } else {
#ifdef NODE_DEBUGGING
                            Serial.println("esp_now_register_recv_cb failed");
#endif
                        }
                    } else {
#ifdef NODE_DEBUGGING
                        Serial.println("esp_now_init_init failed");
#endif
                    }
                } else {
#ifdef NODE_DEBUGGING
                    Serial.println("esp_wifi_start failed");
#endif
                }
            } else {
#ifdef NODE_DEBUGGING
                Serial.println("WiFi.status not disconnected");
#endif
            }
        } else {
#ifdef NODE_DEBUGGING
            Serial.println("WiFi.mode failed");
#endif
        }


        Serial.println("restarting ESPNow");
        status = NODE_STATUS_QUIET;
        setupESPNow();

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

    void printMessage(const NodeMessage* message, signed int rssi) {
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
        // Ignore this message if it isn't a valid message payload.
        if (len != sizeof(*message))
            return;

        // Ignore this message if it's the wrong version.
        if (message->header.version != header.version) {
#ifdef NODE_DEBUGGING
            Serial.print("  -- !version ");
            printMessage(message, rssi);
            Serial.println();
#endif
            return;
        }

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
            if (status == NODE_STATUS_RECEIVING && statusTimer.every(STATUS_CHECK_RATE)) {
                Serial.println("ESPNow started: receiving and broadcasting");
                status = NODE_STATUS_STARTED;
                statusTimer.stop();
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

        static_assert(sizeof(*message) < ESP_NOW_MAX_DATA_LEN, "NodeMessage to large to send");

        static const uint8_t broadcast[] = BROADCAST_ADDR_ARRAY_INITIALIZER;

        auto err = esp_now_send(broadcast, (const uint8_t*)message, sizeof(*message));

#ifdef NODE_DEBUGGING
        if (err != ESP_OK) {
            Serial.printf("esp_now_send() failed: %d\n", err);
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

        // Initialising the UI will init the display too.
#ifdef USERMOD_WLED_TUBES_OLED
        display.init();
        display.flipScreenVertically();
        display.setFont(ArialMT_Plain_10);
        display.setTextAlignment(TEXT_ALIGN_LEFT);
#endif

#ifdef NO_DEBUGGING
        delay(3000);
#endif

        WiFi.onEvent(queueWiFiEvent, ARDUINO_EVENT_WIFI_STA_START);
        WiFi.onEvent(queueWiFiEvent, ARDUINO_EVENT_WIFI_STA_STOP);
        WiFi.onEvent(queueWiFiEvent, ARDUINO_EVENT_WIFI_AP_START);

        setupESPNow();

        Serial.println("setup: ok");
    }

    void update() {

        //process any wifi events to turn on/off ESPNode
        onWiFiEvent();

        onDataReceived();

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

    bool isFollowing() {
        return header.uplinkId != 0;
    }

protected:

    static void queueWiFiEvent(WiFiEvent_t event) {
#ifdef NODE_DEBUGGING
        Serial.printf("WiFiEvent( %S )\n", WiFi.eventName(event) );
#endif
        switch(event) {
            case ARDUINO_EVENT_WIFI_STA_START:
            case ARDUINO_EVENT_WIFI_STA_STOP:
            case ARDUINO_EVENT_WIFI_AP_START:
                xRingbufferSend(instance->_wifiEvents, &event, sizeof(event), 0);
                break;
        }
    }

    void onWiFiEvent() {
        size_t item_size = 0;
        auto *event = (WiFiEvent_t *)xRingbufferReceive(_wifiEvents, &item_size, 0);
        if (event) {
            if (item_size == sizeof(*event)) {
                switch(*event) {
                    case ARDUINO_EVENT_WIFI_STA_START:
                        Serial.printf("onStartESPNow() - %s\n", status_code());
                        if (status == NODE_STATUS_QUIET) {
                            startESPNow();
                        }
                        break;
                    case ARDUINO_EVENT_WIFI_STA_STOP:
                    case ARDUINO_EVENT_WIFI_AP_START:
                        Serial.printf("onStopESPNow() - %s\n", status_code());
                        if (status != NODE_STATUS_QUIET) {
                            Serial.println("WiFi connected: stop broadcasting");
                            esp_err_t err = esp_now_deinit();
                            status = NODE_STATUS_QUIET;
                            rebroadcastTimer.stop();
                        }
                        break;
                }
            } else {
#ifdef NODE_DEBUGGING
                Serial.printf("wrong size WiFiEvent_t received %d\n", item_size);
#endif
            }
            vRingbufferReturnItem(_wifiEvents, (void *)event);
        }
    }

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

    void onWizmote(const uint8_t* address, const wizmote_message* data, uint8_t len) {
        // First make sure this is a WizMote message.
        if (len != sizeof(wizmote_message) || data->byte8 != 1 || data->byte9 != 100 || data->byte5 != 32)
            return;

        static uint32_t last_seq = 0;
        uint32_t cur_seq = data->seq[0] | (data->seq[1] << 8) | (data->seq[2] << 16) | (data->seq[3] << 24);
        if (cur_seq == last_seq)
            return;
        last_seq = cur_seq;

        receiver->onButton(data->button);
    }

    void onDataReceived() {
        size_t item_size = 0;
        auto *msg = (QueuedNodeMessage*)xRingbufferReceive(_networkRxMessages, &item_size, 0);
        if (msg) {
            if(item_size == sizeof(*msg)) {
                onPeerData(msg->mac, &(msg->msg), msg->len, 0, true);
                onWizmote(msg->mac, (wizmote_message*)(&(msg->msg)), msg->len);
            } else {
#ifdef NODE_DEBUGGING
                Serial.printf("wrong size QueueNodeMessage received %d\n", item_size);
#endif
            }
            vRingbufferReturnItem(_networkRxMessages, (void *)msg);
        }
    }

    static void onESPNowRxCallback(const uint8_t *mac_addr, const uint8_t *data, int len) {
        QueuedNodeMessage* msg = nullptr;
        if (len <= sizeof(msg->msg)) {
            if (pdTRUE == xRingbufferSendAcquire(instance->_networkRxMessages, (void**)&msg, sizeof(*msg), 0) ) {
                memcpy(msg->mac, mac_addr, sizeof(msg->mac));
                memcpy(&(msg->msg), data, len);
                msg->len = len;
                xRingbufferSendComplete(instance->_networkRxMessages, msg);
            } else {
                Serial.println("Failed to aquire ring buffer.  Dropping network message");
            }
        } else {
#ifdef NODE_DEBUGGING
            Serial.printf("Receive to large of packet %d bytes. ignoring...\n", len);
#endif
        }
    }
};

LightNode* LightNode::instance = nullptr;


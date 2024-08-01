
#ifndef WLED_DISABLE_ESPNOW_NEW
#include <espnow_broadcast.h>

#if defined ESP32
#include <WiFi.h>

#elif defined ESP8266
#include <ESP8266WiFi.h>
#define WIFI_MODE_STA WIFI_STA
#else
#error "Unsupported platform"
#endif //ESP32

#include <esp_wifi.h>
#include <esp_wifi_types.h>
#include <esp_now.h>
#include <freertos/ringbuf.h>

#ifndef WIFI_EVENT_STA_START // Legacy Event Loop
ESP_EVENT_DECLARE_BASE(WIFI_EVENT);
#define WIFI_EVENT_STA_START SYSTEM_EVENT_STA_START
#define WIFI_EVENT_STA_STOP SYSTEM_EVENT_STA_STOP
#define WIFI_EVENT_AP_START SYSTEM_EVENT_AP_START
#define wifi_event_handler_register(eventId, eventHandler) esp_event_handler_register(WIFI_EVENT, eventId, eventHandler, nullptr)
#else
#define wifi_event_handler_register(eventId, eventHandler) esp_event_handler_instance_register(WIFI_EVENT, eventId, eventHandler, nullptr, nullptr)
#endif

esp_event_base_t x;
#define BROADCAST_ADDR_ARRAY_INITIALIZER {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
#define WLED_ESPNOW_WIFI_CHANNEL 1

typedef struct {
    uint8_t mac[6];
    uint8_t len;
    uint8_t data[WLED_ESPNOW_MAX_MESSAGE_LENGTH];
} QueuedNetworkMessage;
static_assert(WLED_ESPNOW_MAX_MESSAGE_LENGTH <= ESP_NOW_MAX_DATA_LEN, "WLED_ESPNOW_MAX_MESSAGE_LENGTH must be <= 250 bytes");

class QueuedNetworkRingBuffer {
  protected:
    //QueuedNetworkMessage messages[WLED_ESPNOW_MAX_QUEUED_MESSAGES];
    RingbufHandle_t buf = nullptr;

  public:
    QueuedNetworkRingBuffer() {
        buf = xRingbufferCreateNoSplit(sizeof(QueuedNetworkMessage), WLED_ESPNOW_MAX_QUEUED_MESSAGES);
    }

    bool push(const uint8_t* mac, const uint8_t* data, uint8_t len) {
        QueuedNetworkMessage* msg = nullptr;
        if (len <= sizeof(msg->data) &&
            pdTRUE == xRingbufferSendAcquire(buf, (void**)&msg, sizeof(*msg), 0)) {
                memcpy(msg->mac, mac, sizeof(msg->mac));
                memcpy(&(msg->data), data, len);
                msg->len = len;
                xRingbufferSendComplete(buf, msg);
                return true;
        }
        return false;
    }

    QueuedNetworkMessage* pop() {
        size_t size = 0;
        return (QueuedNetworkMessage*)xRingbufferReceive(buf, &size, 0);
    }

    void popComplete(QueuedNetworkMessage* msg) {
        vRingbufferReturnItem(buf, (void *)msg);
    }
};

QueuedNetworkRingBuffer queuedNetworkRingBuffer {};

ESPNOWBroadcast espnowBroadcast;

bool ESPNOWBroadcast::setup() {

    auto err = esp_event_loop_create_default();
    if ( ESP_OK != err && ESP_ERR_INVALID_STATE != err ) {
        return false;
    }
    err = wifi_event_handler_register(WIFI_EVENT_STA_START, onWiFiEvent);
    if ( ESP_OK != err ) {
        return false;
    }
    err = wifi_event_handler_register(WIFI_EVENT_STA_STOP, onWiFiEvent);
    if ( ESP_OK != err ) {
        return false;
    }
    err = wifi_event_handler_register(WIFI_EVENT_AP_START, onWiFiEvent);
    if ( ESP_OK != err ) {
        return false;
    }

    return setupWiFi();
}

bool ESPNOWBroadcast::setupWiFi() {
    Serial.println("ESPNOWBroadcast::setupWiFi()");

    _state.exchange(STOPPED);

    // To enable ESPNow, we need to be in WIFI_STA mode
    if ( !WiFi.mode(WIFI_STA) ) {
        Serial.println("WiFi.mode() failed");
        return false;
    }
    // and not have the WiFi connect
    // Calling discount with tigger an async Wifi Event
    if ( !WiFi.disconnect(false, true) ) {
        Serial.println("WiFi.disconnect() failed");
        return false;
    }

    return true;
}



void ESPNOWBroadcast::loop(size_t maxMessagesToProcess /*= 1*/) {
    switch (espnowBroadcast._state.load()) {
        case ESPNOWBroadcast::STARTING:
            // if WiFI is in starting state, actually stat ESPNow from our main task thread.
            start();
            break;
        case ESPNOWBroadcast::STARTED: {
            auto ndx = maxMessagesToProcess;
            while(ndx-- > 0) {
                size_t size = 0;
                auto *msg = queuedNetworkRingBuffer.pop();
                if (msg) {
                    auto callback = _rxCallback;
                    while( *callback ) {
                        (*callback)(msg->mac, msg->data, msg->len);
                        callback++;
                    }
                    queuedNetworkRingBuffer.popComplete(msg);
                } else {
                    break;
                }
            }
            break;
        }
        default:
            break;
    }
}

ESPNOWBroadcast::err_t ESPNOWBroadcast::send(const uint8_t* msg, size_t len) {
    static const uint8_t broadcast[] = BROADCAST_ADDR_ARRAY_INITIALIZER;
    return static_cast<ESPNOWBroadcast::err_t>(esp_now_send(broadcast, msg, len));
}

void ESPNOWBroadcast::start() {

    Serial.println("starting ESPNow");

    if ( WiFi.mode(WIFI_STA) ) {
        if ( WiFi.status() == WL_DISCONNECTED ) {
            if (esp_wifi_start() == ESP_OK) {
                if (esp_now_init() == ESP_OK) {
                    if (esp_now_register_recv_cb(onESPNowRxCallback) == ESP_OK) {
                        static esp_now_peer_info_t peer = {
                            BROADCAST_ADDR_ARRAY_INITIALIZER,
                            {0},
                            WLED_ESPNOW_WIFI_CHANNEL,
                            WIFI_IF_STA,
                            false,
                            NULL
                            };

                        if (esp_now_add_peer(&peer) == ESP_OK) {
                            ESPNOWBroadcast::STATE starting {ESPNOWBroadcast::STARTING};
                            if (espnowBroadcast._state.compare_exchange_strong(starting, ESPNOWBroadcast::STARTED)) {
                                return;
                            } else {
#ifdef NODE_DEBUGGING
                                Serial.println("atomic state out of sync");
#endif
                            }
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
    setupWiFi();
}


void ESPNOWBroadcast::onWiFiEvent(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if ( event_base == WIFI_EVENT ) {
#ifdef NODE_DEBUGGING
        Serial.printf("WiFiEvent( %S )\n", WiFi.eventName(event) );
#endif
        switch (event_id) {
            case WIFI_EVENT_STA_START: {
                ESPNOWBroadcast::STATE stopped {ESPNOWBroadcast::STOPPED};
                espnowBroadcast._state.compare_exchange_strong(stopped, ESPNOWBroadcast::STARTING);
                break;
            }

            case WIFI_EVENT_STA_STOP:
            case WIFI_EVENT_AP_START: {
                ESPNOWBroadcast::STATE started {ESPNOWBroadcast::STARTED};
                ESPNOWBroadcast::STATE starting {ESPNOWBroadcast::STARTING};
                if (espnowBroadcast._state.compare_exchange_strong(started, ESPNOWBroadcast::STOPPED) ||
                    espnowBroadcast._state.compare_exchange_strong(starting, ESPNOWBroadcast::STOPPED)) {
#ifdef NODE_DEBUGGING
                    Serial.println("WiFi connected: stop broadcasting");
#endif
                    __attribute__((unused)) esp_err_t err = esp_now_deinit();
                }
                break;
            }
        }
    }
}

bool ESPNOWBroadcast::registerCallback( receive_callback_t callback ) {
    // last element is always null
    size_t ndx;
    for (ndx = 0; ndx < _rxCallbackSize-1; ndx++) {
        if (nullptr == _rxCallback[ndx]) {
            _rxCallback[ndx] = callback;
            break;
        }
    }
    return ndx < _rxCallbackSize;
}

bool ESPNOWBroadcast::removeCallback( receive_callback_t callback ) {
    size_t ndx;
    for (ndx = 0; ndx < _rxCallbackSize-1; ndx++) {
        if (_rxCallback[ndx] == callback ) {
            break;
        }
    }

    for (; ndx < _rxCallbackSize-1; ndx++) {
        _rxCallback[ndx] = _rxCallback[ndx+1];
    }

    return ndx < _rxCallbackSize;

}

void ESPNOWBroadcast::onESPNowRxCallback(const uint8_t *mac, const uint8_t *data, int len) {
    if (!queuedNetworkRingBuffer.push(mac, data, len)) {
        if (len > sizeof(WLED_ESPNOW_MAX_MESSAGE_LENGTH)) {
#ifdef NODE_DEBUGGING
            Serial.printf("Receive to large of packet %d bytes. ignoring...\n", len);
#endif
        } else {
            Serial.println("Failed to aquire ring buffer.  Dropping network message");
        }
    }
}

#endif
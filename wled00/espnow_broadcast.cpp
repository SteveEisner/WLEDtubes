
#ifndef WLED_DISABLE_ESPNOW_NEW
#include <Arduino.h>
#include <atomic>

#if defined ESP32
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>

#elif defined ESP8266
#include <ESP8266WiFi.h>
#define WIFI_MODE_STA WIFI_STA
#else
#error "Unsupported platform"
#endif //ESP32

#include "espnow_broadcast.h"

#include <esp_now.h>
#include <esp_idf_version.h>
#include <freertos/ringbuf.h>

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 0, 0)
// Legacy Event Loop
ESP_EVENT_DEFINE_BASE(SYSTEM_EVENT);
#define WIFI_EVENT SYSTEM_EVENT
#define WIFI_EVENT_STA_START SYSTEM_EVENT_STA_START
#define WIFI_EVENT_STA_STOP SYSTEM_EVENT_STA_STOP
#define WIFI_EVENT_AP_START SYSTEM_EVENT_AP_START
#endif

//#define ESPNOW_DEBUGGING

#define BROADCAST_ADDR_ARRAY_INITIALIZER {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
#define WLED_ESPNOW_WIFI_CHANNEL 1

typedef struct {
    uint8_t mac[6];
    uint8_t len;
    uint8_t data[WLED_ESPNOW_MAX_MESSAGE_LENGTH];
} QueuedNetworkMessage;
static_assert(WLED_ESPNOW_MAX_MESSAGE_LENGTH <= ESP_NOW_MAX_DATA_LEN, "WLED_ESPNOW_MAX_MESSAGE_LENGTH must be <= 250 bytes");


class ESPNOWBroadcastImpl : public ESPNOWBroadcast {

    friend ESPNOWBroadcast;

    std::atomic<STATE> _state {STOPPED};
    STATE getState() {
        return _state.load();
    }

    bool setupWiFi();

    void start();

    static esp_err_t onSystemEvent(void *ctx, system_event_t *event);

    static void onWiFiEvent(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

    static void onESPNowRxCallback(const uint8_t *mac_addr, const uint8_t *data, int len);

    class QueuedNetworkRingBuffer {
      protected:
        //QueuedNetworkMessage messages[WLED_ESPNOW_MAX_QUEUED_MESSAGES];
        RingbufHandle_t buf = nullptr;

      public:
        QueuedNetworkRingBuffer() {
            buf = xRingbufferCreateNoSplit(sizeof(QueuedNetworkMessage), WLED_ESPNOW_MAX_QUEUED_MESSAGES);
        }

        bool push(const uint8_t* mac, const uint8_t* data, uint8_t len);

        QueuedNetworkMessage* pop() {
            size_t size = 0;
            return (QueuedNetworkMessage*)xRingbufferReceive(buf, &size, 0);
        }

        void popComplete(QueuedNetworkMessage* msg) {
            vRingbufferReturnItem(buf, (void *)msg);
        }
    };

    QueuedNetworkRingBuffer queuedNetworkRingBuffer {};

};

ESPNOWBroadcastImpl espnow {};


ESPNOWBroadcast& ESPNOWBroadcast::instance() {
    return espnow;
}

ESPNOWBroadcast::STATE ESPNOWBroadcast::getState() {
    return espnow.getState();
}

bool ESPNOWBroadcast::setup() {

    static bool setup = false;
    if (setup) {
        return true;
    }

    #ifdef ESPNOW_DEBUGGING
        delay(2000);
    #endif

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 0, 0)
    tcpip_adapter_init();
    esp_event_loop_init(onSystemEvent, nullptr);
#else

    auto err = esp_event_loop_create_default();
    if ( ESP_OK != err && ESP_ERR_INVALID_STATE != err ) {
        Serial.printf("esp_event_loop_create_default() err %d\n", err);
        return false;
    }
    err = esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_START, ESPNOWBroadcastImpl::onWiFiEvent, nullptr, nullptr);
    if ( ESP_OK != err ) {
        Serial.printf("esp_event_handler_instance_register(WIFI_EVENT_STA_START) err %d\n", err);
        return false;
    }
    err = esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_STOP, ESPNOWBroadcastImpl::onWiFiEvent, nullptr, nullptr);
    if ( ESP_OK != err ) {
        Serial.printf("esp_event_handler_instance_register(WIFI_EVENT_STA_STOP) err %d\n", err);
        return false;
    }
    err = esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_AP_START, ESPNOWBroadcastImpl::onWiFiEvent, nullptr, nullptr);
    if ( ESP_OK != err ) {
        Serial.printf("esp_event_handler_instance_register(WIFI_EVENT_AP_START) err %d\n", err);
        return false;
    }
#endif

    setup = espnow.setupWiFi();
    return setup;
}

bool ESPNOWBroadcastImpl::setupWiFi() {
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
    switch (espnow._state.load()) {
        case ESPNOWBroadcast::STARTING:
            // if WiFI is in starting state, actually stat ESPNow from our main task thread.
            espnow.start();
            break;
        case ESPNOWBroadcast::STARTED: {
            auto ndx = maxMessagesToProcess;
            while(ndx-- > 0) {
                auto *msg = espnow.queuedNetworkRingBuffer.pop();
                if (msg) {
                    auto callback = _rxCallbacks;
                    while( *callback ) {
                        (*callback)(msg->mac, msg->data, msg->len);
                        callback++;
                    }
                    espnow.queuedNetworkRingBuffer.popComplete(msg);
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

bool ESPNOWBroadcast::send(const uint8_t* msg, size_t len) {
    static const uint8_t broadcast[] = BROADCAST_ADDR_ARRAY_INITIALIZER;
    return ERR_OK == esp_now_send(broadcast, msg, len);
}

bool ESPNOWBroadcast::registerCallback( ESPNOWBroadcast::receive_callback_t callback ) {
    // last element is always null
    size_t ndx;
    for (ndx = 0; ndx < _rxCallbacksSize-1; ndx++) {
        if (nullptr == _rxCallbacks[ndx]) {
            _rxCallbacks[ndx] = callback;
            break;
        }
    }
    return ndx < _rxCallbacksSize;
}

bool ESPNOWBroadcast::removeCallback( ESPNOWBroadcast::receive_callback_t callback ) {
    size_t ndx;
    for (ndx = 0; ndx < _rxCallbacksSize-1; ndx++) {
        if (_rxCallbacks[ndx] == callback ) {
            break;
        }
    }

    for (; ndx < _rxCallbacksSize-1; ndx++) {
        _rxCallbacks[ndx] = _rxCallbacks[ndx+1];
    }

    return ndx < _rxCallbacksSize;

}

void ESPNOWBroadcastImpl::start() {

    Serial.println("starting ESPNow");

    if ( WiFi.mode(WIFI_STA) ) {
        auto status = WiFi.status();
        if ( status >= WL_DISCONNECTED ) {
            if (esp_wifi_start() == ESP_OK) {
                if (esp_now_init() == ESP_OK) {
                    if (esp_now_register_recv_cb(ESPNOWBroadcastImpl::onESPNowRxCallback) == ESP_OK) {
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
                            if (_state.compare_exchange_strong(starting, ESPNOWBroadcast::STARTED)) {
                                return;
                            } else {
#ifdef ESPNOW_DEBUGGING
                                Serial.println("atomic state out of sync");
#endif
                            }
                        } else {
#ifdef ESPNOW_DEBUGGING
                            Serial.println("esp_now_add_peer failed");
#endif
                        }
                    } else {
#ifdef ESPNOW_DEBUGGING
                        Serial.println("esp_now_register_recv_cb failed");
#endif
                    }
                } else {
#ifdef ESPNOW_DEBUGGING
                    Serial.println("esp_now_init_init failed");
#endif
                }
            } else {
#ifdef ESPNOW_DEBUGGING
                Serial.println("esp_wifi_start failed");
#endif
            }
        } else {
#ifdef ESPNOW_DEBUGGING
            Serial.printf("WiFi.status not disconnected - %d\n", status);
#endif
        }
    } else {
#ifdef ESPNOW_DEBUGGING
        Serial.println("WiFi.mode failed");
#endif
    }
    Serial.println("restarting ESPNow");
    setupWiFi();
}

esp_err_t ESPNOWBroadcastImpl::onSystemEvent(void *ctx, system_event_t *event) {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 0, 0)
    onWiFiEvent(ctx, SYSTEM_EVENT, event->event_id, nullptr );
#endif
    return ESP_OK;
}


void ESPNOWBroadcastImpl::onWiFiEvent(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if ( event_base == WIFI_EVENT ) {

#ifdef ESPNOW_DEBUGGING
    #if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 0, 0)
            Serial.printf("WiFiEvent( %d )\n", event_id );
    #else
            Serial.printf("WiFiEvent( %s )\n", WiFi.eventName((arduino_event_id_t)event_id) );
    #endif
#endif
        switch (event_id) {
            case WIFI_EVENT_STA_START: {
                ESPNOWBroadcast::STATE stopped {ESPNOWBroadcast::STOPPED};
                espnow._state.compare_exchange_strong(stopped, ESPNOWBroadcast::STARTING);
                break;
            }

            case WIFI_EVENT_STA_STOP:
            case WIFI_EVENT_AP_START: {
                ESPNOWBroadcast::STATE started {ESPNOWBroadcast::STARTED};
                ESPNOWBroadcast::STATE starting {ESPNOWBroadcast::STARTING};
                if (espnow._state.compare_exchange_strong(started, ESPNOWBroadcast::STOPPED) ||
                    espnow._state.compare_exchange_strong(starting, ESPNOWBroadcast::STOPPED)) {
#ifdef ESPNOW_DEBUGGING
                    Serial.println("WiFi connected: stop broadcasting");
#endif
                    esp_now_unregister_recv_cb();
                    esp_now_deinit();
                }
                break;
            }
        }
    }
}

void ESPNOWBroadcastImpl::onESPNowRxCallback(const uint8_t *mac, const uint8_t *data, int len) {
    if (!espnow.queuedNetworkRingBuffer.push(mac, data, len)) {
        if (len > sizeof(WLED_ESPNOW_MAX_MESSAGE_LENGTH)) {
#ifdef ESPNOW_DEBUGGING
            Serial.printf("Receive to large of packet %d bytes. ignoring...\n", len);
#endif
        } else {
            Serial.println("Failed to aquire ring buffer.  Dropping network message");
        }
    }
}

bool ESPNOWBroadcastImpl::QueuedNetworkRingBuffer::push(const uint8_t* mac, const uint8_t* data, uint8_t len) {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 0, 0)
    QueuedNetworkMessage msg[1];
    if (len <= sizeof(msg->data)) {
        memcpy(msg->mac, mac, sizeof(msg->mac));
        memcpy(&(msg->data), data, len);
        msg->len = len;

        if (pdTRUE == xRingbufferSend(buf, (void**)&msg, sizeof(*msg), 0)) {
            return true;
        }
    }
    return false;
#else
    QueuedNetworkMessage* msg = nullptr;
    if (len <= sizeof(msg->data)) {
        if (pdTRUE == xRingbufferSendAcquire(buf, (void**)&msg, sizeof(*msg), 0)) {
            memcpy(msg->mac, mac, sizeof(msg->mac));
            memcpy(&(msg->data), data, len);
            msg->len = len;
            xRingbufferSendComplete(buf, msg);
            return true;
        }
    }
    return false;
#endif
}

#endif
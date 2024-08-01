
#pragma once

#ifndef WLED_DISABLE_ESPNOW_NEW

#include <Arduino.h>
#include <atomic>
#include <WiFi.h>
#include "const.h"


#ifndef WLED_ESPNOW_MAX_QUEUED_MESSAGES
#define WLED_ESPNOW_MAX_QUEUED_MESSAGES 6
#endif

#ifndef WLED_ESPNOW_MAX_MESSAGE_LENGTH
#define WLED_ESPNOW_MAX_MESSAGE_LENGTH 250
#endif

#ifndef WLED_ESPNOW_MAX_REGISTERED_CALLBACKS
#define WLED_ESPNOW_MAX_REGISTERED_CALLBACKS WLED_MAX_USERMODS+1
#endif

class ESPNOWBroadcastClass {

  public:
    bool setup();

    void loop(size_t maxMessagesToProcess = WLED_ESPNOW_MAX_QUEUED_MESSAGES);

    typedef int err_t;
    err_t send(const uint8_t* msg, size_t len);

    typedef void (*receive_callback_t)(const uint8_t *sender, const uint8_t *data, uint8_t len);
    bool registerCallback( receive_callback_t callback );
    bool removeCallback( receive_callback_t callback );

    enum STATE {
        STOPPED = 0,
        STARTING,
        STARTED,
        MAX
    };

    STATE getState() {
        return _state.load();
    }

 protected:
    std::atomic<STATE> _state {STOPPED};

    bool setupWiFi();

    void start();

    static esp_err_t onSystemEvent(void *ctx, system_event_t *event);

    static void onWiFiEvent(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

    static void onESPNowRxCallback(const uint8_t *mac_addr, const uint8_t *data, int len);

    receive_callback_t _rxCallback[WLED_ESPNOW_MAX_REGISTERED_CALLBACKS] = {0};
    static constexpr size_t _rxCallbackSize = sizeof(_rxCallback)/sizeof(_rxCallback[0]);

};

extern ESPNOWBroadcastClass ESPNOWBroadcast;

#endif
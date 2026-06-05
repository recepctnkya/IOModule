#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifndef HEXNET_PLATFORM_BASE_URL
#define HEXNET_PLATFORM_BASE_URL "http://185.33.234.10/myhexnet"
#endif

#define HEXNET_RESOLVE_MQTT_PATH "/api/device/resolve-mqtt.php"
#define HEXNET_DEVICE_HW "Vango-Medium"

#define HEXNET_RESOLVE_TOPIC_MAX 128
#define HEXNET_RESOLVE_UID_MAX 64
#define HEXNET_RESOLVE_STATE_MAX 32

typedef struct {
    bool ok;
    bool registered;
    char state[HEXNET_RESOLVE_STATE_MAX];
    char message[96];
    int company_id;
    int device_pk_id;
    int mqtt_segment;
    char device_uid[HEXNET_RESOLVE_UID_MAX];
    char topic_prefix[HEXNET_RESOLVE_TOPIC_MAX];
    char telemetry_topic[HEXNET_RESOLVE_TOPIC_MAX];
    char status_topic[HEXNET_RESOLVE_TOPIC_MAX];
    char command_topic[HEXNET_RESOLVE_TOPIC_MAX];
    char command_reply_topic[HEXNET_RESOLVE_TOPIC_MAX];
} hexnet_resolve_result_t;

bool hexnet_format_wifi_mac(char *out, size_t out_len);

bool hexnet_resolve_mqtt_is_routing_complete(const hexnet_resolve_result_t *result);

esp_err_t hexnet_resolve_mqtt_fetch(
    const char *base_url,
    const char *fw,
    hexnet_resolve_result_t *out
);

esp_err_t hexnet_resolve_mqtt_save_nvs(const hexnet_resolve_result_t *result);

esp_err_t hexnet_resolve_mqtt_load_nvs(hexnet_resolve_result_t *result);

void hexnet_resolve_mqtt_clear_nvs(void);

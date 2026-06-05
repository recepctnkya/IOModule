#include "hexnet_mqtt.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define LOG_LOCAL_LEVEL HEXNET_LOG_LEVEL_MQTT
#include "hexnet_log.h"
#include "cJSON.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hexnet_bluetooth.h"
#include "hexnet_canbus.h"
#include "hexnet_debug.h"
#include "hexnet_ota_debug.h"
#include "hexnet_resolve_mqtt.h"
#include "hexnet_io_profile.h"
#include "hexnet_version.h"
#include "hexnet_wifi.h"
#include "mqtt_client.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "ota_manager.h"

static const char *TAG = "HEXNET_MQTT";

#define HEXNET_MQTT_BROKER_URI "mqtt://185.33.234.10:1883"
#define HEXNET_MQTT_TOPIC_PREFIX "hexnet/v1"
#define HEXNET_MQTT_PROTOCOL_VERSION "1.0"
#define HEXNET_DEFAULT_COMPANY_ID 1
#define HEXNET_DEFAULT_DEVICE_ID 1
#define TELEMETRY_PUBLISH_PERIOD_MS 5000
#define TELEMETRY_STATE_POLL_PERIOD_MS 500
#define TELEMETRY_CHANGE_MIN_INTERVAL_MS 250
#define MQTT_PUBLISH_READY_DELAY_MS 2500
#define MQTT_PUBLISH_FAIL_LOG_MS 10000
#define MQTT_PUBLISH_FAIL_RECONNECT 6
#define RESOLVE_HTTP_RETRY_MS 60000
#define RESOLVE_HTTP_RETRY_UNREGISTERED_MS 120000
#define RESOLVE_HTTP_REFRESH_MS 300000
#define TELEMETRY_QUEUE_LEN 6
#define TELEMETRY_DEBUG_LOG_EVERY_N 6
#define TELEMETRY_PAYLOAD_MAX_LEN 1536
#define TELEMETRY_QUEUE_FLUSH_BATCH 6
/** ota_start JSON (uzun HTTPS URL) — 512 bayt kesilince komut sessizce bozuluyordu */
#define HEXNET_MQTT_CMD_PAYLOAD_MAX 2048

typedef struct {
    char payload[TELEMETRY_PAYLOAD_MAX_LEN];
} telemetry_queue_item_t;

typedef struct {
    uint16_t outputs_mask;
    int16_t dim[4];
    int16_t water_pct[4];
    int16_t temp_c;
    int16_t hum_pct;
    uint8_t rgb_r;
    uint8_t rgb_g;
    uint8_t rgb_b;
    uint8_t rgb_enable;
} telemetry_state_snapshot_t;

static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static QueueHandle_t s_telemetry_offline_queue = NULL;
static bool s_mqtt_connected = false;
static bool s_mqtt_publish_ready = false;
static int64_t s_mqtt_connected_at_ms = 0;
static int64_t s_last_publish_fail_log_ms = 0;
static int s_publish_fail_streak = 0;
static int s_mqtt_company_id = HEXNET_DEFAULT_COMPANY_ID;
static int s_mqtt_device_id = HEXNET_DEFAULT_DEVICE_ID;
static char s_mqtt_telemetry_topic[128] = {0};
static char s_mqtt_command_topic[128] = {0};
static char s_mqtt_command_reply_topic[128] = {0};
static char s_mqtt_client_id[32] = "hexnet-io";
static bool s_mqtt_route_resolved = false;
static bool s_mqtt_connect_allowed = false;
static bool s_mqtt_nvs_trusted = false;
static int64_t s_last_resolve_attempt_ms = 0;
static int64_t s_last_resolve_success_ms = 0;
static char s_last_offline_enqueued_payload[TELEMETRY_PAYLOAD_MAX_LEN] = {0};
static volatile bool s_mqtt_flush_requested = false;
static int64_t s_mqtt_init_backoff_until_ms = 0;
static volatile bool s_mqtt_restart_pending = false;
static volatile bool s_mqtt_in_publish = false;
static volatile bool s_mqtt_ota_shutdown = false;
static SemaphoreHandle_t s_mqtt_api_mutex = NULL;
static SemaphoreHandle_t s_mqtt_pub_done_sem = NULL;
static volatile int s_mqtt_wait_pub_msg_id = -1;
/** OTA handshake: destroy outbox erken cagrilir — stop sonrasi reboot/aborte kadar tutulur */
static esp_mqtt_client_handle_t s_mqtt_halted_client = NULL;
static uint32_t s_telemetry_tx_count = 0;
static int64_t s_first_telemetry_ok_ms = 0;

extern float batarya_volt;

static void mqtt_api_lock(void);
static void mqtt_api_unlock(void);
static void mqtt_drain_yield_ms(int total_ms);
static bool mqtt_wait_publish_done(int msg_id, int timeout_ms);
static bool mqtt_destroy_halted_client_if_any(void);
static void mqtt_stop_client_safe(void);
static void mqtt_halt_client_for_ota(void);
static void mqtt_start_if_needed(void);
static bool mqtt_publish_ready_now(void);
static bool apply_resolve_routing(const hexnet_resolve_result_t *result);
static bool platform_resolve_mqtt_now(bool force);
static bool mqtt_publish_payload(const char *payload);
static bool mqtt_publish_command_reply(const char *payload);
static void mqtt_cmd_reply_json(cJSON *root, const char *cmd, bool ok, const char *message);
static void telemetry_publish_task(void *arg);
static void mqtt_resolve_task(void *arg);
static char *build_platform_telemetry_json(void);
static bool telemetry_should_publish_now(void);
static void mqtt_handle_command_message(const char *topic, int topic_len, const uint8_t *data, int data_len);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

static void mqtt_api_lock(void)
{
    if (s_mqtt_api_mutex != NULL) {
        (void)xSemaphoreTake(s_mqtt_api_mutex, portMAX_DELAY);
    }
}

static void mqtt_api_unlock(void)
{
    if (s_mqtt_api_mutex != NULL) {
        (void)xSemaphoreGive(s_mqtt_api_mutex);
    }
}

static void mqtt_drain_yield_ms(int total_ms)
{
    const int step_ms = 50;
    for (int left = total_ms; left > 0; left -= step_ms) {
        vTaskDelay(pdMS_TO_TICKS(left > step_ms ? step_ms : left));
    }
}

static bool mqtt_wait_publish_done(int msg_id, int timeout_ms)
{
    if (msg_id < 0 || s_mqtt_pub_done_sem == NULL) {
        return false;
    }
    (void)xSemaphoreTake(s_mqtt_pub_done_sem, 0);
    s_mqtt_wait_pub_msg_id = msg_id;
    const bool ok = xSemaphoreTake(s_mqtt_pub_done_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
    if (!ok) {
        s_mqtt_wait_pub_msg_id = -1;
        ESP_LOGW(TAG, "MQTT publish ack timeout msg_id=%d", msg_id);
    }
    return ok;
}

static bool mqtt_destroy_halted_client_if_any(void)
{
    if (s_mqtt_halted_client == NULL) {
        return true;
    }
    esp_mqtt_client_handle_t halted = s_mqtt_halted_client;
    s_mqtt_halted_client = NULL;
    mqtt_drain_yield_ms(600);
    (void)esp_mqtt_client_unregister_event(halted, ESP_EVENT_ANY_ID, mqtt_event_handler);
    mqtt_drain_yield_ms(400);
    esp_mqtt_client_destroy(halted);
    mqtt_drain_yield_ms(200);
    ESP_LOGI(TAG, "MQTT halted client destroyed (heap_free=%lu)", (unsigned long)esp_get_free_heap_size());

    return true;
}

/** Tam kapatma (restart / abort sonrasi). */
static void mqtt_stop_client_safe(void)
{
    mqtt_destroy_halted_client_if_any();

    esp_mqtt_client_handle_t client = s_mqtt_client;
    if (client == NULL) {
        return;
    }

    s_mqtt_connected = false;
    s_mqtt_publish_ready = false;
    esp_mqtt_client_unregister_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler);
    esp_mqtt_client_stop(client);

    for (int i = 0; i < 20; i++) {
        if (!s_mqtt_in_publish) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    mqtt_drain_yield_ms(500);

    esp_mqtt_client_destroy(client);
    s_mqtt_client = NULL;
}

/**
 * OTA HTTPS oncesi: client stop, destroy YOK (custom outbox QoS1 tamamlanmadan destroy panic).
 * Basarili OTA sonrasi reboot; abort'ta mqtt_destroy_halted_client_if_any().
 */
static void mqtt_halt_client_for_ota(void)
{
    esp_mqtt_client_handle_t client = s_mqtt_client;
    if (client == NULL) {
        return;
    }

    s_mqtt_connected = false;
    s_mqtt_publish_ready = false;
    (void)esp_mqtt_client_disconnect(client);
    mqtt_drain_yield_ms(250);
    (void)esp_mqtt_client_stop(client);
    mqtt_drain_yield_ms(500);
    /* unregister destroy oncesi — halt'ta unregister + destroy panic yapiyordu */

    s_mqtt_halted_client = client;
    s_mqtt_client = NULL;
}

/**
 * HTTPS OTA oncesi: QoS1 command_reply tamamlandiktan sonra MQTT client'i tamamen serbest birak.
 * (Yalnizca disconnect yeterli degil — TLS/outbox ~20KB heap tutuyor.)
 */
static void mqtt_release_client_for_ota(void)
{
    esp_mqtt_client_handle_t client = s_mqtt_client;
    if (client == NULL) {
        return;
    }

    s_mqtt_connected = false;
    s_mqtt_publish_ready = false;
    (void)esp_mqtt_client_disconnect(client);
    mqtt_drain_yield_ms(400);
    (void)esp_mqtt_client_stop(client);
    mqtt_drain_yield_ms(600);
    esp_mqtt_client_unregister_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler);
    esp_mqtt_client_destroy(client);
    s_mqtt_client = NULL;
    s_mqtt_halted_client = NULL;
    ESP_LOGI(TAG, "MQTT released for OTA (heap_free=%lu)", (unsigned long)esp_get_free_heap_size());
}

static void update_mqtt_topics_from_ids(void)
{
    snprintf(
        s_mqtt_telemetry_topic,
        sizeof(s_mqtt_telemetry_topic),
        "%s/%d/%d/telemetry",
        HEXNET_MQTT_TOPIC_PREFIX,
        s_mqtt_company_id,
        s_mqtt_device_id);
    snprintf(
        s_mqtt_command_topic,
        sizeof(s_mqtt_command_topic),
        "%s/%d/%d/command",
        HEXNET_MQTT_TOPIC_PREFIX,
        s_mqtt_company_id,
        s_mqtt_device_id);
    snprintf(
        s_mqtt_command_reply_topic,
        sizeof(s_mqtt_command_reply_topic),
        "%s/%d/%d/command_reply",
        HEXNET_MQTT_TOPIC_PREFIX,
        s_mqtt_company_id,
        s_mqtt_device_id);
}

static bool apply_resolve_routing(const hexnet_resolve_result_t *result)
{
    if (!result || !hexnet_resolve_mqtt_is_routing_complete(result)) {
        return false;
    }

    bool topics_changed = (strncmp(s_mqtt_telemetry_topic, result->telemetry_topic, sizeof(s_mqtt_telemetry_topic)) != 0) ||
                          (strncmp(s_mqtt_command_topic, result->command_topic, sizeof(s_mqtt_command_topic)) != 0);

    snprintf(s_mqtt_telemetry_topic, sizeof(s_mqtt_telemetry_topic), "%s", result->telemetry_topic);
    snprintf(s_mqtt_command_topic, sizeof(s_mqtt_command_topic), "%s", result->command_topic);
    if (result->command_reply_topic[0] != '\0') {
        snprintf(s_mqtt_command_reply_topic, sizeof(s_mqtt_command_reply_topic), "%s", result->command_reply_topic);
    } else {
        snprintf(
            s_mqtt_command_reply_topic,
            sizeof(s_mqtt_command_reply_topic),
            "%s/%d/%d/command_reply",
            HEXNET_MQTT_TOPIC_PREFIX,
            result->company_id > 0 ? result->company_id : s_mqtt_company_id,
            result->mqtt_segment > 0 ? result->mqtt_segment : s_mqtt_device_id);
    }

    if (result->company_id > 0) {
        s_mqtt_company_id = result->company_id;
    }
    if (result->mqtt_segment > 0) {
        s_mqtt_device_id = result->mqtt_segment;
    } else if (result->device_pk_id > 0) {
        s_mqtt_device_id = result->device_pk_id;
    }

    s_mqtt_route_resolved = true;
    ESP_LOGI(
        TAG,
        "MQTT routing resolved: telemetry=%s command=%s company=%d device=%d",
        s_mqtt_telemetry_topic,
        s_mqtt_command_topic,
        s_mqtt_company_id,
        s_mqtt_device_id);
    return topics_changed;
}

static bool platform_resolve_mqtt_now(bool force)
{
    int64_t now_ms = esp_timer_get_time() / 1000LL;
    const int64_t retry_ms = s_mqtt_connect_allowed ? RESOLVE_HTTP_RETRY_MS : RESOLVE_HTTP_RETRY_UNREGISTERED_MS;
    if (!force &&
        s_last_resolve_attempt_ms > 0 &&
        (now_ms - s_last_resolve_attempt_ms) < retry_ms) {
        return s_mqtt_route_resolved;
    }
    s_last_resolve_attempt_ms = now_ms;

    hexnet_resolve_result_t result = {0};
    esp_err_t err = hexnet_resolve_mqtt_fetch(
        HEXNET_PLATFORM_BASE_URL,
        hexnet_firmware_version_string(),
        &result);
    if (err != ESP_OK) {
        if (!s_mqtt_route_resolved) {
            hexnet_resolve_result_t cached = {0};
            if (hexnet_resolve_mqtt_load_nvs(&cached) == ESP_OK) {
                bool changed = apply_resolve_routing(&cached);
                s_mqtt_nvs_trusted = true;
                s_mqtt_connect_allowed = true;
                if (changed) {
                    hexnet_mqtt_stop();
                }
                return true;
            }
        }
        if (s_mqtt_route_resolved && s_mqtt_nvs_trusted) {
            s_mqtt_connect_allowed = true;
            ESP_LOGW(TAG, "resolve HTTP basarisiz; NVS rotasi ile MQTT devam");
        }
        return s_mqtt_route_resolved;
    }

    if (!result.registered || !hexnet_resolve_mqtt_is_routing_complete(&result)) {
        ESP_LOGW(
            TAG,
            "resolve-mqtt not ready: state=%s message=%s",
            result.state[0] ? result.state : "unknown",
            result.message[0] ? result.message : "");
        s_mqtt_connect_allowed = false;
        s_mqtt_nvs_trusted = false;
        s_mqtt_route_resolved = false;
        hexnet_mqtt_stop();
        return false;
    }

    (void)hexnet_resolve_mqtt_save_nvs(&result);
    bool changed = apply_resolve_routing(&result);
    s_mqtt_nvs_trusted = true;
    s_mqtt_connect_allowed = true;
    s_last_resolve_success_ms = now_ms;
    if (changed) {
        hexnet_mqtt_stop();
    }
    return true;
}

static bool load_i32_from_nvs_candidates(
    const char *const *namespaces,
    size_t ns_count,
    const char *const *keys,
    size_t key_count,
    int32_t *out)
{
    if (!namespaces || !keys || !out) {
        return false;
    }

    for (size_t ni = 0; ni < ns_count; ++ni) {
        nvs_handle_t nvs_handle;
        if (nvs_open(namespaces[ni], NVS_READONLY, &nvs_handle) != ESP_OK) {
            continue;
        }

        for (size_t ki = 0; ki < key_count; ++ki) {
            int32_t value = 0;
            if (nvs_get_i32(nvs_handle, keys[ki], &value) == ESP_OK && value > 0) {
                *out = value;
                nvs_close(nvs_handle);
                return true;
            }
        }
        nvs_close(nvs_handle);
    }

    return false;
}

static void load_mqtt_route_ids_from_nvs(void)
{
    int32_t company_id = HEXNET_DEFAULT_COMPANY_ID;
    int32_t device_id = HEXNET_DEFAULT_DEVICE_ID;

    static const char *const kNamespaces[] = {"storage", "hexnet", "config", "settings"};
    static const char *const kCompanyKeys[] = {"mqttCid", "company_id", "companyId", "cid"};
    static const char *const kDeviceKeys[] = {"mqttDid", "mqtt_route_device_id", "device_id", "deviceId", "did"};

    (void)load_i32_from_nvs_candidates(
        kNamespaces,
        sizeof(kNamespaces) / sizeof(kNamespaces[0]),
        kCompanyKeys,
        sizeof(kCompanyKeys) / sizeof(kCompanyKeys[0]),
        &company_id);
    (void)load_i32_from_nvs_candidates(
        kNamespaces,
        sizeof(kNamespaces) / sizeof(kNamespaces[0]),
        kDeviceKeys,
        sizeof(kDeviceKeys) / sizeof(kDeviceKeys[0]),
        &device_id);

    if (company_id > 0) {
        s_mqtt_company_id = (int)company_id;
    }
    if (device_id > 0) {
        s_mqtt_device_id = (int)device_id;
    }
    if (!s_mqtt_route_resolved) {
        update_mqtt_topics_from_ids();
    }
}

static void enqueue_telemetry_offline(const char *payload)
{
    if (!s_telemetry_offline_queue || !payload) {
        return;
    }
    if (strncmp(s_last_offline_enqueued_payload, payload, TELEMETRY_PAYLOAD_MAX_LEN - 1) == 0) {
        return;
    }

    telemetry_queue_item_t item = {0};
    size_t payload_len = strnlen(payload, TELEMETRY_PAYLOAD_MAX_LEN - 1);
    memcpy(item.payload, payload, payload_len);
    item.payload[payload_len] = '\0';

    if (xQueueSend(s_telemetry_offline_queue, &item, 0) != pdTRUE) {
        telemetry_queue_item_t dropped;
        (void)xQueueReceive(s_telemetry_offline_queue, &dropped, 0);
        (void)xQueueSend(s_telemetry_offline_queue, &item, 0);
    }
    memcpy(s_last_offline_enqueued_payload, item.payload, TELEMETRY_PAYLOAD_MAX_LEN);
}

static void flush_telemetry_offline_queue(size_t max_items)
{
    if (!s_mqtt_connected || !s_telemetry_offline_queue) {
        return;
    }

    telemetry_queue_item_t item;
    size_t flushed = 0;
    while (xQueueReceive(s_telemetry_offline_queue, &item, 0) == pdTRUE) {
        if (!mqtt_publish_payload(item.payload)) {
            enqueue_telemetry_offline(item.payload);
            break;
        }
        s_last_offline_enqueued_payload[0] = '\0';
        flushed++;
        if (max_items > 0 && flushed >= max_items) {
            break;
        }
    }
}

static bool capture_telemetry_state_snapshot(telemetry_state_snapshot_t *out)
{
    if (!out) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    out->outputs_mask = (uint16_t)get_outputs();
    for (int i = 0; i < 4; i++) {
        out->dim[i] = (int16_t)get_dimmable_output((uint8_t)i);
        out->water_pct[i] = (int16_t)get_analog_input((uint8_t)i);
    }
    out->temp_c = (int16_t)get_sensorTemp();
    out->hum_pct = (int16_t)get_sensorHumidity();
    out->rgb_r = (uint8_t)get_r_value();
    out->rgb_g = (uint8_t)get_g_value();
    out->rgb_b = (uint8_t)get_b_value();
    out->rgb_enable = (uint8_t)get_rgb_enable();
    return true;
}

static bool telemetry_should_publish_now(void)
{
    static telemetry_state_snapshot_t s_prev = {0};
    static bool s_has_prev = false;
    static int64_t s_last_publish_trigger_ms = 0;

    telemetry_state_snapshot_t current = {0};
    if (!capture_telemetry_state_snapshot(&current)) {
        return false;
    }

    bool changed = (!s_has_prev) || (memcmp(&current, &s_prev, sizeof(current)) != 0);
    int64_t now_ms = esp_timer_get_time() / 1000LL;
    bool heartbeat_due = mqtt_publish_ready_now() &&
                         (s_last_publish_trigger_ms == 0 ||
                          (now_ms - s_last_publish_trigger_ms) >= TELEMETRY_PUBLISH_PERIOD_MS);
    bool min_change_interval_ok = (s_last_publish_trigger_ms == 0 ||
                                   (now_ms - s_last_publish_trigger_ms) >= TELEMETRY_CHANGE_MIN_INTERVAL_MS);

    if (!changed && !heartbeat_due) {
        return false;
    }
    if (changed && !heartbeat_due && !min_change_interval_ok) {
        return false;
    }

    if (changed) {
        s_prev = current;
        s_has_prev = true;
    }
    s_last_publish_trigger_ms = now_ms;
    return true;
}

static int json_bool_like(const cJSON *item, int fallback)
{
    if (!item) {
        return fallback;
    }
    if (cJSON_IsBool(item)) {
        return cJSON_IsTrue(item) ? 1 : 0;
    }
    if (cJSON_IsNumber(item)) {
        return item->valuedouble > 0 ? 1 : 0;
    }
    if (cJSON_IsString(item) && item->valuestring) {
        if (strcmp(item->valuestring, "1") == 0 || strcmp(item->valuestring, "true") == 0 ||
            strcmp(item->valuestring, "on") == 0) {
            return 1;
        }
        if (strcmp(item->valuestring, "0") == 0 || strcmp(item->valuestring, "false") == 0 ||
            strcmp(item->valuestring, "off") == 0) {
            return 0;
        }
    }
    return fallback;
}

static int clamp_int(int value, int lo, int hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

static int resolve_dimmer_index(const cJSON *index_item, const cJSON *channel_item)
{
    const cJSON *pick = index_item ? index_item : channel_item;
    if (!cJSON_IsNumber(pick)) {
        return -1;
    }
    int idx = pick->valueint;
    if (idx >= 1 && idx <= 4) {
        return idx - 1;
    }
    if (idx >= 0 && idx <= 3) {
        return idx;
    }
    return -1;
}

static bool mqtt_apply_dimmer_index(int dim_index, int value_pct)
{
    if (dim_index < 0 || dim_index > 3) {
        return false;
    }
    if (!hexnet_io_profile_dim_slot_enabled((uint8_t)dim_index)) {
        ESP_LOGW(TAG, "MQTT dimmer ignored: slot %d pasif", dim_index);
        return false;
    }
    value_pct = clamp_int(value_pct, 0, 100);
    set_dimmable_output((uint8_t)dim_index, (uint8_t)value_pct);
    ESP_LOGI(TAG, "MQTT dimmer applied index=%d value=%d", dim_index, value_pct);
    return true;
}

static bool mqtt_apply_rgb(int red, int green, int blue, int enable)
{
    if (!hexnet_io_profile_rgb_enabled()) {
        ESP_LOGW(TAG, "MQTT rgb ignored: pasif (profil)");
        return false;
    }
    red = clamp_int(red, 0, 255);
    green = clamp_int(green, 0, 255);
    blue = clamp_int(blue, 0, 255);
    uint8_t ena = (enable < 0) ? 1 : (enable ? 1 : 0);
    set_rgb_values((uint8_t)red, (uint8_t)green, (uint8_t)blue, ena);
    ESP_LOGI(TAG, "MQTT rgb applied r=%d g=%d b=%d enable=%d", red, green, blue, ena);
    return true;
}

static bool mqtt_try_handle_dimmer_command(cJSON *root, const cJSON *args)
{
    const cJSON *idx_item = cJSON_GetObjectItemCaseSensitive(root, "index");
    const cJSON *ch_item = cJSON_GetObjectItemCaseSensitive(root, "channel");
    const cJSON *val_item = cJSON_GetObjectItemCaseSensitive(root, "value");
    if (args) {
        if (!idx_item) {
            idx_item = cJSON_GetObjectItemCaseSensitive(args, "index");
        }
        if (!ch_item) {
            ch_item = cJSON_GetObjectItemCaseSensitive(args, "channel");
        }
        if (!val_item) {
            val_item = cJSON_GetObjectItemCaseSensitive(args, "value");
        }
    }
    int dim_index = resolve_dimmer_index(idx_item, ch_item);
    if (dim_index < 0 || !cJSON_IsNumber(val_item)) {
        return false;
    }
    return mqtt_apply_dimmer_index(dim_index, val_item->valueint);
}

static bool mqtt_try_handle_rgb_command(cJSON *root, const cJSON *args)
{
    const cJSON *r_item = cJSON_GetObjectItemCaseSensitive(root, "r");
    const cJSON *g_item = cJSON_GetObjectItemCaseSensitive(root, "g");
    const cJSON *b_item = cJSON_GetObjectItemCaseSensitive(root, "b");
    const cJSON *enable_item = cJSON_GetObjectItemCaseSensitive(root, "enable");
    if (args) {
        if (!r_item) {
            r_item = cJSON_GetObjectItemCaseSensitive(args, "r");
        }
        if (!g_item) {
            g_item = cJSON_GetObjectItemCaseSensitive(args, "g");
        }
        if (!b_item) {
            b_item = cJSON_GetObjectItemCaseSensitive(args, "b");
        }
        if (!enable_item) {
            enable_item = cJSON_GetObjectItemCaseSensitive(args, "enable");
        }
    }
    const cJSON *rgb_obj = cJSON_GetObjectItemCaseSensitive(root, "rgb");
    if ((!r_item || !g_item || !b_item) && cJSON_IsObject(rgb_obj)) {
        if (!r_item) {
            r_item = cJSON_GetObjectItemCaseSensitive(rgb_obj, "r");
        }
        if (!g_item) {
            g_item = cJSON_GetObjectItemCaseSensitive(rgb_obj, "g");
        }
        if (!b_item) {
            b_item = cJSON_GetObjectItemCaseSensitive(rgb_obj, "b");
        }
    }
    if (!cJSON_IsNumber(r_item) || !cJSON_IsNumber(g_item) || !cJSON_IsNumber(b_item)) {
        return false;
    }
    int enable = enable_item ? json_bool_like(enable_item, 1) : 1;
    return mqtt_apply_rgb(r_item->valueint, g_item->valueint, b_item->valueint, enable);
}

static const char *mqtt_event_id_name(int32_t event_id)
{
    switch (event_id) {
    case MQTT_EVENT_CONNECTED:
        return "CONNECTED";
    case MQTT_EVENT_DISCONNECTED:
        return "DISCONNECTED";
    case MQTT_EVENT_SUBSCRIBED:
        return "SUBSCRIBED";
    case MQTT_EVENT_UNSUBSCRIBED:
        return "UNSUBSCRIBED";
    case MQTT_EVENT_PUBLISHED:
        return "PUBLISHED";
    case MQTT_EVENT_DATA:
        return "DATA";
    case MQTT_EVENT_ERROR:
        return "ERROR";
    case MQTT_EVENT_BEFORE_CONNECT:
        return "BEFORE_CONNECT";
    default:
        return "OTHER";
    }
}

static void mqtt_cmd_reply_json(cJSON *root, const char *cmd, bool ok, const char *message)
{
    if (!cmd) {
        cmd = "unknown";
    }
    cJSON *rep = cJSON_CreateObject();
    if (!rep) {
        return;
    }
    if (root) {
        const cJSON *rid = cJSON_GetObjectItemCaseSensitive(root, "request_id");
        if (cJSON_IsString(rid) && rid->valuestring && rid->valuestring[0] != '\0') {
            cJSON_AddStringToObject(rep, "request_id", rid->valuestring);
        }
    }
    cJSON_AddStringToObject(rep, "cmd", cmd);
    cJSON_AddBoolToObject(rep, "ok", ok);
    cJSON_AddStringToObject(rep, "message", message && message[0] != '\0' ? message : (ok ? "ok" : "error"));
    char *out = cJSON_PrintUnformatted(rep);
    cJSON_Delete(rep);
    if (out) {
        hexnet_mqtt_dbg_tx(s_mqtt_command_reply_topic, out, (int)strlen(out), -1, 1);
        (void)mqtt_publish_command_reply(out);
        free(out);
    }
}

static void mqtt_handle_command_message(const char *topic, int topic_len, const uint8_t *data, int data_len)
{
    if (!topic || topic_len <= 0 || !data || data_len <= 0) {
        return;
    }

    char topic_buf[160];
    int topic_copy_len = topic_len < (int)(sizeof(topic_buf) - 1) ? topic_len : (int)(sizeof(topic_buf) - 1);
    memcpy(topic_buf, topic, (size_t)topic_copy_len);
    topic_buf[topic_copy_len] = '\0';

    if (strcmp(topic_buf, s_mqtt_command_topic) != 0) {
        ESP_LOGW(
            TAG,
            "MQTT cmd topic mismatch (ignored): got=%s expected=%s",
            topic_buf,
            s_mqtt_command_topic[0] != '\0' ? s_mqtt_command_topic : "(unset)");
        return;
    }

    char payload[HEXNET_MQTT_CMD_PAYLOAD_MAX];
    int copy_len = data_len < (int)(sizeof(payload) - 1) ? data_len : (int)(sizeof(payload) - 1);
    if (data_len >= (int)(sizeof(payload) - 1)) {
        ESP_LOGW(TAG, "MQTT cmd payload truncated: %d bytes -> %d (ota_start URL may be broken)", data_len, copy_len);
    }
    memcpy(payload, data, (size_t)copy_len);
    payload[copy_len] = '\0';

    hexnet_debug_platform_cmd("topic=%s payload=%s", topic_buf, payload);
    hexnet_mqtt_dbg_rx(topic_buf, payload, copy_len);

    cJSON *root = cJSON_Parse(payload);
    if (!root) {
        ESP_LOGW(TAG, "MQTT cmd parse failed, topic=%s payload=%s", topic_buf, payload);
        hexnet_debug_platform_cmd("JSON parse HATA");
        return;
    }

    const cJSON *ble_type = cJSON_GetObjectItemCaseSensitive(root, "writeDataType");
    if (cJSON_IsString(ble_type) && ble_type->valuestring) {
        hexnet_debug_platform_cmd("BLE format: writeDataType=%s", ble_type->valuestring);
        parse_write_data(root);
        hexnet_outputs_ble_mirror(get_outputs());
        cJSON_Delete(root);
        return;
    }

    const cJSON *cmd_item = cJSON_GetObjectItemCaseSensitive(root, "cmd");
    const char *cmd = (cJSON_IsString(cmd_item) && cmd_item->valuestring) ? cmd_item->valuestring : NULL;

    if (cmd && strcmp(cmd, "relay_set") == 0) {
        const cJSON *relay_item = cJSON_GetObjectItemCaseSensitive(root, "relay");
        const cJSON *state_item = cJSON_GetObjectItemCaseSensitive(root, "state");
        if (cJSON_IsNumber(relay_item) && state_item) {
            int relay = relay_item->valueint;
            int state = json_bool_like(state_item, 0);
            if (relay >= 0 && relay < 16) {
                if (!hexnet_io_profile_relay_slot_enabled((uint8_t)relay)) {
                    hexnet_debug_platform_cmd("relay_set r%d pasif (profil)", relay);
                    cJSON_Delete(root);
                    return;
                }
                uint16_t mask = get_outputs();
                if (state) {
                    mask |= (uint16_t)(1u << relay);
                } else {
                    mask &= (uint16_t)~(1u << relay);
                }
                set_outputs(mask);
                hexnet_outputs_ble_mirror(mask);
                hexnet_debug_platform_cmd("relay_set r%d=%s mask=0x%04X", relay, state ? "ON" : "OFF", (unsigned)mask);
                ESP_LOGI(TAG, "MQTT relay_set applied relay=%d state=%d mask=%u", relay, state, (unsigned)mask);
            }
        } else {
            const cJSON *args = cJSON_GetObjectItemCaseSensitive(root, "args");
            const cJSON *mask_item = cJSON_IsObject(args) ? cJSON_GetObjectItemCaseSensitive(args, "mask") : NULL;
            if (cJSON_IsNumber(mask_item)) {
                int mask = mask_item->valueint;
                if (mask >= 0 && mask <= 65535) {
                    set_outputs((uint16_t)mask);
                    hexnet_outputs_ble_mirror((uint16_t)mask);
                    hexnet_debug_platform_cmd("relay_set tum mask=0x%04X", (unsigned)mask);
                    ESP_LOGI(TAG, "MQTT relay_set mask applied=%d", mask);
                }
            }
        }
    } else if (cmd && strcmp(cmd, "relay_bit") == 0) {
        const cJSON *args = cJSON_GetObjectItemCaseSensitive(root, "args");
        const cJSON *idx_item = cJSON_IsObject(args) ? cJSON_GetObjectItemCaseSensitive(args, "index") : NULL;
        const cJSON *on_item = cJSON_IsObject(args) ? cJSON_GetObjectItemCaseSensitive(args, "on") : NULL;
        if (cJSON_IsNumber(idx_item) && on_item) {
            int idx = idx_item->valueint;
            int on = json_bool_like(on_item, 0);
            if (idx >= 0 && idx < 16) {
                if (!hexnet_io_profile_relay_slot_enabled((uint8_t)idx)) {
                    hexnet_debug_platform_cmd("relay_bit r%d pasif (profil)", idx);
                } else {
                    uint16_t mask = get_outputs();
                    if (on) {
                        mask |= (uint16_t)(1u << idx);
                    } else {
                        mask &= (uint16_t)~(1u << idx);
                    }
                    set_outputs(mask);
                    hexnet_outputs_ble_mirror(mask);
                    hexnet_debug_platform_cmd("relay_bit r%d=%s mask=0x%04X", idx, on ? "ON" : "OFF", (unsigned)mask);
                    ESP_LOGI(TAG, "MQTT relay_bit applied idx=%d on=%d mask=%u", idx, on, (unsigned)mask);
                }
            }
        }
    } else if (cmd && (strcmp(cmd, "dim_set") == 0 || strcmp(cmd, "dimmer_set") == 0)) {
        const cJSON *args = cJSON_GetObjectItemCaseSensitive(root, "args");
        if (mqtt_try_handle_dimmer_command(root, args)) {
            hexnet_debug_platform_cmd("dim_set OK");
        } else {
            ESP_LOGW(TAG, "MQTT dim_set ignored (need index/channel 0-3 and value 0-100)");
            hexnet_debug_platform_cmd("dim_set HATA (eksik alan)");
        }
    } else if (cmd && strcmp(cmd, "rgb_set") == 0) {
        const cJSON *args = cJSON_GetObjectItemCaseSensitive(root, "args");
        if (mqtt_try_handle_rgb_command(root, args)) {
            hexnet_debug_platform_cmd("rgb_set OK");
        } else {
            ESP_LOGW(TAG, "MQTT rgb_set ignored (need r,g,b in args)");
            hexnet_debug_platform_cmd("rgb_set HATA (eksik alan)");
        }
    } else if (cmd && (strcmp(cmd, "io_config_set") == 0 || strcmp(cmd, "config_set") == 0)) {
        const cJSON *args = cJSON_GetObjectItemCaseSensitive(root, "args");
        const cJSON *cfg = cJSON_IsObject(args) ? args : root;
        if (hexnet_io_profile_apply_json(cfg, HEXNET_IO_LINK_REMOTE) == ESP_OK) {
            hexnet_debug_platform_cmd("io_config_set OK");
            ESP_LOGI(TAG, "MQTT io_config_set applied");
        } else {
            hexnet_debug_platform_cmd("io_config_set HATA");
        }
    } else if (cmd && strcmp(cmd, "motor_set") == 0) {
        if (!hexnet_io_profile_motor_enabled()) {
            hexnet_debug_platform_cmd("motor_set pasif (profil)");
            cJSON_Delete(root);
            return;
        }
        const cJSON *args = cJSON_GetObjectItemCaseSensitive(root, "args");
        const cJSON *val_item = cJSON_GetObjectItemCaseSensitive(root, "value");
        const cJSON *cmd_val = val_item;
        if (!cmd_val && cJSON_IsObject(args)) {
            cmd_val = cJSON_GetObjectItemCaseSensitive(args, "value");
        }
        if (cJSON_IsNumber(cmd_val)) {
            int motor_cmd = cmd_val->valueint;
            if (motor_cmd < 0 || motor_cmd > 2) {
                motor_cmd = 0;
            }
            set_motordata(motor_cmd);
            hexnet_debug_platform_cmd("motor_set value=%d", motor_cmd);
            ESP_LOGI(TAG, "MQTT motor_set applied value=%d", motor_cmd);
        } else {
            hexnet_debug_platform_cmd("motor_set HATA (value 0-2 gerekli)");
        }
    } else if (cmd && strcmp(cmd, "ping") == 0) {
        ESP_LOGI(TAG, "MQTT ping received");
        hexnet_debug_platform_cmd("ping OK");
        mqtt_cmd_reply_json(root, "ping", true, "pong");
    } else if (cmd && strcmp(cmd, "ota_start") == 0) {
        const cJSON *args = cJSON_GetObjectItemCaseSensitive(root, "args");
        const cJSON *rid_item = cJSON_GetObjectItemCaseSensitive(root, "request_id");
        const char *req_id =
            (cJSON_IsString(rid_item) && rid_item->valuestring) ? rid_item->valuestring : "";
        const cJSON *url_item = cJSON_IsObject(args) ? cJSON_GetObjectItemCaseSensitive(args, "url") : NULL;
        const cJSON *ver_item = cJSON_IsObject(args) ? cJSON_GetObjectItemCaseSensitive(args, "version") : NULL;
        const cJSON *force_item = cJSON_IsObject(args) ? cJSON_GetObjectItemCaseSensitive(args, "force") : NULL;
        hexnet_ota_dbg_step(
            10,
            "mqtt_rx ota_start",
            "request_id=%s url=%s ver=%s force=%d",
            req_id[0] ? req_id : "-",
            (cJSON_IsString(url_item) && url_item->valuestring) ? url_item->valuestring : "-",
            (cJSON_IsString(ver_item) && ver_item->valuestring) ? ver_item->valuestring : "-",
            force_item ? json_bool_like(force_item, 0) : 0);
        if (ota_manager_handle_ota_start_json(args, req_id)) {
            hexnet_ota_dbg_step(12, "mqtt_rx ota_start", "kabul — OTA task devralacak");
            ESP_LOGI(TAG, "MQTT ota_start accepted (OTA task will reply and close MQTT)");
        } else {
            hexnet_ota_dbg_step(13, "mqtt_rx ota_start", "RED — requestStart basarisiz");
            ESP_LOGW(TAG, "MQTT ota_start rejected (check url / busy / version)");
            char err[96] = "rejected";
            char st[24] = "idle";
            char te[128] = "";
            int pr = 0;
            ota_manager_get_status(st, sizeof(st), &pr, te, sizeof(te), err, sizeof(err));
            mqtt_cmd_reply_json(root, "ota_start", false, err[0] != '\0' ? err : "rejected");
        }
    } else if (!cmd) {
        const cJSON *type_item = cJSON_GetObjectItemCaseSensitive(root, "type");
        const char *typ = (cJSON_IsString(type_item) && type_item->valuestring) ? type_item->valuestring : NULL;
        if (typ && strcmp(typ, "dimmer") == 0) {
            const cJSON *ch_item = cJSON_GetObjectItemCaseSensitive(root, "channel");
            const cJSON *val_item = cJSON_GetObjectItemCaseSensitive(root, "value");
            int dim_index = resolve_dimmer_index(NULL, ch_item);
            if (dim_index >= 0 && cJSON_IsNumber(val_item)) {
                mqtt_apply_dimmer_index(dim_index, val_item->valueint);
            }
        } else if (typ && strcmp(typ, "rgb") == 0) {
            mqtt_try_handle_rgb_command(root, NULL);
        }
    } else if (cmd) {
        ESP_LOGW(TAG, "MQTT unsupported cmd=%s", cmd);
        hexnet_debug_platform_cmd("desteklenmeyen cmd=%s", cmd);
        mqtt_cmd_reply_json(root, cmd, false, "unsupported_cmd");
    } else {
        hexnet_debug_platform_cmd("cmd alani yok");
        mqtt_cmd_reply_json(root, "unknown", false, "missing_cmd");
    }

    cJSON_Delete(root);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;

    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    if (event_id == MQTT_EVENT_PUBLISHED && event) {
        hexnet_mqtt_dbg_event("PUBLISHED", "msg_id=%d ota_shutdown=%d", event->msg_id, (int)s_mqtt_ota_shutdown);
    } else if (event_id == MQTT_EVENT_DATA && event) {
        hexnet_mqtt_dbg_event("DATA", "topic_len=%d data_len=%d", event->topic_len, event->data_len);
    } else {
        hexnet_mqtt_dbg_event(mqtt_event_id_name(event_id), "ota_shutdown=%d", (int)s_mqtt_ota_shutdown);
    }

    switch (event_id) {
    case MQTT_EVENT_CONNECTED:
        if (s_mqtt_ota_shutdown) {
            break;
        }
        s_mqtt_connected = true;
        s_mqtt_publish_ready = false;
        s_publish_fail_streak = 0;
        s_mqtt_connected_at_ms = esp_timer_get_time() / 1000LL;
        ESP_LOGI(TAG, "MQTT connected -> %s", s_mqtt_telemetry_topic);
        if (s_mqtt_client && s_mqtt_command_topic[0] != '\0') {
            int sub_id = esp_mqtt_client_subscribe(s_mqtt_client, s_mqtt_command_topic, 1);
            ESP_LOGI(TAG, "MQTT subscribe command topic=%s msg_id=%d", s_mqtt_command_topic, sub_id);
        }
        s_mqtt_flush_requested = true;
        break;
    case MQTT_EVENT_DISCONNECTED:
        s_mqtt_connected = false;
        s_mqtt_publish_ready = false;
        if (s_mqtt_ota_shutdown) {
            ESP_LOGD(TAG, "MQTT disconnected during OTA (expected)");
            break;
        }
        s_mqtt_restart_pending = true;
        ESP_LOGW(TAG, "MQTT disconnected");
        break;
    case MQTT_EVENT_ERROR:
        s_mqtt_connected = false;
        s_mqtt_publish_ready = false;
        if (s_mqtt_ota_shutdown) {
            ESP_LOGD(TAG, "MQTT error during OTA (ignored)");
            break;
        }
        s_mqtt_restart_pending = true;
        ESP_LOGE(TAG, "MQTT error event");
        break;
    case MQTT_EVENT_PUBLISHED: {
        esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
        if (event && s_mqtt_wait_pub_msg_id >= 0 && event->msg_id == s_mqtt_wait_pub_msg_id) {
            s_mqtt_wait_pub_msg_id = -1;
            if (s_mqtt_pub_done_sem != NULL) {
                (void)xSemaphoreGive(s_mqtt_pub_done_sem);
            }
        }
        break;
    }
    case MQTT_EVENT_DATA: {
        if (s_mqtt_ota_shutdown) {
            break;
        }
        esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
        if (event) {
            mqtt_handle_command_message(event->topic, event->topic_len, (const uint8_t *)event->data, event->data_len);
        }
        break;
    }
    default:
        break;
    }
}

void hexnet_mqtt_stop(void)
{
    mqtt_api_lock();
    mqtt_stop_client_safe();
    mqtt_api_unlock();
}

void hexnet_mqtt_shutdown_for_ota(void)
{
    mqtt_api_lock();
    s_mqtt_ota_shutdown = true;
    s_mqtt_restart_pending = false;
    s_mqtt_flush_requested = false;
    mqtt_api_unlock();
    ESP_LOGI(TAG, "MQTT paused for OTA");
}

void hexnet_mqtt_resume_after_ota_abort(void)
{
    mqtt_api_lock();
    if (s_mqtt_client != NULL) {
        mqtt_halt_client_for_ota();
    }
    s_mqtt_ota_shutdown = false;
    s_mqtt_restart_pending = true;
    s_mqtt_flush_requested = false;
    mqtt_api_unlock();
    hexnet_mqtt_destroy_halted_client_when_safe();
    ESP_LOGI(TAG, "MQTT OTA abort — reconnect scheduled (heap_free=%lu)",
             (unsigned long)esp_get_free_heap_size());
}

static void mqtt_start_if_needed(void)
{
    const int64_t now_ms = esp_timer_get_time() / 1000LL;
    if (s_mqtt_ota_shutdown) {
        return;
    }

    mqtt_api_lock();
    mqtt_destroy_halted_client_if_any();
    if (s_mqtt_ota_shutdown || s_mqtt_client != NULL) {
        mqtt_api_unlock();
        return;
    }
    if (s_mqtt_init_backoff_until_ms > now_ms) {
        mqtt_api_unlock();
        return;
    }
    if (!s_mqtt_connect_allowed || !s_mqtt_route_resolved || s_mqtt_telemetry_topic[0] == '\0' ||
        s_mqtt_command_topic[0] == '\0') {
        mqtt_api_unlock();
        return;
    }

    {
        uint8_t mac[6] = {0};
        if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
            snprintf(
                s_mqtt_client_id,
                sizeof(s_mqtt_client_id),
                "hexnet-io-%02x%02x%02x%02x%02x%02x",
                mac[0],
                mac[1],
                mac[2],
                mac[3],
                mac[4],
                mac[5]);
        } else {
            strncpy(s_mqtt_client_id, "hexnet-io", sizeof(s_mqtt_client_id) - 1);
            s_mqtt_client_id[sizeof(s_mqtt_client_id) - 1] = '\0';
        }
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = HEXNET_MQTT_BROKER_URI,
        .credentials.client_id = s_mqtt_client_id,
        .session.keepalive = 60,
        .network.timeout_ms = 20000,
        .network.reconnect_timeout_ms = 15000,
        .network.disable_auto_reconnect = false,
        .buffer.size = 2048,
        .buffer.out_size = HEXNET_MQTT_CMD_PAYLOAD_MAX,
    };
    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_mqtt_client == NULL) {
        s_mqtt_init_backoff_until_ms = now_ms + 5000;
        ESP_LOGE(TAG, "MQTT client init failed; 5s sonra tekrar");
        mqtt_api_unlock();
        return;
    }
    s_mqtt_init_backoff_until_ms = 0;
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt_client);
    mqtt_api_unlock();
}

static bool mqtt_publish_ready_now(void)
{
    if (!s_mqtt_connected || !s_mqtt_client) {
        return false;
    }
    if (s_mqtt_publish_ready) {
        return true;
    }
    const int64_t now_ms = esp_timer_get_time() / 1000LL;
    if (s_mqtt_connected_at_ms > 0 &&
        (now_ms - s_mqtt_connected_at_ms) >= MQTT_PUBLISH_READY_DELAY_MS) {
        s_mqtt_publish_ready = true;
        return true;
    }
    return false;
}

static bool mqtt_publish_payload(const char *payload)
{
    if (s_mqtt_ota_shutdown) {
        return false;
    }
    if (!payload || !mqtt_publish_ready_now()) {
        return false;
    }

    const int len = (int)strlen(payload);
    if (len <= 0 || len >= TELEMETRY_PAYLOAD_MAX_LEN) {
        return false;
    }

    hexnet_mqtt_dbg_tx(s_mqtt_telemetry_topic, payload, len, -1, 0);
    s_mqtt_in_publish = true;
    int msg_id = esp_mqtt_client_publish(s_mqtt_client, s_mqtt_telemetry_topic, payload, len, 0, 0);
    s_mqtt_in_publish = false;
    if (msg_id >= 0) {
        //hexnet_mqtt_dbg_event("PUBLISH_OK", "telemetry msg_id=%d", msg_id);
    }
    if (msg_id < 0) {
        if (!s_mqtt_connected && !s_mqtt_ota_shutdown) {
            s_mqtt_restart_pending = true;
            return false;
        }
        s_publish_fail_streak++;
        const int64_t now_ms = esp_timer_get_time() / 1000LL;
        if (s_last_publish_fail_log_ms == 0 ||
            (now_ms - s_last_publish_fail_log_ms) >= MQTT_PUBLISH_FAIL_LOG_MS) {
            ESP_LOGW(TAG, "Telemetry gonderilemedi (msg_id=%d) topic=%s", msg_id, s_mqtt_telemetry_topic);
            s_last_publish_fail_log_ms = now_ms;
        }
        if (s_publish_fail_streak >= MQTT_PUBLISH_FAIL_RECONNECT) {
            s_publish_fail_streak = 0;
            s_mqtt_restart_pending = true;
        }
        return false;
    }

    s_publish_fail_streak = 0;
    s_telemetry_tx_count++;
    const int64_t now_ms = esp_timer_get_time() / 1000LL;
    if (s_first_telemetry_ok_ms == 0) {
        s_first_telemetry_ok_ms = now_ms;
        ESP_LOGI(
            TAG,
            "Telemetry OK (ilk) len=%d aralik=%ds topic=%s",
            len,
            (int)(TELEMETRY_PUBLISH_PERIOD_MS / 1000),
            s_mqtt_telemetry_topic);
    } else if ((s_telemetry_tx_count % TELEMETRY_DEBUG_LOG_EVERY_N) == 0) {
        ESP_LOGI(
            TAG,
            "Telemetry OK #%lu len=%d (~%ds) topic=%s",
            (unsigned long)s_telemetry_tx_count,
            len,
            (int)(TELEMETRY_PUBLISH_PERIOD_MS / 1000),
            s_mqtt_telemetry_topic);
    }
    return true;
}

static bool mqtt_publish_command_reply(const char *payload)
{
    if (!payload || s_mqtt_ota_shutdown || s_mqtt_command_reply_topic[0] == '\0') {
        return false;
    }
    if (!mqtt_publish_ready_now() || !s_mqtt_client) {
        return false;
    }
    const int len = (int)strlen(payload);
    if (len <= 0 || len >= 512) {
        return false;
    }
    /** QoS 1: OTA sonrasi MQTT kapanmadan once broker/webhook ACK alsin (QoS 0 kayboluyordu). */
    int msg_id = esp_mqtt_client_publish(s_mqtt_client, s_mqtt_command_reply_topic, payload, len, 1, 0);
    if (msg_id < 0) {
        ESP_LOGW(TAG, "command_reply publish failed topic=%s", s_mqtt_command_reply_topic);
        return false;
    }
    ESP_LOGI(TAG, "command_reply published (qos1) topic=%s msg_id=%d", s_mqtt_command_reply_topic, msg_id);
    hexnet_mqtt_dbg_event("PUBLISH_SENT", "command_reply msg_id=%d", msg_id);
    if (mqtt_wait_publish_done(msg_id, 4500)) {
        hexnet_ota_dbg_step(22, "mqtt_handshake", "command_reply PUBLISHED ack msg_id=%d", msg_id);
    } else {
        hexnet_ota_dbg_step(22, "mqtt_handshake", "command_reply ack TIMEOUT msg_id=%d", msg_id);
    }
    return true;
}

void hexnet_mqtt_publish_telemetry_now(void)
{
    if (s_mqtt_ota_shutdown) {
        return;
    }
    char *json = build_platform_telemetry_json();
    if (json) {
        (void)mqtt_publish_payload(json);
        free(json);
    }
}

void hexnet_mqtt_finish_ota_mqtt_handshake(const char *request_id, bool ok, const char *message)
{
    if (s_mqtt_ota_shutdown) {
        hexnet_ota_dbg_step(21, "mqtt_handshake skip", "already ota_shutdown");
        return;
    }

    hexnet_ota_dbg_step(21, "mqtt_handshake", "begin ok=%d request_id=%s", ok ? 1 : 0, request_id ? request_id : "-");
    mqtt_api_lock();
    if (request_id && request_id[0] != '\0') {
        cJSON *wrap = cJSON_CreateObject();
        if (wrap) {
            cJSON_AddStringToObject(wrap, "request_id", request_id);
            mqtt_cmd_reply_json(wrap, "ota_start", ok, message && message[0] ? message : (ok ? "accepted" : "rejected"));
            cJSON_Delete(wrap);
        }
    }
    s_mqtt_ota_shutdown = true;
    s_mqtt_restart_pending = false;
    s_mqtt_flush_requested = false;
    mqtt_api_unlock();

    /* Mutex disinda bekle — PUBLISHED callback ve outbox bitsin (hemen destroy = panic) */
    mqtt_drain_yield_ms(400);

    mqtt_api_lock();
    if (ok) {
        if (s_mqtt_client != NULL) {
            mqtt_halt_client_for_ota();
            hexnet_ota_dbg_step(23, "mqtt_handshake", "MQTT halted (destroy gerekirse HTTPS oncesi)");
        }
    }
    mqtt_api_unlock();

    hexnet_ota_dbg_step(24, "mqtt_handshake", "bitti — MQTT durduruldu (HTTPS next)");
    ESP_LOGI(TAG, "MQTT halted for OTA (heap_free=%lu)", (unsigned long)esp_get_free_heap_size());
}

static void mqtt_destroy_halted_worker(void *arg)
{
    (void)arg;
    mqtt_drain_yield_ms(1200);
    mqtt_api_lock();
    (void)mqtt_destroy_halted_client_if_any();
    mqtt_api_unlock();
    vTaskDelete(NULL);
}

bool hexnet_mqtt_halted_client_pending(void)
{
    return s_mqtt_halted_client != NULL;
}

void hexnet_mqtt_destroy_halted_client_when_safe(void)
{
    if (s_mqtt_halted_client == NULL) {
        return;
    }
    (void)xTaskCreate(mqtt_destroy_halted_worker, "mqtt_ota_destroy", 4096, NULL, 4, NULL);
}

static void telemetry_add_device_identity(cJSON *root)
{
    if (!root) {
        return;
    }
    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        char mac_str[24];
        snprintf(
            mac_str,
            sizeof(mac_str),
            "%02X:%02X:%02X:%02X:%02X:%02X",
            mac[0],
            mac[1],
            mac[2],
            mac[3],
            mac[4],
            mac[5]);
        cJSON_AddStringToObject(root, "mac", mac_str);
    } else {
        cJSON_AddStringToObject(root, "mac", "");
    }
    cJSON_AddNumberToObject(root, "id", s_mqtt_device_id);
    cJSON_AddNumberToObject(root, "company_id", s_mqtt_company_id);
}

static char *build_platform_telemetry_json(void)
{
    cJSON *root = cJSON_CreateObject();
    if (!root) {
        return NULL;
    }

    const char *fw_ver = hexnet_firmware_version_string();
    int64_t ts = hexnet_wifi_ntp_unix_ts_sec();
    uint16_t relay_mask = (uint16_t)get_outputs();

    float vin_v = 0.0f;
    float batteryV = 0.0f;
    uint16_t rawBattery = get_voltage_value();
    if (rawBattery > 0) {
        vin_v = (float)rawBattery / 100.0f;
        batteryV = vin_v;
    } else if (batarya_volt > 0.0f) {
        vin_v = batarya_volt;
        batteryV = batarya_volt;
    }
    int battery_pct = 0;
    if (batteryV > 0.0f) {
        float pct = ((batteryV - 3.0f) / 1.2f) * 100.0f;
        if (pct < 0.0f) {
            pct = 0.0f;
        }
        if (pct > 100.0f) {
            pct = 100.0f;
        }
        battery_pct = (int)(pct + 0.5f);
    }

    cJSON_AddStringToObject(root, "protocol", "hexnet-io");
    cJSON_AddStringToObject(root, "version", HEXNET_MQTT_PROTOCOL_VERSION);
    cJSON_AddNumberToObject(root, "ts", (double)ts);

    cJSON *device = cJSON_CreateObject();
    telemetry_add_device_identity(device);
    cJSON_AddNumberToObject(device, "customer_id", 0);
    cJSON_AddStringToObject(device, "fw", fw_ver);
    cJSON_AddItemToObject(root, "device", device);

    cJSON *status = cJSON_CreateObject();
    cJSON_AddBoolToObject(status, "online", hexnet_wifi_is_connected() && s_mqtt_connected);
    cJSON_AddNumberToObject(status, "wifi_rssi", hexnet_wifi_get_sta_rssi());
    cJSON_AddStringToObject(status, "mqtt", s_mqtt_connected ? "connected" : "disconnected");
    cJSON_AddItemToObject(root, "status", status);

    cJSON *power = cJSON_CreateObject();
    cJSON_AddNumberToObject(power, "vin_v", vin_v);
    cJSON_AddNumberToObject(power, "battery_v", batteryV);
    cJSON_AddNumberToObject(power, "battery_pct", battery_pct);
    cJSON_AddItemToObject(root, "power", power);

    cJSON *sensors = cJSON_CreateObject();
    cJSON_AddNumberToObject(sensors, "temperature_c", get_sensorTemp());
    cJSON_AddNumberToObject(sensors, "humidity_pct", get_sensorHumidity());
    cJSON *water_pct = cJSON_CreateArray();
    for (int i = 0; i < 4; i++) {
        cJSON_AddItemToArray(water_pct, cJSON_CreateNumber(get_analog_input((uint8_t)i)));
    }
    cJSON_AddItemToObject(sensors, "water_pct", water_pct);
    cJSON_AddItemToObject(root, "sensors", sensors);

    cJSON *outputs = cJSON_CreateObject();
    cJSON *relays = cJSON_CreateArray();
    for (int i = 0; i < 16; i++) {
        cJSON_AddItemToArray(relays, cJSON_CreateNumber((relay_mask >> i) & 0x01));
    }
    cJSON_AddItemToObject(outputs, "relays", relays);
    cJSON_AddNumberToObject(outputs, "relays_mask", relay_mask);
    cJSON *dimmers = cJSON_CreateArray();
    for (int i = 0; i < 4; i++) {
        int d = get_dimmable_output((uint8_t)i);
        if (d < 0) {
            d = 0;
        }
        if (d > 100) {
            d = 100;
        }
        cJSON_AddItemToArray(dimmers, cJSON_CreateNumber(d));
    }
    cJSON_AddItemToObject(outputs, "dimmers", dimmers);
    cJSON *rgb = cJSON_CreateObject();
    cJSON_AddNumberToObject(rgb, "r", get_r_value());
    cJSON_AddNumberToObject(rgb, "g", get_g_value());
    cJSON_AddNumberToObject(rgb, "b", get_b_value());
    cJSON_AddItemToObject(outputs, "rgb", rgb);
    cJSON_AddItemToObject(root, "outputs", outputs);

    hexnet_io_profile_append_to_json(root, "io_profile");

    cJSON *ota = cJSON_CreateObject();
    {
        char ota_state[24] = "idle";
        char ota_target[128] = "";
        char ota_error[160] = "";
        int ota_progress = 0;
        ota_manager_get_status(
            ota_state,
            sizeof(ota_state),
            &ota_progress,
            ota_target,
            sizeof(ota_target),
            ota_error,
            sizeof(ota_error));
        cJSON_AddStringToObject(ota, "state", ota_state);
        cJSON_AddNumberToObject(ota, "progress", ota_progress);
        cJSON_AddStringToObject(ota, "target_version", ota_target);
        cJSON_AddStringToObject(ota, "last_error", ota_error);
    }
    cJSON_AddItemToObject(root, "ota", ota);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

static void mqtt_resolve_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(4000));
    while (1) {
        if (hexnet_wifi_is_connected() && hexnet_wifi_is_internet_ok() && !s_mqtt_connected) {
            if (!s_mqtt_connect_allowed || !s_mqtt_route_resolved) {
                platform_resolve_mqtt_now(false);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

static void mqtt_restart_client_if_needed(void)
{
    if (s_mqtt_ota_shutdown) {
        return;
    }
    if (!s_mqtt_restart_pending || s_mqtt_in_publish) {
        return;
    }
    if (s_mqtt_client == NULL) {
        s_mqtt_restart_pending = false;
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    if (s_mqtt_in_publish || s_mqtt_connected) {
        return;
    }
    ESP_LOGW(TAG, "MQTT client yeniden baslatiliyor");
    mqtt_api_lock();
    mqtt_stop_client_safe();
    mqtt_api_unlock();
    s_mqtt_restart_pending = false;
}

static void telemetry_publish_task(void *arg)
{
    (void)arg;
    while (1) {
        if (!s_mqtt_ota_shutdown) {
            mqtt_restart_client_if_needed();
            if (hexnet_wifi_is_connected() && s_mqtt_route_resolved && s_mqtt_client == NULL) {
                mqtt_start_if_needed();
            }
            if (hexnet_wifi_is_connected()) {
                hexnet_wifi_start_ntp_if_needed();
            }
            if (telemetry_should_publish_now()) {
                mqtt_api_lock();
                if (!s_mqtt_ota_shutdown && s_mqtt_client != NULL) {
                    char *payload_json = build_platform_telemetry_json();
                    if (payload_json != NULL) {
                        if (!mqtt_publish_payload(payload_json)) {
                            enqueue_telemetry_offline(payload_json);
                        }
                        free(payload_json);
                    }
                }
                mqtt_api_unlock();
            }
            if (s_mqtt_connected && !s_mqtt_ota_shutdown) {
                mqtt_api_lock();
                if (!s_mqtt_ota_shutdown && s_mqtt_connected) {
                    flush_telemetry_offline_queue(s_mqtt_flush_requested ? TELEMETRY_QUEUE_FLUSH_BATCH : 1);
                    if (uxQueueMessagesWaiting(s_telemetry_offline_queue) == 0) {
                        s_mqtt_flush_requested = false;
                    }
                }
                mqtt_api_unlock();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_STATE_POLL_PERIOD_MS));
    }
}

void hexnet_mqtt_init(void)
{
    s_mqtt_connect_allowed = false;
    s_mqtt_nvs_trusted = false;

    if (s_mqtt_api_mutex == NULL) {
        s_mqtt_api_mutex = xSemaphoreCreateMutex();
        if (s_mqtt_api_mutex == NULL) {
            ESP_LOGE(TAG, "MQTT api mutex create failed");
        }
    }
    if (s_mqtt_pub_done_sem == NULL) {
        s_mqtt_pub_done_sem = xSemaphoreCreateBinary();
        if (s_mqtt_pub_done_sem == NULL) {
            ESP_LOGE(TAG, "MQTT pub_done sem create failed");
        }
    }

    load_mqtt_route_ids_from_nvs();
    {
        hexnet_resolve_result_t cached = {0};
        if (hexnet_resolve_mqtt_load_nvs(&cached) == ESP_OK) {
            (void)apply_resolve_routing(&cached);
            s_mqtt_nvs_trusted = true;
            s_mqtt_connect_allowed = true;
            s_last_resolve_success_ms = esp_timer_get_time() / 1000LL;
        } else {
            update_mqtt_topics_from_ids();
        }
    }

    s_telemetry_offline_queue = xQueueCreate(TELEMETRY_QUEUE_LEN, sizeof(telemetry_queue_item_t));
    if (s_telemetry_offline_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create telemetry offline queue");
    }
}

void hexnet_mqtt_start_tasks(void)
{
    ESP_LOGI(
        TAG,
        "Telemetry: heartbeat her %ds, degisim min %dms",
        (int)(TELEMETRY_PUBLISH_PERIOD_MS / 1000),
        TELEMETRY_CHANGE_MIN_INTERVAL_MS);
    xTaskCreate(mqtt_resolve_task, "mqtt_resolve_task", 8192, NULL, 4, NULL);
    xTaskCreate(telemetry_publish_task, "telemetry_publish_task", 6144, NULL, 5, NULL);
}

void hexnet_mqtt_on_sta_got_ip(void)
{
    if (!hexnet_wifi_is_connected()) {
        return;
    }
    if (s_mqtt_connected) {
        return;
    }
    ESP_LOGI(TAG, "STA got IP -> resolve-mqtt + MQTT start");
    if (platform_resolve_mqtt_now(true)) {
        mqtt_start_if_needed();
    } else if (s_mqtt_nvs_trusted && s_mqtt_route_resolved) {
        s_mqtt_connect_allowed = true;
        mqtt_start_if_needed();
    }
}

bool hexnet_mqtt_is_connected(void)
{
    return s_mqtt_connected;
}

bool hexnet_mqtt_has_telemetry_ok(void)
{
    return s_telemetry_tx_count > 0;
}

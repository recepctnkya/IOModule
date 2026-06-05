#include "hexnet_resolve_mqtt.h"

#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "cJSON.h"
#include "hexnet_http.h"
#define LOG_LOCAL_LEVEL HEXNET_LOG_LEVEL_OTHER
#include "hexnet_log.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "HEXNET_RESOLVE";

#define RESOLVE_HTTP_TIMEOUT_MS 20000
#define RESOLVE_HTTP_RESPONSE_MAX 2048

static char s_resolve_http_response[RESOLVE_HTTP_RESPONSE_MAX];
#define RESOLVE_NVS_NAMESPACE "hexnet"

typedef struct {
    char *buffer;
    size_t capacity;
    size_t length;
} http_response_accum_t;

static bool json_truthy(const cJSON *item)
{
    if (!item) {
        return false;
    }
    if (cJSON_IsBool(item)) {
        return cJSON_IsTrue(item);
    }
    if (cJSON_IsNumber(item)) {
        return item->valuedouble != 0.0;
    }
    if (cJSON_IsString(item) && item->valuestring) {
        const char *s = item->valuestring;
        return (strcasecmp(s, "true") == 0) ||
               (strcasecmp(s, "1") == 0) ||
               (strcasecmp(s, "yes") == 0) ||
               (strcasecmp(s, "ok") == 0);
    }
    return false;
}

static void copy_string_field(char *dest, size_t dest_len, const cJSON *item)
{
    if (!dest || dest_len == 0) {
        return;
    }
    dest[0] = '\0';
    if (cJSON_IsString(item) && item->valuestring) {
        snprintf(dest, dest_len, "%s", item->valuestring);
    }
}

static int json_int_field(const cJSON *item, int fallback)
{
    if (cJSON_IsNumber(item)) {
        return item->valueint;
    }
    if (cJSON_IsString(item) && item->valuestring) {
        return atoi(item->valuestring);
    }
    return fallback;
}

static const cJSON *find_topic_key(const cJSON *mqtt, const char *canonical, const char *alt)
{
    if (!mqtt) {
        return NULL;
    }
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(mqtt, canonical);
    if (item && cJSON_IsString(item)) {
        return item;
    }
    if (alt) {
        item = cJSON_GetObjectItemCaseSensitive(mqtt, alt);
        if (item && cJSON_IsString(item)) {
            return item;
        }
    }
    return NULL;
}

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    if (!evt || !evt->user_data) {
        return ESP_OK;
    }
    http_response_accum_t *acc = (http_response_accum_t *)evt->user_data;
    if (evt->event_id != HTTP_EVENT_ON_DATA || !evt->data || evt->data_len <= 0) {
        return ESP_OK;
    }
    if (acc->length >= acc->capacity - 1) {
        return ESP_OK;
    }
    size_t room = (acc->capacity - 1) - acc->length;
    size_t copy_len = (size_t)evt->data_len;
    if (copy_len > room) {
        copy_len = room;
    }
    memcpy(acc->buffer + acc->length, evt->data, copy_len);
    acc->length += copy_len;
    acc->buffer[acc->length] = '\0';
    return ESP_OK;
}

bool hexnet_format_wifi_mac(char *out, size_t out_len)
{
    if (!out || out_len < 18) {
        return false;
    }
    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) {
        return false;
    }
    snprintf(
        out,
        out_len,
        "%02X:%02X:%02X:%02X:%02X:%02X",
        mac[0],
        mac[1],
        mac[2],
        mac[3],
        mac[4],
        mac[5]
    );
    return true;
}

bool hexnet_resolve_mqtt_is_routing_complete(const hexnet_resolve_result_t *result)
{
    if (!result) {
        return false;
    }
    return result->telemetry_topic[0] != '\0' &&
           result->command_topic[0] != '\0' &&
           result->topic_prefix[0] != '\0';
}

static void clear_result(hexnet_resolve_result_t *out)
{
    if (!out) {
        return;
    }
    memset(out, 0, sizeof(*out));
}

static bool parse_resolve_payload(const char *json_text, hexnet_resolve_result_t *out)
{
    if (!json_text || !out) {
        return false;
    }

    cJSON *root = cJSON_Parse(json_text);
    if (!root) {
        snprintf(out->message, sizeof(out->message), "invalid_json");
        return false;
    }

    const cJSON *ok_item = cJSON_GetObjectItemCaseSensitive(root, "ok");
    const cJSON *state_item = cJSON_GetObjectItemCaseSensitive(root, "state");
    const cJSON *msg_item = cJSON_GetObjectItemCaseSensitive(root, "message");
    const cJSON *nested = cJSON_GetObjectItemCaseSensitive(root, "data");

    if (!ok_item && cJSON_IsObject(nested)) {
        ok_item = cJSON_GetObjectItemCaseSensitive(nested, "ok");
        if (!state_item) {
            state_item = cJSON_GetObjectItemCaseSensitive(nested, "state");
        }
        if (!msg_item) {
            msg_item = cJSON_GetObjectItemCaseSensitive(nested, "message");
        }
    }

    out->ok = json_truthy(ok_item);
    copy_string_field(out->state, sizeof(out->state), state_item);
    copy_string_field(out->message, sizeof(out->message), msg_item);

    if (strcasecmp(out->state, "not_registered") == 0) {
        out->registered = false;
        cJSON_Delete(root);
        return true;
    }

    if (strcasecmp(out->state, "rate_limited") == 0) {
        out->registered = false;
        cJSON_Delete(root);
        return true;
    }

    const cJSON *mqtt = cJSON_GetObjectItemCaseSensitive(root, "mqtt");
    if (!mqtt && cJSON_IsObject(nested)) {
        mqtt = cJSON_GetObjectItemCaseSensitive(nested, "mqtt");
    }
    cJSON *mqtt_owned = NULL;
    if (cJSON_IsString(mqtt) && mqtt->valuestring) {
        mqtt_owned = cJSON_Parse(mqtt->valuestring);
        if (mqtt_owned) {
            mqtt = mqtt_owned;
        }
    }

    out->company_id = json_int_field(cJSON_GetObjectItemCaseSensitive(root, "company_id"), 0);
    if (out->company_id <= 0 && cJSON_IsObject(nested)) {
        out->company_id = json_int_field(cJSON_GetObjectItemCaseSensitive(nested, "company_id"), 0);
    }

    out->device_pk_id = json_int_field(cJSON_GetObjectItemCaseSensitive(root, "device_pk_id"), 0);
    if (out->device_pk_id <= 0) {
        out->device_pk_id = json_int_field(cJSON_GetObjectItemCaseSensitive(root, "device_id"), 0);
    }
    if (out->device_pk_id <= 0 && cJSON_IsObject(nested)) {
        out->device_pk_id = json_int_field(cJSON_GetObjectItemCaseSensitive(nested, "device_pk_id"), 0);
        if (out->device_pk_id <= 0) {
            out->device_pk_id = json_int_field(cJSON_GetObjectItemCaseSensitive(nested, "device_id"), 0);
        }
    }

    out->mqtt_segment = json_int_field(cJSON_GetObjectItemCaseSensitive(root, "mqtt_segment"), 0);
    if (out->mqtt_segment <= 0) {
        out->mqtt_segment = json_int_field(
            cJSON_GetObjectItemCaseSensitive(root, "mqtt_effective_device_id"),
            0
        );
    }
    if (out->mqtt_segment <= 0 && cJSON_IsObject(nested)) {
        out->mqtt_segment = json_int_field(cJSON_GetObjectItemCaseSensitive(nested, "mqtt_segment"), 0);
        if (out->mqtt_segment <= 0) {
            out->mqtt_segment = json_int_field(
                cJSON_GetObjectItemCaseSensitive(nested, "mqtt_effective_device_id"),
                0
            );
        }
    }

    copy_string_field(out->device_uid, sizeof(out->device_uid), cJSON_GetObjectItemCaseSensitive(root, "device_uid"));
    if (out->device_uid[0] == '\0' && cJSON_IsObject(nested)) {
        copy_string_field(
            out->device_uid,
            sizeof(out->device_uid),
            cJSON_GetObjectItemCaseSensitive(nested, "device_uid")
        );
    }

    if (cJSON_IsObject(mqtt)) {
        copy_string_field(
            out->topic_prefix,
            sizeof(out->topic_prefix),
            find_topic_key(mqtt, "topic_prefix", "prefix")
        );
        copy_string_field(
            out->telemetry_topic,
            sizeof(out->telemetry_topic),
            find_topic_key(mqtt, "telemetry_topic", "telemetry")
        );
        copy_string_field(
            out->status_topic,
            sizeof(out->status_topic),
            find_topic_key(mqtt, "status_topic", "status")
        );
        copy_string_field(
            out->command_topic,
            sizeof(out->command_topic),
            find_topic_key(mqtt, "command_topic", "command")
        );
        copy_string_field(
            out->command_reply_topic,
            sizeof(out->command_reply_topic),
            find_topic_key(mqtt, "command_reply_topic", "command_reply")
        );
    }

    if (mqtt_owned) {
        cJSON_Delete(mqtt_owned);
    }

    if (out->ok && strcasecmp(out->state, "registered") == 0) {
        out->registered = hexnet_resolve_mqtt_is_routing_complete(out);
    } else if (out->ok && hexnet_resolve_mqtt_is_routing_complete(out)) {
        out->registered = true;
        if (out->state[0] == '\0') {
            snprintf(out->state, sizeof(out->state), "registered");
        }
    } else {
        out->registered = false;
    }

    cJSON_Delete(root);
    return true;
}

esp_err_t hexnet_resolve_mqtt_fetch(
    const char *base_url,
    const char *fw,
    hexnet_resolve_result_t *out
)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }
    clear_result(out);

    const char *base = (base_url && base_url[0] != '\0') ? base_url : HEXNET_PLATFORM_BASE_URL;
    const char *fw_ver = (fw && fw[0] != '\0') ? fw : "unknown";

    char mac_str[24] = {0};
    if (!hexnet_format_wifi_mac(mac_str, sizeof(mac_str))) {
        snprintf(out->message, sizeof(out->message), "mac_read_failed");
        return ESP_FAIL;
    }

    char url[256] = {0};
    snprintf(url, sizeof(url), "%s%s", base, HEXNET_RESOLVE_MQTT_PATH);

    char body[192] = {0};
    snprintf(
        body,
        sizeof(body),
        "{\"mac\":\"%s\",\"fw\":\"%s\",\"hw\":\"%s\"}",
        mac_str,
        fw_ver,
        HEXNET_DEVICE_HW
    );

    memset(s_resolve_http_response, 0, sizeof(s_resolve_http_response));
    http_response_accum_t acc = {
        .buffer = s_resolve_http_response,
        .capacity = sizeof(s_resolve_http_response),
        .length = 0,
    };

    if (!hexnet_http_heap_ok(HEXNET_HTTP_MIN_HEAP_BYTES)) {
        snprintf(out->message, sizeof(out->message), "low_heap");
        return ESP_ERR_NO_MEM;
    }
    if (!hexnet_http_lock(pdMS_TO_TICKS(15000))) {
        snprintf(out->message, sizeof(out->message), "http_busy");
        return ESP_ERR_TIMEOUT;
    }

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = RESOLVE_HTTP_TIMEOUT_MS,
        .event_handler = http_event_handler,
        .user_data = &acc,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        hexnet_http_unlock();
        snprintf(out->message, sizeof(out->message), "http_init_failed");
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_post_field(client, body, (int)strlen(body));

    ESP_LOGI(TAG, "POST %s mac=%s fw=%s hw=%s", url, mac_str, fw_ver, HEXNET_DEVICE_HW);
    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    hexnet_http_unlock();

    if (err != ESP_OK) {
        snprintf(out->message, sizeof(out->message), "http_error:%s", esp_err_to_name(err));
        ESP_LOGW(TAG, "resolve HTTP failed: %s", out->message);
        return err;
    }

    if (status < 200 || status >= 300) {
        snprintf(out->message, sizeof(out->message), "http_status_%d", status);
        ESP_LOGW(TAG, "resolve HTTP status=%d body=%.200s", status, s_resolve_http_response);
        return ESP_FAIL;
    }

    if (!parse_resolve_payload(s_resolve_http_response, out)) {
        ESP_LOGW(TAG, "resolve parse failed body=%.200s", s_resolve_http_response);
        return ESP_FAIL;
    }

    if (out->registered) {
        ESP_LOGI(
            TAG,
            "registered uid=%s company=%d segment=%d telemetry=%s command=%s",
            out->device_uid,
            out->company_id,
            out->mqtt_segment,
            out->telemetry_topic,
            out->command_topic
        );
    } else {
        ESP_LOGW(TAG, "resolve state=%s message=%s", out->state, out->message);
    }

    return ESP_OK;
}

static esp_err_t nvs_put_str_if_nonempty(nvs_handle_t handle, const char *key, const char *value)
{
    if (!value || value[0] == '\0') {
        return ESP_OK;
    }
    return nvs_set_str(handle, key, value);
}

esp_err_t hexnet_resolve_mqtt_save_nvs(const hexnet_resolve_result_t *result)
{
    if (!result || !hexnet_resolve_mqtt_is_routing_complete(result)) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(RESOLVE_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_u8(handle, "resolved", 1);
    if (err == ESP_OK) {
        err = nvs_set_i32(handle, "company_id", result->company_id);
    }
    if (err == ESP_OK) {
        err = nvs_set_i32(handle, "mqtt_segment", result->mqtt_segment);
    }
    if (err == ESP_OK) {
        err = nvs_set_i32(handle, "device_pk_id", result->device_pk_id);
    }
    if (err == ESP_OK) {
        err = nvs_put_str_if_nonempty(handle, "device_uid", result->device_uid);
    }
    if (err == ESP_OK) {
        err = nvs_put_str_if_nonempty(handle, "topic_prefix", result->topic_prefix);
    }
    if (err == ESP_OK) {
        err = nvs_put_str_if_nonempty(handle, "telemetry_topic", result->telemetry_topic);
    }
    if (err == ESP_OK) {
        err = nvs_put_str_if_nonempty(handle, "status_topic", result->status_topic);
    }
    if (err == ESP_OK) {
        err = nvs_put_str_if_nonempty(handle, "command_topic", result->command_topic);
    }
    if (err == ESP_OK) {
        err = nvs_put_str_if_nonempty(handle, "command_reply_topic", result->command_reply_topic);
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t hexnet_resolve_mqtt_load_nvs(hexnet_resolve_result_t *result)
{
    if (!result) {
        return ESP_ERR_INVALID_ARG;
    }
    clear_result(result);

    nvs_handle_t handle;
    esp_err_t err = nvs_open(RESOLVE_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t resolved = 0;
    if (nvs_get_u8(handle, "resolved", &resolved) != ESP_OK || resolved == 0) {
        nvs_close(handle);
        return ESP_ERR_NOT_FOUND;
    }

    int32_t value = 0;
    if (nvs_get_i32(handle, "company_id", &value) == ESP_OK) {
        result->company_id = (int)value;
    }
    if (nvs_get_i32(handle, "mqtt_segment", &value) == ESP_OK) {
        result->mqtt_segment = (int)value;
    }
    if (nvs_get_i32(handle, "device_pk_id", &value) == ESP_OK) {
        result->device_pk_id = (int)value;
    }

    size_t len = sizeof(result->device_uid);
    nvs_get_str(handle, "device_uid", result->device_uid, &len);
    len = sizeof(result->topic_prefix);
    nvs_get_str(handle, "topic_prefix", result->topic_prefix, &len);
    len = sizeof(result->telemetry_topic);
    nvs_get_str(handle, "telemetry_topic", result->telemetry_topic, &len);
    len = sizeof(result->status_topic);
    nvs_get_str(handle, "status_topic", result->status_topic, &len);
    len = sizeof(result->command_topic);
    nvs_get_str(handle, "command_topic", result->command_topic, &len);
    len = sizeof(result->command_reply_topic);
    nvs_get_str(handle, "command_reply_topic", result->command_reply_topic, &len);

    nvs_close(handle);

    result->ok = true;
    result->registered = hexnet_resolve_mqtt_is_routing_complete(result);
    if (result->registered) {
        snprintf(result->state, sizeof(result->state), "registered");
    }
    return result->registered ? ESP_OK : ESP_ERR_NOT_FOUND;
}

void hexnet_resolve_mqtt_clear_nvs(void)
{
    nvs_handle_t handle;
    if (nvs_open(RESOLVE_NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_erase_all(handle);
        nvs_commit(handle);
        nvs_close(handle);
    }
}

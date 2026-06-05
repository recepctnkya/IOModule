#include "hexnet_ota_debug.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

static void log_trunc_payload(char *dst, size_t dst_len, const char *src, int src_len)
{
    if (!dst || dst_len < 8) {
        return;
    }
    if (!src || src_len <= 0) {
        snprintf(dst, dst_len, "(empty)");
        return;
    }
    int copy = src_len;
    if (copy > (int)dst_len - 24) {
        copy = (int)dst_len - 24;
    }
    if (copy < 0) {
        copy = 0;
    }
    memcpy(dst, src, (size_t)copy);
    dst[copy] = '\0';
    if (src_len > copy) {
        snprintf(dst + copy, dst_len - (size_t)copy, "...[%dB]", src_len);
    }
}

#if HEXNET_LOG_OTA

static const char *TAG_OTA = "HEXNET_OTA";

void hexnet_ota_dbg_step(unsigned step, const char *phase, const char *fmt, ...)
{
    char detail[280] = "";
    if (fmt && fmt[0] != '\0') {
        va_list ap;
        va_start(ap, fmt);
        (void)vsnprintf(detail, sizeof(detail), fmt, ap);
        va_end(ap);
    }
    const size_t heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    ESP_LOGI(
        TAG_OTA,
        "[ADIM %02u] %s | %s | heap_free=%u",
        step,
        phase ? phase : "-",
        detail[0] != '\0' ? detail : "-",
        (unsigned)heap);
}

#else

void hexnet_ota_dbg_step(unsigned step, const char *phase, const char *fmt, ...)
{
    (void)step;
    (void)phase;
    (void)fmt;
}

#endif

#if HEXNET_LOG_MQTT

static const char *TAG_MQTT_DBG = "HEXNET_MQTT_DBG";

void hexnet_mqtt_dbg_rx(const char *topic, const char *payload, int payload_len)
{
    char body[400];
    log_trunc_payload(body, sizeof(body), payload, payload_len);
    ESP_LOGI(TAG_MQTT_DBG, "[RX <<] topic=%s len=%d payload=%s", topic ? topic : "-", payload_len, body);
}

void hexnet_mqtt_dbg_tx(const char *topic, const char *payload, int payload_len, int msg_id, int qos)
{
    char body[400];
    log_trunc_payload(body, sizeof(body), payload, payload_len);
    // ESP_LOGI(
    //     TAG_MQTT_DBG,
    //     "[TX >>] topic=%s qos=%d msg_id=%d len=%d payload=%s",
    //     topic ? topic : "-",
    //     qos,
    //     msg_id,
    //     payload_len,
    //     body);
}

void hexnet_mqtt_dbg_event(const char *event_name, const char *detail_fmt, ...)
{
    char detail[200] = "";
    if (detail_fmt && detail_fmt[0] != '\0') {
        va_list ap;
        va_start(ap, detail_fmt);
        (void)vsnprintf(detail, sizeof(detail), detail_fmt, ap);
        va_end(ap);
    }
    ESP_LOGI(
        TAG_MQTT_DBG,
        "[EVT] %s%s%s",
        event_name ? event_name : "?",
        detail[0] != '\0' ? " | " : "",
        detail[0] != '\0' ? detail : "");
}

#else

void hexnet_mqtt_dbg_rx(const char *topic, const char *payload, int payload_len)
{
    (void)topic;
    (void)payload;
    (void)payload_len;
}

void hexnet_mqtt_dbg_tx(const char *topic, const char *payload, int payload_len, int msg_id, int qos)
{
    (void)topic;
    (void)payload;
    (void)payload_len;
    (void)msg_id;
    (void)qos;
}

void hexnet_mqtt_dbg_event(const char *event_name, const char *detail_fmt, ...)
{
    (void)event_name;
    (void)detail_fmt;
}

#endif

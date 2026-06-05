/*
 * VANGO MQTT Handler – HexNet Technology
 *
 * Publishes sensor telemetry every CONFIG_MQTT_TELEMETRY_INTERVAL_MS ms.
 * Applies control commands received on the /control topic immediately.
 */

#include "mqtt_handler.h"

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "mqtt_client.h"
#include "cJSON.h"

#include "hexnet_canbus.h"
#include "adc_monitor.h"

/* ------------------------------------------------------------------ */
/*  Constants                                                          */
/* ------------------------------------------------------------------ */

#define TAG "MQTT"

#define MQTT_CONNECTED_BIT  BIT0

/* ------------------------------------------------------------------ */
/*  State                                                              */
/* ------------------------------------------------------------------ */

static esp_mqtt_client_handle_t s_client      = NULL;
static EventGroupHandle_t       s_mqtt_eg;
static char                     s_topic_tel[96];
static char                     s_topic_ctrl[96];
static char                     s_device_id[16];  /* e.g. "VANGO-AB12CD" */

/* ------------------------------------------------------------------ */
/*  Topic / device-ID helpers                                          */
/* ------------------------------------------------------------------ */

static void build_topics(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    /* Short human-readable device ID shown in telemetry */
    snprintf(s_device_id, sizeof(s_device_id),
             "VANGO-%02X%02X%02X", mac[3], mac[4], mac[5]);

    /* Full MAC for unique per-device topics */
    char mac_hex[13];
    snprintf(mac_hex, sizeof(mac_hex), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    snprintf(s_topic_tel,  sizeof(s_topic_tel),
             CONFIG_MQTT_TOPIC_PREFIX "/%s/telemetry", mac_hex);
    snprintf(s_topic_ctrl, sizeof(s_topic_ctrl),
             CONFIG_MQTT_TOPIC_PREFIX "/%s/control", mac_hex);
}

/* ------------------------------------------------------------------ */
/*  Control message handler                                            */
/* ------------------------------------------------------------------ */

static void apply_control(const char *data, int len)
{
    cJSON *root = cJSON_ParseWithLength(data, len);
    if (!root) {
        ESP_LOGW(TAG, "Invalid control JSON – ignored");
        return;
    }

    cJSON *it;

    /* Digital outputs – full 16-bit bitmask */
    if ((it = cJSON_GetObjectItem(root, "outputs")) && cJSON_IsNumber(it))
        set_outputs((uint16_t)it->valueint);

    /* Digital outputs – individual bits  out_0 … out_15 */
    {
        char key[8];
        uint16_t cur = get_outputs();
        bool changed = false;
        for (int b = 0; b < 16; b++) {
            snprintf(key, sizeof(key), "out_%d", b);
            if ((it = cJSON_GetObjectItem(root, key)) && cJSON_IsNumber(it)) {
                if (it->valueint) cur |=  (uint16_t)(1u << b);
                else              cur &= ~(uint16_t)(1u << b);
                changed = true;
            }
        }
        if (changed) set_outputs(cur);
    }

    /* PWM / dimmable channels 0-3 */
    if ((it = cJSON_GetObjectItem(root, "pwm_0")) && cJSON_IsNumber(it))
        set_dimmable_output(0, (uint8_t)it->valueint);
    if ((it = cJSON_GetObjectItem(root, "pwm_1")) && cJSON_IsNumber(it))
        set_dimmable_output(1, (uint8_t)it->valueint);
    if ((it = cJSON_GetObjectItem(root, "pwm_2")) && cJSON_IsNumber(it))
        set_dimmable_output(2, (uint8_t)it->valueint);
    if ((it = cJSON_GetObjectItem(root, "pwm_3")) && cJSON_IsNumber(it))
        set_dimmable_output(3, (uint8_t)it->valueint);

    /* RGB – read current first so partial updates work */
    uint8_t rr = get_r_value(), gg = get_g_value(),
            bb = get_b_value(), en = get_rgb_enable();
    bool rgb_changed = false;
    if ((it = cJSON_GetObjectItem(root, "rgb_r"))  && cJSON_IsNumber(it)) { rr = (uint8_t)it->valueint; rgb_changed = true; }
    if ((it = cJSON_GetObjectItem(root, "rgb_g"))  && cJSON_IsNumber(it)) { gg = (uint8_t)it->valueint; rgb_changed = true; }
    if ((it = cJSON_GetObjectItem(root, "rgb_b"))  && cJSON_IsNumber(it)) { bb = (uint8_t)it->valueint; rgb_changed = true; }
    if ((it = cJSON_GetObjectItem(root, "rgb_en")) && cJSON_IsNumber(it)) { en = (uint8_t)it->valueint; rgb_changed = true; }
    if (rgb_changed) set_rgb(rr, gg, bb, en);

    /* Motor: 0=stop, 1=forward, 2=backward */
    if ((it = cJSON_GetObjectItem(root, "motor")) && cJSON_IsNumber(it))
        set_motordata(it->valueint);

    cJSON_Delete(root);
    ESP_LOGI(TAG, "Control applied");
}

/* ------------------------------------------------------------------ */
/*  Telemetry publisher                                                */
/* ------------------------------------------------------------------ */

static void publish_telemetry(void)
{
    int wl1 = 0, wl2 = 0, wl3 = 0, wl4 = 0;
    get_average_water_levels(&wl1, &wl2, &wl3, &wl4);

    cJSON *root = cJSON_CreateObject();
    if (!root) return;

    cJSON_AddStringToObject(root, "device",    s_device_id);
    cJSON_AddNumberToObject(root, "uptime_s",  (double)(esp_timer_get_time() / 1000000LL));

    /* Environmental sensors */
    cJSON_AddNumberToObject(root, "temp",      get_sensorTemp());
    cJSON_AddNumberToObject(root, "humidity",  get_sensorHumidity());
    cJSON_AddNumberToObject(root, "water_1",   wl1);
    cJSON_AddNumberToObject(root, "water_2",   wl2);
    cJSON_AddNumberToObject(root, "water_3",   wl3);
    cJSON_AddNumberToObject(root, "water_4",   wl4);
    cJSON_AddNumberToObject(root, "battery_v", (double)get_battery_v());

    /* Output states */
    cJSON_AddNumberToObject(root, "outputs",   get_outputs());
    cJSON_AddNumberToObject(root, "pwm_0",     get_dimmable_output(0));
    cJSON_AddNumberToObject(root, "pwm_1",     get_dimmable_output(1));
    cJSON_AddNumberToObject(root, "pwm_2",     get_dimmable_output(2));
    cJSON_AddNumberToObject(root, "pwm_3",     get_dimmable_output(3));
    cJSON_AddNumberToObject(root, "rgb_r",     get_r_value());
    cJSON_AddNumberToObject(root, "rgb_g",     get_g_value());
    cJSON_AddNumberToObject(root, "rgb_b",     get_b_value());
    cJSON_AddNumberToObject(root, "rgb_en",    get_rgb_enable());
    cJSON_AddNumberToObject(root, "motor",     get_motorData());

    char *js = cJSON_PrintUnformatted(root);
    if (js) {
        esp_mqtt_client_publish(s_client, s_topic_tel, js, 0, /*qos*/1, /*retain*/0);
        ESP_LOGD(TAG, "Telemetry → %s", s_topic_tel);
        free(js);
    }
    cJSON_Delete(root);
}

/* ------------------------------------------------------------------ */
/*  MQTT event handler                                                 */
/* ------------------------------------------------------------------ */

static void mqtt_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t ev = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {

        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Connected – subscribing to %s", s_topic_ctrl);
            esp_mqtt_client_subscribe(s_client, s_topic_ctrl, 1);
            xEventGroupSetBits(s_mqtt_eg, MQTT_CONNECTED_BIT);
            /* Publish an immediate telemetry frame on connect */
            publish_telemetry();
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Disconnected – will auto-reconnect");
            xEventGroupClearBits(s_mqtt_eg, MQTT_CONNECTED_BIT);
            break;

        case MQTT_EVENT_DATA:
            /* Check it is our control topic */
            if (ev->topic && ev->topic_len > 0 &&
                strncmp(ev->topic, s_topic_ctrl, ev->topic_len) == 0) {
                apply_control(ev->data, ev->data_len);
            }
            break;

        case MQTT_EVENT_ERROR:
            if (ev->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGE(TAG, "TCP error %d", ev->error_handle->esp_transport_sock_errno);
            }
            break;

        default:
            break;
    }
}

/* ------------------------------------------------------------------ */
/*  Telemetry task                                                     */
/* ------------------------------------------------------------------ */

static void mqtt_telemetry_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_MQTT_TELEMETRY_INTERVAL_MS));

        if (xEventGroupGetBits(s_mqtt_eg) & MQTT_CONNECTED_BIT) {
            publish_telemetry();
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

void mqtt_handler_start(void)
{
    s_mqtt_eg = xEventGroupCreate();
    build_topics();

    ESP_LOGI(TAG, "Device   : %s",  s_device_id);
    ESP_LOGI(TAG, "Broker   : %s",  CONFIG_MQTT_BROKER_URL);
    ESP_LOGI(TAG, "Telemetry: %s",  s_topic_tel);
    ESP_LOGI(TAG, "Control  : %s",  s_topic_ctrl);
    ESP_LOGI(TAG, "Interval : %d ms", CONFIG_MQTT_TELEMETRY_INTERVAL_MS);

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = CONFIG_MQTT_BROKER_URL,
    };

    s_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_client);

    xTaskCreate(mqtt_telemetry_task, "mqtt_tel", 4096, NULL, 4, NULL);
}

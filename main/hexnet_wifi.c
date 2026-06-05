#include "hexnet_wifi.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

#define LOG_LOCAL_LEVEL HEXNET_LOG_LEVEL_WIFI
#include "hexnet_log.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hexnet_debug.h"
#include "hexnet_http.h"
#include "hexnet_mqtt.h"
#include "hexnet_resolve_mqtt.h"
#include "ota_manager.h"
#include "hexnet_wifi_portal.h"
#include "sdkconfig.h"

static const char *TAG = "HEXNET_WIFI";

#define NTP_VALID_UNIX_THRESHOLD 1700000000
#define WIFI_SUPERVISOR_MS       10000
#define INTERNET_PROBE_TIMEOUT_MS 5000
#define INET_RECHECK_OK_MS       120000
#define STA_BOOT_CONNECT_ATTEMPTS 10
#define INET_FAIL_PORTAL_TICKS   3
#define PORTAL_TEARDOWN_PROBE_MAX  15
#define WIFI_SUP_STACK           16384
#define WIFI_MQTT_UP_STACK       12288
#define WIFI_PORTAL_OFF_STACK    8192

static bool s_wifi_connected = false;
static bool s_internet_ok = false;
static bool s_services_started = false;
static bool s_setup_mode_pending = false;
static bool s_reported_online = false;
static bool s_ntp_started = false;
static char s_sta_ip[16] = {0};
static char s_sta_ssid[33] = {0};
static int s_sta_boot_attempts = 0;
static bool s_boot_portal_opened = false;
static bool s_first_sta_ip_obtained = false;
static int s_inet_fail_streak = 0;
static int64_t s_last_inet_ok_probe_ms = 0;
static TaskHandle_t s_wifi_sup_task = NULL;
static TaskHandle_t s_portal_teardown_task = NULL;
static int64_t s_ntp_epoch_at_sync = 0;
static int64_t s_ntp_mono_ms_at_sync = 0;

typedef enum {
    INET_PROBE_OK = 0,
    INET_PROBE_FAIL,
    INET_PROBE_SKIP,
} inet_probe_result_t;

static inet_probe_result_t run_internet_probe(void);
static esp_event_handler_instance_t s_wifi_evt_instance = NULL;
static esp_event_handler_instance_t s_ip_evt_instance = NULL;

static bool sta_credentials_configured(void)
{
    wifi_config_t cfg = {0};
    if (esp_wifi_get_config(WIFI_IF_STA, &cfg) != ESP_OK) {
        return false;
    }
    return cfg.sta.ssid[0] != '\0';
}

static void cache_sta_ssid(void)
{
    wifi_config_t cfg = {0};
    if (esp_wifi_get_config(WIFI_IF_STA, &cfg) == ESP_OK) {
        strlcpy(s_sta_ssid, (const char *)cfg.sta.ssid, sizeof(s_sta_ssid));
    }
}

static void mqtt_bringup_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(100));
    hexnet_mqtt_on_sta_got_ip();
    vTaskDelete(NULL);
}

static void start_online_services(void)
{
    if (s_services_started) {
        return;
    }
    s_services_started = true;
    s_internet_ok = true;
    s_boot_portal_opened = false;
    hexnet_wifi_portal_mark_provisioned();
    if (hexnet_wifi_portal_is_active()) {
        hexnet_wifi_portal_shutdown();
        hexnet_debug_log("Internet OK; kurulum AP kapandi, STA Wi-Fi, MQTT baslatiliyor");
    } else {
        hexnet_debug_log("Internet OK; STA Wi-Fi, MQTT baslatiliyor");
    }
    xTaskCreate(mqtt_bringup_task, "mqtt_sta_up", WIFI_MQTT_UP_STACK, NULL, 5, NULL);
}

static void close_setup_ap_for_sta(void)
{
    if (!hexnet_wifi_portal_is_active() && !s_boot_portal_opened) {
        return;
    }
    s_boot_portal_opened = false;
    hexnet_wifi_portal_shutdown();
    vTaskDelay(pdMS_TO_TICKS(300));
    if (sta_credentials_configured()) {
        esp_wifi_connect();
    }
}

static void wifi_portal_teardown_task(void *arg)
{
    (void)arg;
    bool ap_closed_for_probe = false;

    for (int attempt = 0; attempt < PORTAL_TEARDOWN_PROBE_MAX; attempt++) {
        if (!s_wifi_connected) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        const inet_probe_result_t probe = run_internet_probe();
        if (probe == INET_PROBE_OK) {
            start_online_services();
            break;
        }
        if (probe == INET_PROBE_SKIP && s_internet_ok) {
            start_online_services();
            break;
        }

        if (probe == INET_PROBE_FAIL &&
            !ap_closed_for_probe &&
            (hexnet_wifi_portal_is_active() || s_boot_portal_opened)) {
            hexnet_debug_log("Internet testi icin kurulum AP kapatiliyor (STA-only)");
            close_setup_ap_for_sta();
            ap_closed_for_probe = true;
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    if (s_wifi_connected && (hexnet_wifi_portal_is_active() || s_boot_portal_opened)) {
        close_setup_ap_for_sta();
    }

    s_portal_teardown_task = NULL;
    vTaskDelete(NULL);
}

static void schedule_portal_teardown_after_sta_ip(void)
{
    if (!hexnet_wifi_portal_is_active() && !s_boot_portal_opened) {
        return;
    }
    if (s_portal_teardown_task != NULL) {
        return;
    }
    if (xTaskCreate(wifi_portal_teardown_task, "wifi_portal_off", WIFI_PORTAL_OFF_STACK, NULL, 5,
                    &s_portal_teardown_task) != pdPASS) {
        s_portal_teardown_task = NULL;
        ESP_LOGW(TAG, "portal teardown task create failed");
    }
}

static void open_boot_setup_portal(const char *reason)
{
    if (s_boot_portal_opened || hexnet_wifi_portal_is_active()) {
        return;
    }
    s_boot_portal_opened = true;
    hexnet_debug_log("%s", reason);
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(400));
    hexnet_wifi_portal_open();
}

static void handle_boot_sta_disconnect(void)
{
    if (s_first_sta_ip_obtained) {
        return;
    }
    if (hexnet_wifi_portal_is_active()) {
        if (sta_credentials_configured()) {
            esp_wifi_connect();
        }
        return;
    }
    if (s_boot_portal_opened) {
        return;
    }
    if (!sta_credentials_configured()) {
        open_boot_setup_portal("Kayitli ag yok; kurulum portali aciliyor");
        return;
    }

    s_sta_boot_attempts++;
    hexnet_debug_log(
        "Kayitli aga baglanilamadi (%d/%d): \"%s\"",
        s_sta_boot_attempts,
        STA_BOOT_CONNECT_ATTEMPTS,
        s_sta_ssid[0] ? s_sta_ssid : "-");

    if (s_sta_boot_attempts >= STA_BOOT_CONNECT_ATTEMPTS) {
        open_boot_setup_portal(
            "Kayitli aga 10 deneme basarisiz; kurulum portali aciliyor -> http://" HEXNET_PORTAL_IP);
        return;
    }
    esp_wifi_connect();
}

static void request_setup_mode(const char *reason)
{
    if (s_setup_mode_pending || hexnet_wifi_portal_is_active()) {
        return;
    }
    if (!hexnet_http_heap_ok(HEXNET_HTTP_MIN_HEAP_BYTES)) {
        ESP_LOGW(TAG, "Kurulum modu ertelendi: dusuk heap (%lu)",
                 (unsigned long)esp_get_free_heap_size());
        return;
    }
    hexnet_debug_log("%s", reason);
    s_setup_mode_pending = true;
    s_inet_fail_streak = 0;
    s_reported_online = false;
    s_internet_ok = false;
    if (s_services_started) {
        hexnet_mqtt_stop();
    }
    s_services_started = false;
    esp_wifi_disconnect();
}

static inet_probe_result_t run_internet_probe(void)
{
    if (ota_manager_is_active()) {
        return INET_PROBE_SKIP;
    }
    if (!hexnet_http_heap_ok(HEXNET_HTTP_MIN_HEAP_BYTES)) {
        ESP_LOGW(TAG, "Internet probe atlandi: dusuk heap (%lu)",
                 (unsigned long)esp_get_free_heap_size());
        return INET_PROBE_SKIP;
    }
    if (!hexnet_http_lock(pdMS_TO_TICKS(8000))) {
        ESP_LOGW(TAG, "Internet probe atlandi: HTTP mesgul");
        return INET_PROBE_SKIP;
    }

    const bool ok = hexnet_wifi_has_internet();
    hexnet_http_unlock();
    return ok ? INET_PROBE_OK : INET_PROBE_FAIL;
}

static void run_connectivity_check(void)
{
    if (!s_wifi_connected) {
        return;
    }

    const int64_t now_ms = esp_timer_get_time() / 1000LL;
    if (s_internet_ok && s_services_started && s_last_inet_ok_probe_ms > 0 &&
        (now_ms - s_last_inet_ok_probe_ms) < INET_RECHECK_OK_MS) {
        start_online_services();
        return;
    }

    const inet_probe_result_t probe = run_internet_probe();
    if (probe == INET_PROBE_SKIP) {
        if (s_internet_ok) {
            start_online_services();
        }
        return;
    }

    if (probe == INET_PROBE_OK) {
        s_inet_fail_streak = 0;
        s_internet_ok = true;
        s_last_inet_ok_probe_ms = now_ms;
        if (!s_reported_online) {
            hexnet_debug_log(
                "Durum: online | SSID=%s | IP=%s | internet=OK",
                s_sta_ssid[0] ? s_sta_ssid : "-",
                s_sta_ip[0] ? s_sta_ip : "-");
            s_reported_online = true;
        }
        start_online_services();
        return;
    }

    s_internet_ok = false;
    s_inet_fail_streak++;
    hexnet_debug_log(
        "Internet probe basarisiz (%d/%d) | IP=%s",
        s_inet_fail_streak,
        INET_FAIL_PORTAL_TICKS,
        s_sta_ip[0] ? s_sta_ip : "-");

    if (s_inet_fail_streak < INET_FAIL_PORTAL_TICKS) {
        return;
    }

    request_setup_mode("Internet yok; kurulum moduna geciliyor");
}

static esp_err_t http_probe_event(esp_http_client_event_t *evt)
{
    (void)evt;
    return ESP_OK;
}

bool hexnet_wifi_has_internet(void)
{
    if (!s_wifi_connected) {
        return false;
    }
    if (!hexnet_http_heap_ok(HEXNET_HTTP_MIN_HEAP_BYTES)) {
        return s_internet_ok;
    }

    char url[96];
    snprintf(url, sizeof(url), "%s/", HEXNET_PLATFORM_BASE_URL);

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_HEAD,
        .timeout_ms = INTERNET_PROBE_TIMEOUT_MS,
        .event_handler = http_probe_event,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (client == NULL) {
        ESP_LOGW(TAG, "Internet probe: http client alloc failed");
        return s_internet_ok;
    }

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    return (err == ESP_OK && status > 0 && status < 500);
}

static void on_sta_online(const char *ip_str)
{
    hexnet_wifi_start_ntp_if_needed();
    hexnet_debug_log("Wi-Fi baglandi. IP: %s", ip_str);
    ESP_LOGI(TAG, "STA IP %s", ip_str);
    schedule_portal_teardown_after_sta_ip();
    if (s_wifi_sup_task != NULL) {
        xTaskNotifyGive(s_wifi_sup_task);
    }
}

static void wifi_supervisor_task(void *arg)
{
    (void)arg;
    s_wifi_sup_task = xTaskGetCurrentTaskHandle();
    vTaskDelay(pdMS_TO_TICKS(3000));

    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(WIFI_SUPERVISOR_MS));

        const bool creds = sta_credentials_configured();
        cache_sta_ssid();

        if (s_wifi_connected) {
            run_connectivity_check();
            continue;
        }

        s_internet_ok = false;
        s_services_started = false;
        hexnet_mqtt_stop();

        if (!s_first_sta_ip_obtained && !s_boot_portal_opened && !hexnet_wifi_portal_is_active()) {
            if (creds) {
                if (s_sta_boot_attempts >= STA_BOOT_CONNECT_ATTEMPTS) {
                    open_boot_setup_portal(
                        "Kayitli aga baglanilamadi; kurulum portali aciliyor");
                } else {
                    hexnet_debug_log("Flash SSID: \"%s\" — baglaniliyor...", s_sta_ssid);
                    esp_wifi_connect();
                }
            } else {
                open_boot_setup_portal("Flash'ta Wi-Fi kaydi yok; kurulum portali aciliyor");
            }
        }
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            cache_sta_ssid();
            if (sta_credentials_configured()) {
                s_sta_boot_attempts = 0;
                s_boot_portal_opened = false;
                hexnet_debug_log(
                    "STA basladi; flash agina baglaniliyor (en fazla %d deneme): \"%s\"",
                    STA_BOOT_CONNECT_ATTEMPTS,
                    s_sta_ssid);
                esp_wifi_connect();
            } else {
                open_boot_setup_portal("STA basladi; kayitli ag yok -> kurulum modu");
            }
        } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
            hexnet_debug_log("Wi-Fi agina baglandi: \"%s\"", s_sta_ssid);
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            const wifi_event_sta_disconnected_t *disc =
                (const wifi_event_sta_disconnected_t *)event_data;
            s_wifi_connected = false;
            s_internet_ok = false;
            s_services_started = false;
            s_reported_online = false;
            s_sta_ip[0] = '\0';
            hexnet_mqtt_stop();
            hexnet_debug_log(
                "Wi-Fi koptu (neden=%d)",
                disc ? (int)disc->reason : -1);

            if (s_setup_mode_pending) {
                s_setup_mode_pending = false;
                vTaskDelay(pdMS_TO_TICKS(300));
                hexnet_wifi_portal_open();
                return;
            }

            if (!s_first_sta_ip_obtained) {
                handle_boot_sta_disconnect();
                return;
            }

            if (sta_credentials_configured()) {
                hexnet_debug_log("Yeniden baglaniliyor...");
                esp_wifi_connect();
            }
        } else if (event_id == WIFI_EVENT_AP_START) {
            hexnet_debug_log("Kurulum AP: \"%s\" -> http://%s",
                             hexnet_wifi_portal_get_ap_ssid(),
                             HEXNET_PORTAL_IP);
            hexnet_wifi_portal_on_ap_started();
        } else if (event_id == WIFI_EVENT_SCAN_DONE) {
            hexnet_wifi_portal_on_scan_done();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)event_data;
        s_wifi_connected = true;
        s_first_sta_ip_obtained = true;
        s_sta_boot_attempts = 0;
        snprintf(s_sta_ip, sizeof(s_sta_ip), IPSTR, IP2STR(&event->ip_info.ip));
        on_sta_online(s_sta_ip);
    }
}

void hexnet_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    esp_err_t loop_ret = esp_event_loop_create_default();
    if (loop_ret != ESP_OK && loop_ret != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(loop_ret);
    }

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &s_wifi_evt_instance));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &s_ip_evt_instance));

    wifi_config_t wifi_cfg = {0};
    if (esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg) != ESP_OK) {
        ESP_LOGW(TAG, "Wi-Fi get_config failed");
    }

    const bool creds = (wifi_cfg.sta.ssid[0] != '\0');
    strlcpy(s_sta_ssid, (const char *)wifi_cfg.sta.ssid, sizeof(s_sta_ssid));

    if (creds) {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        hexnet_debug_log("Acilis: flash'tan SSID=\"%s\" okundu, STA modu", s_sta_ssid);
        ESP_LOGI(TAG, "STA from flash: %s", s_sta_ssid);
    } else {
        esp_netif_create_default_wifi_ap();
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
        hexnet_wifi_portal_enable_ap();
        s_boot_portal_opened = true;
        hexnet_debug_log("Acilis: kayitli Wi-Fi yok; kurulum modu http://%s", HEXNET_PORTAL_IP);
        ESP_LOGI(TAG, "Setup portal: http://%s", HEXNET_PORTAL_IP);
    }

    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_max_tx_power(32);

    xTaskCreate(wifi_supervisor_task, "wifi_sup", WIFI_SUP_STACK, NULL, 4, NULL);
}

bool hexnet_wifi_is_connected(void)
{
    return s_wifi_connected;
}

bool hexnet_wifi_is_internet_ok(void)
{
    return s_internet_ok;
}

bool hexnet_wifi_has_saved_credentials(void)
{
    return sta_credentials_configured();
}

const char *hexnet_wifi_get_sta_ip(void)
{
    return s_sta_ip;
}

const char *hexnet_wifi_get_sta_ssid(void)
{
    cache_sta_ssid();
    return s_sta_ssid;
}

int hexnet_wifi_get_sta_rssi(void)
{
    wifi_ap_record_t ap = {0};
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        return (int)ap.rssi;
    }
    return 0;
}

static void sntp_time_sync_notification_cb(struct timeval *tv)
{
    if (tv == NULL || tv->tv_sec < NTP_VALID_UNIX_THRESHOLD) {
        return;
    }
    s_ntp_epoch_at_sync = (int64_t)tv->tv_sec;
    s_ntp_mono_ms_at_sync = esp_timer_get_time() / 1000LL;
}

void hexnet_wifi_start_ntp_if_needed(void)
{
    if (s_ntp_started) {
        return;
    }
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.google.com");
    esp_sntp_set_time_sync_notification_cb(sntp_time_sync_notification_cb);
    esp_sntp_init();
    s_ntp_started = true;
}

bool hexnet_wifi_ntp_is_synced(void)
{
    time_t now = 0;
    time(&now);
    if (now < NTP_VALID_UNIX_THRESHOLD) {
        return false;
    }
    sntp_sync_status_t st = esp_sntp_get_sync_status();
    return st == SNTP_SYNC_STATUS_COMPLETED;
}

int64_t hexnet_wifi_ntp_unix_ts_sec(void)
{
    time_t now = 0;
    time(&now);
    if (now >= NTP_VALID_UNIX_THRESHOLD) {
        s_ntp_epoch_at_sync = (int64_t)now;
        s_ntp_mono_ms_at_sync = esp_timer_get_time() / 1000LL;
        return (int64_t)now;
    }
    if (s_ntp_epoch_at_sync >= NTP_VALID_UNIX_THRESHOLD && s_ntp_mono_ms_at_sync > 0) {
        const int64_t mono_ms = esp_timer_get_time() / 1000LL;
        return s_ntp_epoch_at_sync + (mono_ms - s_ntp_mono_ms_at_sync) / 1000LL;
    }
    return 0;
}

#include "OtaManager.hpp"
#include "ota_manager.h"

#include <cstring>
#include <strings.h>

#include "cJSON.h"
#include "esp_app_format.h"
#include "esp_https_ota.h"
#include "hexnet_log.h"
#define LOG_LOCAL_LEVEL HEXNET_LOG_LEVEL_OTA
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "hexnet_mqtt.h"
#include "hexnet_ota_debug.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "OtaManager";

static SemaphoreHandle_t s_status_mutex;
static TaskHandle_t s_ota_task;

/** HTTPS/TLS icin yeterli heap (halt sonrasi ~20KB; destroy yalnizca dusuk heap'te). */
static constexpr size_t kOtaMinHeapForHttps = 20000;

static bool ota_wait_heap(size_t min_free, int attempts)
{
    for (int i = 0; i < attempts; i++) {
        if (esp_get_free_heap_size() >= min_free) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    return esp_get_free_heap_size() >= min_free;
}
static bool s_start_pending;

/** Platform "v0.50" / "0.50.0" — baştaki v ve sondaki .0 kırp (tam eşleşme gerektirmez). */
static void normalize_version_label(const char *in, char *out, size_t out_len)
{
    if (!out || out_len == 0) {
        return;
    }
    out[0] = '\0';
    if (!in || in[0] == '\0') {
        return;
    }
    while (*in == ' ' || *in == '\t') {
        in++;
    }
    if (*in == 'v' || *in == 'V') {
        in++;
    }
    size_t n = 0;
    while (in[n] != '\0' && n + 1 < out_len) {
        out[n] = in[n];
        n++;
    }
    out[n] = '\0';
    while (n >= 2 && out[n - 1] == '0' && out[n - 2] == '.') {
        out[n - 2] = '\0';
        n -= 2;
    }
}

static bool versions_equivalent(const char *a, const char *b)
{
    char na[64] = {};
    char nb[64] = {};
    normalize_version_label(a, na, sizeof(na));
    normalize_version_label(b, nb, sizeof(nb));
    if (na[0] == '\0' || nb[0] == '\0') {
        return false;
    }
    return strcmp(na, nb) == 0;
}

static bool json_bool_like(const cJSON *item, bool default_val)
{
    if (!item) {
        return default_val;
    }
    if (cJSON_IsBool(item)) {
        return cJSON_IsTrue(item);
    }
    if (cJSON_IsNumber(item)) {
        return item->valueint != 0;
    }
    if (cJSON_IsString(item) && item->valuestring) {
        return (strcmp(item->valuestring, "1") == 0) ||
               (strcasecmp(item->valuestring, "true") == 0) ||
               (strcasecmp(item->valuestring, "on") == 0);
    }
    return default_val;
}

OtaManager &OtaManager::instance()
{
    static OtaManager inst;
    return inst;
}

const char *OtaManager::stateToString(OtaState state)
{
    switch (state) {
    case OtaState::Idle:
        return "idle";
    case OtaState::Requested:
        return "requested";
    case OtaState::Downloading:
        return "downloading";
    case OtaState::Writing:
        return "writing";
    case OtaState::Verifying:
        return "verifying";
    case OtaState::SetBoot:
        return "set_boot";
    case OtaState::Rebooting:
        return "rebooting";
    case OtaState::Success:
        return "success";
    case OtaState::Failed:
        return "failed";
    default:
        return "idle";
    }
}

void OtaManager::setState(OtaState state)
{
    state_ = state;
}

void OtaManager::setProgress(int pct)
{
    if (pct < 0) {
        pct = 0;
    }
    if (pct > 100) {
        pct = 100;
    }
    progress_ = pct;
}

void OtaManager::setLastError(const char *msg)
{
    if (!msg) {
        last_error_[0] = '\0';
        return;
    }
    strncpy(last_error_, msg, sizeof(last_error_) - 1);
    last_error_[sizeof(last_error_) - 1] = '\0';
}

void OtaManager::init()
{
    if (s_status_mutex == nullptr) {
        s_status_mutex = xSemaphoreCreateMutex();
    }

    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running != nullptr) {
        esp_ota_img_states_t img_state;
        if (esp_ota_get_state_partition(running, &img_state) == ESP_OK &&
            img_state == ESP_OTA_IMG_PENDING_VERIFY) {
            esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
            ESP_LOGI(TAG, "Marked OTA image valid (rollback cancelled): %s", esp_err_to_name(err));
        }
        ESP_LOGI(
            TAG,
            "Running partition: %s @ 0x%lx size 0x%lx",
            running->label,
            (unsigned long)running->address,
            (unsigned long)running->size);
    }

    setState(OtaState::Idle);
    setProgress(0);
    setLastError("");
    target_version_[0] = '\0';
    ota_busy_ = false;
    s_start_pending = false;

    if (s_ota_task == nullptr) {
        xTaskCreate(
            [](void *arg) {
                OtaManager::instance().runOtaTask(arg);
            },
            "ota_manager",
            20480,
            nullptr,
            5,
            &s_ota_task);
    }
}

bool OtaManager::requestStart(const char *url, const char *version, bool force, const char *request_id)
{
    hexnet_ota_dbg_step(
        1,
        "requestStart",
        "url=%s ver=%s force=%d request_id=%s",
        url ? url : "-",
        version ? version : "-",
        force ? 1 : 0,
        request_id ? request_id : "-");

    if (url == nullptr || url[0] == '\0') {
        setLastError("empty url");
        setState(OtaState::Failed);
        hexnet_ota_dbg_step(2, "requestStart", "REJECT empty url");
        return false;
    }

    if (xSemaphoreTake(s_status_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        hexnet_ota_dbg_step(2, "requestStart", "REJECT mutex timeout");
        return false;
    }

    if (ota_busy_ && !force) {
        setLastError("ota already in progress");
        xSemaphoreGive(s_status_mutex);
        hexnet_ota_dbg_step(2, "requestStart", "REJECT ota_busy");
        return false;
    }

    if (state_ == OtaState::Failed && force) {
        setState(OtaState::Idle);
        setProgress(0);
        setLastError("");
        ota_busy_ = false;
    }

    const esp_app_desc_t *app = esp_app_get_description();
    if (!force && version != nullptr && version[0] != '\0' && app != nullptr &&
        app->version[0] != '\0' && versions_equivalent(app->version, version)) {
        setLastError("same version (use force to override)");
        setState(OtaState::Failed);
        xSemaphoreGive(s_status_mutex);
        return false;
    }

    strncpy(pending_url_, url, sizeof(pending_url_) - 1);
    pending_url_[sizeof(pending_url_) - 1] = '\0';

    if (version != nullptr) {
        strncpy(pending_version_, version, sizeof(pending_version_) - 1);
        pending_version_[sizeof(pending_version_) - 1] = '\0';
    } else {
        pending_version_[0] = '\0';
    }

    pending_force_ = force;
    pending_request_id_[0] = '\0';
    if (request_id != nullptr && request_id[0] != '\0') {
        strncpy(pending_request_id_, request_id, sizeof(pending_request_id_) - 1);
        pending_request_id_[sizeof(pending_request_id_) - 1] = '\0';
    }
    strncpy(target_version_, pending_version_, sizeof(target_version_) - 1);
    target_version_[sizeof(target_version_) - 1] = '\0';

    setProgress(0);
    setLastError("");
    setState(OtaState::Requested);
    ota_busy_ = true;
    s_start_pending = true;

    xSemaphoreGive(s_status_mutex);
    hexnet_ota_dbg_step(3, "requestStart", "OK — OTA task kuyruga alindi");
    return true;
}

void OtaManager::getStatus(
    char *state_out,
    size_t state_out_len,
    int *progress_out,
    char *target_version_out,
    size_t target_version_out_len,
    char *last_error_out,
    size_t last_error_out_len) const
{
    if (xSemaphoreTake(s_status_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        if (state_out && state_out_len > 0) {
            strncpy(state_out, "idle", state_out_len - 1);
            state_out[state_out_len - 1] = '\0';
        }
        return;
    }

    if (state_out && state_out_len > 0) {
        strncpy(state_out, stateToString(state_), state_out_len - 1);
        state_out[state_out_len - 1] = '\0';
    }
    if (progress_out) {
        *progress_out = progress_;
    }
    if (target_version_out && target_version_out_len > 0) {
        strncpy(target_version_out, target_version_, target_version_out_len - 1);
        target_version_out[target_version_out_len - 1] = '\0';
    }
    if (last_error_out && last_error_out_len > 0) {
        strncpy(last_error_out, last_error_, last_error_out_len - 1);
        last_error_out[last_error_out_len - 1] = '\0';
    }

    xSemaphoreGive(s_status_mutex);
}

void OtaManager::runOtaTask(void *arg)
{
    (void)arg;
    while (true) {
        if (s_start_pending) {
            if (xSemaphoreTake(s_status_mutex, portMAX_DELAY) == pdTRUE) {
                s_start_pending = false;
                xSemaphoreGive(s_status_mutex);
            }
            hexnet_ota_dbg_step(4, "runOtaTask", "performOta basliyor");
            (void)performOta();
            hexnet_ota_dbg_step(99, "runOtaTask", "performOta bitti");
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

bool OtaManager::performOta()
{
    char url[sizeof(pending_url_)];
    char version[sizeof(pending_version_)];

    if (xSemaphoreTake(s_status_mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }
    strncpy(url, pending_url_, sizeof(url) - 1);
    url[sizeof(url) - 1] = '\0';
    strncpy(version, pending_version_, sizeof(version) - 1);
    version[sizeof(version) - 1] = '\0';
    xSemaphoreGive(s_status_mutex);

    char req_id[sizeof(pending_request_id_)] = {0};
    strncpy(req_id, pending_request_id_, sizeof(req_id) - 1);
    req_id[sizeof(req_id) - 1] = '\0';

    setState(OtaState::Requested);
    hexnet_ota_dbg_step(5, "performOta", "url=%s target=%s request_id=%s", url, version[0] ? version : "-", req_id[0] ? req_id : "-");
    ESP_LOGI(TAG, "OTA start url=%s version=%s", url, version[0] ? version : "(none)");

    hexnet_ota_dbg_step(20, "performOta", "MQTT handshake basliyor");
    hexnet_mqtt_finish_ota_mqtt_handshake(req_id, true, "accepted");
    hexnet_ota_dbg_step(25, "performOta", "MQTT handshake tamam");
    vTaskDelay(pdMS_TO_TICKS(200));
    size_t heap_after_halt = esp_get_free_heap_size();
    hexnet_ota_dbg_step(26, "heap", "post-halt heap_free=%u", (unsigned)heap_after_halt);
    if (heap_after_halt < kOtaMinHeapForHttps && hexnet_mqtt_halted_client_pending()) {
        hexnet_ota_dbg_step(26, "heap", "dusuk — halted MQTT destroy kuyrugu");
        hexnet_mqtt_destroy_halted_client_when_safe();
        for (int i = 0; i < 80; i++) {
            if (!hexnet_mqtt_halted_client_pending()) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        heap_after_halt = esp_get_free_heap_size();
        hexnet_ota_dbg_step(26, "heap", "destroy sonrasi heap_free=%u", (unsigned)heap_after_halt);
    }
    hexnet_ota_dbg_step(27, "heap", "before https heap_free=%u", (unsigned)heap_after_halt);
    if (!ota_wait_heap(kOtaMinHeapForHttps, 15)) {
        hexnet_ota_dbg_step(28, "heap", "YETERSIZ heap_free=%u (min %u)",
                            (unsigned)esp_get_free_heap_size(), (unsigned)kOtaMinHeapForHttps);
        setLastError("ESP_ERR_NO_MEM");
        setState(OtaState::Failed);
        ota_busy_ = false;
        vTaskDelay(pdMS_TO_TICKS(200));
        hexnet_mqtt_resume_after_ota_abort();
        return false;
    }

    const esp_partition_t *update_part = esp_ota_get_next_update_partition(nullptr);
    if (update_part == nullptr) {
        hexnet_ota_dbg_step(30, "performOta", "HATA: ota partition yok");
        ESP_LOGE(TAG, "No OTA update partition available");
        setLastError("no ota partition");
        setState(OtaState::Failed);
        ota_busy_ = false;
        vTaskDelay(pdMS_TO_TICKS(200));
        hexnet_mqtt_resume_after_ota_abort();
        return false;
    }

    hexnet_ota_dbg_step(
        31,
        "partition",
        "label=%s addr=0x%lx size=0x%lx",
        update_part->label,
        (unsigned long)update_part->address,
        (unsigned long)update_part->size);
    ESP_LOGI(
        TAG,
        "Writing to partition %s @ 0x%lx (size 0x%lx)",
        update_part->label,
        (unsigned long)update_part->address,
        (unsigned long)update_part->size);

    esp_http_client_config_t http_cfg = {};
    http_cfg.url = url;
    http_cfg.timeout_ms = 120000;
    http_cfg.keep_alive_enable = false;
    http_cfg.buffer_size = 1536;
    http_cfg.buffer_size_tx = 512;
    http_cfg.disable_auto_redirect = false;

    esp_https_ota_config_t ota_cfg = {};
    ota_cfg.http_config = &http_cfg;

    esp_https_ota_handle_t ota_handle = nullptr;
    hexnet_ota_dbg_step(32, "https", "esp_https_ota_begin url=%s", url);
    ESP_LOGI(TAG, "esp_https_ota_begin...");
    esp_err_t err = esp_https_ota_begin(&ota_cfg, &ota_handle);
    if (err != ESP_OK) {
        hexnet_ota_dbg_step(33, "https", "begin FAILED %s heap=%u", esp_err_to_name(err),
                            (unsigned)esp_get_free_heap_size());
        ESP_LOGE(TAG, "esp_https_ota_begin failed: %s (heap=%u)", esp_err_to_name(err),
                 (unsigned)esp_get_free_heap_size());
        setLastError(esp_err_to_name(err));
        setState(OtaState::Failed);
        ota_busy_ = false;
        vTaskDelay(pdMS_TO_TICKS(300));
        hexnet_mqtt_resume_after_ota_abort();
        return false;
    }

    hexnet_ota_dbg_step(33, "https", "begin OK");
    setState(OtaState::Downloading);
    setProgress(0);

    while (true) {
        err = esp_https_ota_perform(ota_handle);
        if (err == ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            int read_len = esp_https_ota_get_image_len_read(ota_handle);
            int total_len = esp_https_ota_get_image_size(ota_handle);
            if (total_len > 0) {
                int pct = (int)((read_len * 100LL) / total_len);
                setProgress(pct);
                setState(pct < 95 ? OtaState::Downloading : OtaState::Writing);
                if (pct % 10 == 0) {
                    hexnet_ota_dbg_step((unsigned)(40 + (pct / 10)), "download", "%d%% %d/%d byte", pct, read_len, total_len);
                    ESP_LOGI(TAG, "OTA download %d%% (%d/%d)", pct, read_len, total_len);
                }
            } else {
                setState(OtaState::Downloading);
            }
            continue;
        }
        break;
    }

    if (err != ESP_OK) {
        hexnet_ota_dbg_step(55, "https", "perform FAILED %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "esp_https_ota_perform failed: %s", esp_err_to_name(err));
        esp_https_ota_abort(ota_handle);
        setLastError(esp_err_to_name(err));
        setState(OtaState::Failed);
        ota_busy_ = false;
        vTaskDelay(pdMS_TO_TICKS(300));
        hexnet_mqtt_resume_after_ota_abort();
        return false;
    }

    hexnet_ota_dbg_step(56, "https", "perform OK — verify");
    setState(OtaState::Verifying);
    setProgress(98);

    err = esp_https_ota_finish(ota_handle);
    if (err != ESP_OK) {
        hexnet_ota_dbg_step(57, "https", "finish FAILED %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "esp_https_ota_finish failed: %s", esp_err_to_name(err));
        setLastError(esp_err_to_name(err));
        setState(OtaState::Failed);
        ota_busy_ = false;
        vTaskDelay(pdMS_TO_TICKS(300));
        hexnet_mqtt_resume_after_ota_abort();
        return false;
    }

    setState(OtaState::SetBoot);
    setProgress(100);
    setState(OtaState::Success);
    hexnet_ota_dbg_step(58, "success", "finish OK — reboot partition=%s", update_part->label);
    ESP_LOGI(TAG, "OTA complete, rebooting into %s", update_part->label);

    setState(OtaState::Rebooting);
    vTaskDelay(pdMS_TO_TICKS(200));
    hexnet_ota_dbg_step(59, "reboot", "esp_restart");
    esp_restart();
    return true;
}

void ota_manager_init(void)
{
    OtaManager::instance().init();
}

bool ota_manager_is_active(void)
{
    char state[24] = "idle";
    ota_manager_get_status(state, sizeof(state), nullptr, nullptr, 0, nullptr, 0);
    return strcmp(state, "idle") != 0 && strcmp(state, "failed") != 0;
}

bool ota_manager_handle_ota_start_json(const cJSON *args, const char *request_id)
{
    if (!cJSON_IsObject(args)) {
        hexnet_ota_dbg_step(11, "ota_start_json", "REJECT missing args");
        ESP_LOGW(TAG, "ota_start: missing args object");
        return false;
    }

    const cJSON *url_item = cJSON_GetObjectItemCaseSensitive(args, "url");
    const cJSON *version_item = cJSON_GetObjectItemCaseSensitive(args, "version");
    const cJSON *force_item = cJSON_GetObjectItemCaseSensitive(args, "force");

    if (!cJSON_IsString(url_item) || url_item->valuestring == nullptr || url_item->valuestring[0] == '\0') {
        hexnet_ota_dbg_step(11, "ota_start_json", "REJECT invalid url");
        ESP_LOGW(TAG, "ota_start: invalid or empty url");
        return false;
    }

    const char *url = url_item->valuestring;
    const char *version =
        (cJSON_IsString(version_item) && version_item->valuestring) ? version_item->valuestring : "";
    bool force = json_bool_like(force_item, false);

    if (cJSON_IsString(cJSON_GetObjectItemCaseSensitive(args, "sha256"))) {
        ESP_LOGD(TAG, "ota_start: sha256 check skipped (phase 1)");
    }

    hexnet_ota_dbg_step(11, "ota_start_json", "url=%s ver=%s force=%d", url, version, force ? 1 : 0);
    return OtaManager::instance().requestStart(url, version, force, request_id);
}

void ota_manager_get_status(
    char *state_out,
    size_t state_out_len,
    int *progress_out,
    char *target_version_out,
    size_t target_version_out_len,
    char *last_error_out,
    size_t last_error_out_len)
{
    OtaManager::instance().getStatus(
        state_out,
        state_out_len,
        progress_out,
        target_version_out,
        target_version_out_len,
        last_error_out,
        last_error_out_len);
}

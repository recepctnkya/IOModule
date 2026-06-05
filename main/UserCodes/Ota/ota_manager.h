#pragma once

#include <stddef.h>
#include <stdbool.h>

struct cJSON;

#ifdef __cplusplus
extern "C" {
#endif

void ota_manager_init(void);
bool ota_manager_handle_ota_start_json(const struct cJSON *args, const char *request_id);
void ota_manager_get_status(
    char *state_out,
    size_t state_out_len,
    int *progress_out,
    char *target_version_out,
    size_t target_version_out_len,
    char *last_error_out,
    size_t last_error_out_len);
/** OTA indirme/yazma suruyor mu (WiFi probe ve MQTT reconnect icin). */
bool ota_manager_is_active(void);

#ifdef __cplusplus
}
#endif

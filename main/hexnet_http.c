#include "hexnet_http.h"

#include "esp_heap_caps.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t s_http_mutex;

bool hexnet_http_heap_ok(uint32_t min_free_bytes)
{
    return esp_get_free_heap_size() >= min_free_bytes;
}

bool hexnet_http_lock(TickType_t timeout_ticks)
{
    if (s_http_mutex == NULL) {
        s_http_mutex = xSemaphoreCreateMutex();
        if (s_http_mutex == NULL) {
            return false;
        }
    }
    return xSemaphoreTake(s_http_mutex, timeout_ticks) == pdTRUE;
}

void hexnet_http_unlock(void)
{
    if (s_http_mutex != NULL) {
        (void)xSemaphoreGive(s_http_mutex);
    }
}

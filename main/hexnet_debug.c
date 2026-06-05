#include "hexnet_debug.h"

#if HEXNET_LOG_WIFI

#include <stdarg.h>
#include <stdio.h>

#include "esp_log.h"

static const char *TAG = "HEXNET_WIFI";

void hexnet_debug_log(const char *fmt, ...)
{
    if (!fmt) {
        return;
    }
    char buf[240];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) {
        return;
    }
    if ((size_t)n < sizeof(buf) - 2 && buf[n - 1] != '\n') {
        buf[n] = '\n';
        buf[n + 1] = '\0';
    }
    ESP_LOGI(TAG, "%s", buf);
}

#endif

#if HEXNET_LOG_MQTT

#include <stdarg.h>
#include <stdio.h>

#include "esp_log.h"

static const char *TAG_PLATFORM = "HEXNET_MQTT";

void hexnet_debug_platform_cmd(const char *fmt, ...)
{
    if (!fmt) {
        return;
    }
    char buf[360];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) {
        return;
    }
    ESP_LOGI(TAG_PLATFORM, "[Platform komut] %s", buf);
}

#endif

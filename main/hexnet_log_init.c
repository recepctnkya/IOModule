#include "hexnet_log.h"

#include "esp_log.h"

void hexnet_log_init(void)
{
    esp_log_level_set("*", ESP_LOG_ERROR);

#if HEXNET_LOG_WIFI
    esp_log_level_set("HEXNET_WIFI", ESP_LOG_VERBOSE);
    esp_log_level_set("WiFiPortal", ESP_LOG_INFO);
#endif

#if HEXNET_LOG_MQTT
    esp_log_level_set("HEXNET_MQTT", ESP_LOG_VERBOSE);
#endif

#if HEXNET_LOG_BLE
    esp_log_level_set("HEXNET_BLE", ESP_LOG_VERBOSE);
#endif

#if HEXNET_LOG_OTA
    esp_log_level_set("HEXNET_OTA", ESP_LOG_VERBOSE);
    esp_log_level_set("OtaManager", ESP_LOG_INFO);
    esp_log_level_set("APP_MAIN", ESP_LOG_INFO);
#endif

#if HEXNET_LOG_MQTT
    esp_log_level_set("HEXNET_MQTT_DBG", ESP_LOG_VERBOSE);
#endif
}

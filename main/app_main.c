/*
 * Hexnet IO Module – application entry (orchestration only).
 * Pin map: hexnet_io_map.h
 * Modules: hexnet_board, hexnet_wifi, hexnet_mqtt, hexnet_app, …
 */
#include <inttypes.h>

#include "hexnet_log_init.h"
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hexnet_app.h"
#include "hexnet_board.h"
#include "hexnet_io_profile.h"
#include "hexnet_mqtt.h"
#include "hexnet_wifi.h"
#include "hexnet_version.h"
#include "nvs_flash.h"
#include "ota_manager.h"

static const char *TAG = "APP_MAIN";

void app_main(void)
{

    hexnet_board_gpio_init();

    hexnet_board_start_status_led();

    ESP_LOGI(
        TAG,
        "Hexnet IO line=%s release=%s | fw=%s | heap=%" PRIu32 " | IDF %s",
        HEXNET_IO_DEV_LINE,
        HEXNET_IO_RELEASE,
        hexnet_firmware_version_string(),
        esp_get_free_heap_size(),
        esp_get_idf_version());

    ota_manager_init();

    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    hexnet_log_init();

    hexnet_io_profile_init();
 
    hexnet_mqtt_init();
    hexnet_wifi_init();
    
    xTaskCreate(hexnet_app_bringup_task, "app_bringup", 4096, NULL, 4, NULL);

}

#include "hexnet_app.h"



#include "adc_monitor.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "hexnet_log.h"

#include "esp_log.h"

#include "freertos/FreeRTOS.h"

#include "freertos/task.h"

#include "hexnet_bluetooth.h"

#include "hexnet_canbus.h"

#include "hexnet_dim_pwm.h"

#include "hexnet_motor.h"
#include "hexnet_mqtt.h"

#include "hexnet_rgb_strip.h"

#include "hexnet_wifi.h"

#include "hexnet_sensors.h"

#include "hexnet_shift_register.h"

#include "hexnet_wifi_portal.h"



static const char *TAG = "HEXNET_APP";



static void ble_bridge_task(void *arg)

{

    (void)arg;

    while (1) {

        if (get_connection_status()) {

            char *json = NULL;

            get_data_json_format(NULL, 0, &json);

            set_converted_json_data(json);

        }

        vTaskDelay(pdMS_TO_TICKS(200));

    }

}



#define BLE_MIN_DELAY_MS        30000
#define BLE_STABLE_TELEMETRY_MS 15000

static void ble_delayed_init_task(void *arg)

{

    (void)arg;

    ESP_LOGI(TAG, "BLE: min %ds bekleniyor (MQTT oncelikli)", BLE_MIN_DELAY_MS / 1000);
    vTaskDelay(pdMS_TO_TICKS(BLE_MIN_DELAY_MS));

    for (int i = 0; i < 120; ++i) {
        if (hexnet_mqtt_has_telemetry_ok()) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (hexnet_mqtt_has_telemetry_ok()) {
        ESP_LOGI(TAG, "BLE: telemetri OK, %ds stabilite bekleniyor", BLE_STABLE_TELEMETRY_MS / 1000);
        vTaskDelay(pdMS_TO_TICKS(BLE_STABLE_TELEMETRY_MS));
    } else {
        ESP_LOGW(TAG, "BLE: telemetri yok; yine de baslatiliyor");
    }

    ESP_LOGI(TAG, "BLE baslatiliyor");

    ble_init();

    xTaskCreate(ble_bridge_task, "ble_bridge", 4096, NULL, 5, NULL);

    vTaskDelete(NULL);

}



void hexnet_app_bringup_task(void *arg)

{

    (void)arg;



    uint32_t delay_ms = hexnet_wifi_has_saved_credentials() ? 3000 : 20000;

    vTaskDelay(pdMS_TO_TICKS(delay_ms));



    esp_err_t can_init_ret = waveshare_twai_init();
    if (can_init_ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        hexnet_canbus_send_saved_panel_config();
    }


    xTaskCreate(send_frames_task, "can_tx", 4096, NULL, 5, NULL);

    xTaskCreate(receive_frames_task, "can_rx", 4096, NULL, 5, NULL);

    xTaskCreate(can_watchdog_task, "can_wdg", 4096, NULL, 6, NULL);


    xTaskCreate(hexnet_shift_register_task, "shift_reg", 2048, NULL, 5, NULL);

    xTaskCreate(hexnet_rgb_strip_task, "rgb_strip", 4096, NULL, 5, NULL);


    adc_monitor_init();


    xTaskCreate(adc_monitor_task, "adc_mon", 4096, NULL, 5, NULL);

    xTaskCreate(hexnet_dht_task, "dht", 4096, NULL, 5, NULL);

    xTaskCreate(hexnet_dim_pwm_task, "dim_pwm", 4096, NULL, 5, NULL);


    hexnet_mqtt_start_tasks();



    xTaskCreate(hexnet_motor_control_task, "motor", 2048, NULL, 5, NULL);

    xTaskCreate(hexnet_sensor_rs485_task, "rs485", 4096, NULL, 10, NULL);



    xTaskCreate(ble_delayed_init_task, "ble_delay", 4096, NULL, 4, NULL);



    ESP_LOGI(TAG, "Bring-up complete");

    vTaskDelete(NULL);

}



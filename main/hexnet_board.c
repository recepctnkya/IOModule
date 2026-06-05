#include "hexnet_board.h"

#include "driver/gpio.h"
#define LOG_LOCAL_LEVEL HEXNET_LOG_LEVEL_OTHER
#include "hexnet_log.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hexnet_bluetooth.h"
#include "hexnet_io_map.h"
#include "hexnet_motor.h"
#include "hexnet_wifi.h"

static const char *TAG = "HEXNET_BOARD";

#define RUN_LED_MS_IDLE   1000
#define RUN_LED_MS_BLE    500
#define RUN_LED_MS_WIFI   250

static void status_led_task(void *arg)
{
    (void)arg;
    bool led_on = false;

    while (1) {
        uint32_t period_ms;
        if (hexnet_wifi_is_connected()) {
            period_ms = RUN_LED_MS_WIFI;
        } else if (hexnet_ble_is_connected()) {
            period_ms = RUN_LED_MS_BLE;
        } else {
            period_ms = RUN_LED_MS_IDLE;
        }

        gpio_set_level(HEXNET_IO_RUN_LED_GPIO, led_on);
        led_on = !led_on;
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
}

void hexnet_board_gpio_init(void)
{
    gpio_config_t mux_sel = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = HEXNET_IO_MUX_SEL_MASK,
    };
    gpio_config(&mux_sel);

    gpio_config_t mux_z = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = HEXNET_IO_MUX_Z_MASK,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&mux_z);

    gpio_config_t run_led = {
        .pin_bit_mask = (1ULL << HEXNET_IO_RUN_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&run_led);
    gpio_set_level(HEXNET_IO_RUN_LED_GPIO, 0);

    gpio_config_t sr = {
        .pin_bit_mask = HEXNET_IO_SR_MASK,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&sr);
    gpio_set_level(HEXNET_IO_SR_CLK_GPIO, 0);
    gpio_set_level(HEXNET_IO_SR_DATA_GPIO, 0);
    gpio_set_level(HEXNET_IO_SR_STB_GPIO, 0);

    gpio_config_t motor = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = HEXNET_IO_MOTOR_MASK,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&motor);
    hexnet_motor_stop();

    ESP_LOGI(TAG, "GPIO init: MUX, RunLED, shift-reg, motor");
}

void hexnet_board_start_status_led(void)
{
    xTaskCreate(status_led_task, "status_led", 2048, NULL, 5, NULL);
}

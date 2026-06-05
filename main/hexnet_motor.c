#include "hexnet_motor.h"
#include "hexnet_io_profile.h"

#include "driver/gpio.h"
#define LOG_LOCAL_LEVEL HEXNET_LOG_LEVEL_OTHER
#include "hexnet_log.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hexnet_canbus.h"
#include "hexnet_io_map.h"

static const char *TAG = "HEXNET_MOTOR";

int motorDataUpdateCounter = 0;

void hexnet_motor_forward(void)
{
    gpio_set_level(HEXNET_IO_MOTOR_EN_GPIO, 1);
    gpio_set_level(HEXNET_IO_MOTOR_A_GPIO, 1);
    gpio_set_level(HEXNET_IO_MOTOR_B_GPIO, 0);
}

void hexnet_motor_backward(void)
{
    gpio_set_level(HEXNET_IO_MOTOR_EN_GPIO, 1);
    gpio_set_level(HEXNET_IO_MOTOR_A_GPIO, 0);
    gpio_set_level(HEXNET_IO_MOTOR_B_GPIO, 1);
}

void hexnet_motor_stop(void)
{
    gpio_set_level(HEXNET_IO_MOTOR_EN_GPIO, 0);
    gpio_set_level(HEXNET_IO_MOTOR_A_GPIO, 0);
    gpio_set_level(HEXNET_IO_MOTOR_B_GPIO, 0);
}

void hexnet_motor_control_task(void *arg)
{
    (void)arg;
    while (1) {
        if (!hexnet_io_profile_motor_enabled()) {
            hexnet_motor_stop();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        int mData = get_motorData();
        if (mData == 1) {
            hexnet_motor_forward();
            ESP_LOGI(TAG, "forward");
        } else if (mData == 2) {
            hexnet_motor_backward();
            ESP_LOGI(TAG, "backward");
        } else {
            hexnet_motor_stop();
        }
        motorDataUpdateCounter++;
        if (motorDataUpdateCounter >= 2) {
            set_motordata(0);
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

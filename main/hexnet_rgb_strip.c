#include "hexnet_rgb_strip.h"

#define LOG_LOCAL_LEVEL HEXNET_LOG_LEVEL_OTHER
#include "hexnet_log.h"
#include "esp_log.h"
#include "esp_ws28xx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hexnet_bluetooth.h"
#include "hexnet_canbus.h"
#include "hexnet_io_map.h"

static CRGB *s_ws2812_buffer;
static uint8_t s_led_off;

void hexnet_rgb_strip_task(void *arg)
{
    (void)arg;
    ESP_ERROR_CHECK_WITHOUT_ABORT(
        ws28xx_init(HEXNET_IO_RGB_STRIP_GPIO, WS2815, HEXNET_IO_RGB_MATRIX_LEDS, &s_ws2812_buffer));

    while (1) {
        for (int i = 0; i < HEXNET_IO_RGB_LED_COUNT; i++) {
            if (s_led_off) {
                s_ws2812_buffer[i] = (CRGB){.r = 0, .g = 0, .b = 0};
            }
            else {
                s_ws2812_buffer[i] = (CRGB){.r = get_r_value(), .g = get_g_value(), .b = get_b_value()};
            }
        }
        ESP_ERROR_CHECK_WITHOUT_ABORT(ws28xx_update());
        vTaskDelay(pdMS_TO_TICKS(100));
        s_led_off = (get_rgb_enable() != 1);
    }
}

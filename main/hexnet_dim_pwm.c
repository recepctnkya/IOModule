#include "hexnet_dim_pwm.h"

#include <math.h>

#include "driver/ledc.h"
#define LOG_LOCAL_LEVEL HEXNET_LOG_LEVEL_OTHER
#include "hexnet_log.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hexnet_bluetooth.h"
#include "hexnet_canbus.h"
#include "hexnet_io_map.h"

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_CH1        LEDC_CHANNEL_0
#define LEDC_CH2        LEDC_CHANNEL_1
#define LEDC_CH3        LEDC_CHANNEL_2
#define LEDC_CH4        LEDC_CHANNEL_3
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT
#define LEDC_FREQ_HZ    5000
#define MAX_DUTY        ((1 << 10) - 1)

static uint32_t scale_brightness(uint32_t pct)
{
    if (pct == 0) {
        return 0;
    }
    const float gamma = 2.0f;
    float norm = (float)pct / 100.0f;
    return (uint32_t)(powf(norm, gamma) * MAX_DUTY + 0.5f);
}

static void ledc_init_channel(ledc_channel_t ch, gpio_num_t gpio)
{
    ledc_channel_config_t cfg = {
        .speed_mode = LEDC_MODE,
        .channel = ch,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = gpio,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&cfg));
}

void hexnet_dim_pwm_task(void *arg)
{
    (void)arg;

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_init_channel(LEDC_CH1, HEXNET_IO_DIM_PWM_GPIO_1);
    ledc_init_channel(LEDC_CH2, HEXNET_IO_DIM_PWM_GPIO_2);
    ledc_init_channel(LEDC_CH3, HEXNET_IO_DIM_PWM_GPIO_3);
    ledc_init_channel(LEDC_CH4, HEXNET_IO_DIM_PWM_GPIO_4);

    while (1) {
        // Simultaneous BLE + CAN support: read all values
        // BLE updates should sync to CAN state, and vice versa
        // This way both sources control the same output
        uint32_t d1 = scale_brightness(get_dimmable_output(0));
        uint32_t d2 = scale_brightness(get_dimmable_output(1));
        uint32_t d3 = scale_brightness(get_dimmable_output(2));
        uint32_t d4 = scale_brightness(get_dimmable_output(3));
        ledc_set_duty(LEDC_MODE, LEDC_CH1, d1);
        ledc_update_duty(LEDC_MODE, LEDC_CH1);
        ledc_set_duty(LEDC_MODE, LEDC_CH2, d2);
        ledc_update_duty(LEDC_MODE, LEDC_CH2);
        ledc_set_duty(LEDC_MODE, LEDC_CH3, d3);
        ledc_update_duty(LEDC_MODE, LEDC_CH3);
        ledc_set_duty(LEDC_MODE, LEDC_CH4, d4);
        ledc_update_duty(LEDC_MODE, LEDC_CH4);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

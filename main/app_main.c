/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* MQTT (over TCP) Example with custom outbox

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "waveshare_twai_port.h" // Include the Waveshare TWAI port library 
#include "esp_log.h"
#include "driver/gpio.h"
#include "mqtt_client.h"
#include "driver/i2c.h"
#include "esp_ws28xx.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "adc_monitor.h"

#include "dht_sensor.h"
#include "driver/ledc.h"


static const char *TAG = "MQTT_EXAMPLE";


#define SAMPLE_COUNT 100
#define DELAY_MS 10
#define NUM_CHANNELS 4


// MUX control pins
#define S0 GPIO_NUM_10
#define S1 GPIO_NUM_11
#define S2 GPIO_NUM_12

#define NUM_CHANNELS 8

#define DHT_GPIO 6
#define DHT_TYPE DHT21


// MUX output pins
#define MUX1_Z GPIO_NUM_13
#define MUX2_Z GPIO_NUM_14

adc1_channel_t adc_channels[NUM_CHANNELS] = {
    ADC1_CHANNEL_3,  // GPIO4
    ADC1_CHANNEL_4,  // GPIO5
    ADC1_CHANNEL_5,  // GPIO6
    ADC1_CHANNEL_6   // GPIO7
};


uint32_t sample_buffer[NUM_CHANNELS][SAMPLE_COUNT];
esp_adc_cal_characteristics_t adc_chars;

#define FRAME_1  "F1"
#define FRAME_2  "F2"
#define FRAME_3  "F3"
#define LED_GPIO_1 35
#define LED_GPIO_2 45
#define TOGGLE_INTERVAL_MS 200

#define SR_CLK 42   // Clock pin
#define SR_DATA 41  // Data pin
#define SR_STB 40   // Strobe/Latch pin
#define DELAY_MS 1000

#define DIM_GPIO        18
#define PWM_FREQ_HZ     1000        // 1kHz PWM for dimming
#define PWM_RES         LEDC_TIMER_10_BIT
#define MAX_DUTY        ((1 << 10) - 1)  // 1023



#define I2C_MASTER_SCL_IO 1
#define I2C_MASTER_SDA_IO 2
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define MCP4728_ADDR 0x60 // Default I2C address

#define MCP4728_WRITE_DAC 0x40 // Fast Write Command

#define LDAC_GPIO 39
#define RDY_BSY_GPIO 38




#define RGB_GPIO 37
#define LED_NUM 65  
#define RGB_RELAY 36

static uint8_t led_state_off = 0;
CRGB* ws2812_buffer;


typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} CRGB2;

// Hue: 0-360, sat: 0-255, val: 0-255
CRGB2 hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v) {
    float hh = h / 60.0f;
    int i = (int)hh;
    float ff = hh - i;
    float p = v * (1.0f - s / 255.0f);
    float q = v * (1.0f - (s / 255.0f) * ff);
    float t = v * (1.0f - (s / 255.0f) * (1.0f - ff));

    float r = 0, g = 0, b = 0;  // <-- Default init here
    switch (i % 6) {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        case 5: r = v; g = p; b = q; break;
    }
    return (CRGB2){.r = (uint8_t)r, .g = (uint8_t)g, .b = (uint8_t)b};
}

#define ROWS 10
#define COLS 20
#define TOTAL_LEDS (ROWS * COLS)

int get_index(int row, int col) {
    if (row % 2 == 0)
        return row * COLS + col;           // left to right
    else
        return row * COLS + (COLS - 1 - col); // right to left
}

void fade_all() {
    for (int i = 0; i < TOTAL_LEDS; i++) {
        ws2812_buffer[i].r = get_r_value();
        ws2812_buffer[i].g = get_g_value();
        ws2812_buffer[i].b = get_b_value();
    }
}

void app_rgb(void *arg) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(ws28xx_init(RGB_GPIO, WS2815, TOTAL_LEDS, &ws2812_buffer));

    int hue_base = 0;
    int color_offset = 20; // Different hue per step in trail
    int snake_len = 6;

    while (1) {
        for(int i = 0; i < LED_NUM; i++) {
            if (led_state_off) ws2812_buffer[i] = (CRGB){.r=0, .g=0, .b=0};
            else ws2812_buffer[i] = (CRGB){.r=get_g_value(), .g=get_r_value(), .b=get_b_value()};
        }
        ESP_LOGI(TAG, "Updating RGB LEDs with values: R=%d, G=%d, B=%d", get_r_value(), get_g_value(), get_b_value());
        ESP_ERROR_CHECK_WITHOUT_ABORT(ws28xx_update());
        vTaskDelay(pdMS_TO_TICKS(100));

        if(get_rgb_enable() == 1) {
            gpio_set_level(RGB_RELAY, 1);
        } else {
            gpio_set_level(RGB_RELAY, 0);
        }
    }
}


void set_mux_channel(uint8_t channel) {
    gpio_set_level(S0, channel & 0x01);
    gpio_set_level(S1, (channel >> 1) & 0x01);
    gpio_set_level(S2, (channel >> 2) & 0x01);
    vTaskDelay(1);
}

void read_input_task(void *arg) {

    while (1) {
        printf("Reading 16 inputs:\n");

        for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
            set_mux_channel(ch);

            // Short delay for signal stabilization
            vTaskDelay(1);

            int val1 = gpio_get_level(MUX1_Z);  // inputs 0-7
            int val2 = gpio_get_level(MUX2_Z);  // inputs 8-15

            printf("Input %2d = %d\t", ch, val1);
            printf("Input %2d = %d\n", ch + 8, val2);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Write all 4 channels (A, B, C, D) with values
esp_err_t mcp4728_write_all_channels(uint16_t a, uint16_t b, uint16_t c, uint16_t d)
{
    uint8_t buf[9];

    buf[0] = 0x50; // Multi-write command

    // DAC A
    buf[1] = ((a >> 8) & 0x0F) | 0x00; // Vref = VDD, gain x1, PD normal
    buf[2] = a & 0xFF;

    // DAC B
    buf[3] = ((b >> 8) & 0x0F) | 0x10;
    buf[4] = b & 0xFF;

    // DAC C
    buf[5] = ((c >> 8) & 0x0F) | 0x20;
    buf[6] = c & 0xFF;

    // DAC D
    buf[7] = ((d >> 8) & 0x0F) | 0x30;
    buf[8] = d & 0xFF;

    return i2c_master_write_to_device(I2C_MASTER_NUM, MCP4728_ADDR, buf, sizeof(buf), pdMS_TO_TICKS(100));
}


// Pulse LDAC to update DAC outputs
void pulse_ldac()
{
    gpio_set_level(LDAC_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(LDAC_GPIO, 1);
}

// Wait until RDY/BSY goes high (ready)
void wait_for_ready()
{
    while (gpio_get_level(RDY_BSY_GPIO) == 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


void smooth_dim_task(void *arg)
{
    const int steps = 100;
    const int delay_per_step_ms = 10000 / steps; // 5 sec up/down
    const int max_val = 3500;
    esp_err_t err;
    uint16_t val = 0;

    while (1) {
        // Fade up
        for (int i = 0; i <= steps; i++) {
            val = (i * max_val) / steps;
            err = mcp4728_write_all_channels(val, val, val, val);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "I2C write failed during fade-up, step %d, err=0x%x", i, err);
            }
            wait_for_ready();
            pulse_ldac();
            vTaskDelay(pdMS_TO_TICKS(delay_per_step_ms));
        }
    }
}





void toggle_led_task(void *arg)
{
    bool led_state = false;

    while (1) {
        gpio_set_level(LED_GPIO_1, led_state);
        gpio_set_level(LED_GPIO_2, led_state);
        led_state = !led_state;
        vTaskDelay(pdMS_TO_TICKS(TOGGLE_INTERVAL_MS));
        //ESP_LOGI(TAG, "LED State ----------------------------------------------------------: %s", esp_get_idf_version());
    }
}

static void shift_register_write(uint16_t value)
{
    // Send bits MSB first for two 74HC4094 chips
    for (int i = 15; i >= 0; i--) {
        gpio_set_level(SR_CLK, 0);
        gpio_set_level(SR_DATA, (value >> i) & 0x01);
        gpio_set_level(SR_CLK, 1);
    }

    // Toggle latch to output the new data
    gpio_set_level(SR_STB, 0);
    gpio_set_level(SR_STB, 1);
}

void shift_register_task(void *arg)
{
    uint16_t outputs = get_outputs(); // Get current outputs from CAN/TWAI logic
    while (1) {
        // Turn ON all outputs (1s)
        shift_register_write(get_outputs());
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    //outputs = get_outputs();
}


// void i2c_scan() {
//     esp_err_t err;

//     for (uint8_t addr = 0x03; addr < 0x78; addr++) {
//         i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//         i2c_master_start(cmd);
//         i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
//         i2c_master_stop(cmd);
//         err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(20));
//         i2c_cmd_link_delete(cmd);


//         if (err == ESP_OK) {
//             ESP_LOGW(TAG, "#####################I2C device found at address 0x%02X", addr);
//         } else if (err != ESP_ERR_TIMEOUT) {
//             ESP_LOGD(TAG, "No device at 0x%02X (err=0x%x)", addr, err);
//         }
//     }
// }


#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT // 10-bit resolution (0-1023)
#define LEDC_FREQUENCY          5000              // 5 kHz

// PWM fade task
void rgb_pwm_task(void *pvParameters)
{
    // Timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = RGB_RELAY,
        .duty           = 0, // Initially off
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // Fade service
    ledc_fade_func_install(0);

    while (1) {
        // Fade in
        ESP_ERROR_CHECK(ledc_set_fade_time_and_start(
            LEDC_MODE, LEDC_CHANNEL, 1023, 1000, LEDC_FADE_NO_WAIT));
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Fade out
        ESP_ERROR_CHECK(ledc_set_fade_time_and_start(
            LEDC_MODE, LEDC_CHANNEL, 0, 1000, LEDC_FADE_NO_WAIT));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}




void analog_main_task(void *arg) {

        // Configure ADC1
        adc1_config_width(ADC_WIDTH_BIT_12);  // 0–4095
        for (int i = 0; i < NUM_CHANNELS; i++) {
            adc1_config_channel_atten(adc_channels[i], ADC_ATTEN_DB_11);  // 0–3.3V
        }

        // Calibrate (optional but recommended)
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    while (1) 
    {


        // Sampling loop
        for (int sample = 0; sample < SAMPLE_COUNT; sample++) {
            for (int i = 0; i < NUM_CHANNELS; i++) {
                sample_buffer[i][sample] = adc1_get_raw(adc_channels[i]);
            }
        }

        // Calculate and print averages
        for (int i = 0; i < NUM_CHANNELS; i++) {
            uint32_t sum = 0;
            for (int s = 0; s < SAMPLE_COUNT; s++) {
                sum += sample_buffer[i][s];
            }
            uint32_t avg_raw = sum / SAMPLE_COUNT;
            uint32_t voltage = esp_adc_cal_raw_to_voltage(avg_raw, &adc_chars);
            printf(" GPIO %d average raw: %d, voltage: %d mV", 4 + i, (int)avg_raw, (int)voltage);
        }
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void dht_task(void *pvParameters) {
    dht_init(6);  // Initialize DHT on GPIO4
    
    while(1) {
        float temp, hum;
        int result = dht_read_data(&temp, &hum);
        
        if(result == 0) {
            ESP_LOGI("DHT", "Temperature: %.1fC, Humidity: %.1f%%", temp, hum);
        } else {
            ESP_LOGE("DHT", "Error reading sensor: %d", result);
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main(void)
{




    // Configure MUX control pins
    gpio_config_t io_confz = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << S0) | (1ULL << S1) | (1ULL << S2),
    };
    gpio_config(&io_confz);

    // Configure MUX Z output pins (as input to MCU)
    gpio_config_t in_conf = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << MUX1_Z) | (1ULL << MUX2_Z),
        .pull_up_en = GPIO_PULLUP_ENABLE,  // optional based on sensor
    };
    gpio_config(&in_conf);



    // Configure both GPIOs as outputs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO_1)  | (1ULL << LED_GPIO_2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);






    // Configure GPIOs as outputs
    gpio_config_t io_confSR = {
        .pin_bit_mask = (1ULL << SR_CLK) | (1ULL << SR_DATA) | (1ULL << SR_STB),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_confSR);


    // Configure I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);



        // Configure GPIOs
        gpio_config_t io_conf2 = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << LDAC_GPIO),
            .pull_down_en = 0,
            .pull_up_en = 0,
        };
        gpio_config(&io_conf2);
        gpio_set_level(LDAC_GPIO, 1);
        
    
        gpio_config_t rdy_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = (1ULL << RDY_BSY_GPIO),
            .pull_down_en = 0,
            .pull_up_en = 1,
        };
        gpio_config(&rdy_conf);


        // Initialize lines low
        gpio_set_level(SR_CLK, 0);
        gpio_set_level(SR_DATA, 0);
        gpio_set_level(SR_STB, 0);


    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("custom_outbox", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    //ESP_ERROR_CHECK(esp_netif_init());
    //ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    //ESP_ERROR_CHECK(example_connect());



    waveshare_twai_init(); // Initialize the Waveshare TWAI module 
    xTaskCreate(send_frames_task, "send_frames_task", 4096, NULL, 5, NULL);
    xTaskCreate(receive_frames_task, "receive_frames_task", 4096, NULL, 5, NULL);
    xTaskCreate(toggle_led_task, "toggle_led_task", 2048, NULL, 5, NULL);
    xTaskCreate(shift_register_task, "shift_register_task", 2048, NULL, 5, NULL);
    //xTaskCreate(smooth_dim_task, "smooth_dim_task", 4096, NULL, 5, NULL);
    xTaskCreate(app_rgb, "rbg_", 4096, NULL, 5, NULL);
    //xTaskCreate(analog_main_task, "analog_", 4096, NULL, 5, NULL);
    //adc_monitor_init();
    //xTaskCreate(adc_monitor_task, "adc_monitor_task", 4096, NULL, 5, NULL);
    //xTaskCreate(read_input_task, "DI_", 4096, NULL, 5, NULL);
       
    xTaskCreate(dht_task, "dht_task", 4096, NULL, 5, NULL); 
    xTaskCreate(rgb_pwm_task, "rgb_pwm_task", 2048, NULL, 5, NULL);  

    //i2c_scan();
}

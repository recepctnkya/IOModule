#include "hexnet_sensors.h"



#include <stdlib.h>

#include <string.h>



#include "driver/gpio.h"

#include "driver/uart.h"

#include "esp_heap_caps.h"

#define LOG_LOCAL_LEVEL HEXNET_LOG_LEVEL_OTHER

#include "hexnet_log.h"

#include "esp_log.h"

#include "esp_timer.h"

#include "freertos/FreeRTOS.h"

#include "freertos/task.h"

#include "hexnet_canbus.h"

#include "hexnet_io_map.h"

#include "dht_sensor.h"



static const char *TAG = "HEXNET_SENSORS";



#define RS485_START_DELAY_MS    12000

#define RS485_INSTALL_MIN_HEAP  32768

#define RS485_INSTALL_RETRIES   8



int hexnet_sensor_rs485_temperature = 0;

int hexnet_sensor_rs485_humidity = 0;

int hexnet_sensor_comm_status = HEXNET_SENSOR_COMM_LOST;



static int64_t s_last_rs485_rx_us;

static bool s_rs485_uart_ready;



static esp_err_t rs485_uart_init(void)

{

    if (s_rs485_uart_ready) {

        return ESP_OK;

    }



    gpio_config_t rs485_en = {

        .pin_bit_mask = (1ULL << HEXNET_IO_RS485_EN_GPIO),

        .mode = GPIO_MODE_OUTPUT,

        .pull_up_en = GPIO_PULLUP_DISABLE,

        .pull_down_en = GPIO_PULLDOWN_DISABLE,

        .intr_type = GPIO_INTR_DISABLE,

    };

    gpio_config(&rs485_en);

    gpio_set_level(HEXNET_IO_RS485_EN_GPIO, 0);



    uart_config_t uart_cfg = {

        .baud_rate = HEXNET_IO_UART_RS485_BAUD,

        .data_bits = UART_DATA_8_BITS,

        .parity = UART_PARITY_DISABLE,

        .stop_bits = UART_STOP_BITS_1,

        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,

        .source_clk = UART_SCLK_DEFAULT,

    };



    esp_err_t err = uart_driver_install(

        HEXNET_IO_UART_RS485_PORT, HEXNET_IO_UART_RS485_BUF_SIZE * 2, 0, 0, NULL, 0);

    if (err == ESP_ERR_INVALID_STATE) {

        err = ESP_OK;

    }

    if (err != ESP_OK) {

        return err;

    }



    err = uart_param_config(HEXNET_IO_UART_RS485_PORT, &uart_cfg);

    if (err != ESP_OK) {

        uart_driver_delete(HEXNET_IO_UART_RS485_PORT);

        return err;

    }



    err = uart_set_pin(

        HEXNET_IO_UART_RS485_PORT,

        HEXNET_IO_UART_RS485_TX_GPIO,

        HEXNET_IO_UART_RS485_RX_GPIO,

        UART_PIN_NO_CHANGE,

        UART_PIN_NO_CHANGE);

    if (err != ESP_OK) {

        uart_driver_delete(HEXNET_IO_UART_RS485_PORT);

        return err;

    }



    s_rs485_uart_ready = true;

    return ESP_OK;

}



void hexnet_dht_task(void *arg)

{

    (void)arg;

    dht_init(HEXNET_IO_DHT_GPIO_PRIMARY);



    while (1) {

        float temp1 = 0.0f;

        float hum1 = 0.0f;

        float temp2 = 0.0f;

        float hum2 = 0.0f;



        vTaskDelay(pdMS_TO_TICKS(1000));



        if (dht_read_data_gpio(HEXNET_IO_DHT_GPIO_PRIMARY, &temp2, &hum2) != 0) {

            temp2 = 0.0f;

            hum2 = 0.0f;

        }

        update_analog_inputs(temp1, temp2, 0, 0);

        vTaskDelay(pdMS_TO_TICKS(2000));

    }

}



void hexnet_sensor_rs485_task(void *arg)

{

    (void)arg;



    vTaskDelay(pdMS_TO_TICKS(RS485_START_DELAY_MS));



    esp_err_t init_err = ESP_FAIL;

    for (int attempt = 0; attempt < RS485_INSTALL_RETRIES; attempt++) {

        const uint32_t heap = esp_get_free_heap_size();

        if (heap < RS485_INSTALL_MIN_HEAP) {

            ESP_LOGW(TAG, "RS485 bekliyor: heap=%lu", (unsigned long)heap);

            vTaskDelay(pdMS_TO_TICKS(3000));

            continue;

        }

        init_err = rs485_uart_init();

        if (init_err == ESP_OK) {

            ESP_LOGI(TAG, "RS485 UART hazir (heap=%lu)", (unsigned long)heap);

            break;

        }

        ESP_LOGW(TAG, "RS485 UART kurulumu basarisiz (%s), tekrar...", esp_err_to_name(init_err));

        vTaskDelay(pdMS_TO_TICKS(3000));

    }



    if (init_err != ESP_OK) {

        ESP_LOGE(TAG, "RS485 devre disi; UART kurulamadi");

        hexnet_sensor_comm_status = HEXNET_SENSOR_COMM_LOST;

        while (1) {

            vTaskDelay(pdMS_TO_TICKS(60000));

        }

    }



    uint8_t *rx = malloc(HEXNET_IO_UART_RS485_BUF_SIZE);

    if (!rx) {

        ESP_LOGE(TAG, "RS485 RX malloc failed");

        vTaskDelete(NULL);

        return;

    }



    s_last_rs485_rx_us = esp_timer_get_time();



    while (1) {

        int len = uart_read_bytes(

            HEXNET_IO_UART_RS485_PORT, rx, HEXNET_IO_UART_RS485_BUF_SIZE - 1, pdMS_TO_TICKS(100));



        if (len > 0) {

            rx[len] = '\0';

            if (sscanf((char *)rx, "$TEMP,%d;HUM,%d#", &hexnet_sensor_rs485_temperature, &hexnet_sensor_rs485_humidity) == 2) {

                set_sensorTemp(hexnet_sensor_rs485_temperature);

                set_sensorHumidity(hexnet_sensor_rs485_humidity);

                s_last_rs485_rx_us = esp_timer_get_time();

                hexnet_sensor_comm_status = HEXNET_SENSOR_COMM_OK;

            }

        }



        if ((esp_timer_get_time() - s_last_rs485_rx_us) > 10 * 1000000LL) {

            hexnet_sensor_comm_status = HEXNET_SENSOR_COMM_LOST;

        }

        vTaskDelay(pdMS_TO_TICKS(100));

    }

}



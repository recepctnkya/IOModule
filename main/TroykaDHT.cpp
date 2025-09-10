#include "TroykaDHT.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

DHT::DHT(gpio_num_t pin, uint8_t type) {
    _pin = pin;
    _type = type;
    _state = DHT_ERROR_NO_REPLY;
}

void DHT::begin() {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << _pin);
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
}

int8_t DHT::read() {
    uint8_t data[5] = {0};
    uint8_t dataBit;
    uint8_t checkSum;

    // Send start signal (18ms low)
    gpio_set_direction(_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(_pin, 0);
    esp_rom_delay_us(18000);
    
    // Switch to input with pullup
    gpio_set_direction(_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(_pin, GPIO_PULLUP_ONLY);
    
    // Wait for sensor response
    if (pulseInLength(HIGH, 40) == 0) return _state = DHT_ERROR_NO_REPLY;
    if (pulseInLength(LOW, 80) == 0) return _state = DHT_ERROR_NO_REPLY;
    if (pulseInLength(HIGH, 80) == 0) return _state = DHT_ERROR_NO_REPLY;

    // Read 40 bits of data
    for (int i = 0; i < 40; i++) {
        pulseInLength(LOW, 50);
        dataBit = pulseInLength(HIGH, 100);
        if (dataBit) {
            data[i/8] <<= 1;
            data[i/8] += (dataBit > 45) ? 1 : 0;
        } else {
            return _state = DHT_ERROR_TIMEOUT;
        }
    }

    // Verify checksum
    checkSum = data[0] + data[1] + data[2] + data[3];
    if (data[4] != checkSum) return _state = DHT_ERROR_CHECKSUM;

    // Process data
    switch (_type) {
        case DHT11:
            _humidity = data[0];
            _temperatureC = data[3] & 0x80 ? 
                (data[2] + (1 - (data[3] & 0x7F) * 0.1f)) * -1 : 
                (data[2] + (data[3] & 0x7F) * 0.1f);
            break;
        case DHT21:
        case DHT22:
            _humidity = ((data[0] << 8) + data[1]) * 0.1f;
            _temperatureC = (((data[2] & 0x7F) << 8) + data[3]) * 
                (data[2] & 0x80 ? -0.1f : 0.1f);
            break;
    }

    _temperatureF = (_temperatureC * 9.0f / 5.0f) + 32.0f;
    _temperatureK = _temperatureC + CELSIUS_TO_KELVIN;
    
    return _state = DHT_OK;
}

uint64_t DHT::pulseInLength(bool state, uint64_t timeout) {
    uint64_t start = esp_timer_get_time();
    while (gpio_get_level(_pin) == state) {
        if (esp_timer_get_time() - start > timeout) return 0;
    }
    return esp_timer_get_time() - start;
}
#ifndef TROYKA_DHT_H
#define TROYKA_DHT_H

#include "driver/gpio.h"

#define DHT11 11
#define DHT21 21
#define DHT22 22

#define HIGH 1
#define LOW 0

#define DHT_OK 0
#define DHT_ERROR_CHECKSUM -1
#define DHT_ERROR_TIMEOUT -2
#define DHT_ERROR_NO_REPLY -3

#define CELSIUS_TO_KELVIN 273.15f

class DHT {
public:
    DHT(gpio_num_t pin, uint8_t type);
    void begin();
    int8_t read();
    
    float getTemperatureC() const { return _temperatureC; }
    float getHumidity() const { return _humidity; }
    int8_t getState() const { return _state; }

private:
    uint64_t pulseInLength(bool state, uint64_t timeout);
    
    gpio_num_t _pin;
    uint8_t _type;
    int8_t _state;
    float _temperatureC;
    float _temperatureF;
    float _temperatureK;
    float _humidity;
};

#endif // TROYKA_DHT_H
#include "dht_sensor.h"
#include "TroykaDHT.h"
#include <map>

static std::map<int, DHT*> dht_sensors;

extern "C" void dht_init(int gpio_pin) {
    // Check if sensor for this GPIO already exists
    if (dht_sensors.find(gpio_pin) == dht_sensors.end()) {
        DHT* sensor = new DHT(static_cast<gpio_num_t>(gpio_pin), DHT22);
        sensor->begin();
        dht_sensors[gpio_pin] = sensor;
    }
}

extern "C" int dht_read_data(float *temperature, float *humidity) {
    // Find the first available sensor (for backward compatibility)
    if (dht_sensors.empty()) return -1;
    
    DHT* sensor = dht_sensors.begin()->second;
    int8_t result = sensor->read();
    if (result == DHT_OK) {
        *temperature = sensor->getTemperatureC();
        *humidity = sensor->getHumidity();
    }
    return result;
}

extern "C" int dht_read_data_gpio(int gpio_pin, float *temperature, float *humidity) {
    auto it = dht_sensors.find(gpio_pin);
    if (it == dht_sensors.end()) return -1;
    
    DHT* sensor = it->second;
    int8_t result = sensor->read();
    if (result == DHT_OK) {
        *temperature = sensor->getTemperatureC();
        *humidity = sensor->getHumidity();
    }
    return result;
}
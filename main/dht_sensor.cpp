#include "dht_sensor.h"
#include "TroykaDHT.h"

static DHT* dht_sensor = nullptr;

extern "C" void dht_init(int gpio_pin) {
    if (dht_sensor == nullptr) {
        dht_sensor = new DHT(static_cast<gpio_num_t>(gpio_pin), DHT22);
        dht_sensor->begin();
    }
}

extern "C" int dht_read_data(float *temperature, float *humidity) {
    if (dht_sensor == nullptr) return -1;
    
    int8_t result = dht_sensor->read();
    if (result == DHT_OK) {
        *temperature = dht_sensor->getTemperatureC();
        *humidity = dht_sensor->getHumidity();
    }
    return result;
}
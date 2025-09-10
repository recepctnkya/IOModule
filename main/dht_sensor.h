#ifndef DHT_SENSOR_H
#define DHT_SENSOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void dht_init(int gpio_pin);
int dht_read_data(float *temperature, float *humidity);

#ifdef __cplusplus
}
#endif

#endif // DHT_SENSOR_H
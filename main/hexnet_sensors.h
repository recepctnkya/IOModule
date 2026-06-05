#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HEXNET_SENSOR_COMM_OK    0
#define HEXNET_SENSOR_COMM_LOST  1

extern int hexnet_sensor_rs485_temperature;
extern int hexnet_sensor_rs485_humidity;
extern int hexnet_sensor_comm_status;

void hexnet_dht_task(void *arg);
void hexnet_sensor_rs485_task(void *arg);

#ifdef __cplusplus
}
#endif

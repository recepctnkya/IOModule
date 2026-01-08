#ifndef ADC_MONITOR_H
#define ADC_MONITOR_H

#include "esp_adc_cal.h"

#define ADC_WIDTH       ADC_WIDTH_BIT_12
#define ADC_ATTEN       ADC_ATTEN_DB_11
#define DEFAULT_VREF    1100 // in mV

#define ADC2_BATTERY_CHANNEL  ADC1_CHANNEL_2   // GPIO3
#define ADC1_SENSOR_CH0       ADC1_CHANNEL_3   // GPIO4
#define ADC1_SENSOR_CH1       ADC1_CHANNEL_4   // GPIO5
#define ADC1_SENSOR_CH2       ADC1_CHANNEL_5   // GPIO6
#define ADC1_SENSOR_CH3       ADC1_CHANNEL_6   // GPIO7

void adc_monitor_init(void);
void adc_monitor_task(void *param);

#endif // ADC_MONITOR_H

#ifndef ADC_MONITOR_H
#define ADC_MONITOR_H

#include "esp_adc_cal.h"

#define ADC_WIDTH       ADC_WIDTH_BIT_12
#define ADC_ATTEN       ADC_ATTEN_DB_11
#define DEFAULT_VREF    1100 // in mV

#define ADC2_BATTERY_CHANNEL  ADC2_CHANNEL_4   // GPIO15
#define ADC1_SENSOR_CH0       ADC1_CHANNEL_3   // GPIO39
#define ADC1_SENSOR_CH1       ADC1_CHANNEL_4   // GPIO32
#define ADC1_SENSOR_CH2       ADC1_CHANNEL_5   // GPIO33
#define ADC1_SENSOR_CH3       ADC1_CHANNEL_6   // GPIO34

void adc_monitor_init(void);
void adc_monitor_task(void *param);

#endif // ADC_MONITOR_H

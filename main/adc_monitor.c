#include "adc_monitor.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "waveshare_twai_port.h"

#define TAG "ADC_MONITOR"

// Calibration points for battery input voltage (in raw)
#define BATTERY_RAW_AT_11V5   2460
#define BATTERY_RAW_AT_14V5   3950
#define BATTERY_REF_VOLTAGE   12000  // 12.0 V in mV
#define WATERLEVEL_BUF_SIZE 100
// Calibration points for water level % mapping
#define SENSOR1_RAW_AT_0PCT    3181
#define SENSOR1_RAW_AT_100PCT  3213

#define SENSOR2_RAW_AT_0PCT    3182
#define SENSOR2_RAW_AT_100PCT  3216

#define ADC_SAMPLE_COUNT      100
static int waterLevel1_buf[WATERLEVEL_BUF_SIZE] = {0};
static int waterLevel2_buf[WATERLEVEL_BUF_SIZE] = {0};
static int waterLevel_buf_idx = 0;
int waterLevel1 = 0; // Global variable to store water level percentage
int waterLevel2 = 0; // Global variable to store water level percentage

static esp_adc_cal_characteristics_t adc1_chars;

// Helper to calculate average of buffer
static int average_buf(const int *buf) {
    int sum = 0;
    for (int i = 0; i < WATERLEVEL_BUF_SIZE; i++) {
        sum += buf[i];
    }
    return sum / WATERLEVEL_BUF_SIZE;
}

// Returns average water levels via pointers
void get_average_water_levels(int *pWaterLevel1, int *pWaterLevel2) {
    if (pWaterLevel1) *pWaterLevel1 = average_buf(waterLevel1_buf);
    if (pWaterLevel2) *pWaterLevel2 = average_buf(waterLevel2_buf);
}

static void init_adc1_channels()
{
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC1_SENSOR_CH0, ADC_ATTEN); // IO4
    adc1_config_channel_atten(ADC1_SENSOR_CH1, ADC_ATTEN);  // IO5
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, DEFAULT_VREF, &adc1_chars);
}

static uint32_t read_adc1_avg(adc1_channel_t channel)
{
    uint32_t sum = 0;
    for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
        sum += adc1_get_raw(channel);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return sum / ADC_SAMPLE_COUNT;
}

static uint32_t read_adc2_avg(adc2_channel_t channel)
{
    int raw;
    uint32_t sum = 0;
    for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
        if (adc2_get_raw(channel, ADC_WIDTH, &raw) == ESP_OK) {
            sum += raw;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return sum / ADC_SAMPLE_COUNT;
}

static float calculate_input_voltage(uint32_t raw)
{
    // Linear interpolation between calibration points
    float voltage = 11.5f + ((float)(raw - BATTERY_RAW_AT_11V5) / (BATTERY_RAW_AT_14V5 - BATTERY_RAW_AT_11V5)) * (14.5f - 11.5f);
    return voltage;
}

static int calculate_percentage_water_level_1(uint32_t raw, int32_t voltage_offset)
{
    int32_t adjusted_raw = raw - voltage_offset;
    int pct = ((adjusted_raw - SENSOR1_RAW_AT_0PCT) * 100) / (SENSOR1_RAW_AT_100PCT - SENSOR1_RAW_AT_0PCT);
    if (pct > 100) pct = 100;
    if (pct < 0) pct = 0;
    return pct;
}
static int calculate_percentage_water_level_2(uint32_t raw, int32_t voltage_offset)
{
    int32_t adjusted_raw = raw - voltage_offset;
    int pct = ((adjusted_raw - SENSOR2_RAW_AT_0PCT) * 100) / (SENSOR2_RAW_AT_100PCT - SENSOR2_RAW_AT_0PCT);
    if (pct > 100) pct = 100;
    if (pct < 0) pct = 0;
    return pct;
}

void adc_monitor_init(void)
{
    init_adc1_channels();
    // ADC2 calibration (battery) does not require adc2_config_channel_atten if not used with WiFi
}

void adc_monitor_task(void *param)
{
    while (1) {
        int battery_raw = read_adc2_avg(ADC2_BATTERY_CHANNEL);
        float voltage = calculate_input_voltage(battery_raw);
        int voltage_offset = 0;

        int raw0 = read_adc1_avg(ADC1_SENSOR_CH0);
        int raw1 = read_adc2_avg(ADC1_SENSOR_CH1);


        waterLevel1 = calculate_percentage_water_level_1(raw0, voltage_offset);
        waterLevel2 = calculate_percentage_water_level_2(raw1, voltage_offset);

        // Store in buffer and update index
        waterLevel1_buf[waterLevel_buf_idx] = waterLevel1;
        waterLevel2_buf[waterLevel_buf_idx] = waterLevel2;
        waterLevel_buf_idx = (waterLevel_buf_idx + 1) % WATERLEVEL_BUF_SIZE;

        get_average_water_levels(&waterLevel1, &waterLevel2);

        //ESP_LOGI(TAG, "Raw ADC values - Sensor 0: %d, Sensor 1: %d , Battery voltage: %.2f V | Water Level 1: %d%% | Water Level 2: %d%%", raw0, raw1,  voltage, waterLevel1, waterLevel2);
        // ESP_LOGI(TAG, "Battery voltage: %.2f V | Water Level 1: %d%% | Water Level 2: %d%%",
        //          voltage, waterLevel1, waterLevel2);
        
        // Update analog inputs with water level data
        update_water_levels_only(waterLevel1, waterLevel2); // DHT temps will be updated by DHT task
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

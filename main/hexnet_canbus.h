#ifndef __HEXNET_CANBUS_H // Header guard to prevent multiple inclusions 

#include <stdio.h>             // Standard I/O library 
#include <stdlib.h>            // Standard library for memory allocation, etc. 
#include "freertos/FreeRTOS.h" // FreeRTOS core header 
#include "freertos/task.h"     // FreeRTOS task management header 
#include "freertos/queue.h"    // FreeRTOS queue management header 
#include "freertos/semphr.h"   // FreeRTOS semaphore management header 
#include "esp_err.h"           // ESP-IDF error handling header 
#include "esp_log.h"           // ESP-IDF logging header 
#include "driver/twai.h"       // TWAI driver header 
#include <esp_timer.h>         // ESP timer header 

#include "driver/i2c.h"  // I2C driver header 
#include "driver/gpio.h" // GPIO driver header 

/* --------------------- Definitions and static variables ------------------ */
// Example Configuration 
#define TX_GPIO_NUM 18 // Transmit GPIO pin number 
#define RX_GPIO_NUM 17 // Receive GPIO pin number 

// #define TX_GPIO_NUM 15 // Transmit GPIO pin number 
// #define RX_GPIO_NUM 16 // Receive GPIO pin number 

#define EXAMPLE_TAG "TWAI Master"              // Log tag for TWAI master 

// Interval definitions 
#define TRANSMIT_RATE_MS 1000 // Message transmit rate in milliseconds 
#define POLLING_RATE_MS 1000  // Polling rate in milliseconds 

// Function prototypes 
esp_err_t waveshare_twai_init();     // Initialize TWAI 
esp_err_t waveshare_twai_transmit(); // Transmit data via TWAI 
void send_frames_task(void *arg);    // Task to send frames via TWAI
void receive_frames_task(void *pvParameter); // Task to receive frames via TWAI
void send_can_frame(uint32_t id, uint8_t *data); // Send a CAN frame via TWAI
void populate_frame_2(uint8_t analog_inputs[4], uint8_t dimmable_outputs[4]); // Populate frame 2 data
void populate_frame_3(uint8_t r, uint8_t g, uint8_t b); // Populate frame 3 data
void populate_frame_1(uint16_t voltage, uint16_t outputs, uint16_t inputs); // Populate frame 1 data
void handle_rx_message(twai_message_t message); // Handle received CAN messages
void can_watchdog_task(void *pvParameter); // Watchdog task for CAN monitoring
void update_analog_inputs(float dht1_temp, float dht2_temp, int waterlevel1, int waterlevel2); // Update analog inputs with sensor data
void update_water_levels_only(int waterlevel1, int waterlevel2); // Update only water levels while preserving temperature values
void set_framedata(uint8_t frame_id, uint8_t *data, size_t len); // Set frame data

// Getter functions for RGB values
uint8_t get_r_value(); 
uint8_t get_g_value();
uint8_t get_b_value();
uint8_t get_rgb_enable(); // Get RGB enable flag
uint16_t get_outputs();
uint8_t get_dimmable_output(uint8_t index); // Get outputs value
uint8_t get_analog_input(uint8_t index); // Get analog input value

#endif // End of header guard 

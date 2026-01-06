#include "hexnet_canbus.h" // Include the Waveshare TWAI port library
#include "stdio.h"
#include "stdint.h"
#include "string.h"
static bool driver_installed = false; // Driver installation status
unsigned long previousMillis = 0;     // Will store last time a message was sent
static uint32_t error_recovery_count = 0; // Error recovery counter
static uint32_t last_successful_tx = 0;   // Last successful transmission timestamp

// Forward declarations
static void check_and_recover_from_errors(void);
static void monitor_can_health(void);

// TWAI timing configuration for 50 Kbits
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
// TWAI filter configuration to accept all messages
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
// General configuration for TWAI
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NO_ACK);
#define TAG "CAN_BUS"

// CAN frame IDs
#define FRAME_1_ID 0x100
#define FRAME_2_ID 0x200
#define FRAME_3_ID 0x300
uint8_t rgb_enable = 0; // Enable RGB output

// Frame data
uint8_t frame_1_data[8] = {0};  // First frame data (Voltage, Outputs, Inputs)
uint8_t frame_2_data[8] = {0};  // Second frame data (Analog inputs, Dimmable outputs)
uint8_t frame_3_data[8] = {0};  // Third frame data (RGB colors)
uint16_t simvoltage = 1350;
uint16_t simoutputs = 0;
uint16_t siminputs = 0;

// Helper function to populate Frame 1 data
void populate_frame_1(uint16_t voltage, uint16_t outputs, uint16_t inputs) {
    frame_1_data[0] = (voltage >> 8) & 0xFF;  // Voltage MSB
    frame_1_data[1] = voltage & 0xFF;         // Voltage LSB
    frame_1_data[2] = (outputs >> 8) & 0xFF;  // Outputs MSB
    frame_1_data[3] = outputs & 0xFF;         // Outputs LSB
    frame_1_data[4] = (inputs >> 8) & 0xFF;   // Inputs MSB
    frame_1_data[5] = inputs & 0xFF;          // Inputs LSB
}


// Helper function to populate Frame 3 data
void populate_frame_3(uint8_t r, uint8_t g, uint8_t b) {
    frame_3_data[0] = r;  // Red
    frame_3_data[1] = g;  // Green
    frame_3_data[2] = b;  // Blue
    frame_3_data[3] = rgb_enable; // RGB enable flag
}

// Helper function to populate Frame 2 data
void populate_frame_2(uint8_t analog_inputs[4], uint8_t dimmable_outputs[4]) {
    for (int i = 0; i < 4; i++) {
        frame_2_data[i] = analog_inputs[i];        // Analog inputs
        frame_2_data[i + 4] = dimmable_outputs[i]; // Dimmable outputs
    }
}


// Function to send a CAN frame
void send_can_frame(uint32_t id, uint8_t *data) {
    twai_message_t message;
    message.identifier = id;
    message.rtr = 0;  // Data frame
    message.data_length_code = 8;  // Max CAN data length is 8 bytes
    memcpy(message.data, data, 8);

    // Send the message over the CAN bus
    esp_err_t res = twai_transmit(&message, pdMS_TO_TICKS(100));  // 100 ms timeout
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send frame ID: 0x%03X, error: 0x%x", (unsigned int)id, res);
        // Check if we need to recover from errors
        check_and_recover_from_errors();
    } else {
        last_successful_tx = esp_timer_get_time() / 1000; // Update successful TX timestamp
    }
}


uint8_t analog_inputs[4] = {0, 0, 0, 0};       // DHT1_temp, DHT2_temp, waterlevel1, waterlevel2
uint8_t dimmable_outputs[4] = {0,0,0,0}; // Example dimmable outputs
uint8_t r = 0, g = 255, b = 0; // Example RGB values (Red color)

// Function to update analog inputs with sensor data
void update_analog_inputs(float dht1_temp, float dht2_temp, int waterlevel1, int waterlevel2) {
    // Convert temperature to uint8_t (clamp to 0-255 range)
    analog_inputs[2] = (uint8_t)(dht1_temp);
    analog_inputs[3] = (uint8_t)(dht2_temp);
    
}

// Function to update only water levels while preserving temperature values
void update_water_levels_only(int waterlevel1, int waterlevel2) {
    // Only update water level values, preserve existing temperature values
    analog_inputs[0] = (uint8_t)(waterlevel1 > 0 ? (waterlevel1 < 255 ? waterlevel1 : 255) : 0);
    analog_inputs[1] = (uint8_t)(waterlevel2 > 0 ? (waterlevel2 < 255 ? waterlevel2 : 255) : 0);
}

// Task to send frames every 100ms
void send_frames_task(void *arg) {

    while (1) {
        // Populate the frames with custom data
        populate_frame_1(simvoltage, simoutputs, siminputs);  // Example: Voltage=1350, Outputs=0xFF00, Inputs=0xAA00
        populate_frame_2(analog_inputs, dimmable_outputs); // Example analog and dimmable output values
        populate_frame_3(r, g, b);  // Example RGB values

        send_can_frame(FRAME_1_ID, frame_1_data);
        vTaskDelay(pdMS_TO_TICKS(100));

        send_can_frame(FRAME_2_ID, frame_2_data);
        // ESP_LOGI(TAG, "Frame 2 - DHT1: %d°C, DHT2: %d°C, Water1: %d%%, Water2: %d%%, Dim1: %d, Dim2: %d, Dim3: %d, Dim4: %d", 
        //          analog_inputs[2], analog_inputs[3], analog_inputs[0], analog_inputs[1],
        //          dimmable_outputs[0], dimmable_outputs[1], dimmable_outputs[2], dimmable_outputs[3]);
        vTaskDelay(pdMS_TO_TICKS(100));

        send_can_frame(FRAME_3_ID, frame_3_data);
        vTaskDelay(pdMS_TO_TICKS(100));  // Wait for 100ms before sending the next cycle
    }
}

// Getter functions for RGB values
uint8_t get_r_value() {
    return r;
}

uint8_t get_g_value() {
    return g;
}

uint8_t get_b_value() {
    return b;
}
uint8_t get_rgb_enable() {
    return rgb_enable;
}

uint16_t get_outputs() {
    return simoutputs;
}

uint8_t get_dimmable_output(uint8_t index) {
    if (index < 4) {
        return dimmable_outputs[index];
    }
    return 0;
}

uint8_t get_analog_input(uint8_t index) {
    if (index < 4) {
        return analog_inputs[index];
    }
    return 0;
}


void handle_rx_message(twai_message_t message) {
    // CAN frame id 0x720: (index, val) -> update analog_inputs[index]
    if (message.identifier == 0x720 && message.data_length_code >= 2) {
        uint8_t index = message.data[0];
        uint8_t val = message.data[1];
        if (val) {
            simoutputs |= (1 << index);  // set bit at index
            ESP_LOGI(EXAMPLE_TAG, "Setting output %d to ON simoutput: %d", index , simoutputs);
        } else {
            simoutputs &= ~(1 << index); // clear bit at index
        }
    }
    // CAN frame id 0x730: (index, val) -> update dimmable_outputs[index]
    else if (message.identifier == 0x730 && message.data_length_code >= 2) {
        uint8_t index = message.data[0];
        uint8_t val = message.data[1];
        dimmable_outputs[index] = val;
        ESP_LOGI(EXAMPLE_TAG, "Setting dimmable %d to ON simoutput: %d", index , val);
    }
    // CAN frame id 0x740: (r, g, b) -> update RGB values
    else if (message.identifier == 0x740 && message.data_length_code >= 3) {
        r = message.data[0];
        g = message.data[1];
        b = message.data[2];
        rgb_enable = message.data[3]; // Update RGB enable flag
    }
    else if (message.identifier == 0x750 && message.data_length_code >= 1) {
        int motorData = message.data[0];
    }
}

// Error recovery function
static void check_and_recover_from_errors() {
    twai_status_info_t status;
    twai_get_status_info(&status);
    
    // Check for Bus-Off condition
    if (status.state == TWAI_STATE_BUS_OFF) {
        ESP_LOGW(TAG, "CAN Bus-Off detected! Attempting recovery...");
        error_recovery_count++;
        
        // Stop and restart the driver
        twai_stop();
        vTaskDelay(pdMS_TO_TICKS(100));
        
        if (twai_start() == ESP_OK) {
            ESP_LOGI(TAG, "CAN driver restarted successfully (recovery #%"PRIu32")", error_recovery_count);
        } else {
            ESP_LOGE(TAG, "Failed to restart CAN driver (recovery #%"PRIu32")", error_recovery_count);
        }
    }
    
    // Check for high error counts
    if (status.tx_error_counter > 100 || status.rx_error_counter > 100) {
        ESP_LOGW(TAG, "High error counts detected - TX: %"PRIu32", RX: %"PRIu32"", 
                 status.tx_error_counter, status.rx_error_counter);
        
        // Reset error counters
        twai_initiate_recovery();
        ESP_LOGI(TAG, "CAN error recovery initiated");
    }
}

// Watchdog monitoring function
static void monitor_can_health() {
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Check if we haven't had a successful transmission in 10 seconds
    if (last_successful_tx > 0 && (current_time - last_successful_tx) > 10000) {
        ESP_LOGW(TAG, "No successful CAN transmission for %"PRIu32" seconds", 
                 (current_time - last_successful_tx) / 1000);
        check_and_recover_from_errors();
    }
}

// Watchdog task to monitor CAN bus health
void can_watchdog_task(void *pvParameter) {
    uint32_t last_status_check = 0;
    uint32_t consecutive_failures = 0;
    
    while(1) {
        uint32_t current_time = esp_timer_get_time() / 1000;
        
        // Check CAN status every 5 seconds
        if (current_time - last_status_check >= 5000) {
            twai_status_info_t status;
            twai_get_status_info(&status);
            
            ESP_LOGI(TAG, "CAN Status - State: %d, TX Errors: %"PRIu32", RX Errors: %"PRIu32", Recovery Count: %"PRIu32"",
                     status.state, status.tx_error_counter, status.rx_error_counter, error_recovery_count);
            
            // Check for problematic states
            if (status.state == TWAI_STATE_BUS_OFF) {
                ESP_LOGE(TAG, "CAN Bus-Off state detected by watchdog!");
                check_and_recover_from_errors();
                consecutive_failures++;
            } else if (status.state == TWAI_STATE_RECOVERING) {
                ESP_LOGW(TAG, "CAN in recovery state");
            } else if (status.state == TWAI_STATE_RUNNING) {
                consecutive_failures = 0; // Reset failure counter on successful state
            }
            
            // If we have too many consecutive failures, try a full restart
            if (consecutive_failures >= 3) {
                ESP_LOGE(TAG, "Too many consecutive CAN failures (%"PRIu32"), attempting full restart", consecutive_failures);
                
                // Stop and uninstall driver
                twai_stop();
                vTaskDelay(pdMS_TO_TICKS(1000));
                twai_driver_uninstall();
                vTaskDelay(pdMS_TO_TICKS(1000));
                
                // Reinitialize
                esp_err_t ret = waveshare_twai_init();
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "CAN driver fully restarted successfully");
                    consecutive_failures = 0;
                } else {
                    ESP_LOGE(TAG, "Failed to restart CAN driver: 0x%x", ret);
                }
            }
            
            last_status_check = current_time;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
    }
}

int twaiCounter = 0;
void receive_frames_task(void *pvParameter)
{
    twai_message_t message;
    
    while(1)
    {
        // Check if alert happened with timeout
        uint32_t alerts_triggered;
        esp_err_t alert_result = twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(100));
        
        if (alert_result == ESP_OK) {
            twai_status_info_t twaistatus;
            twai_get_status_info(&twaistatus);

            // Handle alerts
            if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
                ESP_LOGW(EXAMPLE_TAG,"Alert: TWAI controller has become error passive.");
                check_and_recover_from_errors();
            }
            
            if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
                ESP_LOGW(EXAMPLE_TAG,"Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
                ESP_LOGW(EXAMPLE_TAG,"Bus error count: %"PRIu32, twaistatus.bus_error_count);
                check_and_recover_from_errors();
            }

            if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
                ESP_LOGW(EXAMPLE_TAG,"Alert: The RX queue is full causing a received frame to be lost.");
                ESP_LOGW(EXAMPLE_TAG,"RX buffered: %"PRIu32, twaistatus.msgs_to_rx);
                ESP_LOGW(EXAMPLE_TAG,"RX missed: %"PRIu32, twaistatus.rx_missed_count);
                ESP_LOGW(EXAMPLE_TAG,"RX overrun %"PRIu32, twaistatus.rx_overrun_count);
                
                // Clear the queue aggressively to prevent further issues
                int cleared_count = 0;
                while (twai_receive(&message, 0) == ESP_OK) {
                    cleared_count++;
                    // Process the message if it's important, otherwise discard
                    if (message.identifier == 0x720 || message.identifier == 0x730 || message.identifier == 0x740) {
                        handle_rx_message(message);
                    }
                }
                ESP_LOGW(EXAMPLE_TAG,"Cleared %d messages from RX queue", cleared_count);
            }

            // Check if message is received
            if (alerts_triggered & TWAI_ALERT_RX_DATA) {     
                if (twai_receive(&message, 0) == ESP_OK) {
                   handle_rx_message(message);
                }
            }
        } else if (alert_result == ESP_ERR_TIMEOUT) {
            // Timeout is normal, continue monitoring
        } else {
            ESP_LOGE(EXAMPLE_TAG, "Error reading alerts: 0x%x", alert_result);
        }
        
        // Monitor CAN health periodically
        monitor_can_health();
        
        // Proactive queue monitoring - check queue level and clear if getting full
        twai_status_info_t status;
        twai_get_status_info(&status);
        if (status.msgs_to_rx > 3) { // If more than 3 messages queued
            ESP_LOGW(EXAMPLE_TAG, "RX queue getting full (%"PRIu32" msgs), clearing proactively", status.msgs_to_rx);
            int cleared_count = 0;
            while (twai_receive(&message, 0) == ESP_OK && cleared_count < 5) { // Clear up to 5 messages
                cleared_count++;
                if (message.identifier == 0x720 || message.identifier == 0x730 || message.identifier == 0x740) {
                    handle_rx_message(message);
                }
            }
            if (cleared_count > 0) {
                ESP_LOGW(EXAMPLE_TAG, "Proactively cleared %d messages from RX queue", cleared_count);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); // Reduced delay for faster message processing
    }
}


// Function to send a message
static void send_message()
{
    // Configure message to transmit
    twai_message_t message;       // Message structure
    message.identifier = 0x0F6;   // Message identifier
    message.data_length_code = 8; // Data length code
    for (int i = 0; i < 8; i++)
    {
        message.data[i] = i; // Set message data
    }

    // Queue message for transmission
    esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(1000));
    if (result == ESP_OK)
    {
        ESP_LOGD(EXAMPLE_TAG, "Message queued for transmission successfully");
        last_successful_tx = esp_timer_get_time() / 1000; // Update successful TX timestamp
    }
    else
    {
        ESP_LOGE(EXAMPLE_TAG, "Failed to queue message for transmission: 0x%x", result);
        check_and_recover_from_errors();
    }
}

// Function to initialize Waveshare TWAI
esp_err_t waveshare_twai_init()
{
    esp_err_t ret;
    
    // Install TWAI driver
    ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret == ESP_OK) {
        ESP_LOGI(EXAMPLE_TAG,"Driver installed");
    } else {
        ESP_LOGE(EXAMPLE_TAG,"Failed to install driver: 0x%x", ret);
        return ret;
    }
    
    // Start TWAI driver
    ret = twai_start();
    if (ret == ESP_OK) {
        ESP_LOGI(EXAMPLE_TAG,"Driver started");
    } else {
        ESP_LOGE(EXAMPLE_TAG,"Failed to start driver: 0x%x", ret);
        twai_driver_uninstall();
        return ret;
    }

    // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | 
                               TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_FAILED | TWAI_ALERT_TX_SUCCESS;
    ret = twai_reconfigure_alerts(alerts_to_enable, NULL);
    if (ret == ESP_OK) {
        ESP_LOGI(EXAMPLE_TAG,"CAN Alerts reconfigured");
    } else {
        ESP_LOGE(EXAMPLE_TAG,"Failed to reconfigure alerts: 0x%x", ret);
        twai_stop();
        twai_driver_uninstall();
        return ret;
    }

    // Initialize monitoring variables
    last_successful_tx = esp_timer_get_time() / 1000;
    error_recovery_count = 0;
    
    // TWAI driver is now successfully installed and started
    driver_installed = true;
    ESP_LOGI(EXAMPLE_TAG,"CAN bus initialization completed successfully");
    return ESP_OK;           // Return success status
}

// Function to transmit messages
esp_err_t waveshare_twai_transmit()
{
    if (!driver_installed)
    {
        ESP_LOGE(EXAMPLE_TAG, "Driver not installed, cannot transmit");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait before retrying
        return ESP_FAIL;                 // Return failure status
    }
    
    // Check if alert happened with timeout
    uint32_t alerts_triggered;
    esp_err_t alert_result = twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(100));
    
    if (alert_result == ESP_OK) {
        twai_status_info_t twaistatus;
        twai_get_status_info(&twaistatus);

        // Handle alerts
        if (alerts_triggered & TWAI_ALERT_ERR_PASS)
        {
            ESP_LOGW(EXAMPLE_TAG, "Alert: TWAI controller has become error passive.");
            check_and_recover_from_errors();
        }
        if (alerts_triggered & TWAI_ALERT_BUS_ERROR)
        {
            ESP_LOGW(EXAMPLE_TAG, "Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
            ESP_LOGW(EXAMPLE_TAG, "Bus error count: %" PRIu32, twaistatus.bus_error_count);
            check_and_recover_from_errors();
        }
        if (alerts_triggered & TWAI_ALERT_TX_FAILED)
        {
            ESP_LOGW(EXAMPLE_TAG, "Alert: The Transmission failed.");
            ESP_LOGW(EXAMPLE_TAG, "TX buffered: %" PRIu32, twaistatus.msgs_to_tx);
            ESP_LOGW(EXAMPLE_TAG, "TX error: %" PRIu32, twaistatus.tx_error_counter);
            ESP_LOGW(EXAMPLE_TAG, "TX failed: %" PRIu32, twaistatus.tx_failed_count);
            check_and_recover_from_errors();
        }
        if (alerts_triggered & TWAI_ALERT_TX_SUCCESS)
        {
            ESP_LOGD(EXAMPLE_TAG, "Alert: The Transmission was successful.");
            ESP_LOGD(EXAMPLE_TAG, "TX buffered: %" PRIu32, twaistatus.msgs_to_tx);
            last_successful_tx = esp_timer_get_time() / 1000; // Update successful TX timestamp
        }
    } else if (alert_result == ESP_ERR_TIMEOUT) {
        // Timeout is normal, continue
    } else {
        ESP_LOGE(EXAMPLE_TAG, "Error reading alerts: 0x%x", alert_result);
        return alert_result;
    }

    // Send message
    unsigned long currentMillis = esp_timer_get_time() / 1000; // Get current time in milliseconds
    if (currentMillis - previousMillis >= TRANSMIT_RATE_MS)
    {                                   // Check if it's time to send the message
        previousMillis = currentMillis; // Update last send time
        send_message();                 // Call send message function
    }
    return ESP_OK; // Return success status
}

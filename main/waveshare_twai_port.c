#include "waveshare_twai_port.h" // Include the Waveshare TWAI port library
#include "stdio.h"
#include "stdint.h"
#include "string.h"
static bool driver_installed = false; // Driver installation status
unsigned long previousMillis = 0;     // Will store last time a message was sent

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
        ESP_LOGE(TAG, "Failed to send frame ID: 0x%03X", (unsigned int)id);
    }
}


uint8_t analog_inputs[4] = {76, 90, 30, 32};       // Example analog inputs
uint8_t dimmable_outputs[4] = {0,0,0,0}; // Example dimmable outputs
uint8_t r = 0, g = 255, b = 0; // Example RGB values (Red color)

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
}

int twaiCounter = 0;
void receive_frames_task(void *pvParameter)
{
    twai_message_t message;
    
    while(1)
    {

        // Check if alert happened
        uint32_t alerts_triggered;
        twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS-900));
        twai_status_info_t twaistatus;
        twai_get_status_info(&twaistatus);

        // Handle alerts
        if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
            ESP_LOGI(EXAMPLE_TAG,"Alert: TWAI controller has become error passive.");
        }
        if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
            ESP_LOGI(EXAMPLE_TAG,"Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
            ESP_LOGI(EXAMPLE_TAG,"Bus error count: %"PRIu32, twaistatus.bus_error_count);
        }

        if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
            ESP_LOGI(EXAMPLE_TAG,"Alert: The RX queue is full causing a received frame to be lost.");
            ESP_LOGI(EXAMPLE_TAG,"RX buffered: %"PRIu32, twaistatus.msgs_to_rx);
            ESP_LOGI(EXAMPLE_TAG,"RX missed: %"PRIu32, twaistatus.rx_missed_count);
            ESP_LOGI(EXAMPLE_TAG,"RX overrun %"PRIu32, twaistatus.rx_overrun_count);
        }



        // Check if message is received
        if (alerts_triggered & TWAI_ALERT_RX_DATA)
        {     
            if (twai_receive(&message, 0) == ESP_OK) {
               handle_rx_message(message);
            }
            else {
                
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
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
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK)
    {
        printf("Message queued for transmission\n"); // Success message
    }
    else
    {
        printf("Failed to queue message for transmission\n"); // Failure message
    }
}

// Function to initialize Waveshare TWAI
esp_err_t waveshare_twai_init()
{
    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        ESP_LOGI(EXAMPLE_TAG,"Driver installed");
    } else {
        ESP_LOGE(EXAMPLE_TAG,"Failed to install driver");
    }
    
    // Start TWAI driver
    if (twai_start() == ESP_OK) {
        ESP_LOGI(EXAMPLE_TAG,"Driver started");
    } else {
        ESP_LOGE(EXAMPLE_TAG,"Failed to start driver");
    }

    // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
    if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
        ESP_LOGI(EXAMPLE_TAG,"CAN Alerts reconfigured");
    } else {
        ESP_LOGE(EXAMPLE_TAG,"Failed to reconfigure alerts");
    }

    // TWAI driver is now successfully installed and started
    driver_installed = true;
    return ESP_OK;           // Return success status
}

// Function to transmit messages
esp_err_t waveshare_twai_transmit()
{

    if (!driver_installed)
    {
        // Driver not installed
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait before retrying
        return ESP_FAIL;                 // Return failure status
    }
    // Check if alert happened
    uint32_t alerts_triggered;                                           // Variable to store triggered alerts
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS)); // Read alerts
    twai_status_info_t twaistatus;                                       // Variable to store TWAI status information
    twai_get_status_info(&twaistatus);                                   // Get TWAI status information

    // Handle alerts
    if (alerts_triggered & TWAI_ALERT_ERR_PASS)
    {
        ESP_LOGI(EXAMPLE_TAG, "Alert: TWAI controller has become error passive."); // Log error passive alert
    }
    if (alerts_triggered & TWAI_ALERT_BUS_ERROR)
    {
        ESP_LOGI(EXAMPLE_TAG, "Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus."); // Log bus error alert
        ESP_LOGI(EXAMPLE_TAG, "Bus error count: %" PRIu32, twaistatus.bus_error_count);                // Log bus error count
    }
    if (alerts_triggered & TWAI_ALERT_TX_FAILED)
    {
        ESP_LOGI(EXAMPLE_TAG, "Alert: The Transmission failed.");                 // Log transmission failure alert
        ESP_LOGI(EXAMPLE_TAG, "TX buffered: %" PRIu32, twaistatus.msgs_to_tx);    // Log buffered messages count
        ESP_LOGI(EXAMPLE_TAG, "TX error: %" PRIu32, twaistatus.tx_error_counter); // Log transmission error count
        ESP_LOGI(EXAMPLE_TAG, "TX failed: %" PRIu32, twaistatus.tx_failed_count); // Log failed transmission count
    }
    if (alerts_triggered & TWAI_ALERT_TX_SUCCESS)
    {
        ESP_LOGI(EXAMPLE_TAG, "Alert: The Transmission was successful.");      // Log transmission success alert
        ESP_LOGI(EXAMPLE_TAG, "TX buffered: %" PRIu32, twaistatus.msgs_to_tx); // Log buffered messages count
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

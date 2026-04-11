#pragma once

#include "esp_err.h"
#include <stdbool.h>

/** WiFi provisioning state – readable by other tasks (e.g. LED indicator). */
typedef enum {
    WIFI_PROV_STATE_IDLE,           /* Not yet started                          */
    WIFI_PROV_STATE_DISCONNECTED,   /* AP active, no client connected – 2000 ms */
    WIFI_PROV_STATE_AP_CLIENT,      /* Phone/PC connected to our AP – 100 ms    */
    WIFI_PROV_STATE_CONNECTED,      /* STA got IP from router – solid ON        */
} wifi_prov_state_t;

/** Return current provisioning / connection state. */
wifi_prov_state_t wifi_prov_get_state(void);

/**
 * @brief Initialize WiFi with captive-portal provisioning.
 *
 * Flow:
 *  1. Read SSID / password from NVS ("wifi_creds" namespace).
 *  2. If credentials exist → try to connect as STA (30 s timeout).
 *     On success → return ESP_OK (state = CONNECTED).
 *  3. If no credentials → state = PROVISIONING.
 *     If credentials exist but fail → state = CREDS_FAILED, then PROVISIONING.
 *  4. Start AP "ESP32-Setup" + DNS redirect + HTTP config page and
 *     BLOCK until the user has submitted valid credentials.
 *
 * Call this once from app_main() before starting any network tasks.
 * Start the LED indicator task BEFORE calling this so it can reflect state.
 */
esp_err_t wifi_prov_init(void);

/**
 * @brief Erase stored WiFi credentials and restart provisioning on
 *        the next boot (call before esp_restart()).
 */
void wifi_prov_erase_credentials(void);

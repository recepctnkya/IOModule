#include "hexnet_nvs.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "HEXNET_NVS";

// Function to write an integer to NVS
esp_err_t nvs_write_int(const char* key, int value) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Open NVS handle
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    // Write integer to NVS
    err = nvs_set_i32(nvs_handle, key, value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error writing integer to NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    // Commit written value
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    return err;
}

// Function to read an integer from NVS
esp_err_t nvs_read_int(const char* key, int* value) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Open NVS handle
    err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    // Read integer from NVS
    err = nvs_get_i32(nvs_handle, key, value);
    nvs_close(nvs_handle);
    return err;
}

// Function to write a string to NVS
esp_err_t nvs_write_string(const char* key, const char* value) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Open NVS handle
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    // Write string to NVS
    err = nvs_set_str(nvs_handle, key, value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error writing string to NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    // Commit written value
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    return err;
}

// Function to read a string from NVS
esp_err_t nvs_read_string(const char* key, char* value, size_t length) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Open NVS handle
    err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    // Read string from NVS
    err = nvs_get_str(nvs_handle, key, value, &length);
    nvs_close(nvs_handle);
    return err;
}
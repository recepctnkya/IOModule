#ifndef HEXNET_NVS_H
#define HEXNET_NVS_H

#include "esp_err.h"

// Function to write an integer to NVS
esp_err_t nvs_write_int(const char* key, int value);

// Function to read an integer from NVS
esp_err_t nvs_read_int(const char* key, int* value);

// Function to write a string to NVS
esp_err_t nvs_write_string(const char* key, const char* value);

// Function to read a string from NVS
esp_err_t nvs_read_string(const char* key, char* value, size_t length);

#endif // HEXNET_NVS_H
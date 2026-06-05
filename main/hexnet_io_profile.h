#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cJSON.h"
#include "esp_err.h"

#define HEXNET_IO_RELAY_SLOTS   16
#define HEXNET_IO_DIM_SLOTS     4
#define HEXNET_IO_SENSOR_SLOTS  5
#define HEXNET_IO_TANK_SLOTS    4

/** Kaynak bayrakları (hangi istemci profili güncelledi / bağlı sayılır). */
#define HEXNET_IO_LINK_DISPLAY 0x01
#define HEXNET_IO_LINK_BLE     0x02
#define HEXNET_IO_LINK_REMOTE  0x04

typedef struct {
    uint8_t num_relays;
    uint8_t num_dims;
    uint8_t num_sensors;
    uint8_t num_tanks;
    uint8_t motor_enabled;
    uint8_t rgb_enabled;
    uint16_t relay_enable_mask;
    uint8_t dim_enable_mask;
    uint8_t sensor_enable_mask;
    uint8_t tank_enable_mask;
    uint8_t relay_type[HEXNET_IO_RELAY_SLOTS];
    uint8_t dim_type[HEXNET_IO_DIM_SLOTS];
    uint8_t sensor_type[HEXNET_IO_SENSOR_SLOTS];
    uint8_t link_flags;
} hexnet_io_profile_t;

void hexnet_io_profile_init(void);

const hexnet_io_profile_t *hexnet_io_profile_get(void);

esp_err_t hexnet_io_profile_save(void);

/** BLE Configuration JSON veya platform io_config_set. */
esp_err_t hexnet_io_profile_apply_json(const cJSON *json, uint8_t link_flag);

void hexnet_io_profile_append_to_json(cJSON *parent, const char *object_name);

bool hexnet_io_profile_relay_slot_enabled(uint8_t index);
bool hexnet_io_profile_dim_slot_enabled(uint8_t index);
bool hexnet_io_profile_sensor_slot_enabled(uint8_t index);
bool hexnet_io_profile_motor_enabled(void);
bool hexnet_io_profile_rgb_enabled(void);

/** numOfOutputs / buffers ile uyum (BLE telemetri). */
void hexnet_io_profile_sync_legacy_globals(void);

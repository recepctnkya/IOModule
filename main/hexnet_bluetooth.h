/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
        
#ifndef HEXNET_BLUETOOTH_H
#define HEXNET_BLUETOOTH_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"
#include "esp_timer.h"
#include "cJSON.h"

#define GATTS_TABLE_TAG  "HEXNET_BLE"

#define SPP_PROFILE_NUM             1
#define SPP_PROFILE_APP_IDX         0
#define ESP_SPP_APP_ID              0x56
#define SAMPLE_DEVICE_NAME          "VANGO_"    //The Device Name Characteristics in GAP
#define SPP_SVC_INST_ID	            0

#define BLE_ENB 0

/// SPP Service
static const uint16_t spp_service_uuid = 0xABF0;
/// Characteristic UUID
#define ESP_GATT_UUID_SPP_DATA_RECEIVE      0xABF1
#define ESP_GATT_UUID_SPP_DATA_NOTIFY       0xABF2
#define ESP_GATT_UUID_SPP_COMMAND_RECEIVE   0xABF3
#define ESP_GATT_UUID_SPP_COMMAND_NOTIFY    0xABF4

#ifdef SUPPORT_HEARTBEAT
#define ESP_GATT_UUID_SPP_HEARTBEAT         0xABF5
#endif


#define spp_sprintf(s,...)         sprintf((char*)(s), ##__VA_ARGS__)
#define SPP_DATA_MAX_LEN           (512)
#define SPP_CMD_MAX_LEN            (20)
#define SPP_STATUS_MAX_LEN         (20)
#define SPP_DATA_BUFF_MAX_LEN      (2*1024)
///Attributes State Machine
enum{
    SPP_IDX_SVC,

    SPP_IDX_SPP_DATA_RECV_CHAR,
    SPP_IDX_SPP_DATA_RECV_VAL,

    SPP_IDX_SPP_DATA_NOTIFY_CHAR,
    SPP_IDX_SPP_DATA_NTY_VAL,
    SPP_IDX_SPP_DATA_NTF_CFG,

    SPP_IDX_SPP_COMMAND_CHAR,
    SPP_IDX_SPP_COMMAND_VAL,

    SPP_IDX_SPP_STATUS_CHAR,
    SPP_IDX_SPP_STATUS_VAL,
    SPP_IDX_SPP_STATUS_CFG,

#ifdef SUPPORT_HEARTBEAT
    SPP_IDX_SPP_HEARTBEAT_CHAR,
    SPP_IDX_SPP_HEARTBEAT_VAL,
    SPP_IDX_SPP_HEARTBEAT_CFG,
#endif

    SPP_IDX_NB,
};

void ble_init(void);
static uint8_t find_char_and_desr_index(uint16_t handle);
static bool store_wr_buffer(esp_ble_gatts_cb_param_t *p_data);
static void free_write_buffer(void);
static void process_complete_data(void);
bool get_connection_status();
void set_converted_json_data(char* converted_json_data);
uint8_t* get_spp_cmd_buff();
void reset_spp_cmd_buff();
void parse_ble_data(const char* json_data);
static void parse_read_data(cJSON* json);
static void parse_write_data(cJSON* json);
static void parse_configuration_data(cJSON* json);
static void parse_rules_data(cJSON* json);
void load_theme_configuration_from_nvs(int* themeType, int* wallpaperEnabled, int* wallpaperTimeIndex);
void load_panel_configuration_from_nvs(int *totalOutpts, int buffer1[16], int *totalSensors, int buffer2[5], int *totalDims, int buffer3[4]);
void save_panel_configuration_to_nvs(int totalOutps, int buffer1[16], int totalSensors, int buffer2[5], int totalDims, int buffer3[4]);
void get_data_json_format(const uint16_t* regs_data, int txPacketType, char** json_str);
uint8_t get_rgb_val(uint8_t index);
uint8_t get_dim_value(int index);
uint16_t get_outputsFromBle();

#ifdef SUPPORT_HEARTBEAT
void spp_heartbeat_task(void * arg);
#endif

void bluetooth_send_periodic_callback(TimerHandle_t xTimer);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#endif /* HEXNET_BLUETOOTH_H */

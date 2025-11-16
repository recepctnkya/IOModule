/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "hexnet_bluetooth.h"
#include "cJSON.h"
#include "hexnet_canbus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"


static const uint8_t spp_adv_data[23] = {
    /* Flags */
    0x02,0x01,0x06,
    /* Complete List of 16-bit Service Class UUIDs */
    0x03,0x03,0xF0,0xAB,
    /* Complete Local Name in advertising */
    0x0F,0x09, 'V', 'A', 'N', 'G', 'O', '_'
};


char* converted_json_data2 = NULL;
// Example data to save
int numOfOutputs = 16;
int numOfDims = 4;
int numOfSensors = 5;
float batarya_volt = 0;
bool slaveConnectionStatus = true;
int panelThemeType = 0;
int panelWallpaperEnable = false;
int panelWallpaperTime = 0;
int numberOfNotifications = 0;
cJSON* notifications = NULL;
// Declare the global outputsBuffer
int outputsBuffer[16] = {0};
int sensorsBuffer[5] = {0};
int dimsBuffer[4] = {0};
int rgbBuffer[4] = {0};
int btn_index = 0;
uint8_t rgbEna = 0; // RGB LED enable variable
uint8_t fromBleConfig = 0;
int panelWallpaperEnableCounter = 1;
uint16_t outputsFromBle = 0;
int dimsDataBuffer[4] = {0};



// Define the characteristic UUID
#define CHAR_UUID_REGS_DATA 0x2A56

uint8_t spp_cmd_buff[1000];

static uint16_t spp_mtu_size = 500;  // Initialize to requested MTU, will be updated after negotiation
static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;
QueueHandle_t spp_uart_queue = NULL;
static QueueHandle_t cmd_cmd_queue = NULL;
static QueueHandle_t data_transmit_queue = NULL;

#ifdef SUPPORT_HEARTBEAT
static QueueHandle_t cmd_heartbeat_queue = NULL;
static uint8_t  heartbeat_s[9] = {'E','s','p','r','e','s','s','i','f'};
static bool enable_heart_ntf = false;
static uint8_t heartbeat_count_num = 0;
#endif

static bool enable_data_ntf = false;
static bool is_connected = false;
static esp_bd_addr_t spp_remote_bda = {0x0,};

// MTU negotiation state tracking
static bool mtu_negotiation_pending = false;

static uint16_t spp_handle_table[SPP_IDX_NB];

static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

typedef struct spp_receive_data_node{
    int32_t len;
    uint8_t * node_buff;
    struct spp_receive_data_node * next_node;
}spp_receive_data_node_t;

static spp_receive_data_node_t * temp_spp_recv_data_node_p1 = NULL;
static spp_receive_data_node_t * temp_spp_recv_data_node_p2 = NULL;

typedef struct spp_receive_data_buff{
    int32_t node_num;
    int32_t buff_size;
    spp_receive_data_node_t * first_node;
}spp_receive_data_buff_t;

static spp_receive_data_buff_t SppRecvDataBuff = {
    .node_num   = 0,
    .buff_size  = 0,
    .first_node = NULL
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/*
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_READ;

#ifdef SUPPORT_HEARTBEAT
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
#endif

///SPP Service - data receive characteristic, read&write without response
static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t  spp_data_receive_val[20] = {0x00};

///SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t  spp_data_notify_val[20] = {0x00};
static const uint8_t  spp_data_notify_ccc[2] = {0x00, 0x00};

///SPP Service - command characteristic, read&write without response
static const uint16_t spp_command_uuid = ESP_GATT_UUID_SPP_COMMAND_RECEIVE;
static const uint8_t  spp_command_val[10] = {0x00};

///SPP Service - status characteristic, notify&read
static const uint16_t spp_status_uuid = ESP_GATT_UUID_SPP_COMMAND_NOTIFY;
static const uint8_t  spp_status_val[10] = {0x00};
static const uint8_t  spp_status_ccc[2] = {0x00, 0x00};

#ifdef SUPPORT_HEARTBEAT
///SPP Server - Heart beat characteristic, notify&write&read
static const uint16_t spp_heart_beat_uuid = ESP_GATT_UUID_SPP_HEARTBEAT;
static const uint8_t  spp_heart_beat_val[2] = {0x00, 0x00};
static const uint8_t  spp_heart_beat_ccc[2] = {0x00, 0x00};
#endif

///Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] =
{
    //SPP -  Service Declaration
    [SPP_IDX_SVC]                      	=
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
    sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

    //SPP -  data receive characteristic Declaration
    [SPP_IDX_SPP_DATA_RECV_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  data receive characteristic Value
    [SPP_IDX_SPP_DATA_RECV_VAL]             	=
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_DATA_MAX_LEN,sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val}},

    //SPP -  data notify characteristic Declaration
    [SPP_IDX_SPP_DATA_NOTIFY_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  data notify characteristic Value
    [SPP_IDX_SPP_DATA_NTY_VAL]   =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_notify_uuid, ESP_GATT_PERM_READ,
    SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *)spp_data_notify_val}},

    //SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_DATA_NTF_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},

    //SPP -  command characteristic Declaration
    [SPP_IDX_SPP_COMMAND_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  command characteristic Value
    [SPP_IDX_SPP_COMMAND_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_command_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_CMD_MAX_LEN,sizeof(spp_command_val), (uint8_t *)spp_command_val}},

    //SPP -  status characteristic Declaration
    [SPP_IDX_SPP_STATUS_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  status characteristic Value
    [SPP_IDX_SPP_STATUS_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_status_uuid, ESP_GATT_PERM_READ,
    SPP_STATUS_MAX_LEN,sizeof(spp_status_val), (uint8_t *)spp_status_val}},

    //SPP -  status characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_STATUS_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_status_ccc), (uint8_t *)spp_status_ccc}},

#ifdef SUPPORT_HEARTBEAT
    //SPP -  Heart beat characteristic Declaration
    [SPP_IDX_SPP_HEARTBEAT_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    //SPP -  Heart beat characteristic Value
    [SPP_IDX_SPP_HEARTBEAT_VAL]   =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_heart_beat_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(spp_heart_beat_val), sizeof(spp_heart_beat_val), (uint8_t *)spp_heart_beat_val}},

    //SPP -  Heart beat characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_HEARTBEAT_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_heart_beat_ccc}},
#endif
};

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;

    for(int i = 0; i < SPP_IDX_NB ; i++){
        if(handle == spp_handle_table[i]){
            return i;
        }
    }

    return error;
}

static bool store_wr_buffer(esp_ble_gatts_cb_param_t *p_data)
{
    temp_spp_recv_data_node_p1 = (spp_receive_data_node_t *)malloc(sizeof(spp_receive_data_node_t));

    if(temp_spp_recv_data_node_p1 == NULL){
        ESP_LOGI(GATTS_TABLE_TAG, "malloc error %s %d", __func__, __LINE__);
        return false;
    }
    if(temp_spp_recv_data_node_p2 != NULL){
        temp_spp_recv_data_node_p2->next_node = temp_spp_recv_data_node_p1;
    }
    temp_spp_recv_data_node_p1->len = p_data->write.len;
    SppRecvDataBuff.buff_size += p_data->write.len;
    temp_spp_recv_data_node_p1->next_node = NULL;
    temp_spp_recv_data_node_p1->node_buff = (uint8_t *)malloc(p_data->write.len);
    temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1;
    if (temp_spp_recv_data_node_p1->node_buff == NULL) {
        ESP_LOGI(GATTS_TABLE_TAG, "malloc error %s %d\n", __func__, __LINE__);
        temp_spp_recv_data_node_p1->len = 0;
    } else {
        memcpy(temp_spp_recv_data_node_p1->node_buff,p_data->write.value,p_data->write.len);
    }

    if(SppRecvDataBuff.node_num == 0){
        SppRecvDataBuff.first_node = temp_spp_recv_data_node_p1;
        SppRecvDataBuff.node_num++;
    }else{
        SppRecvDataBuff.node_num++;
    }

    return true;
}

static void process_complete_data(void)
{
    if (SppRecvDataBuff.first_node == NULL || SppRecvDataBuff.buff_size == 0) {
        return;
    }

    // Allocate buffer for complete data
    char *complete_data = malloc(SppRecvDataBuff.buff_size + 1);
    if (complete_data == NULL) {
        ESP_LOGE(GATTS_TABLE_TAG, "Failed to allocate memory for complete data");
        free_write_buffer();
        return;
    }

    // Reassemble data from all chunks
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;
    int offset = 0;
    
    while(temp_spp_recv_data_node_p1 != NULL){
        if (temp_spp_recv_data_node_p1->node_buff && temp_spp_recv_data_node_p1->len > 0) {
            memcpy(complete_data + offset, temp_spp_recv_data_node_p1->node_buff, temp_spp_recv_data_node_p1->len);
            offset += temp_spp_recv_data_node_p1->len;
        }
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p1->next_node;
    }
    
    complete_data[offset] = '\0'; // Null terminate
    
    ESP_LOGI(GATTS_TABLE_TAG, "Complete data received (%d bytes): %s", offset, complete_data);
    
    // Process the complete JSON data
    parse_ble_data(complete_data);
    
    // Clean up
    free(complete_data);
    free_write_buffer();
}

static void free_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while(temp_spp_recv_data_node_p1 != NULL){
        temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1->next_node;
        if (temp_spp_recv_data_node_p1->node_buff) {
            free(temp_spp_recv_data_node_p1->node_buff);
        }
        free(temp_spp_recv_data_node_p1);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p2;
    }

    SppRecvDataBuff.node_num = 0;
    SppRecvDataBuff.buff_size = 0;
    SppRecvDataBuff.first_node = NULL;
}

//BURASSSSSSSSSSSSSSSSSIIIIIIIIIIIIIIIIIII  COOOOOOOOKKKK    ONEEEEEEEEEEMMMMLLLIIII
#ifdef SUPPORT_HEARTBEAT
void spp_heartbeat_task(void * arg)
{
    uint16_t cmd_id;

    for(;;) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if(xQueueReceive(cmd_heartbeat_queue, &cmd_id, portMAX_DELAY)) {
            while(1){
                heartbeat_count_num++;
                vTaskDelay(5000/ portTICK_PERIOD_MS);
                if((heartbeat_count_num >3)&&(is_connected)){
                    esp_ble_gap_disconnect(spp_remote_bda);
                }
                if(is_connected && enable_heart_ntf){
                    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_HEARTBEAT_VAL],sizeof(heartbeat_s), heartbeat_s, false);
                }else if(!is_connected){
                    break;
                }
            }
        }
    }
    vTaskDelete(NULL);
}
#endif


const char example_json_data_packet[] = "{\"temperature\": 25.0, \"humidity\": 50.0, \"pressure\": 1013.25}";

// Data transmission task to handle chunking
void data_transmit_task(void *arg)
{
    char *data_to_send = NULL;
    
    for(;;) {
        if(xQueueReceive(data_transmit_queue, &data_to_send, portMAX_DELAY)) {
            if (data_to_send != NULL && is_connected) {
                int data_len = strlen(data_to_send);
                // Use conservative MTU if negotiation is still pending
                uint16_t effective_mtu = mtu_negotiation_pending ? 23 : spp_mtu_size;
                int max_payload = effective_mtu - 3; // Account for GATT protocol overhead
                
                //ESP_LOGI(GATTS_TABLE_TAG, "Sending JSON data, length: %d, MTU: %d", data_len, effective_mtu);
                
                if (data_len <= max_payload) {
                    // Data fits in single packet
                    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],
                                                data_len, (uint8_t *)data_to_send, false);
                } else {
                    // // Send first chunk only to avoid stack overflow
                    // ESP_LOGW(GATTS_TABLE_TAG, "Data too large (%d bytes), sending first %d bytes", data_len, max_payload);
                    // esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],
                    //                             max_payload, (uint8_t *)data_to_send, false);
                }
                
                // Note: We don't free data_to_send here since it's the original converted_json_data2 pointer
                // The memory is managed by the caller (display_manager)
            }
        }
    }
    vTaskDelete(NULL);
}
void bluetooth_send_periodic_callback(TimerHandle_t xTimer)
{
    static int counter = 0;
    
    // Send data if connected and data is available - minimal operations only
    if (is_connected && converted_json_data2 != NULL) {
        char *data_ptr = converted_json_data2;
        xQueueSend(data_transmit_queue, &data_ptr, 0); // Non-blocking send
        counter++;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    ESP_LOGI(GATTS_TABLE_TAG, "GAP_EVT, event %d", event);

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "Advertising started successfully");
        }
        break;
    // Note: ESP_GAP_BLE_CONN_PARAMS_UPD_EVT and ESP_GAP_BLE_CONN_PARAMS_UPD_FAIL_EVT 
    // are no longer available in ESP-IDF v5.3. Connection parameter updates are now
    // handled automatically by the stack or through different mechanisms.
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *) param;
    uint8_t res = 0xff;

    //ESP_LOGI(GATTS_TABLE_TAG, "event = %x",event);
    switch (event) {
    	case ESP_GATTS_REG_EVT:
    	    ESP_LOGI(GATTS_TABLE_TAG, "%s %d", __func__, __LINE__);
        	esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);

        	ESP_LOGI(GATTS_TABLE_TAG, "%s %d", __func__, __LINE__);
        	esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, sizeof(spp_adv_data));

        	ESP_LOGI(GATTS_TABLE_TAG, "%s %d", __func__, __LINE__);
        	esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
       	break;
    	case ESP_GATTS_READ_EVT:
            res = find_char_and_desr_index(p_data->read.handle);
            if(res == SPP_IDX_SPP_STATUS_VAL){
                //TODO:client read the status characteristic
            }
       	 break;
    	case ESP_GATTS_WRITE_EVT: {
    	    res = find_char_and_desr_index(p_data->write.handle);
            if(p_data->write.is_prep == false){
                //ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT : handle = %d", res);
                if(res == SPP_IDX_SPP_COMMAND_VAL){
                    // Use the buffer directly instead of malloc
                    memset(spp_cmd_buff, 0x0, sizeof(spp_cmd_buff));
                    memcpy(spp_cmd_buff, p_data->write.value, p_data->write.len);
                    ESP_LOGI(GATTS_TABLE_TAG, "Received Buffer Data: %s", (char *)spp_cmd_buff);
                    
                    // Process the JSON data for command messages
                    parse_ble_data((const char*)spp_cmd_buff);


                }else if(res == SPP_IDX_SPP_DATA_NTF_CFG){
                    if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)){
                        enable_data_ntf = true;
                    }else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)){
                        enable_data_ntf = false;
                        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT : handle = ");
                    }
                }
#ifdef SUPPORT_HEARTBEAT
                else if(res == SPP_IDX_SPP_HEARTBEAT_CFG){
                    if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)){
                        enable_heart_ntf = true;
                    }else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)){
                        enable_heart_ntf = false;
                    }
                }else if(res == SPP_IDX_SPP_HEARTBEAT_VAL){
                    if((p_data->write.len == sizeof(heartbeat_s))&&(memcmp(heartbeat_s,p_data->write.value,sizeof(heartbeat_s)) == 0)){
                        heartbeat_count_num = 0;
                    }
                }
#endif
                else if(res == SPP_IDX_SPP_DATA_RECV_VAL){
                    ESP_LOGI(GATTS_TABLE_TAG, "Single write data received (%d bytes): %.*s", p_data->write.len, p_data->write.len, (char *)p_data->write.value);
                    esp_log_buffer_char(GATTS_TABLE_TAG,(char *)(p_data->write.value),p_data->write.len);
                    
                    // Process the JSON data directly for single writes
                    parse_ble_data((const char*)p_data->write.value);
                }else{
                    //TODO:
                }
            }else if((p_data->write.is_prep == true)&&(res == SPP_IDX_SPP_DATA_RECV_VAL)){
                
                ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_PREP_WRITE_EVT : handle = %d", res);
                store_wr_buffer(p_data);
            }
      	 	break;
    	}
    	case ESP_GATTS_EXEC_WRITE_EVT:{
    	    ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
    	    if(p_data->exec_write.exec_write_flag){
    	        // Process the complete reassembled data
    	        process_complete_data();
    	    }
    	    break;
    	}
    	case ESP_GATTS_MTU_EVT:
    	    spp_mtu_size = p_data->mtu.mtu;
    	    mtu_negotiation_pending = false; // MTU negotiation completed
    	    ESP_LOGI(GATTS_TABLE_TAG, "MTU size updated to: %d", spp_mtu_size);
    	    break;
    	case ESP_GATTS_CONF_EVT:
    	    break;
    	case ESP_GATTS_UNREG_EVT:
        	break;
    	case ESP_GATTS_DELETE_EVT:
        	break;
    	case ESP_GATTS_START_EVT:
        	break;
    	case ESP_GATTS_STOP_EVT:
        	break;
    	case ESP_GATTS_CONNECT_EVT:
    	    spp_conn_id = p_data->connect.conn_id;
    	    spp_gatts_if = gatts_if;
    	    is_connected = true;

            ESP_LOGI(GATTS_TABLE_TAG, "Client connected, current MTU: %d", spp_mtu_size);

            // Initialize MTU negotiation state
            mtu_negotiation_pending = true;

            // Note: MTU exchange is initiated by the client, server responds to it
            // The local MTU is already set to 500 in ble_init()
            ESP_LOGI(GATTS_TABLE_TAG, "Waiting for client MTU exchange request...");

            // Set connection parameters for better stability
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.min_int = 0x10;    // 20ms
            conn_params.max_int = 0x20;    // 40ms  
            conn_params.latency = 0;       // No latency
            conn_params.timeout = 400;     // 4 seconds
            esp_ble_gap_update_conn_params(&conn_params);

            // Create a FreeRTOS timer for periodic data transmission
            TimerHandle_t periodic_timer = xTimerCreate("PeriodicTimer", pdMS_TO_TICKS(500), pdTRUE, (void *)0, bluetooth_send_periodic_callback);

            if (periodic_timer == NULL) {
                ESP_LOGE(GATTS_TABLE_TAG, "Failed to create timer");
            } else {
                // Start the timer with a block time of 0 (start immediately)
                if (xTimerStart(periodic_timer, 0) != pdPASS) {
                    ESP_LOGE(GATTS_TABLE_TAG, "Failed to start timer");
                }
            }

    	    memcpy(&spp_remote_bda,&p_data->connect.remote_bda,sizeof(esp_bd_addr_t));
#ifdef SUPPORT_HEARTBEAT
    	    uint16_t cmd = 0;
            xQueueSend(cmd_heartbeat_queue,&cmd,10/portTICK_PERIOD_MS);
#endif
        	break;
    	case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "Client disconnected, reason: %d", p_data->disconnect.reason);
            spp_mtu_size = 500;  // Reset to requested MTU for next connection
            mtu_negotiation_pending = false; // Reset MTU negotiation state
    	    is_connected = false;
    	    enable_data_ntf = false;
#ifdef SUPPORT_HEARTBEAT
    	    enable_heart_ntf = false;
    	    heartbeat_count_num = 0;
#endif
    	    esp_ble_gap_start_advertising(&spp_adv_params);
    	    break;
    	case ESP_GATTS_OPEN_EVT:
    	    break;
    	case ESP_GATTS_CANCEL_OPEN_EVT:
    	    break;
    	case ESP_GATTS_CLOSE_EVT:
    	    break;
    	case ESP_GATTS_LISTEN_EVT:
    	    break;
    	case ESP_GATTS_CONGEST_EVT:
    	    break;
    	case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
    	    ESP_LOGI(GATTS_TABLE_TAG, "The number handle =%x",param->add_attr_tab.num_handle);
    	    if (param->add_attr_tab.status != ESP_GATT_OK){
    	        ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
    	    }
    	    else if (param->add_attr_tab.num_handle != SPP_IDX_NB){
    	        ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, SPP_IDX_NB);
    	    }
    	    else {
    	        memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
    	        esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
    	    }
    	    break;
    	}
    	default:
    	    break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if; 
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d",param->reg.app_id, param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < SPP_PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == spp_profile_tab[idx].gatts_if) {
                if (spp_profile_tab[idx].gatts_cb) {
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

uint8_t get_rgb_val(uint8_t index) {
    if (index < 4) {
        return rgbBuffer[index];
    }
    return 0;  // Hatalı index
}

uint8_t get_dim_value(int index) {
    return dimsDataBuffer[index];
}

uint16_t get_outputsFromBle() {
    return outputsFromBle;
}

// Function to parse BLE data and call the appropriate parsing function
void parse_ble_data(const char* json_data) {
    cJSON* json = cJSON_Parse(json_data);

    if (json == NULL) {
        //ESP_LOGE(GATTS_TABLE_TAG, "Invalid JSON data");
        return;
    }

    cJSON* messageType = cJSON_GetObjectItem(json, "MessageType");
    if (messageType == NULL || !cJSON_IsString(messageType)) {
        ESP_LOGE(GATTS_TABLE_TAG, "MessageType not found or invalid");
        cJSON_Delete(json);
        return;
    }

    if (strcmp(messageType->valuestring, "Read") == 0) {
        parse_read_data(json);
    } else if (strcmp(messageType->valuestring, "Write") == 0) {
        parse_write_data(json);
    } else if (strcmp(messageType->valuestring, "Configuration") == 0) {
        parse_configuration_data(json);
    } else if (strcmp(messageType->valuestring, "Rules") == 0) {
        parse_rules_data(json);
    } else {
        ESP_LOGE(GATTS_TABLE_TAG, "Unknown MessageType: %s", messageType->valuestring);
    }

    cJSON_Delete(json);
}

// Stub implementations for parsing helper functions
// These should be implemented based on your specific requirements
static void parse_read_data(cJSON* json) {
    ESP_LOGI(GATTS_TABLE_TAG, "parse_read_data called");
    // TODO: Implement read data parsing
}

// Function to parse Write data
void parse_write_data(cJSON* json) {
    cJSON* writeDataType = cJSON_GetObjectItem(json, "writeDataType");
    cJSON* writeNo = cJSON_GetObjectItem(json, "writeNo");
    cJSON* writeData = cJSON_GetObjectItem(json, "writeData");
    uint8_t can_data[8] = {0}; // CAN verisi için buffer

    if (writeDataType && cJSON_IsString(writeDataType)) {
        ESP_LOGI("PARSE_WRITE_DATA", "writeDataType: %s", writeDataType->valuestring);
    }
    if (writeNo && cJSON_IsNumber(writeNo)) {
        ESP_LOGI("PARSE_WRITE_DATA", "writeNo: %d", writeNo->valueint);
    }

    if (writeDataType && cJSON_IsString(writeDataType) && writeNo && cJSON_IsNumber(writeNo)) {
        if (strcmp(writeDataType->valuestring, "Output") == 0) {
            if (writeData && cJSON_IsNumber(writeData)) {
                ESP_LOGI("PARSE_WRITE_DATA", "Output Value: %d", writeData->valueint);

                if ((uint8_t)writeData->valueint) {
                    outputsFromBle |= (1 << (uint8_t)writeNo->valueint);  // set bit at index
                    ESP_LOGI(EXAMPLE_TAG, "Setting output %d to ON simoutput: %d", (uint8_t)writeNo->valueint , outputsFromBle);
                } else {
                    outputsFromBle &= ~(1 << (uint8_t)writeNo->valueint); // clear bit at index
                }
            }
        } else if (strcmp(writeDataType->valuestring, "Dim") == 0) {
            if (writeData && cJSON_IsNumber(writeData)) {
                ESP_LOGI("PARSE_WRITE_DATA", "Dim Value: %d", writeData->valueint);
                dimsDataBuffer[(uint8_t)writeNo->valueint] = (uint8_t)writeData->valueint;
            }
        } else if (strcmp(writeDataType->valuestring, "RGB") == 0) {
            cJSON* rgbArray = cJSON_GetObjectItem(json, "writeData");
            if (rgbArray && cJSON_IsArray(rgbArray) && cJSON_GetArraySize(rgbArray) == 3) {
                rgbBuffer[0] = (uint8_t)cJSON_GetArrayItem(rgbArray, 0)->valueint; // Red
                rgbBuffer[1] = (uint8_t)cJSON_GetArrayItem(rgbArray, 1)->valueint; // Green
                rgbBuffer[2] = (uint8_t)cJSON_GetArrayItem(rgbArray, 2)->valueint; // Blue
                rgbEna = 1; // Enable RGB
                rgbBuffer[3] = rgbEna; // RgbEnable
                //rgbBuffer[3] = (uint8_t)cJSON_GetArrayItem(rgbArray, 3)->valueint; // RgbEnable
                
                ESP_LOGI("PARSE_WRITE_DATA", "RGB Values: R=%d, G=%d, B=%d rgbEna = %d", rgbBuffer[0], rgbBuffer[1], rgbBuffer[2], rgbEna);

            } else {
                ESP_LOGE("PARSE_WRITE_DATA", "RGB writeData must be an array of 4 values.");
            }
        } else {
            ESP_LOGI("PARSE_WRITE_DATA", "Unknown writeDataType: %s", writeDataType->valuestring);
        }
    }
}
// Function to parse Configuration data
void parse_configuration_data(cJSON* json) {
    cJSON* numOfOutputs_json = cJSON_GetObjectItem(json, "numOfOutputs");
    cJSON* outputsNameBuffer = cJSON_GetObjectItem(json, "OutputsNameBuffer");
    cJSON* numOfDims_json = cJSON_GetObjectItem(json, "numOfDims");
    cJSON* dimsNameBuffer = cJSON_GetObjectItem(json, "DimsNameBuffer");
    cJSON* numOfSensors_json = cJSON_GetObjectItem(json, "numOfSensors");
    cJSON* sensorsNameBuffer = cJSON_GetObjectItem(json, "SensorsNameBuffer");
    cJSON* rgbEnabled = cJSON_GetObjectItem(json, "RGBEnabled");
    cJSON* theme = cJSON_GetObjectItem(json, "Theme");

    int outputsBuf[16] = {0};
    int dimsBuf[4] = {0};
    int sensorsBuf[5] = {0};

    // Extract values and update global variables
    if (numOfOutputs_json && cJSON_IsNumber(numOfOutputs_json)) {
        numOfOutputs = numOfOutputs_json->valueint;
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "numOfOutputs: %d", numOfOutputs);
    }
    if (outputsNameBuffer && cJSON_IsArray(outputsNameBuffer)) {
        int size = cJSON_GetArraySize(outputsNameBuffer);
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "OutputsNameBuffer:");
        for (int i = 0; i < size && i < 16; i++) {
            cJSON* item = cJSON_GetArrayItem(outputsNameBuffer, i);
            if (cJSON_IsNumber(item)) {
                outputsBuf[i] = item->valueint;
                ESP_LOGI("PARSE_CONFIGURATION_DATA", "  %d", item->valueint);
            }
        }
    }
    if (numOfDims_json && cJSON_IsNumber(numOfDims_json)) {
        numOfDims = numOfDims_json->valueint;
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "numOfDims: %d", numOfDims);
    }
    if (dimsNameBuffer && cJSON_IsArray(dimsNameBuffer)) {
        int size = cJSON_GetArraySize(dimsNameBuffer);
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "DimsNameBuffer:");
        for (int i = 0; i < size && i < 4; i++) {
            cJSON* item = cJSON_GetArrayItem(dimsNameBuffer, i);
            if (cJSON_IsNumber(item)) {
                dimsBuf[i] = item->valueint;
                ESP_LOGI("PARSE_CONFIGURATION_DATA", "  %d", item->valueint);
            }
        }
    }
    if (numOfSensors_json && cJSON_IsNumber(numOfSensors_json)) {
        numOfSensors = numOfSensors_json->valueint;
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "numOfSensors: %d", numOfSensors);
    }
    if (sensorsNameBuffer && cJSON_IsArray(sensorsNameBuffer)) {
        int size = cJSON_GetArraySize(sensorsNameBuffer);
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "SensorsNameBuffer:");
        for (int i = 0; i < size && i < 5; i++) {
            cJSON* item = cJSON_GetArrayItem(sensorsNameBuffer, i);
            if (cJSON_IsNumber(item)) {
                sensorsBuf[i] = item->valueint;
                ESP_LOGI("PARSE_CONFIGURATION_DATA", "  %d", item->valueint);
            }
        }
    }
    if (rgbEnabled && cJSON_IsString(rgbEnabled)) {
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "RGBEnables: %s", rgbEnabled->valuestring);
    }
    if (theme && cJSON_IsString(theme)) {
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "Theme: %s", theme->valuestring);
    }
    
    // Save to NVS using the global variables
    save_panel_configuration_to_nvs(numOfOutputs, outputsBuf, numOfSensors, sensorsBuf, numOfDims, dimsBuf);
    
    ESP_LOGI("PARSE_CONFIGURATION_DATA", "Updated globals - numOfOutputs: %d, numOfSensors: %d, numOfDims: %d", 
             numOfOutputs, numOfSensors, numOfDims);

    // Copy buffers to global buffers
    for (int i = 0; i < numOfOutputs && i < 16; i++)
    {
        outputsBuffer[i] = outputsBuf[i];
    }
    for (int i = 0; i < numOfDims && i < 4; i++)
    {
        dimsBuffer[i] = dimsBuf[i];
    }
    for (int i = 0; i < numOfSensors && i < 5; i++)
    {
        sensorsBuffer[i] = sensorsBuf[i];
    }
    
}

static void parse_rules_data(cJSON* json) {
    ESP_LOGI(GATTS_TABLE_TAG, "parse_rules_data called");
    // TODO: Implement rules data parsing
}




// Example function to create JSON data packet as a C string
char* create_json_data_packet(const uint16_t* regs_data, int numOfOutputs, int numOfDims, int numOfSensors, bool slaveConnectionStatus, int themeType, int numberOfNotifications, cJSON* notifications) {
    // Create a JSON object
    cJSON *json = cJSON_CreateObject();

    // Add number of outputs, dims, sensors, slave connection status, and theme type to the JSON object
    cJSON_AddStringToObject(json, "slvConn", "Yes");
    cJSON_AddNumberToObject(json, "numOfO", numOfOutputs);
    cJSON_AddNumberToObject(json, "numOfD", numOfDims);
    cJSON_AddNumberToObject(json, "numOfS", numOfSensors);
    cJSON_AddStringToObject(json, "RGBE", "yes");
    cJSON_AddNumberToObject(json, "Th", themeType);
    cJSON_AddNumberToObject(json, "volt", batarya_volt);


    // Add outputsBuffer to the JSON object
    cJSON *outputnames = cJSON_CreateIntArray(outputsBuffer, numOfOutputs);
    cJSON_AddItemToObject(json, "outNB", outputnames);

    // Add dimsBuffer to the JSON object
    cJSON *dimnames = cJSON_CreateIntArray(dimsBuffer, numOfDims);
    cJSON_AddItemToObject(json, "dimNB", dimnames);

    // Add sensorsBuffer to the JSON object
    cJSON *sensornames = cJSON_CreateIntArray(sensorsBuffer, 5);
    cJSON_AddItemToObject(json, "senNB", sensornames);


    // Fetch outputsBuffer from regs_data
    int buf[16];
    for (int i = 0; i < numOfOutputs; i++) {
#if BLE_ENB 
        buf[i] = get_outputsFromBle() >> i & 0x01; // Get the output state from the bitmask
#else
        buf[i] = get_outputs() >> i & 0x01; // Get the output state from the bitmask
#endif
    }
    cJSON *outputs = cJSON_CreateIntArray(buf, numOfOutputs);
    cJSON_AddItemToObject(json, "outDB", outputs);

    // Fetch dimsBuffer from regs_data
    for (int i = 0; i < numOfDims; i++) {
        buf[i] = get_dimmable_output(i);
    }
    cJSON *dims = cJSON_CreateIntArray(buf, numOfDims);
    cJSON_AddItemToObject(json, "dDB", dims);


    for (int i = 0; i < numOfSensors; i++) {
        buf[i] = get_analog_input(i);
    }
    cJSON *sensors = cJSON_CreateIntArray(buf, numOfSensors);
    cJSON_AddItemToObject(json, "sDB", sensors);


    rgbBuffer[0] = get_rgb_val(0);
    rgbBuffer[1] = get_rgb_val(1);
    rgbBuffer[2] = get_rgb_val(2);
    rgbBuffer[3] = get_rgb_val(3);
    //ESP_LOGI(TAG, "RGB Values: %d, %d, %d, %d", get_rgb_value(0), get_rgb_value(1), get_rgb_value(2), get_rgb_value(3));
    // Add rgbBuffer to the JSON object
    cJSON *rgb = cJSON_CreateIntArray(rgbBuffer, 4);
    cJSON_AddItemToObject(json, "RGBDB", rgb);
    


    // Convert JSON object to string
    char *json_str = cJSON_PrintUnformatted(json);
    //ESP_LOGI("JSON_DATA_PACKET", "%s", json_str);

    // Free the JSON object
    cJSON_Delete(json);

    return json_str; // Caller is responsible for freeing the returned string
}


void get_data_json_format(const uint16_t* regs_data, int txPacketType, char** json_str)  {

    // Call create_json_data_packet function
    *json_str = create_json_data_packet(regs_data, numOfOutputs, numOfDims, numOfSensors, slaveConnectionStatus, panelThemeType, numberOfNotifications, notifications);
    // Check if notifications is not NULL before deleting
    if (notifications != NULL) {
        cJSON_Delete(notifications);
        notifications = NULL; // Set to NULL after deletion to avoid dangling pointer
    }
} 


// Function to read an integer from NVS
esp_err_t nvs_read_int(const char* key, int* value) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    int32_t temp_value;

    // Open NVS handle
    err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS_WRITE_PARAMS", "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    // Read integer from NVS
    err = nvs_get_i32(nvs_handle, key, &temp_value);
    if (err == ESP_OK) {
        *value = (int)temp_value;
    }
    nvs_close(nvs_handle);
    return err;
}

// Function to write an integer to NVS
esp_err_t nvs_write_int(const char* key, int value) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Open NVS handle
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS_WRITE_PARAMS", "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    // Write integer to NVS
    err = nvs_set_i32(nvs_handle, key, value);
    if (err != ESP_OK) {
        ESP_LOGE("NVS_WRITE_PARAMS", "Error writing integer to NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    // Commit written value
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    return err;
}

// Function to read configuration data from NVS
void load_panel_configuration_from_nvs(int *totalOutpts, int buffer1[16], int *totalSensors, int buffer2[5], int *totalDims, int buffer3[4]) {
    int32_t value;
    
    // Read totalOutpts from NVS
    if (nvs_read_int("numOfOutputs", &value) != ESP_OK || value < 0 || value > 16) {
        *totalOutpts = 4; // Set to default value if out of range
        ESP_LOGE("NVS_READ_PARAMS", "Error reading Total Outputs from NVS: %d", *totalOutpts);  
        save_panel_configuration_to_nvs(*totalOutpts, buffer1, *totalSensors, buffer2, *totalDims, buffer3);
    } else {
        *totalOutpts = (int)value;
    }

    // Read buffer1 from NVS
    for (int i = 0; i < 16; i++) {
        char key[16];
        snprintf(key, sizeof(key), "outBuf%d", i);
        if (nvs_read_int(key, &value) != ESP_OK || value < 1 || value > 18) {
            buffer1[i] = 1; // Set to default value if out of range
        } else {
            buffer1[i] = (int)value;
        }
    }

    // Read totalSensors from NVS
    if (nvs_read_int("numSens", &value) != ESP_OK || value < 0 || value > 5) {
        *totalSensors = 1; // Set to default value if out of range
        ESP_LOGE("NVS_READ_PARAMS", "Error reading Total Sensors from NVS: %d", *totalSensors);
        save_panel_configuration_to_nvs(*totalOutpts, buffer1, *totalSensors, buffer2, *totalDims, buffer3);
    } else {
        *totalSensors = (int)value;
    }

    // Read buffer2 from NVS
    for (int i = 0; i < 5; i++) {
        char key[16];
        snprintf(key, sizeof(key), "sensBuf%d", i);
        if (nvs_read_int(key, &value) != ESP_OK || value < 0 || value > 1) {
            buffer2[i] = 1; // Set to default value if out of range
        } else {
            buffer2[i] = (int)value;
        }
    }

    // Read totalDims from NVS
    if (nvs_read_int("numDims", &value) != ESP_OK || value < 0 || value > 4) {
        *totalDims = 1; // Set to default value if out of range
        ESP_LOGE("NVS_READ_PARAMS", "Error reading Total Dims from NVS: %d", *totalDims);
        save_panel_configuration_to_nvs(*totalOutpts, buffer1, *totalSensors, buffer2, *totalDims, buffer3);
    } else {
        *totalDims = (int)value;
    }

    // Read buffer3 from NVS
    for (int i = 0; i < 4; i++) {
        char key[16];
        snprintf(key, sizeof(key), "dimsBuf%d", i);
        if (nvs_read_int(key, &value) != ESP_OK || value < 1 || value > 8) {
            buffer3[i] = 1; // Set to default value if out of range
        } else {
            buffer3[i] = (int)value;
        }
    }
}

// Load the theme settings from NVS
void load_theme_configuration_from_nvs(int* themeType, int* wallpaperEnabled, int* wallpaperTimeIndex) {
    int32_t value;

    // Read themeType from NVS
    if (nvs_read_int("thmTyp", &value) != ESP_OK || value < 0 || value > 1) {
        *themeType = 1; // Set to default value if out of range
    } else {
        *themeType = (int)value;
    }

    // Read wallpaperEnabled from NVS
    if (nvs_read_int("wallpEn", &value) != ESP_OK || value < 0 || value > 1) {
        *wallpaperEnabled = 1; // Set to default value if out of range
    } else {
        *wallpaperEnabled = (int)value;
    }

    // Read wallpaperTimeIndex from NVS
    if (nvs_read_int("wllTimI", &value) != ESP_OK || value < 0 || value > 600) {
        *wallpaperTimeIndex = 30; // Set to default value if out of range
    } else {
        *wallpaperTimeIndex = (int)value;
    }

    // Log the loaded configuration
    ESP_LOGI(GATTS_TABLE_TAG, "Loaded Theme Configuration: Theme=%d, WallpaperEnabled=%d, WallpaperTimeIndex=%d",
             *themeType, *wallpaperEnabled, *wallpaperTimeIndex);
}



void save_panel_configuration_to_nvs(int totalOutps, int buffer1[16], int totalSensors, int buffer2[5], int totalDims, int buffer3[4]) {
    // Ensure the values do not exceed the maximum allowed sizes
    if (totalOutps > 16) {
        totalOutps = 16;
    }
    if (totalSensors > 5) {
        totalSensors = 5;
    }
    if (totalDims > 4) {
        totalDims = 4;
    }

    // Save totalOutps to NVS
    nvs_write_int("numOfOutputs", totalOutps);
    ESP_LOGW("NVS_WRITE_PARAMS", "Total Outputs to save: %d", totalOutps);

    // Save buffer1 to NVS
    for (int i = 0; i < 16; i++) {
        if (buffer1[i] < 1 || buffer1[i] > 18) {
            buffer1[i] = 1; // Set to default value if out of range
        }
        char key[16];
        snprintf(key, sizeof(key), "outBuf%d", i);
        nvs_write_int(key, buffer1[i]);
    }

    // Save totalSensors to NVS
    nvs_write_int("numSens", totalSensors);
    ESP_LOGW("NVS_WRITE_PARAMS", "Total Sensors to save: %d", totalSensors);

    // Save buffer2 to NVS
    for (int i = 0; i < 5; i++) {
        if (buffer2[i] < 0 || buffer2[i] > 1) {
            buffer2[i] = 0; // Set to default value if out of range
        }
        char key[16];
        snprintf(key, sizeof(key), "sensBuf%d", i);
        nvs_write_int(key, buffer2[i]);
    }

    // Save totalDims to NVS
    nvs_write_int("numDims", totalDims);
    ESP_LOGW("NVS_WRITE_PARAMS", "Total Dims to save: %d", totalDims);

    // Save buffer3 to NVS
    for (int i = 0; i < 4; i++) {
        if (buffer3[i] < 0 || buffer3[i] > 8) {
            buffer3[i] = 0; // Set to default value if out of range
        }
        char key[16];
        snprintf(key, sizeof(key), "dimsBuf%d", i);
        nvs_write_int(key, buffer3[i]);        
    }
    ESP_LOGI("NVS_WRITE_PARAMS", "##### NVS Parameters Saved Successfully! #####");
}

void save_theme_configuration_to_nvs(int16_t* themeType, uint16_t* wallpaperEnabled, uint16_t* wallpaperTimeIndex){


    //Save themeType to NVS
    nvs_write_int("thmTyp", panelThemeType);

    //Save wallpaperEnabled to NVS
    nvs_write_int("wallpEn", panelWallpaperEnable);

    //Save wallpaperTimeIndex to NVS
    nvs_write_int("wllTimI", panelWallpaperTime);


}




void ble_init(void)
{
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();



    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    load_panel_configuration_from_nvs(&numOfOutputs, outputsBuffer, &numOfSensors, sensorsBuffer, &numOfDims, dimsBuffer);
    load_theme_configuration_from_nvs(&panelThemeType, &panelWallpaperEnable, &panelWallpaperTime);


    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth", __func__);

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);

    // Create queues
    cmd_cmd_queue = xQueueCreate(10, sizeof(uint16_t));
    data_transmit_queue = xQueueCreate(5, sizeof(char*));
    
    if (cmd_cmd_queue == NULL || data_transmit_queue == NULL) {
        ESP_LOGE(GATTS_TABLE_TAG, "Failed to create queues");
        return;
    }

    // Create data transmission task
    xTaskCreate(data_transmit_task, "data_transmit", 4096, NULL, 5, NULL);

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    return;
}

// Function to get the connection status
bool get_connection_status() {
    return is_connected;
}


// Function to set the global variable converted_json_data2
void set_converted_json_data(char* converted_json_data) {
    if (converted_json_data2 != NULL) {
        free(converted_json_data2);
    }
    converted_json_data2 = converted_json_data;
    ESP_LOGI(GATTS_TABLE_TAG, "Converted JSON Data: %s", converted_json_data2);
}

// Function to get the spp_cmd_buff
uint8_t* get_spp_cmd_buff() {
    return spp_cmd_buff;
}

// Function to reset the spp_cmd_buff
void reset_spp_cmd_buff() {
    memset(spp_cmd_buff, 0x0, sizeof(spp_cmd_buff));
}
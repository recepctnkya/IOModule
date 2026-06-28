#include "hexnet_io_profile.h"

#include <string.h>

#include "hexnet_canbus.h"
#define LOG_LOCAL_LEVEL HEXNET_LOG_LEVEL_OTHER
#include "hexnet_log.h"
#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "HEXNET_IO_PROFILE";

#define NVS_NS "hexnet"
#define NVS_KEY "io_profile"
#define PROFILE_MAGIC 0x314F4948u /* "HIO1" */
#define PROFILE_VERSION 1

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t version;
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
    uint8_t reserved;
} hexnet_io_profile_blob_t;

static hexnet_io_profile_t s_profile;
static bool s_loaded;

extern int numOfOutputs;
extern int numOfDims;
extern int numOfSensors;
extern int outputsBuffer[16];
extern int sensorsBuffer[5];
extern int dimsBuffer[4];

void load_panel_configuration_from_nvs(
    int *totalOutpts,
    int buffer1[16],
    int *totalSensors,
    int buffer2[5],
    int *totalDims,
    int buffer3[4]);
void save_panel_configuration_to_nvs(
    int totalOutps,
    int buffer1[16],
    int totalSensors,
    int buffer2[5],
    int totalDims,
    int buffer3[4]);

static uint8_t count_bits8(uint8_t v)
{
    uint8_t n = 0;
    while (v) {
        n += (uint8_t)(v & 1u);
        v >>= 1;
    }
    return n;
}

static uint8_t count_bits16(uint16_t v)
{
    uint8_t n = 0;
    while (v) {
        n += (uint8_t)(v & 1u);
        v >>= 1;
    }
    return n;
}

static void profile_set_defaults(hexnet_io_profile_t *p)
{
    memset(p, 0, sizeof(*p));
    p->num_relays = 4;
    p->num_dims = 4;
    p->num_sensors = 5;
    p->num_tanks = 3;
    p->motor_enabled = 1;
    p->rgb_enabled = 1;
    p->relay_enable_mask = 0x000Fu;
    p->dim_enable_mask = 0x0Fu;
    p->sensor_enable_mask = 0x1Fu;
    p->tank_enable_mask = 0x07u;
    for (int i = 0; i < HEXNET_IO_RELAY_SLOTS; ++i) {
        p->relay_type[i] = (uint8_t)((i < 4) ? 1 : 0);
    }
    for (int i = 0; i < HEXNET_IO_DIM_SLOTS; ++i) {
        p->dim_type[i] = 1;
    }
    for (int i = 0; i < HEXNET_IO_SENSOR_SLOTS; ++i) {
        p->sensor_type[i] = 1;
    }
}

static void profile_recompute_counts(hexnet_io_profile_t *p)
{
    p->num_relays = count_bits16(p->relay_enable_mask);
    p->num_dims = count_bits8(p->dim_enable_mask);
    p->num_sensors = count_bits8(p->sensor_enable_mask);
    p->num_tanks = count_bits8(p->tank_enable_mask);
    if (p->num_relays == 0) {
        p->num_relays = 1;
        p->relay_enable_mask |= 1u;
    }
}

static void blob_from_profile(const hexnet_io_profile_t *p, hexnet_io_profile_blob_t *b)
{
    memset(b, 0, sizeof(*b));
    b->magic = PROFILE_MAGIC;
    b->version = PROFILE_VERSION;
    b->num_relays = p->num_relays;
    b->num_dims = p->num_dims;
    b->num_sensors = p->num_sensors;
    b->num_tanks = p->num_tanks;
    b->motor_enabled = p->motor_enabled ? 1 : 0;
    b->rgb_enabled = p->rgb_enabled ? 1 : 0;
    b->relay_enable_mask = p->relay_enable_mask;
    b->dim_enable_mask = p->dim_enable_mask;
    b->sensor_enable_mask = p->sensor_enable_mask;
    b->tank_enable_mask = p->tank_enable_mask;
    memcpy(b->relay_type, p->relay_type, sizeof(b->relay_type));
    memcpy(b->dim_type, p->dim_type, sizeof(b->dim_type));
    memcpy(b->sensor_type, p->sensor_type, sizeof(b->sensor_type));
    b->link_flags = p->link_flags;
}

static void profile_from_blob(const hexnet_io_profile_blob_t *b, hexnet_io_profile_t *p)
{
    memset(p, 0, sizeof(*p));
    p->num_relays = b->num_relays;
    p->num_dims = b->num_dims;
    p->num_sensors = b->num_sensors;
    p->num_tanks = b->num_tanks;
    p->motor_enabled = b->motor_enabled;
    p->rgb_enabled = b->rgb_enabled;
    p->relay_enable_mask = b->relay_enable_mask;
    p->dim_enable_mask = b->dim_enable_mask;
    p->sensor_enable_mask = b->sensor_enable_mask;
    p->tank_enable_mask = b->tank_enable_mask;
    memcpy(p->relay_type, b->relay_type, sizeof(p->relay_type));
    memcpy(p->dim_type, b->dim_type, sizeof(p->dim_type));
    memcpy(p->sensor_type, b->sensor_type, sizeof(p->sensor_type));
    p->link_flags = b->link_flags;
    profile_recompute_counts(p);
}

static void profile_from_legacy_panel(void)
{
    profile_set_defaults(&s_profile);

    int out_cnt = 0;
    int sens_cnt = 0;
    int dim_cnt = 0;

    if (out_cnt > 0 && out_cnt <= HEXNET_IO_RELAY_SLOTS) {
        s_profile.relay_enable_mask = (uint16_t)((1u << out_cnt) - 1u);
        s_profile.num_relays = (uint8_t)out_cnt;
    }
    for (int i = 0; i < HEXNET_IO_RELAY_SLOTS; ++i) {
        if (outputsBuffer[i] >= 1 && outputsBuffer[i] <= 18) {
            s_profile.relay_type[i] = (uint8_t)outputsBuffer[i];
            s_profile.relay_enable_mask |= (uint16_t)(1u << i);
        } else if (outputsBuffer[i] == 0) {
            s_profile.relay_enable_mask &= (uint16_t)~(1u << i);
        }
    }

    if (dim_cnt > 0 && dim_cnt <= HEXNET_IO_DIM_SLOTS) {
        s_profile.dim_enable_mask = (uint8_t)((1u << dim_cnt) - 1u);
    }
    for (int i = 0; i < HEXNET_IO_DIM_SLOTS; ++i) {
        if (dimsBuffer[i] >= 1 && dimsBuffer[i] <= 8) {
            s_profile.dim_type[i] = (uint8_t)dimsBuffer[i];
            s_profile.dim_enable_mask |= (uint8_t)(1u << i);
        } else {
            s_profile.dim_enable_mask &= (uint8_t)~(1u << i);
        }
    }

    if (sens_cnt > 0 && sens_cnt <= HEXNET_IO_SENSOR_SLOTS) {
        s_profile.sensor_enable_mask = (uint8_t)((1u << sens_cnt) - 1u);
    }
    s_profile.tank_enable_mask = 0;
    for (int i = 0; i < HEXNET_IO_SENSOR_SLOTS; ++i) {
        if (sensorsBuffer[i]) {
            s_profile.sensor_enable_mask |= (uint8_t)(1u << i);
            s_profile.sensor_type[i] = (uint8_t)sensorsBuffer[i];
            if (i >= 2) {
                s_profile.tank_enable_mask |= (uint8_t)(1u << (i - 2));
            }
        } else {
            s_profile.sensor_enable_mask &= (uint8_t)~(1u << i);
        }
    }

    s_profile.motor_enabled = 1;
    s_profile.rgb_enabled = (get_rgb_enable() != 0) ? 1 : 1;
    profile_recompute_counts(&s_profile);
}

static esp_err_t profile_load_blob(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return err;
    }

    hexnet_io_profile_blob_t blob;
    size_t len = sizeof(blob);
    err = nvs_get_blob(handle, NVS_KEY, &blob, &len);
    nvs_close(handle);
    if (err != ESP_OK || len != sizeof(blob) || blob.magic != PROFILE_MAGIC || blob.version != PROFILE_VERSION) {
        return ESP_ERR_NOT_FOUND;
    }

    profile_from_blob(&blob, &s_profile);

    // load_panel_configuration_from_nvs(&out_cnt, outputsBuffer, &sens_cnt, sensorsBuffer, &dim_cnt, dimsBuffer);
    // printf("###########################Legacy panel config loaded from NVS: out_cnt=%d sens_cnt=%d dim_cnt=%d\n", out_cnt, sens_cnt, dim_cnt);

    return ESP_OK;
}

void hexnet_io_profile_sync_legacy_globals(void)
{
    numOfOutputs = s_profile.num_relays;
    numOfDims = s_profile.num_dims;
    numOfSensors = s_profile.num_sensors;

    for (int i = 0; i < HEXNET_IO_RELAY_SLOTS; ++i) {
        outputsBuffer[i] = (s_profile.relay_enable_mask & (1u << i)) ? (int)s_profile.relay_type[i] : 0;
        if (outputsBuffer[i] == 0 && (s_profile.relay_enable_mask & (1u << i))) {
            outputsBuffer[i] = 1;
        }
    }
    for (int i = 0; i < HEXNET_IO_DIM_SLOTS; ++i) {
        dimsBuffer[i] = (s_profile.dim_enable_mask & (1u << i)) ? (int)s_profile.dim_type[i] : 0;
        if (dimsBuffer[i] == 0 && (s_profile.dim_enable_mask & (1u << i))) {
            dimsBuffer[i] = 1;
        }
    }
    for (int i = 0; i < HEXNET_IO_SENSOR_SLOTS; ++i) {
        sensorsBuffer[i] = (s_profile.sensor_enable_mask & (1u << i)) ? (int)s_profile.sensor_type[i] : 0;
        if (sensorsBuffer[i] == 0 && (s_profile.sensor_enable_mask & (1u << i))) {
            sensorsBuffer[i] = 1;
        }
    }

    save_panel_configuration_to_nvs(numOfOutputs, outputsBuffer, numOfSensors, sensorsBuffer, numOfDims, dimsBuffer);
}

static void profile_apply_hw_enable_flags(void)
{
    if (!s_profile.rgb_enabled) {
        set_rgb_values(get_r_value(), get_g_value(), get_b_value(), 0);
    }
}

void hexnet_io_profile_init(void)
{
    if (profile_load_blob() == ESP_OK) {
        printf("Profil NVS yuklendi: R=%u D=%u S=%u tank=%u motor=%u rgb=%u link=0x%02X\n",
                 (unsigned)s_profile.num_relays,
                 (unsigned)s_profile.num_dims,
                 (unsigned)s_profile.num_sensors,
                 (unsigned)s_profile.num_tanks,
                 (unsigned)s_profile.motor_enabled,
                 (unsigned)s_profile.rgb_enabled,
                 (unsigned)s_profile.link_flags);
    } else {
        printf("Profil blob yok; legacy NVS'ten migrate\n");
        profile_from_legacy_panel();
        (void)hexnet_io_profile_save();
    }

    hexnet_io_profile_sync_legacy_globals();
    profile_apply_hw_enable_flags();
    s_loaded = true;
}

const hexnet_io_profile_t *hexnet_io_profile_get(void)
{
    return &s_profile;
}

esp_err_t hexnet_io_profile_save(void)
{
    hexnet_io_profile_blob_t blob;
    blob_from_profile(&s_profile, &blob);

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(handle, NVS_KEY, &blob, sizeof(blob));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err == ESP_OK) {
        hexnet_io_profile_sync_legacy_globals();
        profile_apply_hw_enable_flags();
        ESP_LOGI(TAG, "Profil flash'a yazildi");
    }
    return err;
}

static int clamp_panel_count(int value, int max_value)
{
    if (value < 0) {
        return 0;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

esp_err_t hexnet_io_profile_apply_panel_config(
    int totalOutps,
    const uint8_t output_types[HEXNET_IO_RELAY_SLOTS],
    int totalSensors,
    const uint8_t sensor_types[HEXNET_IO_SENSOR_SLOTS],
    int totalDims,
    const uint8_t dim_types[HEXNET_IO_DIM_SLOTS],
    uint8_t link_flag)
{
    if (!s_loaded) {
        return ESP_ERR_INVALID_STATE;
    }

    const int relay_count = clamp_panel_count(totalOutps, HEXNET_IO_RELAY_SLOTS);
    const int sensor_count = clamp_panel_count(totalSensors, HEXNET_IO_SENSOR_SLOTS);
    const int dim_count = clamp_panel_count(totalDims, HEXNET_IO_DIM_SLOTS);

    s_profile.relay_enable_mask = (uint16_t)((relay_count > 0) ? ((1u << relay_count) - 1u) : 0);
    for (int i = 0; i < HEXNET_IO_RELAY_SLOTS; ++i) {
        if (i < relay_count) {
            uint8_t type = output_types ? output_types[i] : 1;
            s_profile.relay_type[i] = (type >= 1 && type <= 18) ? type : 1;
        } else {
            s_profile.relay_type[i] = 0;
        }
    }

    s_profile.sensor_enable_mask = (uint8_t)((sensor_count > 0) ? ((1u << sensor_count) - 1u) : 0);
    s_profile.tank_enable_mask = 0;
    for (int i = 0; i < HEXNET_IO_SENSOR_SLOTS; ++i) {
        if (i < sensor_count) {
            uint8_t type = sensor_types ? sensor_types[i] : 1;
            s_profile.sensor_type[i] = (type == 1) ? type : 1;
            if (i >= 2 && (i - 2) < HEXNET_IO_TANK_SLOTS) {
                s_profile.tank_enable_mask |= (uint8_t)(1u << (i - 2));
            }
        } else {
            s_profile.sensor_type[i] = 0;
        }
    }

    s_profile.dim_enable_mask = (uint8_t)((dim_count > 0) ? ((1u << dim_count) - 1u) : 0);
    for (int i = 0; i < HEXNET_IO_DIM_SLOTS; ++i) {
        if (i < dim_count) {
            uint8_t type = dim_types ? dim_types[i] : 1;
            s_profile.dim_type[i] = (type >= 1 && type <= 8) ? type : 1;
        } else {
            s_profile.dim_type[i] = 0;
        }
    }

    if (link_flag) {
        s_profile.link_flags |= link_flag;
    }

    profile_recompute_counts(&s_profile);
    return hexnet_io_profile_save();
}

static int json_bool(const cJSON *item, int default_val)
{
    if (!item) {
        return default_val;
    }
    if (cJSON_IsBool(item)) {
        return cJSON_IsTrue(item) ? 1 : 0;
    }
    if (cJSON_IsNumber(item)) {
        return item->valueint ? 1 : 0;
    }
    if (cJSON_IsString(item) && item->valuestring) {
        if (strcmp(item->valuestring, "yes") == 0 || strcmp(item->valuestring, "true") == 0 ||
            strcmp(item->valuestring, "1") == 0) {
            return 1;
        }
    }
    return default_val;
}

static void apply_enable_mask_u16(const cJSON *arr, uint16_t *mask, uint8_t *types, int max_slots)
{
    if (!cJSON_IsArray(arr)) {
        return;
    }
    const int n = cJSON_GetArraySize(arr);
    for (int i = 0; i < n && i < max_slots; ++i) {
        const cJSON *item = cJSON_GetArrayItem(arr, i);
        int active = 1;
        int type_code = 1;
        if (cJSON_IsNumber(item)) {
            type_code = item->valueint;
            active = (type_code != 0) ? 1 : 0;
        } else if (cJSON_IsObject(item)) {
            const cJSON *en = cJSON_GetObjectItem(item, "enabled");
            const cJSON *act = cJSON_GetObjectItem(item, "active");
            const cJSON *tp = cJSON_GetObjectItem(item, "type");
            active = json_bool(en, json_bool(act, 1));
            if (cJSON_IsNumber(tp)) {
                type_code = tp->valueint;
            }
        }
        if (active) {
            *mask |= (uint16_t)(1u << i);
            if (type_code > 0) {
                types[i] = (uint8_t)type_code;
            }
        } else {
            *mask &= (uint16_t)~(1u << i);
            types[i] = 0;
        }
    }
}

static void apply_enable_mask_u8(const cJSON *arr, uint8_t *mask, uint8_t *types, int max_slots)
{
    if (!cJSON_IsArray(arr)) {
        return;
    }
    const int n = cJSON_GetArraySize(arr);
    for (int i = 0; i < n && i < max_slots; ++i) {
        const cJSON *item = cJSON_GetArrayItem(arr, i);
        int active = 1;
        int type_code = 1;
        if (cJSON_IsNumber(item)) {
            type_code = item->valueint;
            active = (type_code != 0) ? 1 : 0;
        } else if (cJSON_IsObject(item)) {
            const cJSON *en = cJSON_GetObjectItem(item, "enabled");
            const cJSON *act = cJSON_GetObjectItem(item, "active");
            const cJSON *tp = cJSON_GetObjectItem(item, "type");
            active = json_bool(en, json_bool(act, 1));
            if (cJSON_IsNumber(tp)) {
                type_code = tp->valueint;
            }
        }
        if (active) {
            *mask |= (uint8_t)(1u << i);
            if (type_code > 0) {
                types[i] = (uint8_t)type_code;
            }
        } else {
            *mask &= (uint8_t)~(1u << i);
            types[i] = 0;
        }
    }
}

esp_err_t hexnet_io_profile_apply_json(const cJSON *json, uint8_t link_flag)
{
    if (!json || !s_loaded) {
        return ESP_ERR_INVALID_ARG;
    }

    const cJSON *nested = cJSON_GetObjectItem(json, "io_profile");
    if (cJSON_IsObject(nested)) {
        (void)hexnet_io_profile_apply_json(nested, 0);
    }

    const cJSON *num_relays = cJSON_GetObjectItem(json, "numOfOutputs");
    const cJSON *num_dims = cJSON_GetObjectItem(json, "numOfDims");
    const cJSON *num_sensors = cJSON_GetObjectItem(json, "numOfSensors");
    const cJSON *num_tanks = cJSON_GetObjectItem(json, "numOfTanks");
    const cJSON *motor_en = cJSON_GetObjectItem(json, "motorEnabled");
    const cJSON *rgb_en = cJSON_GetObjectItem(json, "RGBEnabled");
    const cJSON *rgb_en2 = cJSON_GetObjectItem(json, "rgbEnabled");
    const cJSON *relay_mask = cJSON_GetObjectItem(json, "relayEnableMask");
    const cJSON *dim_mask = cJSON_GetObjectItem(json, "dimEnableMask");
    const cJSON *sensor_mask = cJSON_GetObjectItem(json, "sensorEnableMask");
    const cJSON *tank_mask = cJSON_GetObjectItem(json, "tankEnableMask");

    if (cJSON_IsNumber(num_relays) && num_relays->valueint >= 0 && num_relays->valueint <= HEXNET_IO_RELAY_SLOTS) {
        const int n = num_relays->valueint;
        s_profile.relay_enable_mask = (uint16_t)((n > 0) ? ((1u << n) - 1u) : 0);
    }
    if (cJSON_IsNumber(num_dims) && num_dims->valueint >= 0 && num_dims->valueint <= HEXNET_IO_DIM_SLOTS) {
        const int n = num_dims->valueint;
        s_profile.dim_enable_mask = (uint8_t)((n > 0) ? ((1u << n) - 1u) : 0);
    }
    if (cJSON_IsNumber(num_sensors) && num_sensors->valueint >= 0 && num_sensors->valueint <= HEXNET_IO_SENSOR_SLOTS) {
        const int n = num_sensors->valueint;
        s_profile.sensor_enable_mask = (uint8_t)((n > 0) ? ((1u << n) - 1u) : 0);
    }
    if (cJSON_IsNumber(num_tanks) && num_tanks->valueint >= 0 && num_tanks->valueint <= HEXNET_IO_TANK_SLOTS) {
        const int n = num_tanks->valueint;
        s_profile.tank_enable_mask = (uint8_t)((n > 0) ? ((1u << n) - 1u) : 0);
    }

    if (cJSON_IsNumber(relay_mask)) {
        s_profile.relay_enable_mask = (uint16_t)relay_mask->valueint;
    }
    if (cJSON_IsNumber(dim_mask)) {
        s_profile.dim_enable_mask = (uint8_t)dim_mask->valueint;
    }
    if (cJSON_IsNumber(sensor_mask)) {
        s_profile.sensor_enable_mask = (uint8_t)sensor_mask->valueint;
    }
    if (cJSON_IsNumber(tank_mask)) {
        s_profile.tank_enable_mask = (uint8_t)tank_mask->valueint;
    }

    apply_enable_mask_u16(
        cJSON_GetObjectItem(json, "OutputsNameBuffer"),
        &s_profile.relay_enable_mask,
        s_profile.relay_type,
        HEXNET_IO_RELAY_SLOTS);
    apply_enable_mask_u16(
        cJSON_GetObjectItem(json, "relays"),
        &s_profile.relay_enable_mask,
        s_profile.relay_type,
        HEXNET_IO_RELAY_SLOTS);

    apply_enable_mask_u8(
        cJSON_GetObjectItem(json, "DimsNameBuffer"),
        &s_profile.dim_enable_mask,
        s_profile.dim_type,
        HEXNET_IO_DIM_SLOTS);
    apply_enable_mask_u8(
        cJSON_GetObjectItem(json, "dimmers"),
        &s_profile.dim_enable_mask,
        s_profile.dim_type,
        HEXNET_IO_DIM_SLOTS);

    apply_enable_mask_u8(
        cJSON_GetObjectItem(json, "SensorsNameBuffer"),
        &s_profile.sensor_enable_mask,
        s_profile.sensor_type,
        HEXNET_IO_SENSOR_SLOTS);
    apply_enable_mask_u8(
        cJSON_GetObjectItem(json, "sensors"),
        &s_profile.sensor_enable_mask,
        s_profile.sensor_type,
        HEXNET_IO_SENSOR_SLOTS);

    const cJSON *tanks = cJSON_GetObjectItem(json, "tanks");
    if (cJSON_IsArray(tanks)) {
        s_profile.tank_enable_mask = 0;
        const int n = cJSON_GetArraySize(tanks);
        for (int i = 0; i < n && i < HEXNET_IO_TANK_SLOTS; ++i) {
            const cJSON *item = cJSON_GetArrayItem(tanks, i);
            if (json_bool(item, 0)) {
                s_profile.tank_enable_mask |= (uint8_t)(1u << i);
            }
        }
    }

    if (motor_en) {
        s_profile.motor_enabled = (uint8_t)json_bool(motor_en, s_profile.motor_enabled);
    }
    if (rgb_en || rgb_en2) {
        s_profile.rgb_enabled = (uint8_t)json_bool(rgb_en ? rgb_en : rgb_en2, s_profile.rgb_enabled);
    }

    const cJSON *links = cJSON_GetObjectItem(json, "links");
    if (cJSON_IsObject(links)) {
        if (cJSON_GetObjectItem(links, "display")) {
            s_profile.link_flags = (uint8_t)(
                (s_profile.link_flags & ~(uint8_t)HEXNET_IO_LINK_DISPLAY) |
                (json_bool(cJSON_GetObjectItem(links, "display"), 0) ? HEXNET_IO_LINK_DISPLAY : 0));
        }
        if (cJSON_GetObjectItem(links, "ble")) {
            s_profile.link_flags = (uint8_t)(
                (s_profile.link_flags & ~(uint8_t)HEXNET_IO_LINK_BLE) |
                (json_bool(cJSON_GetObjectItem(links, "ble"), 0) ? HEXNET_IO_LINK_BLE : 0));
        }
        if (cJSON_GetObjectItem(links, "remote")) {
            s_profile.link_flags = (uint8_t)(
                (s_profile.link_flags & ~(uint8_t)HEXNET_IO_LINK_REMOTE) |
                (json_bool(cJSON_GetObjectItem(links, "remote"), 0) ? HEXNET_IO_LINK_REMOTE : 0));
        }
    }

    if (link_flag) {
        s_profile.link_flags |= link_flag;
    }

    profile_recompute_counts(&s_profile);
    return hexnet_io_profile_save();
}

void hexnet_io_profile_append_to_json(cJSON *parent, const char *object_name)
{
    if (!parent) {
        return;
    }
    const char *name = (object_name && object_name[0]) ? object_name : "io_profile";
    cJSON *io = cJSON_CreateObject();
    if (!io) {
        return;
    }

    cJSON_AddNumberToObject(io, "relays", s_profile.num_relays);
    cJSON_AddNumberToObject(io, "dimmers", s_profile.num_dims);
    cJSON_AddNumberToObject(io, "sensors", s_profile.num_sensors);
    cJSON_AddNumberToObject(io, "tanks", s_profile.num_tanks);
    cJSON_AddBoolToObject(io, "motor", s_profile.motor_enabled != 0);
    cJSON_AddBoolToObject(io, "rgb", s_profile.rgb_enabled != 0);
    cJSON_AddNumberToObject(io, "relay_mask", s_profile.relay_enable_mask);
    cJSON_AddNumberToObject(io, "dim_mask", s_profile.dim_enable_mask);
    cJSON_AddNumberToObject(io, "sensor_mask", s_profile.sensor_enable_mask);
    cJSON_AddNumberToObject(io, "tank_mask", s_profile.tank_enable_mask);

    cJSON *links = cJSON_CreateObject();
    cJSON_AddBoolToObject(links, "display", (s_profile.link_flags & HEXNET_IO_LINK_DISPLAY) != 0);
    cJSON_AddBoolToObject(links, "ble", (s_profile.link_flags & HEXNET_IO_LINK_BLE) != 0);
    cJSON_AddBoolToObject(links, "remote", (s_profile.link_flags & HEXNET_IO_LINK_REMOTE) != 0);
    cJSON_AddItemToObject(io, "links", links);

    cJSON_AddItemToObject(parent, name, io);
}

bool hexnet_io_profile_relay_slot_enabled(uint8_t index)
{
    return index < HEXNET_IO_RELAY_SLOTS && (s_profile.relay_enable_mask & (1u << index)) != 0;
}

bool hexnet_io_profile_dim_slot_enabled(uint8_t index)
{
    return index < HEXNET_IO_DIM_SLOTS && (s_profile.dim_enable_mask & (1u << index)) != 0;
}

bool hexnet_io_profile_sensor_slot_enabled(uint8_t index)
{
    return index < HEXNET_IO_SENSOR_SLOTS && (s_profile.sensor_enable_mask & (1u << index)) != 0;
}

bool hexnet_io_profile_motor_enabled(void)
{
    return s_profile.motor_enabled != 0;
}

bool hexnet_io_profile_rgb_enabled(void)
{
    return s_profile.rgb_enabled != 0;
}

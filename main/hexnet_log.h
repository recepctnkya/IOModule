#pragma once

/*
 * Compile-time log groups. Set to 0 in CMakeLists to strip I/D/W from that area.
 * Only Wi-Fi, MQTT and BLE are enabled by default.
 */
#ifndef HEXNET_LOG_WIFI
#define HEXNET_LOG_WIFI 1
#endif
#ifndef HEXNET_LOG_MQTT
#define HEXNET_LOG_MQTT 1
#endif
#ifndef HEXNET_LOG_BLE
#define HEXNET_LOG_BLE 1
#endif
#ifndef HEXNET_LOG_OTA
#define HEXNET_LOG_OTA 1
#endif

#if HEXNET_LOG_WIFI
#define HEXNET_LOG_LEVEL_WIFI ESP_LOG_VERBOSE
#else
#define HEXNET_LOG_LEVEL_WIFI ESP_LOG_ERROR
#endif

#if HEXNET_LOG_MQTT
#define HEXNET_LOG_LEVEL_MQTT ESP_LOG_VERBOSE
#else
#define HEXNET_LOG_LEVEL_MQTT ESP_LOG_ERROR
#endif

#if HEXNET_LOG_BLE
#define HEXNET_LOG_LEVEL_BLE ESP_LOG_VERBOSE
#else
#define HEXNET_LOG_LEVEL_BLE ESP_LOG_ERROR
#endif

#if HEXNET_LOG_OTA
#define HEXNET_LOG_LEVEL_OTA ESP_LOG_VERBOSE
#else
#define HEXNET_LOG_LEVEL_OTA ESP_LOG_ERROR
#endif

#define HEXNET_LOG_LEVEL_OTHER ESP_LOG_DEBUG

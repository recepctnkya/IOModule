#pragma once

#include "hexnet_log.h"

#ifdef __cplusplus
extern "C" {
#endif

#if HEXNET_LOG_WIFI
void hexnet_debug_log(const char *fmt, ...);
#else
#define hexnet_debug_log(...) ((void)0)
#endif

#if HEXNET_LOG_MQTT
void hexnet_debug_platform_cmd(const char *fmt, ...);
#else
#define hexnet_debug_platform_cmd(...) ((void)0)
#endif

#ifdef __cplusplus
}
#endif

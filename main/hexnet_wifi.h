#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void hexnet_wifi_init(void);
bool hexnet_wifi_is_connected(void);
bool hexnet_wifi_has_internet(void);
bool hexnet_wifi_is_internet_ok(void);
bool hexnet_wifi_has_saved_credentials(void);
const char *hexnet_wifi_get_sta_ip(void);
const char *hexnet_wifi_get_sta_ssid(void);
int hexnet_wifi_get_sta_rssi(void);

void hexnet_wifi_start_ntp_if_needed(void);
bool hexnet_wifi_ntp_is_synced(void);
int64_t hexnet_wifi_ntp_unix_ts_sec(void);

#ifdef __cplusplus
}
#endif

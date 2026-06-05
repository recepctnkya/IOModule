#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HEXNET_PORTAL_IP "192.168.1.1"

bool hexnet_wifi_portal_is_factory_boot(void);
bool hexnet_wifi_portal_is_provisioned(void);
bool hexnet_wifi_portal_is_active(void);
const char *hexnet_wifi_portal_get_ap_ssid(void);
void hexnet_wifi_portal_enable_ap(void);
void hexnet_wifi_portal_open(void);
void hexnet_wifi_portal_on_ap_started(void);
void hexnet_wifi_portal_shutdown(void);
void hexnet_wifi_portal_start_http(void);
void hexnet_wifi_portal_mark_provisioned(void);
void hexnet_wifi_portal_request_scan(void);
void hexnet_wifi_portal_on_scan_done(void);

#ifdef __cplusplus
}
#endif

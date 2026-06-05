#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** Deferred init: CAN, BLE, IO tasks, MQTT (after Wi-Fi portal window). */
void hexnet_app_bringup_task(void *arg);

#ifdef __cplusplus
}
#endif

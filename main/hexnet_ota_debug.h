#pragma once

#include "hexnet_log.h"

#ifdef __cplusplus
extern "C" {
#endif

/** OTA adim loglari (HEXNET_LOG_OTA=1). step: 1..N sabit numara. */
void hexnet_ota_dbg_step(unsigned step, const char *phase, const char *fmt, ...);

/** MQTT gelen mesaj (HEXNET_LOG_MQTT=1). */
void hexnet_mqtt_dbg_rx(const char *topic, const char *payload, int payload_len);

/** MQTT giden publish (HEXNET_LOG_MQTT=1). */
void hexnet_mqtt_dbg_tx(const char *topic, const char *payload, int payload_len, int msg_id, int qos);

/** MQTT olay (connected, published, ...). */
void hexnet_mqtt_dbg_event(const char *event_name, const char *detail_fmt, ...);

#ifdef __cplusplus
}
#endif

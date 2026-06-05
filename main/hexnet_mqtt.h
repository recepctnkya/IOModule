#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void hexnet_mqtt_init(void);
void hexnet_mqtt_start_tasks(void);
void hexnet_mqtt_on_sta_got_ip(void);
void hexnet_mqtt_stop(void);
/** OTA indirme oncesi: MQTT client kapatilir (HTTPS ile cakisma / panic onlenir). */
void hexnet_mqtt_shutdown_for_ota(void);
/** command_reply + telemetri + MQTT kapat (OTA task icinden, MQTT callback ile yarisma yok). */
void hexnet_mqtt_finish_ota_mqtt_handshake(const char *request_id, bool ok, const char *message);
/** OTA handshake sonrasi: durdurulmus client bellegi bosalt (ayri gorev, panic onlenir). */
void hexnet_mqtt_destroy_halted_client_when_safe(void);
/** Halted MQTT client hala bellekte mi (destroy bekleniyor). */
bool hexnet_mqtt_halted_client_pending(void);
/** OTA basarisiz: MQTT yeniden baglanabilir. */
void hexnet_mqtt_resume_after_ota_abort(void);
bool hexnet_mqtt_is_connected(void);
bool hexnet_mqtt_has_telemetry_ok(void);
/** OTA oncesi son telemetri (ota.state) — MQTT kapatilmadan once */
void hexnet_mqtt_publish_telemetry_now(void);

#ifdef __cplusplus
}
#endif

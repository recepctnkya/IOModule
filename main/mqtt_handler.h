#pragma once

/**
 * @brief Start the MQTT client and telemetry task.
 *
 * Call this once after wifi_prov_init() returns (WiFi connected).
 *
 * Topics (MAC address uniquely identifies the device):
 *   Publish:   {CONFIG_MQTT_TOPIC_PREFIX}/{MAC}/telemetry  – sensor data JSON, QoS 1
 *   Subscribe: {CONFIG_MQTT_TOPIC_PREFIX}/{MAC}/control    – command JSON, QoS 1
 *
 * Telemetry JSON keys:
 *   device, uptime_s, temp, humidity,
 *   water_1..4, battery_v,
 *   outputs, pwm_0, pwm_1,
 *   rgb_r, rgb_g, rgb_b, rgb_en, motor
 *
 * Control JSON keys (all optional):
 *   outputs  (0-65535)   – 16 digital outputs bitmask
 *   pwm_0    (0-100)     – dimmable channel 0 %
 *   pwm_1    (0-100)     – dimmable channel 1 %
 *   rgb_r/g/b (0-255)    – RGB colour
 *   rgb_en   (0/1)       – RGB strip on/off
 *   motor    (0/1/2)     – 0 stop, 1 forward, 2 backward
 */
void mqtt_handler_start(void);

<<<<<<< HEAD
idf_component_register(SRCS
                    "TroykaDHT.cpp"
                    "dht_sensor.cpp"
                    "adc_monitor.c"
                    "esp_ws28xx.c"
                    "hexnet_canbus.c"
                    "hexnet_bluetooth.c"
                    "hexnet_resolve_mqtt.c"
                    "hexnet_wifi_portal.c"
                    "dns_server.c"
                    "hexnet_debug.c"
                    "hexnet_ota_debug.c"
                    "hexnet_log_init.c"
                    "hexnet_http.c"
                    "hexnet_wifi.c"
                    "hexnet_mqtt.c"
                    "hexnet_board.c"
                    "hexnet_io_profile.c"
                    "hexnet_version.c"
                    "hexnet_motor.c"
                    "hexnet_shift_register.c"
                    "hexnet_rgb_strip.c"
                    "hexnet_dim_pwm.c"
                    "hexnet_sensors.c"
                    "hexnet_app.c"
                    "hexnet_nvs.c"
                    "UserCodes/Ota/OtaManager.cpp"
                    "app_main.c" 
                    INCLUDE_DIRS "." "UserCodes/Ota"
                    REQUIRES 
=======
idf_component_register(SRCS  "TroykaDHT.cpp" "dht_sensor.cpp"  "adc_monitor.c" "esp_ws28xx.c" "hexnet_canbus.c" "hexnet_bluetooth.c" "app_main.c" "wifi_prov_mgr.c"
                    INCLUDE_DIRS "."
                    REQUIRES
>>>>>>> 4ada128a93d42e7407371d5f2fbc719e66ac861f
                    esp_system
                    nvs_flash
                    esp_event
                    esp_netif
                    esp_wifi
                    esp_http_server
                    lwip
                    protocol_examples_common
                    driver
                    mqtt
                    esp_http_client
                    esp_http_server
                    esp_https_ota
                    app_update
                    esp_adc
                    bt
                    json
                    )

target_add_binary_data(${COMPONENT_LIB} "hexnet_logo.png" BINARY)

idf_build_get_property(project_ver PROJECT_VER)
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    HEXNET_LOG_WIFI=1
    HEXNET_LOG_MQTT=1
    HEXNET_LOG_BLE=1
    HEXNET_LOG_OTA=1
    PROJECT_VER="${project_ver}"
)


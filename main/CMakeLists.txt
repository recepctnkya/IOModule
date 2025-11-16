idf_component_register(SRCS  "TroykaDHT.cpp" "dht_sensor.cpp"  "adc_monitor.c" "esp_ws28xx.c" "hexnet_canbus.c" "hexnet_bluetooth.c" "app_main.c" 
                    INCLUDE_DIRS "."
                    REQUIRES 
                    esp_system
                    nvs_flash
                    esp_event
                    esp_netif
                    protocol_examples_common
                    driver
                    mqtt
                    esp_adc
                    bt
                    json
                    )

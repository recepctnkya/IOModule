idf_component_register(SRCS  "TroykaDHT.cpp" "dht_sensor.cpp"  "adc_monitor.c" "esp_ws28xx.c" "app_main.c" "waveshare_twai_port.c" 
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
                    )

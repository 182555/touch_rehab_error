idf_component_register(SRCS "can_open_comm.c" "wifiConnection.c" "led_strip_rmt_ws2812.c" "StateStatusLED.c" "stateMachine.c" 
                    INCLUDE_DIRS "."
                    REQUIRES esp_netif driver nvs_flash wpa_supplicant esp_wifi
                
                    )

idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS ""
                       REQUIRES
                           ble_manager
                           nvs_flash
                           freertos
                           esp_timer
                           esp_event
                           esp_netif
                           esp_wifi
                           esp-tls
                           espressif__mqtt)


idf_component_register(SRCS usb_cdc_basic_main.c
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES usb esp_wifi esp_event nvs_flash esp_system esp_netif lwip)

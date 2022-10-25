idf_component_register(SRCS "tusb_serial_device_main.c"
INCLUDE_DIRS "."
PRIV_REQUIRES nvs_flash esp_netif driver
PRIV_REQUIRES tinyusb
REQUIRES StateMachine
REQUIRES protocol_examples_common
REQUIRES tinyusb)

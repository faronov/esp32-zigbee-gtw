if(CONFIG_NCP_BUS_MODE_USB)
    set(BUS_SRC "src/esp_ncp_bus_usb.c")
else()
    set(BUS_SRC "src/esp_ncp_bus.c")
endif()

idf_component_register(SRCS "${BUS_SRC}"
                             "src/esp_ncp_frame.c"
                             "src/esp_ncp_main.c"
                             "src/esp_ncp_zb.c"
                             "src/slip.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "src/priv"
                       PRIV_REQUIRES esp-zigbee-lib nvs_flash driver)
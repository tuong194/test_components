
idf_component_register(SRCS "hello_world_main.c" "rd_ble_mesh.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES freertos log esp_hw_support bt nvs_flash
                    REQUIRES driver esp_rom esp_event
                    )

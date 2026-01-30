idf_component_register(SRCS "pressure.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver nvs_flash esp_adc
)
idf_component_register(SRCS "pressure.c"
                       INCLUDE_DIRS "." "${IDF_PATH}/components/esp_adc/deprecated/include"
                       REQUIRES driver)
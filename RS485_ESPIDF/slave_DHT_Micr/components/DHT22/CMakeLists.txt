idf_component_register(SRCS "DHT22.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES driver log freertos esp_system)
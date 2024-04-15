idf_component_register(SRCS "adxl355.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver
                    REQUIRES esp_timer
                    REQUIRES freertos)

idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "")
list(APPEND EXTRA_COMPONENT_DIRS components)
spiffs_create_partition_image(storage ../partition FLASH_IN_PROJECT)
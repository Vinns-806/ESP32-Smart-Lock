idf_component_register(SRCS "smart_lock_main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES bt nvs_flash esp_driver_ledc esp_driver_i2c)
idf_component_register(
    SRCS "i2c.c" "adxl345.c"
    INCLUDE_DIRS "."
    REQUIRES "esp_driver_gpio"
)
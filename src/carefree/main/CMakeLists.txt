idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash esp_wifi mqtt esp_http_client adxl345)

component_compile_options(-Wno-error=format= -Wno-format)

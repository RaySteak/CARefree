idf_component_register(SRCS "main.cpp" "mqtt_auth.cpp"
                       INCLUDE_DIRS ".")

component_compile_options(-Wno-error=format= -Wno-format)

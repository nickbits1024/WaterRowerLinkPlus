
idf_component_register(SRCS "ble.c" "s4.c" "heart_rate_manager.c" "antplus.c" "main.c" "usb.c"
                    INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-missing-field-initializers" "-Wno-switch" "-Wno-unused-variable")
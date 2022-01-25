
idf_component_register(SRCS "antplus.c" "waterrower.c" "main.c" "waterrower.c" "usb.c"
                    INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-missing-field-initializers" "-Wno-switch" "-Wno-unused-variable")
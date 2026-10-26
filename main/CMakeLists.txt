set(srcs "main.c" "gamepad.c" "crsf.c" "crc.c" "lizard.c")

set(requires "bluepad32" "btstack")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")

# idf_component_register(SRCS "uart_async_rxtxtasks_main.c"
#                     INCLUDE_DIRS ".")
file(GLOB_RECURSE BLE "./BLE/*.cpp")
file(GLOB_RECURSE MAIN "main.cpp")

idf_component_register(SRCS ${MAIN} ${BLE}
                       PRIV_REQUIRES spi_flash
                       INCLUDE_DIRS  "Headers" "BLE")

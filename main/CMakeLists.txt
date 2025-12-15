file(GLOB_RECURSE BLE "./BLE/*.cpp")
file(GLOB_RECURSE MAIN "main.cpp")
file(GLOB_RECURSE DATAMODULE "./components/data_module/data_module.cpp")

idf_component_register(SRCS ${MAIN} ${BLE} ${DATAMODULE}
                       PRIV_REQUIRES spi_flash spiffs
                       INCLUDE_DIRS  "Headers" "BLE")

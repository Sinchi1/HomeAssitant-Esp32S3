file(GLOB_RECURSE BLE "./BLE/*.cpp")
file(GLOB_RECURSE MAIN "main.cpp")
file(GLOB_RECURSE DATAMODULE "./components/data_module/data_module.cpp")
file(GLOB_RECURSE Model "./Model/*.cpp")
file(GLOB_RECURSE adapter "./adapt/*.cpp")


idf_component_register(SRCS ${MAIN} ${BLE} ${DATAMODULE} ${Model} ${adapter}
                       PRIV_REQUIRES spi_flash spiffs
                       INCLUDE_DIRS  "Headers" "BLE" "Model" "adapt")

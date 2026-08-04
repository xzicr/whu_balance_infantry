add_library(chassis_build_config INTERFACE)
target_link_libraries(chassis_build_config INTERFACE stm32f407_options)

message(STATUS "Chassis target placeholder enabled")

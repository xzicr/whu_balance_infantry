add_library(gimbal_build_config INTERFACE)
target_link_libraries(gimbal_build_config INTERFACE stm32f407_options)

message(STATUS "Gimbal target placeholder enabled")

add_library(stm32f407_options INTERFACE)

target_compile_definitions(
    stm32f407_options
    INTERFACE
        USE_HAL_DRIVER
        STM32F407xx
        ARM_MATH_CM4
)

target_compile_options(
    stm32f407_options
    INTERFACE
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        -ffunction-sections
        -fdata-sections
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wall>
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wextra>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
)

target_link_options(
    stm32f407_options
    INTERFACE
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        -Wl,--gc-sections
        -Wl,--print-memory-usage
)

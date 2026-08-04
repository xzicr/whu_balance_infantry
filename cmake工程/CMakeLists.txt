cmake_minimum_required(VERSION 3.22)

project(
    rm_balance_firmware
    VERSION 0.1.0
    LANGUAGES C CXX ASM
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

include(cmake/stm32f407-options.cmake)

add_subdirectory(targets/chassis)
add_subdirectory(targets/gimbal)

message(STATUS "Configured isolated RM Balance firmware workspace")
message(STATUS "Firmware source migration has not started yet")

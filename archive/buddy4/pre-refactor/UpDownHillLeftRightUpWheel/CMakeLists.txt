cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico_w CACHE STRING "Target board")

# Accept a path supplied through -DPICO_SDK_PATH or the environment.
if(NOT PICO_SDK_PATH AND DEFINED ENV{PICO_SDK_PATH})
    file(TO_CMAKE_PATH "$ENV{PICO_SDK_PATH}" PICO_SDK_PATH)
endif()

# Usual Windows Raspberry Pi Pico extension SDK location.
if(NOT PICO_SDK_PATH AND DEFINED ENV{USERPROFILE})
    file(TO_CMAKE_PATH "$ENV{USERPROFILE}" IMU_USER_DIR)
    set(IMU_DEFAULT_SDK "${IMU_USER_DIR}/.pico-sdk/sdk/2.3.1")

    if(EXISTS "${IMU_DEFAULT_SDK}/external/pico_sdk_import.cmake")
        set(PICO_SDK_PATH "${IMU_DEFAULT_SDK}")
    endif()
endif()

if(NOT EXISTS "${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
    message(FATAL_ERROR
        "Pico SDK not found. Set PICO_SDK_PATH to your SDK 2.3.1 directory.")
endif()

set(PICO_SDK_PATH "${PICO_SDK_PATH}" CACHE PATH "Pico SDK directory" FORCE)

# Must be included before project().
include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")

project(IMUProject C CXX ASM)

pico_sdk_init()

if(NOT PICO_SDK_VERSION_STRING VERSION_EQUAL "2.3.1")
    message(WARNING
        "This project targets Pico SDK 2.3.1; found ${PICO_SDK_VERSION_STRING}.")
endif()

add_executable(IMUProject
    IMUProject.c
)

target_link_libraries(IMUProject PRIVATE
    pico_stdlib
    hardware_i2c
    m
)

target_compile_options(IMUProject PRIVATE
    -Wall
    -Wextra
)

pico_enable_stdio_usb(IMUProject 1)
pico_enable_stdio_uart(IMUProject 0)

pico_add_extra_outputs(IMUProject)
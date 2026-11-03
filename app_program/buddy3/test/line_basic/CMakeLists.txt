# Basic Buddy 2 + 3 line following test (own UF2, USB serial, no barcode, no IMU).
# Build from this folder:  cmake -S . -B build -G Ninja && cmake --build build
# Output: build/line_basic.uf2

# == Raspberry Pi Pico VS Code extension block ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ===============================================

cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w CACHE STRING "Board")
include(${CMAKE_CURRENT_LIST_DIR}/../../../pico_sdk_import.cmake)
project(line_basic C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

set(BUDDY2_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../buddy2)
set(BUDDY3_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)

add_executable(line_basic
    line_basic_main.c
    ${BUDDY3_DIR}/line_follow.c
    ${BUDDY3_DIR}/line_sensors.c
    ${BUDDY2_DIR}/motion_control.c
    ${BUDDY2_DIR}/motion_pid.c
    ${BUDDY2_DIR}/motion_motor.c
    ${BUDDY2_DIR}/motion_encoder.c)
target_include_directories(line_basic PRIVATE ${BUDDY2_DIR} ${BUDDY3_DIR})
target_compile_options(line_basic PRIVATE -Wall -Wextra)
# Original team settings (same as app_program/integration_test). Nothing experimental:
# left wheel scale back to 1.0, right-turn speed and the rest from line_config.h.
target_compile_definitions(line_basic PRIVATE
    LINE_LEFT_WHEEL_SCALE=1.0f
    MOTION_MIN_DUTY=0.10f MOTION_BREAKAWAY_DUTY=0.22f
    MOTION_FEEDFORWARD_DUTY_PER_CM_S=0.0125f
    MOTION_STALL_DUTY=0.6f MOTION_STALL_TIME_MS=1500u
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=500)
target_link_libraries(line_basic PRIVATE pico_stdlib hardware_gpio
    hardware_pwm hardware_clocks hardware_sync m)
pico_enable_stdio_usb(line_basic 1)
pico_enable_stdio_uart(line_basic 0)
pico_add_extra_outputs(line_basic)

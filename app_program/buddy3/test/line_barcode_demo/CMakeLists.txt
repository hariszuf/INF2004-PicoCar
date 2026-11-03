# Buddy 2 + 3 line following with barcode reading (own UF2, USB serial, no IMU).
# Build from this folder:  cmake -S . -B build -G Ninja && cmake --build build
# Output: build/line_barcode.uf2

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
project(line_barcode C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

set(BUDDY2_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../buddy2)
set(BUDDY3_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)

add_executable(line_barcode
    line_barcode_main.c
    ${BUDDY3_DIR}/line_follow.c
    ${BUDDY3_DIR}/line_sensors.c
    ${BUDDY3_DIR}/barcode_scan.c
    ${BUDDY3_DIR}/barcode39.c
    ${BUDDY2_DIR}/motion_control.c
    ${BUDDY2_DIR}/motion_pid.c
    ${BUDDY2_DIR}/motion_motor.c
    ${BUDDY2_DIR}/motion_encoder.c)
target_include_directories(line_barcode PRIVATE ${BUDDY2_DIR} ${BUDDY3_DIR})
target_compile_options(line_barcode PRIVATE -Wall -Wextra)
# Motion tuning. Baseline = the 3 Nov "ff025" build (min duty 0.15, feed-forward 0.025: more
# drive authority than integration_test's 0.10 / 0.0125, which only reached ~25% of target
# speed while line following). One steering change: LINE_RIGHT_OUTER_SPEED_CM_S=3.5f so
# right and left corrections are equal (was 5.0f for right turns).
# MOTION_PID_HOLD_INTEGRAL_ON_ZERO (Buddy 2) was tried in v3 and left the car not moving,
# so it is OFF here. Also enables the barcode hook in line following.
target_compile_definitions(line_barcode PRIVATE
    LINE_BARCODE_DECODE=1
    LINE_RIGHT_OUTER_SPEED_CM_S=3.5f
    MOTION_MIN_DUTY=0.15f MOTION_BREAKAWAY_DUTY=0.22f
    MOTION_FEEDFORWARD_DUTY_PER_CM_S=0.025f
    MOTION_STALL_DUTY=0.6f MOTION_STALL_TIME_MS=1500u
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=500)
target_link_libraries(line_barcode PRIVATE pico_stdlib hardware_gpio
    hardware_pwm hardware_clocks hardware_sync m)
pico_enable_stdio_usb(line_barcode 1)
pico_enable_stdio_uart(line_barcode 0)
pico_add_extra_outputs(line_barcode)

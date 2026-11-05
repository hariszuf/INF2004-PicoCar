# Buddy 2 serial bench (motion_bench) - own UF2, USB serial.
# Build from this folder:  cmake -S . -B build -G Ninja && cmake --build build
# Output: build/motion_bench.uf2
# Uses the settings in app_program/buddy2/motion_config.h unchanged.

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
project(motion_bench C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

set(BUDDY2_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)

add_executable(motion_bench
    ${BUDDY2_DIR}/test/motion_bench_main.c
    ${BUDDY2_DIR}/motion_control.c
    ${BUDDY2_DIR}/motion_pid.c
    ${BUDDY2_DIR}/motion_motor.c
    ${BUDDY2_DIR}/motion_encoder.c)
target_include_directories(motion_bench PRIVATE ${BUDDY2_DIR})
target_compile_options(motion_bench PRIVATE -Wall -Wextra)
target_link_libraries(motion_bench
    pico_stdlib hardware_gpio hardware_pwm hardware_clocks hardware_sync m)
pico_enable_stdio_usb(motion_bench 1)
pico_enable_stdio_uart(motion_bench 0)
pico_add_extra_outputs(motion_bench)

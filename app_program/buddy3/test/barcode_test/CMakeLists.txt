# Stand-alone barcode reader test (own UF2, USB serial, no motors).
# Build from this folder:  cmake -S . -B build -G Ninja && cmake --build build
# Output: build/barcode_test.uf2

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
project(barcode_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

set(BUDDY3_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)

add_executable(barcode_test
    barcode_test_main.c
    ${BUDDY3_DIR}/barcode39.c
    ${BUDDY3_DIR}/barcode_scan.c
    ${BUDDY3_DIR}/line_sensors.c)
target_include_directories(barcode_test PRIVATE ${BUDDY3_DIR})
target_compile_options(barcode_test PRIVATE -Wall -Wextra)
target_link_libraries(barcode_test PRIVATE pico_stdlib hardware_gpio)
pico_enable_stdio_usb(barcode_test 1)
pico_enable_stdio_uart(barcode_test 0)
pico_add_extra_outputs(barcode_test)

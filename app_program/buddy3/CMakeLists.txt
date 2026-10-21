# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
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
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")


cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(ir_sensor_test C CXX ASM)

pico_sdk_init()

add_executable(ir_sensor_test
    ir_motor_curve_test.c
)

target_link_libraries(ir_sensor_test
    pico_stdlib
    hardware_pwm
)

pico_enable_stdio_usb(ir_sensor_test 1)
pico_enable_stdio_uart(ir_sensor_test 0)

pico_add_extra_outputs(ir_sensor_test)

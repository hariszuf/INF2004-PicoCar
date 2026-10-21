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
cmake_minimum_required(VERSION 3.13)

# Board must be Pico W for the optional on-board Wi-Fi/MQTT stage.
# VS Code Pico extension: choose board "pico_w" when importing.
if (NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico_w CACHE STRING "Pico board type")
endif()

include(pico_sdk_import.cmake)
project(buddy4 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

option(BUDDY4_ENABLE_MQTT "Build with Pico W Wi-Fi and MQTT" OFF)

add_executable(buddy4
    main.c
    lsm303d.c
    terrain.c
)

target_include_directories(buddy4 PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(buddy4 pico_stdlib hardware_i2c m)

if(BUDDY4_ENABLE_MQTT)
    if(NOT PICO_BOARD STREQUAL "pico_w")
        message(FATAL_ERROR "MQTT mode requires PICO_BOARD=pico_w for this project")
    endif()
    target_sources(buddy4 PRIVATE mqtt_telemetry.c)
    target_compile_definitions(buddy4 PRIVATE BUDDY4_ENABLE_MQTT=1)
    target_link_libraries(buddy4
        pico_cyw43_arch_lwip_threadsafe_background
        pico_lwip_mqtt
    )
else()
    target_compile_definitions(buddy4 PRIVATE BUDDY4_ENABLE_MQTT=0)
endif()

pico_enable_stdio_usb(buddy4 1)
pico_enable_stdio_uart(buddy4 0)
pico_add_extra_outputs(buddy4)

cmake_minimum_required(VERSION 3.13)

# Include the Raspberry Pi Pico C/C++ SDK build system
include(pico_sdk_import.cmake)

project(RoboCar C CXX ASM)

# Initialize the SDK
pico_sdk_init()

# Define the executable and its source files
add_executable(robocar
    src/main.c
    src/communication/wifi_mqtt.c
    src/communication/telemetry.c
    
)

# Target the Raspberry Pi Pico W hardware
set(PICO_BOARD pico_w)

# Link the executable with the C-SDK libraries
target_link_libraries(robocar
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    # Note: Link your μT-Kernel 3.0 static library (.a) here if required by your lab setup
)

# Enable USB standard I/O for debugging
pico_enable_stdio_usb(robocar 1)
pico_enable_stdio_uart(robocar 0)

# Direct the build system to generate the final binary (.bin/.uf2) files
pico_add_extra_outputs(robocar)
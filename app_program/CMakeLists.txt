# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

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
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(EmbeddedProject C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

# Autonomous line following (Buddy 3) on top of Motion Control (Buddy 2).
# To run the Buddy 2 square-route test instead, use "Buddy 2/test/motion_demo_main.c"
# in place of the line_follow_demo_main.c line and remove the Buddy 3 sources.
add_executable(EmbeddedProject
        "Buddy 3/test/line_follow_demo_main.c"
        "Buddy 3/line_follow.c"
        "Buddy 3/line_sensors.c"
        "Buddy 2/motion_control.c"
        "Buddy 2/motion_pid.c"
        "Buddy 2/motion_motor.c"
        "Buddy 2/motion_encoder.c"
)

pico_set_program_name(EmbeddedProject "EmbeddedProject")
pico_set_program_version(EmbeddedProject "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(EmbeddedProject 0)
pico_enable_stdio_usb(EmbeddedProject 1)

# Add the standard library to the build
target_link_libraries(EmbeddedProject
        pico_stdlib
        hardware_gpio
        hardware_pwm
        hardware_clocks
        hardware_sync)

# Add the standard include files to the build
target_include_directories(EmbeddedProject PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        "${CMAKE_CURRENT_LIST_DIR}/Buddy 2"
        "${CMAKE_CURRENT_LIST_DIR}/Buddy 3"
)

# Line following runs much slower than the Buddy 2 demo: lower the minimum motor duty.
target_compile_definitions(EmbeddedProject PRIVATE
        MOTION_MIN_DUTY=0.10f
        MOTION_BREAKAWAY_DUTY=0.22f
        MOTION_FEEDFORWARD_DUTY_PER_CM_S=0.0125f
        MOTION_STALL_DUTY=0.6f
        MOTION_STALL_TIME_MS=1500u
)

# Add any user requested libraries
target_link_libraries(EmbeddedProject 
        
        )

pico_add_extra_outputs(EmbeddedProject)


# Separate motor diagnostic firmware - does not change EmbeddedProject.uf2
add_executable(RightTurnDiagnostic
        "Buddy 3/test/right_turn_diagnostic_main.c"
        "Buddy 2/motion_control.c"
        "Buddy 2/motion_pid.c"
        "Buddy 2/motion_motor.c"
        "Buddy 2/motion_encoder.c"
)
pico_enable_stdio_uart(RightTurnDiagnostic 0)
pico_enable_stdio_usb(RightTurnDiagnostic 1)
target_include_directories(RightTurnDiagnostic PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/Buddy 2"
        "${CMAKE_CURRENT_LIST_DIR}/Buddy 3"
)
target_link_libraries(RightTurnDiagnostic
        pico_stdlib hardware_gpio hardware_pwm hardware_clocks hardware_sync
)
target_compile_definitions(RightTurnDiagnostic PRIVATE
        MOTION_MIN_DUTY=0.10f
        MOTION_BREAKAWAY_DUTY=0.22f
        MOTION_FEEDFORWARD_DUTY_PER_CM_S=0.0125f
        MOTION_STALL_DUTY=0.6f
        MOTION_STALL_TIME_MS=1500u
)
pico_add_extra_outputs(RightTurnDiagnostic)

cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w CACHE STRING "Board")
include(${CMAKE_CURRENT_LIST_DIR}/../pico_sdk_import.cmake)
project(buddy234_driving_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()
add_executable(buddy234_driving_test main.c drive_control.c
    ../buddy2/motion_control.c ../buddy2/motion_pid.c
    ../buddy2/motion_motor.c ../buddy2/motion_encoder.c
    ../buddy3/line_follow.c ../buddy3/line_sensors.c
    ../buddy4/buddy4_imu.c ../buddy4/buddy4_motion.c ../buddy4/buddy4_impact.c ../buddy4/buddy4_encoder_yaw.c)
target_include_directories(buddy234_driving_test PRIVATE ../buddy2 ../buddy3 ../buddy4 ../shared)
target_compile_options(buddy234_driving_test PRIVATE -Wall -Wextra)
target_compile_definitions(buddy234_driving_test PRIVATE
    MOTION_MIN_DUTY=0.10f MOTION_BREAKAWAY_DUTY=0.22f
    MOTION_FEEDFORWARD_DUTY_PER_CM_S=0.0125f
    MOTION_STALL_DUTY=0.6f MOTION_STALL_TIME_MS=1500u
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=500)
target_link_libraries(buddy234_driving_test PRIVATE pico_stdlib hardware_gpio
    hardware_pwm hardware_clocks hardware_sync hardware_i2c m)
pico_enable_stdio_usb(buddy234_driving_test 1)
pico_enable_stdio_uart(buddy234_driving_test 0)
pico_add_extra_outputs(buddy234_driving_test)


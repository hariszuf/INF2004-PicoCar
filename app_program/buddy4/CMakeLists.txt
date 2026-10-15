cmake_minimum_required(VERSION 3.16)
project(buddy4_imu_tests LANGUAGES C)

# This builds the standalone IMU logic test for Windows/Linux/macOS.
# buddy4.c uses the repository's micro T-Kernel and must be built in its
# existing firmware build system; this target does NOT produce a UF2.
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_library(imu_detect STATIC imu_detect.c)
target_include_directories(imu_detect PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(test_imu_detect tests/test_imu_detect.c)
target_link_libraries(test_imu_detect PRIVATE imu_detect)

enable_testing()
add_test(NAME imu_detection COMMAND test_imu_detect)

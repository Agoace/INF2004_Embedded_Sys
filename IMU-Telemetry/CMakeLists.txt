cmake_minimum_required(VERSION 3.13)

# Standalone pico_w firmware target for the IMU producer slice.
#
# This project is deliberately independent of the parent coursework repository:
# it builds on its own with the Pico SDK and links no Wi-Fi/CYW43 code.
include(${CMAKE_CURRENT_LIST_DIR}/pico_sdk_import.cmake)

project(imu_telemetry C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

pico_sdk_init()

add_executable(imu_telemetry
    src/imu_telemetry.c
    src/imu_core.c
    src/imu_lsm303.c
    src/imu_snapshot.c
    src/imu_hal_pico.c
)

target_include_directories(imu_telemetry PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)

target_compile_options(imu_telemetry PRIVATE -Wall -Wextra -Wshadow)

# hardware_i2c only. No pico_cyw43_arch, no Wi-Fi.
target_link_libraries(imu_telemetry PRIVATE
    pico_stdlib
    hardware_i2c
)

pico_enable_stdio_usb(imu_telemetry 1)
pico_enable_stdio_uart(imu_telemetry 0)

pico_add_extra_outputs(imu_telemetry)

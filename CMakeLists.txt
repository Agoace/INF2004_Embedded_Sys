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
cmake_minimum_required(VERSION 3.13...3.27)

# Select Pico W unless the caller supplies another compatible board.
if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico_w CACHE STRING "Target board")
endif()

# The SDK import must run before project().
include(pico_sdk_import.cmake)

project(inf2004_embedded_sys LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

add_executable(pin_probe
    src/pin_probe.c
)

target_link_libraries(pin_probe PRIVATE
    pico_stdlib
    hardware_gpio
)

# Send the probe report over USB. Leave the default UART pins free.
pico_enable_stdio_usb(pin_probe 1)
pico_enable_stdio_uart(pin_probe 0)

# Generate ELF, BIN, HEX, map, and UF2 files.
pico_add_extra_outputs(pin_probe)

add_executable(board_diagnostics
    src/board_diagnostics.c
)

target_link_libraries(board_diagnostics PRIVATE
    pico_stdlib
    hardware_adc
    hardware_gpio
    hardware_i2c
)

pico_enable_stdio_usb(board_diagnostics 1)
pico_enable_stdio_uart(board_diagnostics 0)
pico_add_extra_outputs(board_diagnostics)

add_executable(servo_test
    src/servo_test.c
)

target_link_libraries(servo_test PRIVATE
    pico_stdlib
    hardware_clocks
    hardware_pwm
)

pico_enable_stdio_usb(servo_test 1)
pico_enable_stdio_uart(servo_test 0)
pico_add_extra_outputs(servo_test)

add_executable(servo_calibration
    src/servo_calibration.c
)

target_link_libraries(servo_calibration PRIVATE
    pico_stdlib
    hardware_clocks
    hardware_pwm
)

pico_enable_stdio_usb(servo_calibration 1)
pico_enable_stdio_uart(servo_calibration 0)
pico_add_extra_outputs(servo_calibration)

add_executable(motor_encoder_calibration
    src/motor_encoder_calibration.c
)

target_link_libraries(motor_encoder_calibration PRIVATE
    pico_stdlib
    hardware_gpio
    hardware_pwm
)

pico_enable_stdio_usb(motor_encoder_calibration 1)
pico_enable_stdio_uart(motor_encoder_calibration 0)
pico_add_extra_outputs(motor_encoder_calibration)

add_executable(motor_duty_calibration
    src/motor_duty_calibration.c
)

target_link_libraries(motor_duty_calibration PRIVATE
    pico_stdlib
    hardware_gpio
    hardware_pwm
)

pico_enable_stdio_usb(motor_duty_calibration 1)
pico_enable_stdio_uart(motor_duty_calibration 0)
pico_add_extra_outputs(motor_duty_calibration)

add_executable(motor_speed_calibration
    src/motor_speed_calibration.c
)

target_link_libraries(motor_speed_calibration PRIVATE
    pico_stdlib
    hardware_gpio
    hardware_pwm
)

pico_enable_stdio_usb(motor_speed_calibration 1)
pico_enable_stdio_uart(motor_speed_calibration 0)
pico_add_extra_outputs(motor_speed_calibration)

add_executable(motor_rotation_drift
    src/motor_rotation_drift.c
    src/telemetry.c
)

target_include_directories(motor_rotation_drift PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_compile_definitions(motor_rotation_drift PRIVATE
    TELEMETRY_VERBOSE=0
)

target_link_libraries(motor_rotation_drift PRIVATE
    pico_stdlib
    hardware_gpio
    hardware_pwm
    pico_cyw43_arch_lwip_threadsafe_background
)

pico_enable_stdio_usb(motor_rotation_drift 1)
pico_enable_stdio_uart(motor_rotation_drift 0)
pico_add_extra_outputs(motor_rotation_drift)

add_executable(wifi_telemetry_probe
    src/wifi_telemetry_probe.c
    src/telemetry.c
)

target_include_directories(wifi_telemetry_probe PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(wifi_telemetry_probe PRIVATE
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
)

pico_enable_stdio_usb(wifi_telemetry_probe 1)
pico_enable_stdio_uart(wifi_telemetry_probe 0)
pico_add_extra_outputs(wifi_telemetry_probe)

add_executable(ultrasonic_test
    src/ultrasonic_test.c
    src/ultrasonic.c
)

target_include_directories(ultrasonic_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(ultrasonic_test PRIVATE
    pico_stdlib
    hardware_gpio
    hardware_irq
    hardware_sync
)

pico_enable_stdio_usb(ultrasonic_test 1)
pico_enable_stdio_uart(ultrasonic_test 0)
pico_add_extra_outputs(ultrasonic_test)

add_executable(obstacle_test
    src/obstacle_test.c
    src/obstacle.c
    src/ultrasonic.c
)

target_include_directories(obstacle_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(obstacle_test PRIVATE
    pico_stdlib
    hardware_gpio
    hardware_irq
    hardware_sync
)

pico_enable_stdio_usb(obstacle_test 1)
pico_enable_stdio_uart(obstacle_test 0)
pico_add_extra_outputs(obstacle_test)

add_executable(scan_test
    src/scan_test.c
    src/obstacle_profile.c
    src/obstacle_scan.c
    src/servo.c
    src/ultrasonic.c
)

target_include_directories(scan_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(scan_test PRIVATE
    pico_stdlib
    hardware_clocks
    hardware_gpio
    hardware_irq
    hardware_pwm
    hardware_sync
)

pico_enable_stdio_usb(scan_test 1)
pico_enable_stdio_uart(scan_test 0)
pico_add_extra_outputs(scan_test)

add_executable(line_test
    src/line_test.c
    src/line_position.c
    src/line_sensors.c
    src/line_spin.c
    src/drive.c
)

target_include_directories(line_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(line_test PRIVATE
    pico_stdlib
    hardware_adc
    hardware_gpio
    hardware_pwm
    hardware_sync
)

pico_enable_stdio_usb(line_test 1)
pico_enable_stdio_uart(line_test 0)
pico_add_extra_outputs(line_test)

add_executable(steer_test
    src/steer_test.c
    src/line_steer.c
    src/line_position.c
    src/line_sensors.c
    src/line_spin.c
    src/drive.c
    src/steer_log.c
    src/wifi_log.c
)

target_include_directories(steer_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(steer_test PRIVATE
    pico_stdlib
    pico_rand
    hardware_adc
    hardware_gpio
    hardware_pwm
    hardware_sync
    pico_cyw43_arch_lwip_threadsafe_background
)

pico_enable_stdio_usb(steer_test 1)
pico_enable_stdio_uart(steer_test 0)
pico_add_extra_outputs(steer_test)

# Bench test: line following with the obstacle bypass (docs/OBSTACLE_BYPASS.md).
add_executable(bypass_test
    src/bypass_test.c
    src/obstacle_bypass.c
    src/obstacle.c
    src/obstacle_profile.c
    src/obstacle_scan.c
    src/servo.c
    src/ultrasonic.c
    src/line_steer.c
    src/line_position.c
    src/line_sensors.c
    src/line_spin.c
    src/drive.c
    src/steer_log.c
    src/wifi_log.c
)

target_include_directories(bypass_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(bypass_test PRIVATE
    pico_stdlib
    pico_rand
    hardware_adc
    hardware_gpio
    hardware_pwm
    hardware_sync
    pico_cyw43_arch_lwip_threadsafe_background
)

pico_enable_stdio_usb(bypass_test 1)
pico_enable_stdio_uart(bypass_test 0)
pico_add_extra_outputs(bypass_test)
# Independent IR line-detection and Code 39 barcode-reading firmware.
add_executable(robocar_ir_sensors
    src/robocar_ir_sensors.c
    src/line_detector.c
    src/ir_line_data.c
    src/barcode_reader.c
)

target_include_directories(robocar_ir_sensors PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(robocar_ir_sensors PRIVATE
    pico_stdlib
    hardware_adc
)

pico_enable_stdio_usb(robocar_ir_sensors 1)
pico_enable_stdio_uart(robocar_ir_sensors 0)
pico_add_extra_outputs(robocar_ir_sensors)

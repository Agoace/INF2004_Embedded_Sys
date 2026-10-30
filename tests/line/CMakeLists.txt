# Host-only build for the line component. No Pico SDK, no firmware targets.
#
#   cmake -S tests/line -B tests/line/build
#   cmake --build tests/line/build
#   ctest --test-dir tests/line/build --output-on-failure
#
# This lives here on purpose. The component's own project used to be the
# repository root CMakeLists.txt, which meant merging this branch replaced the
# firmware project and could drop firmware targets (RT-0036 B3). The root file is
# now identical to the branch point, and the firmware build is untouched.
#
# src/line/line_adc_pico.c is the component's only platform file. It needs
# hardware/adc.h from the Pico SDK, so the firmware target that embeds the
# component compiles it, not this project.

cmake_minimum_required(VERSION 3.13)

project(line_component_host LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(LINE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../src/line")

add_library(line_core STATIC
    ${LINE_DIR}/line_types.c
    ${LINE_DIR}/line_position.c
    ${LINE_DIR}/line_steer.c
    ${LINE_DIR}/line_spin.c
    ${LINE_DIR}/line_health.c
    ${LINE_DIR}/line_sampler.c
    ${LINE_DIR}/line_producer.c
)

target_include_directories(line_core PUBLIC ${LINE_DIR})

target_compile_options(line_core PRIVATE -Wall -Wextra -Wshadow)

enable_testing()

add_executable(test_line test_line.c)

target_link_libraries(test_line PRIVATE line_core)

target_link_libraries(test_line PRIVATE m)

target_compile_options(test_line PRIVATE -Wall -Wextra -Wshadow)

add_test(NAME line_component COMMAND test_line)

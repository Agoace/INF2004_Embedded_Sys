cmake_minimum_required(VERSION 3.13)

# Host unit tests for the IMU producer. Built with the host compiler, not the
# Pico toolchain, so this project is configured separately from the firmware:
#
#   cmake -S tests -B build-host && cmake --build build-host && ctest --test-dir build-host --output-on-failure
project(imu_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(test_imu
    test_imu.c
    ../src/imu_core.c
    ../src/imu_lsm303.c
    ../src/imu_snapshot.c
)

target_include_directories(test_imu PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../src)
target_compile_options(test_imu PRIVATE -Wall -Wextra -Wshadow)
target_link_libraries(test_imu PRIVATE m)

# IMU_TEST_SNAPSHOT_HOOK lets the test force a publish between the two word
# loads of imu_snapshot_read(), which proves the two-word seqlock retries
# instead of returning a torn pair. Production builds compile the hook away.
target_compile_definitions(test_imu PRIVATE IMU_TEST_SNAPSHOT_HOOK=1)

enable_testing()
add_test(NAME imu_host COMMAND test_imu)

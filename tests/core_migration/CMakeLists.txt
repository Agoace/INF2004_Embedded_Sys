cmake_minimum_required(VERSION 3.20)
project(core_migration_blocker_tests C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
get_filename_component(REPO "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
enable_testing()
add_compile_options(-Wall -Wextra -Wconversion -Wshadow -Wformat=2 -Werror
    -UNDEBUG)
set(LOCK_SECTIONS "${CMAKE_CURRENT_BINARY_DIR}/lock_sections.inc")
add_custom_command(OUTPUT "${LOCK_SECTIONS}"
    COMMAND "${Python3_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/extract_lock_sections.py"
        "${REPO}" "${LOCK_SECTIONS}"
    DEPENDS extract_lock_sections.py "${REPO}/kernel/tkernel/smp_lock.c"
        "${REPO}/app_program/subsystem1/robot_mqtt.c"
        "${REPO}/lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c"
    VERBATIM)
add_executable(test_b1_lock_sections test_b1_lock_sections.c "${LOCK_SECTIONS}")
target_include_directories(test_b1_lock_sections PRIVATE
    "${REPO}" "${CMAKE_CURRENT_BINARY_DIR}")
add_test(NAME b1_lock_sections COMMAND test_b1_lock_sections)
add_test(NAME b1_plain_lock_inventory COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/extract_lock_sections.py" "${REPO}")

set(LIFECYCLE "${CMAKE_CURRENT_BINARY_DIR}/lifecycle.inc")
add_custom_command(OUTPUT "${LIFECYCLE}"
    COMMAND "${Python3_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/extract_lifecycle.py"
        "${REPO}" "${LIFECYCLE}"
    DEPENDS extract_lifecycle.py extract_lock_sections.py
        "${REPO}/app_program/platform/app_lock_stress.c"
    VERBATIM)
add_custom_target(b2_lifecycle_source DEPENDS "${LIFECYCLE}")
foreach(smp 0 1)
    set(target "test_b2_lifecycle_smp${smp}")
    add_executable(${target} test_b2_lifecycle.c)
    add_dependencies(${target} b2_lifecycle_source)
    target_include_directories(${target} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
    target_compile_definitions(${target} PRIVATE TK_SUPPORT_SMP=${smp})
    add_test(NAME "b2_lifecycle_smp${smp}" COMMAND ${target})
endforeach()

include("${REPO}/tests/shim_checks.cmake")

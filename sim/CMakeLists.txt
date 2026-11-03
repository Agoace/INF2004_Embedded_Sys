cmake_minimum_required(VERSION 3.20)
project(robot_sim C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(Python3 3.9 REQUIRED COMPONENTS Interpreter)
get_filename_component(REPO "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(SIM_FIRMWARE detour_test CACHE STRING "Firmware source set")
if(NOT SIM_FIRMWARE STREQUAL "detour_test")
    message(FATAL_ERROR "Unsupported SIM_FIRMWARE: ${SIM_FIRMWARE}")
endif()
option(SIM_OBSERVE_HOOKS "Observe caller module calls without changing time" ON)
include("${CMAKE_CURRENT_SOURCE_DIR}/targets/${SIM_FIRMWARE}.cmake")
set(SIM_SOURCES core/runtime.c core/world.c devices/ultrasonic.c devices/placeholders.c ${SIM_TARGET_SOURCES})
add_executable(firmware_sim ${SIM_SOURCES} ${SIM_FIRMWARE_SOURCES})
target_include_directories(firmware_sim PRIVATE include "${REPO}/src")
target_link_libraries(firmware_sim PRIVATE m)
target_compile_options(firmware_sim PRIVATE -Wall -Wextra -Wconversion -Wshadow)
set_source_files_properties(${SIM_SOURCES} PROPERTIES
    COMPILE_OPTIONS "-Werror;-Wpedantic;-Wstrict-prototypes;-Wmissing-prototypes")
execute_process(COMMAND "${CMAKE_C_COMPILER}" --version
    OUTPUT_VARIABLE SIM_COMPILER OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ERROR_IS_FATAL ANY)
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/compiler.txt" "${SIM_COMPILER}\n")
list(JOIN SIM_HOOKS "\",\"" SIM_HOOK_JSON)
if(SIM_HOOKS)
    set(SIM_HOOK_JSON "\"${SIM_HOOK_JSON}\"")
endif()
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/target.json"
    "{\"target\":\"${SIM_FIRMWARE}\",\"hooks\":[${SIM_HOOK_JSON}],\"entry\":\"firmware_main\"}\n")
configure_file(core/robot_sim.in robot_sim @ONLY)
file(CHMOD "${CMAKE_CURRENT_BINARY_DIR}/robot_sim"
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
add_executable(sim_clock_probe core/runtime.c core/world.c devices/ultrasonic.c devices/placeholders.c tests/probe.c)
target_link_libraries(sim_clock_probe PRIVATE m)
add_executable(sim_model_probe core/world.c tests/model_probe.c)
target_include_directories(sim_model_probe PRIVATE include)
target_link_libraries(sim_model_probe PRIVATE m)
target_compile_options(sim_model_probe PRIVATE -Wall -Wextra -Wconversion -Wshadow -Werror -Wpedantic -UNDEBUG)
set_source_files_properties(tests/model_probe.c PROPERTIES COMPILE_OPTIONS "-Wstrict-prototypes;-Wmissing-prototypes")
target_include_directories(sim_clock_probe PRIVATE include "${REPO}/src")
target_compile_options(sim_clock_probe PRIVATE -Wall -Wextra -Wconversion -Wshadow -Werror -Wpedantic -UNDEBUG)
set_source_files_properties(tests/probe.c PROPERTIES COMPILE_OPTIONS "-Wstrict-prototypes;-Wmissing-prototypes")
add_executable(sim_ultrasonic_probe core/runtime.c core/world.c devices/ultrasonic.c
    devices/placeholders.c tests/ultrasonic_probe.c "${REPO}/src/obstacle_scan.c")
target_include_directories(sim_ultrasonic_probe PRIVATE include "${REPO}/src")
target_link_libraries(sim_ultrasonic_probe PRIVATE m)
target_compile_options(sim_ultrasonic_probe PRIVATE -Wall -Wextra -Wconversion -Wshadow -Werror -Wpedantic -UNDEBUG)
set_source_files_properties(tests/ultrasonic_probe.c PROPERTIES COMPILE_OPTIONS "-Wstrict-prototypes;-Wmissing-prototypes")
enable_testing()
add_test(NAME sim_contracts COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_milestone1.py"
    "${CMAKE_CURRENT_BINARY_DIR}/robot_sim")
add_test(NAME sim_motion_ir COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_milestone2.py"
    "${CMAKE_CURRENT_BINARY_DIR}/robot_sim")
add_test(NAME sim_echo_detour COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_milestone3.py"
    "${CMAKE_CURRENT_BINARY_DIR}/robot_sim")
add_test(NAME sim_specular_echo COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_specular.py"
    "${CMAKE_CURRENT_BINARY_DIR}/robot_sim")
add_test(NAME sim_environment_server COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_server.py"
    "${CMAKE_CURRENT_BINARY_DIR}/robot_sim")
add_test(NAME sim_offline_viewer COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_milestone4.py"
    "${CMAKE_CURRENT_BINARY_DIR}/robot_sim")

cmake_minimum_required(VERSION 3.20)
project(robot_host_tests C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
get_filename_component(REPO "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
enable_testing()
add_compile_options(-Wall -Wextra -Wconversion -Wshadow -Wformat=2 -Werror -UNDEBUG)
include_directories("${REPO}/src" "${REPO}/app_program/subsystem2"
    "${REPO}/app_program/subsystem3" "${REPO}/app_program/subsystem5"
    "${REPO}/app_program/vehicle")
function(host_test name test_file)
    set(sources "${REPO}/${test_file}")
    foreach(source IN LISTS ARGN)
        if(NOT EXISTS "${REPO}/${source}")
            return()
        endif()
        list(APPEND sources "${REPO}/${source}")
    endforeach()
    if(NOT EXISTS "${REPO}/${test_file}")
        return()
    endif()
    add_executable(${name} ${sources})
    add_test(NAME ${name} COMMAND ${name})
endfunction()

function(robot_source old new output)
    if(EXISTS "${REPO}/${new}")
        set(${output} "${new}" PARENT_SCOPE)
    else()
        set(${output} "${old}" PARENT_SCOPE)
    endif()
endfunction()
robot_source(src/line_position.c app_program/subsystem3/line_position.c line_position_source)
robot_source(src/line_steer.c app_program/subsystem3/line_steer.c line_steer_source)
robot_source(src/line_trace.c app_program/subsystem3/line_trace.c line_trace_source)
robot_source(src/line_search.c app_program/subsystem3/line_search.c line_search_source)
robot_source(src/obstacle.c app_program/subsystem5/obstacle.c obstacle_source)
robot_source(src/obstacle_profile.c app_program/subsystem5/obstacle_profile.c obstacle_profile_source)
robot_source(src/servo_scan.c app_program/subsystem5/servo_scan.c servo_scan_source)
robot_source(src/detour_run.c app_program/subsystem5/detour_run.c detour_run_source)
robot_source(src/obstacle_detour.c app_program/subsystem5/obstacle_detour.c obstacle_detour_source)
robot_source(src/motor_ctl.c app_program/subsystem2/motor_ctl.c motor_ctl_source)
robot_source(src/vehicle_sm.c app_program/vehicle/vehicle_sm.c vehicle_sm_source)
host_test(test_detour_sweep tests/test_detour_sweep.c ${obstacle_detour_source})
host_test(test_ir_modules tests/test_ir_modules.c src/barcode_reader.c src/ir_line_data.c)
host_test(test_obstacle_scan tests/test_obstacle_scan.c )
host_test(test_line_trace tests/test_line_trace.c ${line_trace_source} ${line_search_source} ${line_steer_source} ${obstacle_source} ${vehicle_sm_source} ${line_position_source})
host_test(test_obstacle_modules tests/test_obstacle_modules.c ${servo_scan_source} ${detour_run_source} ${obstacle_detour_source} ${obstacle_profile_source} ${line_trace_source} ${line_search_source} ${line_steer_source} ${obstacle_source} ${vehicle_sm_source} ${line_position_source})
host_test(test_heartbeat tests/test_heartbeat.c src/heartbeat.c)
host_test(test_line_position tests/test_line_position.c ${line_position_source})
host_test(test_line_steer tests/test_line_steer.c ${line_steer_source})
host_test(test_motor_ctl tests/test_motor_ctl.c ${motor_ctl_source})
host_test(test_obstacle_bypass tests/test_obstacle_bypass.c src/obstacle_bypass.c)
host_test(test_obstacle tests/test_obstacle.c ${obstacle_source})
host_test(test_obstacle_detour tests/test_obstacle_detour.c ${obstacle_detour_source})
host_test(test_obstacle_profile tests/test_obstacle_profile.c ${obstacle_profile_source})
host_test(test_steer_log tests/test_steer_log.c src/steer_log.c)
host_test(test_vehicle_controller tests/test_vehicle_controller.c src/vehicle_controller.c)
host_test(test_vehicle_sm tests/test_vehicle_sm.c ${vehicle_sm_source})
host_test(module_line host-tests/test_line.c ${line_trace_source} ${line_steer_source} ${obstacle_source} ${line_position_source})
host_test(module_search host-tests/test_search.c ${line_search_source} ${line_position_source})
host_test(module_scan host-tests/test_scan.c ${servo_scan_source} ${obstacle_profile_source})
host_test(module_detour host-tests/test_detour.c ${detour_run_source} ${obstacle_detour_source} ${line_position_source})

# Kernel-side glue tests use explicit host doubles, never real kernel headers.
host_test(test_motion_task tests/test_motion_task.c
    app_program/subsystem2/motion.c app_program/subsystem2/motor_ctl.c
    app_program/subsystem2/subsystem2.c)
host_test(test_gpio_irq tests/test_gpio_irq.c app_program/platform/drv_gpio_irq.c)
foreach(test IN ITEMS test_motion_task test_gpio_irq)
    if(TARGET ${test})
        target_include_directories(${test} BEFORE PRIVATE "${REPO}/tests/mtk_stubs")
    endif()
endforeach()
include("${CMAKE_CURRENT_LIST_DIR}/mqtt_protocol.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/template_checks.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/network_checks.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/vehicle_network_checks.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/mqtt_client_checks.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/dashboard_checks.cmake")

# Kernel time checks use host doubles, never real kernel/SDK headers.
host_test(test_drv_time tests/test_drv_time.c app_program/platform/drv_time.c)
if(TARGET test_drv_time)
    target_include_directories(test_drv_time BEFORE PRIVATE "${REPO}/tests/mtk_stubs")
endif()

host_test(test_drv_line tests/test_drv_line.c app_program/subsystem3/drv_line.c)
if(TARGET test_drv_line)
    target_include_directories(test_drv_line BEFORE PRIVATE "${REPO}/tests/mtk_stubs")
endif()

host_test(test_drv_ultrasonic tests/test_drv_ultrasonic.c app_program/subsystem5/drv_ultrasonic.c)
if(TARGET test_drv_ultrasonic)
    target_include_directories(test_drv_ultrasonic BEFORE PRIVATE "${REPO}/tests/mtk_stubs")
endif()

host_test(test_vehicle_log tests/test_vehicle_log.c app_program/vehicle/vehicle_log.c)
if(TARGET test_vehicle_log)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME test_robot_contract COMMAND "${Python3_EXECUTABLE}"
        "${REPO}/tests/test_robot_contract.py" "$<TARGET_FILE:test_vehicle_log>")
endif()
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME test_task_priorities COMMAND "${Python3_EXECUTABLE}"
    "${REPO}/tests/test_task_priorities.py")
# The two button owners are mutually exclusive build modes.
host_test(test_motion_task_bench tests/test_motion_task.c
    app_program/subsystem2/motion.c app_program/subsystem2/motor_ctl.c
    app_program/subsystem2/subsystem2.c)
if(TARGET test_motion_task_bench)
    target_include_directories(test_motion_task_bench BEFORE PRIVATE "${REPO}/tests/mtk_stubs")
    target_compile_definitions(test_motion_task_bench PRIVATE APP_BENCH=1)
endif()

host_test(test_vehicle_task tests/test_vehicle_task.c app_program/vehicle/vehicle_task.c
    app_program/vehicle/vehicle_log.c app_program/subsystem4/imu_hump.c ${vehicle_sm_source} ${line_position_source}
    ${line_trace_source} ${line_steer_source} ${line_search_source} ${obstacle_source}
    ${servo_scan_source} ${obstacle_profile_source} ${detour_run_source} ${obstacle_detour_source})
if(TARGET test_vehicle_task)
    target_include_directories(test_vehicle_task BEFORE PRIVATE "${REPO}/tests/mtk_stubs")
    target_link_libraries(test_vehicle_task PRIVATE m)
endif()

host_test(test_watchdog tests/test_watchdog.c app_program/platform/drv_watchdog.c)
if(TARGET test_watchdog)
    target_include_directories(test_watchdog BEFORE PRIVATE "${REPO}/tests/mtk_stubs")
    target_compile_definitions(test_watchdog PRIVATE APP_TEST=1)
endif()
host_test(test_watchdog_hw tests/test_watchdog_hw.c app_program/platform/drv_watchdog_hw.c)
if(TARGET test_watchdog_hw)
    target_include_directories(test_watchdog_hw BEFORE PRIVATE "${REPO}/tests/watchdog_stubs")
    target_compile_definitions(test_watchdog_hw PRIVATE APP_TEST=1)
endif()

if(TARGET test_motion_task)
    add_test(NAME test_motion_start_task_failure COMMAND test_motion_task task)
    add_test(NAME test_motion_start_cyclic_failure COMMAND test_motion_task cyclic)
    add_test(NAME test_motion_start_watchdog_failure COMMAND test_motion_task watchdog)
endif()
host_test(test_motion_watchdog tests/test_motion_task.c app_program/subsystem2/motion.c
    app_program/subsystem2/motor_ctl.c app_program/subsystem2/subsystem2.c)
if(TARGET test_motion_watchdog)
    target_include_directories(test_motion_watchdog BEFORE PRIVATE "${REPO}/tests/mtk_stubs")
    target_compile_definitions(test_motion_watchdog PRIVATE APP_TEST=1 APP_BENCH=1)
endif()

host_test(test_app_task tests/test_app_task.c app_program/platform/app_task.c)
if(TARGET test_app_task)
    target_include_directories(test_app_task BEFORE PRIVATE "${REPO}/tests/mtk_stubs")
endif()

host_test(test_line_service tests/test_line_service.c app_program/subsystem3/line_service.c ${line_position_source})
if(TARGET test_line_service)
    target_include_directories(test_line_service BEFORE PRIVATE "${REPO}/tests/mtk_stubs")
endif()

host_test(test_sonar_history tests/test_sonar_history.c)
if(TARGET test_sonar_history)
    target_include_directories(test_sonar_history BEFORE PRIVATE "${REPO}/tests/mtk_stubs")
endif()
host_test(test_sonar_service tests/test_sonar_service.c app_program/subsystem5/sonar_service.c)
if(TARGET test_sonar_service)
    target_include_directories(test_sonar_service BEFORE PRIVATE "${REPO}/tests/mtk_stubs")
    add_test(NAME test_sonar_cyclic_failure COMMAND test_sonar_service cyclic)
endif()

host_test(test_telemetry tests/test_telemetry.c app_program/subsystem1/telemetry.c)
if(TARGET test_telemetry)
    target_include_directories(test_telemetry BEFORE PRIVATE "${REPO}/tests/mtk_stubs")
endif()

if(TARGET test_vehicle_task)
    add_test(NAME test_vehicle_stale_line COMMAND test_vehicle_task stale)
    add_test(NAME test_vehicle_deadline_overrun COMMAND test_vehicle_task overrun)
    add_test(NAME test_vehicle_stale_sonar COMMAND test_vehicle_task sonarstale)
endif()

# TEST runner policy and buffered timing evidence; no real scheduler or hardware.
host_test(test_test_runner tests/test_test_runner.c app_program/app_main.c)
host_test(test_production_runner tests/test_test_runner.c app_program/app_main.c)
host_test(test_timing_test_hooks tests/test_timing_test_hooks.c
    app_program/subsystem1/subsystem1_test.c app_program/subsystem2/subsystem2_test.c)
foreach(test IN ITEMS test_test_runner test_production_runner test_timing_test_hooks)
    target_include_directories(${test} BEFORE PRIVATE "${REPO}/tests/mtk_stubs")
endforeach()
target_compile_definitions(test_test_runner PRIVATE APP_TEST=1)
target_compile_definitions(test_timing_test_hooks PRIVATE APP_TEST=1)
add_test(NAME test_test_runner_priority_failure COMMAND test_test_runner priority_failure)
add_test(NAME test_timing_window_threshold COMMAND test_timing_test_hooks bad_window)
include("${CMAKE_CURRENT_SOURCE_DIR}/imu_checks.cmake")

add_test(NAME test_imu_source_make COMMAND "${Python3_EXECUTABLE}"
    "${REPO}/tests/test_imu_source_make.py")
host_test(test_production_runner_filtered tests/test_test_runner.c
    app_program/app_main.c)
target_include_directories(test_production_runner_filtered BEFORE PRIVATE
    "${REPO}/tests/mtk_stubs")
target_compile_definitions(test_production_runner_filtered PRIVATE
    IMU_HUMP_SOURCE=IMU_HUMP_SOURCE_FILTERED)

host_test(test_imu_hump_diag tests/test_imu_hump_diag.c
    app_program/subsystem4/imu_hump.c
    app_program/subsystem4/imu_hump_diag.c
    app_program/subsystem4/imu_hump_diag_print.c)
target_include_directories(test_imu_hump_diag BEFORE PRIVATE
    "${REPO}/tests/mtk_stubs")
target_link_libraries(test_imu_hump_diag PRIVATE m)
foreach(test IN ITEMS test_imu_service test_imu_capture_service
        test_imu_stream test_vehicle_task test_vehicle_network_task)
    target_sources(${test} PRIVATE
        "${REPO}/app_program/subsystem4/imu_hump_diag.c")
endforeach()
foreach(test IN ITEMS test_imu_stream test_vehicle_task
        test_vehicle_network_task test_telemetry test_telemetry_log)
    target_sources(${test} PRIVATE
        "${REPO}/app_program/subsystem4/imu_hump_diag_print.c")
    target_link_libraries(${test} PRIVATE m)
endforeach()

set(junction_sources app_program/vehicle/junction_action.c
    app_program/vehicle/junction_source.c)
host_test(test_junction tests/test_junction.c ${junction_sources}
    app_program/vehicle/junction_line.c ${line_search_source}
    ${line_trace_source} ${line_steer_source} ${line_position_source}
    ${obstacle_source})
foreach(test IN ITEMS test_vehicle_task test_vehicle_network_task)
    foreach(source IN LISTS junction_sources)
        target_sources(${test} PRIVATE "${REPO}/${source}")
    endforeach()
    target_sources(${test} PRIVATE
        "${REPO}/app_program/vehicle/junction_line.c")
endforeach()
foreach(test IN ITEMS test_telemetry test_telemetry_log)
    foreach(source IN LISTS junction_sources)
        target_sources(${test} PRIVATE "${REPO}/${source}")
    endforeach()
    target_sources(${test} PRIVATE
        "${REPO}/app_program/vehicle/junction_print.c"
        "${REPO}/${line_search_source}" "${REPO}/${obstacle_source}")
endforeach()
add_test(NAME test_junction_make COMMAND "${Python3_EXECUTABLE}"
    "${REPO}/tests/test_junction_make.py")
get_target_property(vehicle_test_sources test_vehicle_task SOURCES)
set(junction_id 0)
foreach(command IN ITEMS A B C D)
    math(EXPR junction_id "${junction_id} + 1")
    set(test test_vehicle_junction_${command})
    add_executable(${test} ${vehicle_test_sources})
    target_include_directories(${test} BEFORE PRIVATE "${REPO}/tests/mtk_stubs")
    target_compile_definitions(${test} PRIVATE JUNCTION_TEST_CMD=${junction_id})
    target_link_libraries(${test} PRIVATE m)
    add_test(NAME ${test} COMMAND ${test} junction)
endforeach()
foreach(command A B C D)
    add_test(NAME test_vehicle_junction_${command}_estop
        COMMAND test_vehicle_junction_${command} junctionstop)
endforeach()
add_test(NAME test_vehicle_junction_start_busy
    COMMAND test_vehicle_junction_A junctionbusy)
add_test(NAME test_vehicle_junction_start_stale
    COMMAND test_vehicle_junction_A junctionstale)

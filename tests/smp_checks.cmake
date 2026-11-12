# M1: contract tests for the host double, and both placement seams.
# M8: startup publication and early MQTT entry in all placement profiles.
host_test(test_smp_double tests/test_smp_double.c)
target_include_directories(test_smp_double BEFORE PRIVATE
    "${REPO}/tests/mtk_stubs")
foreach(scenario locks nested atomics wait invalid)
    add_test(NAME test_smp_double_${scenario}
        COMMAND test_smp_double ${scenario})
endforeach()

# Copy the completed target, including late-added sources and fixture flags.
# Only placement changes; no existing scenario or assertion is removed.
function(placement_variant original suffix)
    set(target "${original}_${suffix}")
    get_target_property(sources ${original} SOURCES)
    add_executable(${target} ${sources})
    foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS
            LINK_LIBRARIES)
        get_target_property(value ${original} ${property})
        if(value)
            set_property(TARGET ${target} PROPERTY ${property} "${value}")
        endif()
    endforeach()
    if(suffix STREQUAL "up")
        target_compile_definitions(${target} PRIVATE TK_SUPPORT_SMP=0)
    else()
        target_compile_definitions(${target} PRIVATE APP_FORCE_CORE0=1)
    endif()
    add_test(NAME ${target} COMMAND ${target})
endfunction()

foreach(mode normal bench mqtt test)
    placement_variant(test_app_task_${mode} control)
    placement_variant(test_app_task_${mode} up)
endforeach()
foreach(net 0 1)
    set(original "test_template_wifi_smp1_net${net}")
    placement_variant(${original} control)
    add_test(NAME ${original}_control_wrap COMMAND ${original}_control wrap)
endforeach()
foreach(target test_imu_service test_imu_capture_service test_telemetry
        test_telemetry_log)
    placement_variant(${target} control)
endforeach()
foreach(failure create_failure start_failure cyclic_failure count_wrap
        fifo_empty)
    add_test(NAME test_imu_service_control_${failure}
        COMMAND test_imu_service_control ${failure})
endforeach()
add_test(NAME test_imu_capture_service_control_handoff
    COMMAND test_imu_capture_service_control capture)

foreach(placement up control)
    foreach(target test_app_startup test_production_runner test_test_runner
            test_imu_stream_runner test_startup_mqtt_runner test_robot_mqtt)
        placement_variant(${target} ${placement})
    endforeach()
    set_tests_properties(test_app_startup_${placement} PROPERTIES TIMEOUT 10)
    add_test(NAME test_startup_provider_failure_${placement}
        COMMAND test_production_runner_${placement} startup_failure)
    add_test(NAME test_startup_mqtt_provider_failure_${placement}
        COMMAND test_startup_mqtt_runner_${placement} startup_failure)
    add_test(NAME test_robot_mqtt_startup_${placement}
        COMMAND test_robot_mqtt_${placement} startup)
    add_test(NAME test_robot_mqtt_no_boot_id_${placement}
        COMMAND test_robot_mqtt_${placement} no_boot_id)
endforeach()

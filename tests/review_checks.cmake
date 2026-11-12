add_test(NAME test_sdk_profiles COMMAND "${Python3_EXECUTABLE}"
    "${REPO}/tests/test_sdk_profiles.py")

# M4: never mutate the live stream from the TEST runner, at any placement.
get_target_property(probe_sources test_telemetry SOURCES)
list(REMOVE_ITEM probe_sources "${REPO}/tests/test_telemetry.c")
foreach(placement up split control)
    set(smp 1)
    set(force 0)
    if(placement STREQUAL "up")
        set(smp 0)
    elseif(placement STREQUAL "control")
        set(force 1)
    endif()
    foreach(mqtt 0 1)
        set(target "test_stream_runner_${placement}_mqtt${mqtt}")
        host_test(${target} tests/test_log_stream_selftest.c
            app_program/subsystem1/log_stream_selftest.c)
        target_include_directories(${target} BEFORE PRIVATE
            "${REPO}/tests/mtk_stubs")
        target_compile_definitions(${target} PRIVATE APP_TEST=1 TM_MQTT=${mqtt}
            TK_SUPPORT_SMP=${smp} TK_MAX_CORE=2 APP_FORCE_CORE0=${force})
    endforeach()
    set(target "test_telemetry_probe_${placement}")
    add_executable(${target} test_telemetry_probe.c ${probe_sources})
    target_include_directories(${target} BEFORE PRIVATE
        "${REPO}/tests/mtk_stubs")
    target_compile_definitions(${target} PRIVATE APP_TEST=1 TM_MQTT=1
        TK_SUPPORT_SMP=${smp} TK_MAX_CORE=2 APP_FORCE_CORE0=${force})
    target_link_libraries(${target} PRIVATE m)
    add_test(NAME ${target} COMMAND ${target})

    set(target "test_storage_probe_race_${placement}")
    host_test(${target} tests/test_storage_probe_race.c)
    target_include_directories(${target} BEFORE PRIVATE
        "${REPO}/tests/mtk_stubs")
    target_compile_definitions(${target} PRIVATE APP_TEST=1 TM_MQTT=0
        TK_SUPPORT_SMP=${smp} TK_MAX_CORE=2 APP_FORCE_CORE0=${force})
    foreach(scenario immediate noise reset)
        add_test(NAME "${target}_${scenario}" COMMAND ${target} ${scenario})
    endforeach()
endforeach()

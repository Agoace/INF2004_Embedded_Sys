# M4: never mutate the live stream from the TEST runner, at any placement.
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
endforeach()

host_test(test_app_board tests/test_app_board.c)
host_test(test_network_queue tests/test_network_queue.c app_program/subsystem1/network_queue.c)
if(TARGET test_network_queue)
    target_include_directories(test_network_queue BEFORE PRIVATE "${REPO}/tests/network_stubs")
    foreach(failure 1 2 3)
        add_test(NAME test_network_queue_init_failure_${failure} COMMAND test_network_queue ${failure})
    endforeach()
endif()
# M2: instrument only this translation unit, never production sources.
foreach(smp 0 1)
    set(mask_test "test_network_mask_smp${smp}")
    host_test(${mask_test} tests/test_network_mask.c)
    target_include_directories(${mask_test} BEFORE PRIVATE
        "${REPO}/tests/network_stubs")
    target_compile_definitions(${mask_test} PRIVATE TK_SUPPORT_SMP=${smp})
    foreach(scenario RANGE 0 5)
        add_test(NAME "${mask_test}_${scenario}"
            COMMAND ${mask_test} ${scenario})
    endforeach()
endforeach()
host_test(test_telemetry_log tests/test_telemetry_log.c)
if(TARGET test_telemetry_log)
    target_include_directories(test_telemetry_log BEFORE PRIVATE "${REPO}/tests/mtk_stubs")
    target_compile_definitions(test_telemetry_log PRIVATE TM_MQTT=1)
endif()
host_test(test_telemetry_mode tests/test_telemetry_mode.c
    app_program/subsystem1/telemetry_mode.c)
host_test(test_log_stream tests/test_log_stream.c
    app_program/subsystem1/log_stream.c app_program/subsystem1/mqtt_wire.c)
if(TARGET test_log_stream)
    target_include_directories(test_log_stream BEFORE PRIVATE "${REPO}/tests/mqtt_stubs")
    target_compile_definitions(test_log_stream PRIVATE TM_MQTT=1)
    foreach(scenario two_boot_ids live_batches live_steady live_jump_lag live_jump_reconnect
            live_generation live_failures dump_flow dump_complete dump_failures
            dump_link_down dump_abort dump_throughput shared_budget live_paused)
        add_test(NAME test_log_stream_${scenario} COMMAND test_log_stream ${scenario})
    endforeach()
endif()

host_test(test_imu_fifo tests/test_imu_fifo.c
    app_program/subsystem4/imu_fifo.c)
target_link_libraries(test_imu_fifo PRIVATE m)
host_test(test_imu_capture tests/test_imu_capture.c
    app_program/subsystem4/imu_capture.c)
host_test(test_imu_support tests/test_imu_support.c
    app_program/subsystem4/imu_support.c)
target_link_libraries(test_imu_support PRIVATE m)
host_test(test_imu_drive tests/test_imu_drive.c
    app_program/subsystem4/imu_drive.c)
target_link_libraries(test_imu_drive PRIVATE m)
foreach(window 15 20 31)
    host_test(test_imu_roll_${window} tests/test_imu_roll.c
        app_program/subsystem4/imu_roll.c)
    target_compile_definitions(test_imu_roll_${window} PRIVATE
        IMU_ROLL_PLATEAU_SAMPLES=${window}u)
    target_link_libraries(test_imu_roll_${window} PRIVATE m)
endforeach()
host_test(test_imu_core tests/test_imu_core.c
    app_program/subsystem4/imu_core.c app_program/subsystem4/imu_support.c
    app_program/subsystem4/imu_roll.c)
target_link_libraries(test_imu_core PRIVATE m)
host_test(test_imu_core_benchmark tests/test_imu_core_benchmark.c
    app_program/subsystem4/imu_core.c app_program/subsystem4/imu_support.c
    app_program/subsystem4/imu_roll.c)
target_compile_options(test_imu_core_benchmark PRIVATE -Os)
target_link_libraries(test_imu_core_benchmark PRIVATE m)
host_test(test_imu_service tests/test_imu_service.c
    app_program/subsystem4/imu_service.c app_program/subsystem4/imu_core.c
    app_program/subsystem4/imu_support.c app_program/subsystem4/imu_roll.c)
target_include_directories(test_imu_service BEFORE PRIVATE
    "${REPO}/tests/mtk_stubs")
target_compile_definitions(test_imu_service PRIVATE APP_TEST=1 APP_BENCH=0)
target_compile_options(test_imu_service PRIVATE
    -include "${REPO}/tests/imu_service_hook.h")
target_link_libraries(test_imu_service PRIVATE m)
host_test(test_imu_capture_service tests/test_imu_service.c
    app_program/subsystem4/imu_service.c app_program/subsystem4/imu_core.c
    app_program/subsystem4/imu_support.c app_program/subsystem4/imu_roll.c
    app_program/subsystem4/imu_capture.c)
target_include_directories(test_imu_capture_service BEFORE PRIVATE
    "${REPO}/tests/mtk_stubs")
target_compile_definitions(test_imu_capture_service PRIVATE
    APP_TEST=1 APP_BENCH=0 APP_IMU_STREAM=1)
target_compile_options(test_imu_capture_service PRIVATE
    -include "${REPO}/tests/imu_service_hook.h")
target_link_libraries(test_imu_capture_service PRIVATE m)
add_test(NAME test_imu_capture_service_handoff
    COMMAND test_imu_capture_service capture)
foreach(failure create_failure start_failure cyclic_failure count_wrap
        fifo_empty)
    add_test(NAME test_imu_service_${failure}
        COMMAND test_imu_service ${failure})
endforeach()
host_test(test_imu_bench tests/test_imu_bench.c
    app_program/subsystem4/imu_service.c)
target_include_directories(test_imu_bench BEFORE PRIVATE
    "${REPO}/tests/mtk_stubs")
target_compile_definitions(test_imu_bench PRIVATE APP_BENCH=1)
host_test(test_imu_stream tests/test_imu_stream.c
    app_program/subsystem4/subsystem4_test.c app_program/subsystem4/imu_mark.c
    app_program/subsystem4/imu_drive.c)
target_include_directories(test_imu_stream BEFORE PRIVATE
    "${REPO}/tests/mtk_stubs")
target_compile_definitions(test_imu_stream PRIVATE
    APP_TEST=1 APP_BENCH=0 APP_IMU_STREAM=1)
target_link_libraries(test_imu_stream PRIVATE m)
foreach(failure create_failure start_failure stop_failure)
    add_test(NAME test_imu_stream_${failure}
        COMMAND test_imu_stream ${failure})
endforeach()
foreach(scenario distance timeout button fault mag_fault unavailable motor_fault
                 command_fault wake_fault stale double_stop double_cleanup
                 held_boot)
    add_test(NAME test_imu_drive_stream_${scenario}
        COMMAND test_imu_stream drive_${scenario})
endforeach()
host_test(test_imu_decode tests/test_imu_decode.c
    app_program/subsystem4/imu_decode.c)
target_link_libraries(test_imu_decode PRIVATE m)
host_test(test_imu_probe tests/test_imu_probe.c
    app_program/subsystem4/imu_probe.c app_program/subsystem4/imu_decode.c
    app_program/subsystem4/imu_fifo.c)
target_link_libraries(test_imu_probe PRIVATE m)
host_test(test_imu_hw tests/test_imu_hw.c app_program/subsystem4/imu_hw.c)
target_include_directories(test_imu_hw BEFORE PRIVATE
    "${REPO}/tests/imu_stubs")
target_compile_definitions(test_imu_hw PRIVATE
    APP_TEST=1 BOARD_UART0_FREE=1 TM_CONSOLE_UART=0)
host_test(test_imu_mark tests/test_imu_mark.c
    app_program/subsystem4/imu_mark.c)
host_test(test_imu_stream_runner tests/test_test_runner.c app_program/app_main.c)
target_include_directories(test_imu_stream_runner BEFORE PRIVATE
    "${REPO}/tests/mtk_stubs")
target_compile_definitions(test_imu_stream_runner PRIVATE
    APP_TEST=1 APP_IMU_STREAM=1)
add_test(NAME test_imu_stream_priority_failure
    COMMAND test_imu_stream_runner priority_failure)

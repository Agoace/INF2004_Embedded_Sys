host_test(test_barcode tests/test_barcode.c
    app_program/subsystem3/barcode.c)
target_link_libraries(test_barcode PRIVATE m)
host_test(test_barcode_capture tests/test_barcode_capture.c
    app_program/subsystem3/barcode_capture.c)
target_compile_definitions(test_barcode_capture PRIVATE APP_BARCODE_CAPTURE=1)
host_test(test_barcode_runner tests/test_test_runner.c app_program/app_main.c)
target_include_directories(test_barcode_runner BEFORE PRIVATE
    "${REPO}/tests/mtk_stubs")
target_compile_definitions(test_barcode_runner PRIVATE
    APP_TEST=1 APP_BARCODE_CAPTURE=1)
add_test(NAME test_barcode_priority_failure
    COMMAND test_barcode_runner priority_failure)
host_test(test_barcode_test tests/test_barcode_test.c
    app_program/subsystem3/barcode_test.c
    app_program/subsystem3/barcode_capture.c app_program/subsystem3/barcode.c
    app_program/subsystem3/barcode_drive.c)
target_include_directories(test_barcode_test BEFORE PRIVATE
    "${REPO}/tests/barcode_stubs" "${REPO}/tests/mtk_stubs")
target_compile_definitions(test_barcode_test PRIVATE
    APP_TEST=1 APP_BARCODE_CAPTURE=1 APP_BENCH=0 APP_IMU_STREAM=0 TM_MQTT=0)
target_link_libraries(test_barcode_test PRIVATE m)
foreach(case IN ITEMS button_stop stop_retry motion_fault sensor_fault
        command_fail missing_motor stale_motor acquisition_loss wake_fault
        missed_wake moving_start timeout capacity held)
    add_test(NAME test_barcode_${case} COMMAND test_barcode_test ${case})
endforeach()
host_test(test_barcode_packed tests/test_barcode_packed.c
    app_program/subsystem3/barcode.c)
target_link_libraries(test_barcode_packed PRIVATE m)
host_test(test_barcode_hw tests/test_barcode_hw.c
    app_program/subsystem3/barcode_hw.c)
target_include_directories(test_barcode_hw BEFORE PRIVATE
    "${REPO}/tests/barcode_stubs")
target_compile_definitions(test_barcode_hw PRIVATE APP_BARCODE_CAPTURE=1)
host_test(test_barcode_drive tests/test_barcode_drive.c
    app_program/subsystem3/barcode_drive.c)
host_test(test_barcode_drive_clamp tests/test_barcode_drive.c
    app_program/subsystem3/barcode_drive.c)
target_compile_definitions(test_barcode_drive_clamp PRIVATE
    BARCODE_PASS_DISTANCE_MM=850u EXPECT_CLAMP=1)
host_test(test_barcode_line_copy tests/test_line_service.c
    app_program/subsystem3/line_service.c
    app_program/subsystem3/line_position.c
    app_program/subsystem3/barcode_capture.c)
target_include_directories(test_barcode_line_copy BEFORE PRIVATE
    "${REPO}/tests/mtk_stubs")
target_compile_definitions(test_barcode_line_copy PRIVATE
    APP_TEST=1 APP_BARCODE_CAPTURE=1)

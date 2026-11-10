host_test(test_barcode_live_source tests/test_barcode_live_source.c
    app_program/subsystem3/barcode_live_source.c)
target_include_directories(test_barcode_live_source BEFORE PRIVATE
    "${REPO}/tests/mtk_stubs")
target_compile_definitions(test_barcode_live_source PRIVATE
    BC_LIVE_SOURCE_HOST_TEST=1)

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
    app_program/subsystem3/barcode_drive.c
    app_program/subsystem3/barcode_peak.c)
target_include_directories(test_barcode_test BEFORE PRIVATE
    "${REPO}/tests/barcode_stubs" "${REPO}/tests/mtk_stubs")
target_compile_definitions(test_barcode_test PRIVATE
    APP_TEST=1 APP_BARCODE_CAPTURE=1 APP_BENCH=0 APP_IMU_STREAM=0 TM_MQTT=0
    TM_CONSOLE_USB_CDC=1)
target_link_libraries(test_barcode_test PRIVATE m)
foreach(case IN ITEMS button_stop stop_retry motion_fault sensor_fault
        command_fail missing_motor stale_motor acquisition_loss wake_fault
        missed_wake moving_start timeout capacity held
        line_busy line_busy_twice line_busy_start line_age_jitter line_stale
        line_invalid line_raw line_read_error motor_busy motor_busy_twice
        motor_read_error watchdog_start watchdog_lost line_stale_start
        dump_press dump_held motor_busy_phase
        usb_offline usb_wait usb_reconnect usb_held usb_wrap)
    add_test(NAME test_barcode_${case} COMMAND test_barcode_test ${case})
endforeach()
host_test(test_barcode_packed tests/test_barcode_packed.c
    app_program/subsystem3/barcode.c)
target_link_libraries(test_barcode_packed PRIVATE m)
host_test(test_barcode_real tests/test_barcode_real.c
    app_program/subsystem3/barcode.c)
target_link_libraries(test_barcode_real PRIVATE m)
host_test(test_barcode_blur tests/test_barcode_blur.c
    app_program/subsystem3/barcode.c)
target_link_libraries(test_barcode_blur PRIVATE m)
target_compile_definitions(test_barcode_real PRIVATE
    BC_FIXTURE="${REPO}/tests/fixtures/barcode_large_f_forward.csv")
host_test(test_barcode_peak tests/test_barcode_peak.c
    app_program/subsystem3/barcode.c app_program/subsystem3/barcode_peak.c)
target_link_libraries(test_barcode_peak PRIVATE m)
host_test(test_barcode_peak_real tests/test_barcode_peak_real.c
    app_program/subsystem3/barcode.c app_program/subsystem3/barcode_peak.c)
target_link_libraries(test_barcode_peak_real PRIVATE m)
target_compile_definitions(test_barcode_peak_real PRIVATE
    BC_FIXTURES="${REPO}/tests/fixtures")
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
    app_program/subsystem3/barcode_calibration.c
    app_program/subsystem3/line_position.c
    app_program/subsystem3/barcode_capture.c)
target_include_directories(test_barcode_line_copy BEFORE PRIVATE
    "${REPO}/tests/mtk_stubs")
target_compile_definitions(test_barcode_line_copy PRIVATE
    APP_TEST=1 APP_BARCODE_CAPTURE=1)

# Keep the common LINE owner linked in both normal and capture host profiles.
if(TARGET test_line_service)
    target_sources(test_line_service PRIVATE
        "${REPO}/app_program/subsystem3/barcode_calibration.c")
endif()
host_test(test_barcode_calibration tests/test_barcode_calibration.c
    app_program/subsystem3/barcode_calibration.c)
foreach(case IN ITEMS white_only air dark spike white_short black_short bounds
        aborted no_data gap duplicate reverse_stamp wrap clipping capacity
        restart terminal dark_line cap)
    add_test(NAME test_barcode_calibration_${case}
        COMMAND test_barcode_calibration ${case})
endforeach()

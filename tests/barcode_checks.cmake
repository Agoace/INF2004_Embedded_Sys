add_test(NAME test_barcode_fail_analyze COMMAND ${Python3_EXECUTABLE} "${REPO}/tests/test_barcode_fail_analyze.py")
add_test(NAME test_barcode_session_report COMMAND ${Python3_EXECUTABLE} "${REPO}/tests/test_barcode_session_report.py")
add_test(NAME test_barcode_fail_make COMMAND ${Python3_EXECUTABLE} "${REPO}/tests/test_barcode_fail_make.py")
host_test(test_barcode_fail_capture tests/test_barcode_fail_capture.c)
target_include_directories(test_barcode_fail_capture BEFORE PRIVATE "${REPO}/tests/mtk_stubs")
target_compile_definitions(test_barcode_fail_capture PRIVATE BARCODE_FAIL_CAPTURE=1)
target_link_libraries(test_barcode_fail_capture PRIVATE m)
host_test(test_barcode_fail_capture_one tests/test_barcode_fail_capture.c)
target_include_directories(test_barcode_fail_capture_one BEFORE PRIVATE "${REPO}/tests/mtk_stubs")
target_compile_definitions(test_barcode_fail_capture_one PRIVATE BARCODE_FAIL_CAPTURE=1 BCFC_FAILED_SLOTS=1)
target_link_libraries(test_barcode_fail_capture_one PRIVATE m)
host_test(test_barcode_fail_feed tests/test_barcode_live_feed.c
    app_program/subsystem3/barcode_attempt.c
    app_program/subsystem3/barcode_live_source.c
    app_program/subsystem3/barcode_live_producer.c
    app_program/subsystem3/barcode_window.c app_program/subsystem3/barcode.c)
target_include_directories(test_barcode_fail_feed BEFORE PRIVATE "${REPO}/tests/mtk_stubs")
target_compile_definitions(test_barcode_fail_feed PRIVATE
    BC_LIVE_COUNTS_HOST_TEST=1 TM_MQTT=1 BARCODE_FAIL_CAPTURE=1)
target_link_libraries(test_barcode_fail_feed PRIVATE m)
foreach(scenario start_fail counts_cancel counts_epoch race)
    add_test(NAME test_barcode_fail_feed_${scenario} COMMAND test_barcode_fail_feed ${scenario})
endforeach()
host_test(test_barcode_wire tests/test_barcode_wire.c app_program/subsystem1/mqtt_wire.c)
host_test(test_barcode_attempt tests/test_barcode_attempt.c app_program/subsystem3/barcode_attempt.c)
target_compile_definitions(test_barcode_attempt PRIVATE TM_MQTT=1)
add_test(NAME test_barcode_v2_python COMMAND ${Python3_EXECUTABLE} "${REPO}/tests/test_barcode_v2.py")

host_test(test_barcode_live_feed tests/test_barcode_live_feed.c
    app_program/subsystem3/barcode_attempt.c
    app_program/subsystem3/barcode_live_source.c
    app_program/subsystem3/barcode_live_producer.c
    app_program/subsystem3/barcode_window.c app_program/subsystem3/barcode.c)
target_include_directories(test_barcode_live_feed BEFORE PRIVATE
    "${REPO}/tests/mtk_stubs")
target_link_libraries(test_barcode_live_feed PRIVATE m)
target_compile_definitions(test_barcode_live_feed PRIVATE BC_LIVE_COUNTS_HOST_TEST=1 TM_MQTT=1)
add_test(NAME test_barcode_live_feed_race COMMAND test_barcode_live_feed race)
add_test(NAME test_barcode_live_feed_counts_epoch COMMAND test_barcode_live_feed counts_epoch)
add_test(NAME test_barcode_live_feed_counts_cancel COMMAND test_barcode_live_feed counts_cancel)
add_test(NAME test_barcode_live_feed_none COMMAND test_barcode_live_feed none)
# HOST ONLY live feed: custom A=LEFT map id2, mode2, synthetic clear bound.
host_test(test_barcode_live_feed_live tests/test_barcode_live_feed.c
    app_program/subsystem3/barcode_attempt.c
    app_program/subsystem3/barcode_live_source.c
    app_program/subsystem3/barcode_live_producer.c
    app_program/subsystem3/barcode_window.c app_program/subsystem3/barcode.c)
target_include_directories(test_barcode_live_feed_live BEFORE PRIVATE
    "${REPO}/tests/mtk_stubs")
target_link_libraries(test_barcode_live_feed_live PRIVATE m)
target_compile_definitions(test_barcode_live_feed_live PRIVATE
    BC_LIVE_COUNTS_HOST_TEST=1 TM_MQTT=1 BARCODE_LIVE_ENABLE=1
    BARCODE_MEASURED_CLEAR_MM=1
    BW_MAP_VALUES=1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
    BW_MAP_POLICY_ID=2u)
foreach(case IN ITEMS normal race counts_epoch counts_cancel none)
    add_test(NAME test_barcode_live_feed_live_${case}
        COMMAND test_barcode_live_feed_live ${case})
endforeach()

host_test(test_barcode_live_producer tests/test_barcode_live_producer.c
    app_program/subsystem3/barcode_live_producer.c
    app_program/subsystem3/barcode_window.c app_program/subsystem3/barcode.c)
target_link_libraries(test_barcode_live_producer PRIVATE m)
foreach(case IN ITEMS reverse fallback mode2 no_progress stale pause epoch
        revision cut_change cut_equal disabled loss missed generation
        motor_fault motor_old raw_clip clock read_fail ack no_ack busy_ack
        us_wrap ms_wrap count_wrap old_epoch_ack old_epoch_no_ack sequence_wrap
        nonletter mode2_ack)
    add_test(NAME test_barcode_producer_${case}
        COMMAND test_barcode_live_producer ${case})
endforeach()

# HOST ONLY live-gate variants (synthetic clear bound; never a firmware build).
# custom: A=LEFT,B=RIGHT id2 -> mode2 acts. builtin: id1 never acts.
# none: A=NONE,B=RIGHT id2 -> unmapped A stays SHADOW, B acts.
foreach(variant IN ITEMS custom builtin none)
    set(target test_barcode_live_producer_live_${variant})
    host_test(${target} tests/test_barcode_live_producer.c
        app_program/subsystem3/barcode_live_producer.c
        app_program/subsystem3/barcode_window.c app_program/subsystem3/barcode.c)
    target_link_libraries(${target} PRIVATE m)
    target_compile_definitions(${target} PRIVATE BARCODE_LIVE_ENABLE=1
        BARCODE_MEASURED_CLEAR_MM=1)
    foreach(case IN ITEMS normal mode2 mode2_ack ack revision cut_change epoch
            disabled nonletter)
        add_test(NAME test_barcode_producer_live_${variant}_${case}
            COMMAND ${target} ${case})
    endforeach()
endforeach()
target_compile_definitions(test_barcode_live_producer_live_custom PRIVATE
    BW_MAP_VALUES=1,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
    BW_MAP_POLICY_ID=2u)
target_compile_definitions(test_barcode_live_producer_live_none PRIVATE
    BW_MAP_VALUES=0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
    BW_MAP_POLICY_ID=2u)

host_test(test_barcode_quiet_real tests/test_barcode_quiet_real.c
    app_program/subsystem3/barcode_window.c app_program/subsystem3/barcode.c)
target_compile_definitions(test_barcode_quiet_real PRIVATE BC_FIXTURES="${REPO}/tests/fixtures")
target_link_libraries(test_barcode_quiet_real PRIVATE m)

host_test(test_barcode_motion_twitch tests/test_barcode_motion_twitch.c
    app_program/subsystem3/barcode_window.c app_program/subsystem3/barcode.c)
target_compile_definitions(test_barcode_motion_twitch PRIVATE BC_FIXTURES="${REPO}/tests/fixtures")
target_link_libraries(test_barcode_motion_twitch PRIVATE m)
foreach(case IN ITEMS geometry straight twitch_star twitch_edge kr retreat reverse stall)
    add_test(NAME test_barcode_motion_${case} COMMAND test_barcode_motion_twitch ${case})
endforeach()

host_test(test_barcode_window_real tests/test_barcode_window_real.c
    app_program/subsystem3/barcode_window.c app_program/subsystem3/barcode.c)
target_compile_definitions(test_barcode_window_real PRIVATE
    BC_FIXTURES="${REPO}/tests/fixtures")
target_link_libraries(test_barcode_window_real PRIVATE m)

foreach(window_target IN ITEMS test_barcode_window test_barcode_window_map
        test_barcode_window_invalid_map test_barcode_window_unconfirmed)
    host_test(${window_target} tests/test_barcode_window.c
        app_program/subsystem3/barcode_window.c
        app_program/subsystem3/barcode.c)
    target_link_libraries(${window_target} PRIVATE m)
endforeach()
target_compile_definitions(test_barcode_window_map PRIVATE
    BW_MAP_VALUES=1,2,3,4,0,3,4,1,2,3,4,0,1,2,3,4,0,1,2,3,4,0,1,2,3,4
    BW_MAP_POLICY_ID=2u)
target_compile_definitions(test_barcode_window_invalid_map PRIVATE
    BW_MAP_VALUES=255 BW_MAP_POLICY_ID=3u)
target_compile_definitions(test_barcode_window_unconfirmed PRIVATE
    BW_MAP_CONFIRMED=0)
foreach(case IN ITEMS sizes normal variable wrap two_ms freeze tracking short_quiet
        immutable missing merged calibration queue brackets leading short_tail
        gap extra multi small air reversal motor_fault motor_gap sample_gap
        adc_fault paper flat_dark line_only alias one_missing)
    add_test(NAME test_barcode_window_${case}
        COMMAND test_barcode_window ${case})
endforeach()

foreach(absent_profile IN ITEMS test bench)
    host_test(test_barcode_counts_absent_${absent_profile}
        tests/test_barcode_live_counts_absent.c
        app_program/subsystem3/barcode_live_source.c)
    target_include_directories(test_barcode_counts_absent_${absent_profile} BEFORE PRIVATE
        "${REPO}/tests/mtk_stubs")
endforeach()
target_compile_definitions(test_barcode_counts_absent_test PRIVATE APP_TEST=1)
target_compile_definitions(test_barcode_counts_absent_bench PRIVATE APP_BENCH=1)

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
    app_program/subsystem3/barcode_window.c
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
    app_program/subsystem3/barcode_live_source.c
    app_program/subsystem3/line_position.c
    app_program/subsystem3/barcode_capture.c)
target_include_directories(test_barcode_line_copy BEFORE PRIVATE
    "${REPO}/tests/mtk_stubs")
target_compile_definitions(test_barcode_line_copy PRIVATE
    APP_TEST=1 APP_BARCODE_CAPTURE=1)

# Keep the common LINE owner linked in both normal and capture host profiles.
if(TARGET test_line_service)
    target_sources(test_line_service PRIVATE
        "${REPO}/app_program/subsystem3/barcode_calibration.c"
        "${REPO}/app_program/subsystem3/barcode_attempt.c"
        "${REPO}/app_program/subsystem3/barcode_live_source.c"
        "${REPO}/app_program/subsystem3/barcode_live_producer.c"
        "${REPO}/app_program/subsystem3/barcode_window.c"
        "${REPO}/app_program/subsystem3/barcode.c")
    target_link_libraries(test_line_service PRIVATE m)
endif()
host_test(test_barcode_calibration tests/test_barcode_calibration.c
    app_program/subsystem3/barcode_calibration.c)
foreach(case IN ITEMS white_only air dark spike white_short black_short bounds
        aborted no_data gap duplicate reverse_stamp wrap clipping capacity
        restart terminal dark_line cap)
    add_test(NAME test_barcode_calibration_${case}
        COMMAND test_barcode_calibration ${case})
endforeach()

# HOST ONLY end-to-end S3 producer -> vehicle jlive -> ja. Live variants use
# a synthetic 1 mm clear bound; they are not firmware builds.
foreach(variant IN ITEMS default custom builtin expire search
        decode_grace decode_timeout decode_end)
    set(target test_barcode_live_e2e_${variant})
    set(action app_program/barcode_live_vehicle/action.c)
    if(variant STREQUAL "default")
        set(action app_program/vehicle/junction_action.c)
    endif()
    host_test(${target} tests/test_barcode_live_e2e.c
        app_program/barcode_live_vehicle/policy.c
        app_program/barcode_live_vehicle/hook.c
        app_program/subsystem3/barcode_attempt.c
        app_program/subsystem3/barcode_live_source.c
        app_program/subsystem3/barcode_live_producer.c
        app_program/subsystem3/barcode_window.c app_program/subsystem3/barcode.c
        app_program/vehicle/junction_source.c app_program/vehicle/junction_live.c
        ${action}
        ${line_search_source} ${obstacle_source})
    target_include_directories(${target} BEFORE PRIVATE "${REPO}/tests/mtk_stubs")
    target_link_libraries(${target} PRIVATE m)
    target_compile_definitions(${target} PRIVATE BC_LIVE_COUNTS_HOST_TEST=1
        TM_MQTT=1)
endforeach()
target_compile_definitions(test_barcode_live_e2e_custom PRIVATE
    BARCODE_LIVE_ENABLE=1 BARCODE_MEASURED_CLEAR_MM=1 E2E_EXPECT_OFFER=1
    E2E_POLICY=3
    BW_MAP_VALUES=0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
    BW_MAP_POLICY_ID=2u)
# V1/V3 live policy: J1 beyond 400 mm from the decode, or SEARCH first.
foreach(variant IN ITEMS expire search)
    target_compile_definitions(test_barcode_live_e2e_${variant} PRIVATE
        BARCODE_LIVE_ENABLE=1 BARCODE_MEASURED_CLEAR_MM=1 E2E_EXPECT_OFFER=1
        BW_MAP_VALUES=0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
        BW_MAP_POLICY_ID=2u)
endforeach()
target_compile_definitions(test_barcode_live_e2e_expire PRIVATE
    J1_MM=850.0f E2E_POLICY=1)
target_compile_definitions(test_barcode_live_e2e_search PRIVATE
    E2E_POLICY=2)
host_test(test_blv_policy tests/test_blv_policy.c
    app_program/barcode_live_vehicle/policy.c
    app_program/barcode_live_vehicle/hook.c
    app_program/vehicle/junction_action.c
    ${line_search_source} ${obstacle_source})
target_link_libraries(test_blv_policy PRIVATE m)
add_test(NAME test_barcode_live_vehicle_make
    COMMAND ${Python3_EXECUTABLE} ${REPO}/tests/test_barcode_live_vehicle_make.py)
target_compile_definitions(test_barcode_live_e2e_builtin PRIVATE
    BARCODE_LIVE_ENABLE=1 BARCODE_MEASURED_CLEAR_MM=1 E2E_LETTER='A')

host_test(test_blv_turn tests/test_blv_turn.c
    app_program/barcode_live_vehicle/action.c
    app_program/barcode_live_vehicle/line.c
    ${line_search_source} ${obstacle_source})
target_link_libraries(test_blv_turn PRIVATE m)

# Live-only control collision regression; initially red against shared producer.
host_test(test_barcode_control_cache tests/test_barcode_control_cache.c
    app_program/subsystem3/barcode_live_producer.c
    app_program/subsystem3/barcode_window.c app_program/subsystem3/barcode.c)
target_compile_definitions(test_barcode_control_cache PRIVATE
    BARCODE_LIVE_ENABLE=1 BARCODE_MEASURED_CLEAR_MM=1)
target_link_libraries(test_barcode_control_cache PRIVATE m)

# Compile shared producer with the SAME forced-include seam as live firmware.
add_library(barcode_control_raw OBJECT "${REPO}/app_program/subsystem3/barcode_live_producer.c")
target_compile_definitions(barcode_control_raw PRIVATE BARCODE_LIVE_ENABLE=1 BARCODE_MEASURED_CLEAR_MM=1)
target_compile_options(barcode_control_raw PRIVATE -include "${REPO}/app_program/barcode_live_vehicle/producer_impl.h")
foreach(t IN ITEMS test_barcode_control_cache test_barcode_live_feed_live
        test_barcode_live_producer_live_custom test_barcode_live_producer_live_builtin
        test_barcode_live_producer_live_none test_barcode_live_e2e_custom
        test_barcode_live_e2e_builtin test_barcode_live_e2e_expire test_barcode_live_e2e_search
        test_barcode_live_e2e_decode_grace test_barcode_live_e2e_decode_timeout
        test_barcode_live_e2e_decode_end)
    get_target_property(srcs ${t} SOURCES)
    list(REMOVE_ITEM srcs "${REPO}/app_program/subsystem3/barcode_live_producer.c")
    set_property(TARGET ${t} PROPERTY SOURCES "${srcs}")
    target_sources(${t} PRIVATE $<TARGET_OBJECTS:barcode_control_raw>
        "${REPO}/app_program/barcode_live_vehicle/control.c")
    target_compile_definitions(${t} PRIVATE TEST_CONTROL_CACHE=1)
endforeach()
foreach(v IN ITEMS custom builtin none)
    add_test(NAME test_barcode_producer_live_${v}_read_fail
        COMMAND test_barcode_live_producer_live_${v} read_fail)
endforeach()

add_test(NAME test_barcode_live_feed_live_recheck_fail
    COMMAND test_barcode_live_feed_live recheck_fail)

foreach(case IN ITEMS search30 search199 search200 search_context search_hard
        search_end30 search_endlate search_end50 search_end51 search_unbound
        search_motorfault search_loss)
    add_test(NAME test_barcode_producer_grace_${case}
        COMMAND test_barcode_live_producer_live_custom ${case})
endforeach()

foreach(v IN ITEMS decode_grace decode_timeout decode_end)
    target_compile_definitions(test_barcode_live_e2e_${v} PRIVATE
        BARCODE_LIVE_ENABLE=1 BARCODE_MEASURED_CLEAR_MM=1 E2E_EXPECT_OFFER=1
        E2E_POLICY=3
        BW_MAP_VALUES=0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
        BW_MAP_POLICY_ID=2u)
endforeach()
target_compile_definitions(test_barcode_live_e2e_decode_grace PRIVATE
    E2E_DECODE_START=1200u E2E_DECODE_DURATION=30u)
target_compile_definitions(test_barcode_live_e2e_decode_timeout PRIVATE
    E2E_DECODE_START=1200u E2E_DECODE_DURATION=200u E2E_DECODE_RECORDS=0u)
target_compile_definitions(test_barcode_live_e2e_decode_end PRIVATE
    E2E_DECODE_START=2450u E2E_DECODE_DURATION=30u)

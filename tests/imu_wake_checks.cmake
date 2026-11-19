# IMU INT1 watermark wake (IMU_INT1_WAKE=1): policy, host SIL, probe
# registers and the service wake path. The =0 tests above stay unchanged.
host_test(test_imu_wake tests/test_imu_wake.c)
# Red safety acceptance (backlog == 0 in dead and missed-edge INT1 cases).
# Expected FAIL until fix F1; remove WILL_FAIL in the F1 change.
add_test(NAME test_imu_wake_acceptance COMMAND test_imu_wake acceptance)
set_tests_properties(test_imu_wake_acceptance PROPERTIES WILL_FAIL TRUE)
host_test(test_imu_probe_int1 tests/test_imu_probe.c
    app_program/subsystem4/imu_probe.c app_program/subsystem4/imu_decode.c
    app_program/subsystem4/imu_fifo.c)
target_compile_definitions(test_imu_probe_int1 PRIVATE IMU_INT1_WAKE=1)
target_link_libraries(test_imu_probe_int1 PRIVATE m)
host_test(test_imu_wake_service tests/test_imu_wake_service.c
    app_program/subsystem4/imu_hump_diag.c app_program/subsystem4/imu_safety.c
    app_program/subsystem4/imu_service.c app_program/subsystem4/imu_core.c
    app_program/subsystem4/imu_support.c app_program/subsystem4/imu_roll.c
    app_program/subsystem4/imu_hump.c)
target_include_directories(test_imu_wake_service BEFORE PRIVATE
    "${REPO}/tests/mtk_stubs")
target_compile_definitions(test_imu_wake_service PRIVATE
    APP_TEST=1 APP_BENCH=0 IMU_INT1_WAKE=1)
target_link_libraries(test_imu_wake_service PRIVATE m)
# imu_service.c links the safety board when the CR1 IMU packing is merged.
if(EXISTS "${REPO}/app_program/subsystem4/imu_safety_board.c")
    target_sources(test_imu_wake_service PRIVATE
        "${REPO}/app_program/subsystem4/imu_safety_board.c")
endif()
add_test(NAME test_imu_wake_service_add_failure
    COMMAND test_imu_wake_service add_failure)

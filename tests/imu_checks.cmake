host_test(test_imu_decode tests/test_imu_decode.c
    app_program/subsystem4/imu_decode.c)
target_link_libraries(test_imu_decode PRIVATE m)
host_test(test_imu_probe tests/test_imu_probe.c
    app_program/subsystem4/imu_probe.c app_program/subsystem4/imu_decode.c)
target_link_libraries(test_imu_probe PRIVATE m)
host_test(test_imu_hw tests/test_imu_hw.c app_program/subsystem4/imu_hw.c)
target_include_directories(test_imu_hw BEFORE PRIVATE
    "${REPO}/tests/imu_stubs")
target_compile_definitions(test_imu_hw PRIVATE
    APP_TEST=1 BOARD_UART0_FREE=1 TM_CONSOLE_UART=0)

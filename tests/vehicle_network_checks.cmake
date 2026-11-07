host_test(test_vehicle_mqtt tests/test_vehicle_mqtt.c app_program/vehicle/vehicle_mqtt.c app_program/subsystem1/mqtt_wire.c
    app_program/vehicle/vehicle_log.c app_program/vehicle/odometry.c)
if(TARGET test_vehicle_mqtt)
    target_include_directories(test_vehicle_mqtt BEFORE PRIVATE "${REPO}/tests/mtk_stubs")
    target_compile_definitions(test_vehicle_mqtt PRIVATE TM_MQTT=1)
endif()
host_test(test_vehicle_network_policy tests/test_vehicle_network_policy.c
    app_program/vehicle/vehicle_network_policy.c)
host_test(test_vehicle_network_task tests/test_vehicle_network_task.c
    app_program/vehicle/vehicle_task.c app_program/vehicle/vehicle_log.c
    app_program/vehicle/vehicle_network_policy.c app_program/vehicle/obstacle_report.c app_program/vehicle/run_stats.c
    app_program/vehicle/odometry.c app_program/subsystem1/mqtt_wire.c ${vehicle_sm_source} ${line_position_source}
    ${line_trace_source} ${line_steer_source} ${line_search_source} ${obstacle_source}
    ${servo_scan_source} ${obstacle_profile_source} ${detour_run_source} ${obstacle_detour_source})
if(TARGET test_vehicle_network_task)
    target_include_directories(test_vehicle_network_task BEFORE PRIVATE "${REPO}/tests/mtk_stubs")
    target_compile_definitions(test_vehicle_network_task PRIVATE TM_MQTT=1)
    add_test(NAME test_vehicle_network_uncalibrated COMMAND test_vehicle_network_task uncalibrated)
    add_test(NAME test_vehicle_network_estop_priority COMMAND test_vehicle_network_task estop)
endif()
host_test(test_obstacle_report tests/test_obstacle_report.c app_program/vehicle/obstacle_report.c ${obstacle_profile_source})
host_test(test_odometry tests/test_odometry.c app_program/vehicle/odometry.c)
host_test(test_run_stats tests/test_run_stats.c app_program/vehicle/run_stats.c
    app_program/vehicle/odometry.c)
foreach(odo_test test_odometry test_run_stats)
    if(TARGET ${odo_test})
        target_link_libraries(${odo_test} PRIVATE m)
    endif()
endforeach()

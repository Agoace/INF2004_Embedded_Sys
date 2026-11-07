host_test(test_robot_mqtt tests/test_robot_mqtt.c
    app_program/subsystem1/robot_mqtt.c app_program/subsystem1/mqtt_wire.c
    app_program/subsystem1/log_stream.c app_program/subsystem1/telemetry_mode.c)
if(TARGET test_robot_mqtt)
    target_include_directories(test_robot_mqtt BEFORE PRIVATE "${REPO}/tests/mqtt_stubs")
    # Public, non-secret fixture: TEST-NET broker and a synthetic command token.
    target_compile_definitions(test_robot_mqtt PRIVATE TM_MQTT=1 ROBOT_MQTT_NO_PRIVATE_CONFIG=1
        ROBOT_MQTT_BROKER_IP="192.0.2.10"
        ROBOT_MQTT_COMMAND_TOKEN_HEX="00112233445566778899aabbccddeeff")
    foreach(scenario connection commands schedule retained no_boot_id status obstacle detour summary log_live log_dump
            log_dump_ack_error log_disconnect health_stalled health_missing
            idle_rates idle_dump idle_no_vehicle idle_busy)
        add_test(NAME test_robot_mqtt_${scenario} COMMAND test_robot_mqtt ${scenario})
    endforeach()
endif()
foreach(config no_token zero malformed short)
    set(name test_robot_mqtt_${config})
    add_executable(${name} "${REPO}/tests/test_robot_mqtt.c"
        "${REPO}/app_program/subsystem1/robot_mqtt.c" "${REPO}/app_program/subsystem1/mqtt_wire.c"
        "${REPO}/app_program/subsystem1/log_stream.c"
        "${REPO}/app_program/subsystem1/telemetry_mode.c")
    target_include_directories(${name} BEFORE PRIVATE "${REPO}/tests/mqtt_stubs")
    target_compile_definitions(${name} PRIVATE TM_MQTT=1 ROBOT_MQTT_NO_PRIVATE_CONFIG=1 ROBOT_MQTT_BROKER_IP="192.0.2.10")
    if(NOT config STREQUAL "no_token")
        set(token "00000000000000000000000000000000")
        if(config STREQUAL "malformed")
            set(token "00112233445566778899aabbccddeefz")
        elseif(config STREQUAL "short")
            set(token "public-short-fixture")
        endif()
        target_compile_definitions(${name} PRIVATE ROBOT_MQTT_COMMAND_TOKEN_HEX="${token}")
    endif()
    add_test(NAME ${name} COMMAND ${name} unconfigured)
endforeach()

host_test(test_robot_mqtt tests/test_robot_mqtt.c
    app_program/subsystem1/robot_mqtt.c app_program/subsystem1/mqtt_wire.c)
if(TARGET test_robot_mqtt)
    target_include_directories(test_robot_mqtt BEFORE PRIVATE "${REPO}/tests/mqtt_stubs")
    # Public, non-secret fixture: TEST-NET broker and a synthetic command token.
    target_compile_definitions(test_robot_mqtt PRIVATE TM_MQTT=1
        ROBOT_MQTT_BROKER_IP="192.0.2.10" ROBOT_MQTT_COMMANDS_ENABLED=1
        ROBOT_MQTT_COMMAND_TOKEN_HEX="00112233445566778899aabbccddeeff")
    foreach(scenario connection commands schedule retained abort begin_snapshot
            begin_async begin_generation row_mem open_busy disconnect disconnect_abort)
        add_test(NAME test_robot_mqtt_${scenario} COMMAND test_robot_mqtt ${scenario})
    endforeach()
endif()
foreach(config disabled zero malformed)
    set(name test_robot_mqtt_${config})
    add_executable(${name} "${REPO}/tests/test_robot_mqtt.c"
        "${REPO}/app_program/subsystem1/robot_mqtt.c" "${REPO}/app_program/subsystem1/mqtt_wire.c")
    target_include_directories(${name} BEFORE PRIVATE "${REPO}/tests/mqtt_stubs")
    set(enabled 1)
    set(token "00112233445566778899aabbccddeeff")
    if(config STREQUAL "disabled")
        set(enabled 0)
    elseif(config STREQUAL "zero")
        set(token "00000000000000000000000000000000")
    else()
        set(token "00112233445566778899aabbccddeefz")
    endif()
    target_compile_definitions(${name} PRIVATE TM_MQTT=1 ROBOT_MQTT_BROKER_IP="192.0.2.10"
        ROBOT_MQTT_COMMANDS_ENABLED=${enabled} ROBOT_MQTT_COMMAND_TOKEN_HEX="${token}")
    add_test(NAME ${name} COMMAND ${name} disabled)
endforeach()

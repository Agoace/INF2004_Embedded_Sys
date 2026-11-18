host_test(test_mqtt_wire tests/test_mqtt_wire.c app_program/subsystem1/mqtt_wire.c)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
if(TARGET test_mqtt_wire)
    add_test(NAME test_mqtt_robot COMMAND "${Python3_EXECUTABLE}"
        "${REPO}/tests/test_mqtt_robot.py" "$<TARGET_FILE:test_mqtt_wire>")
endif()

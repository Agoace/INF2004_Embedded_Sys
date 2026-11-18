# Dashboard model, commands and HTTP rules. No broker, no Paho; C-packed samples.
if(TARGET test_mqtt_wire)
    add_test(NAME test_mqtt_dashboard COMMAND "${Python3_EXECUTABLE}"
        "${REPO}/tests/test_mqtt_dashboard.py" "$<TARGET_FILE:test_mqtt_wire>")
endif()

add_executable(test_pil_wire
    "${REPO}/tests/test_pil_wire.c"
    "${REPO}/app_program/sim/pil_wire.c")
set_property(TARGET test_pil_wire PROPERTY LINK_LIBRARIES "")
target_include_directories(test_pil_wire PRIVATE "${REPO}/app_program/sim")
add_test(NAME test_pil_wire_core COMMAND test_pil_wire)
file(READ "${REPO}/tests/fixtures/pil_wire_v1.json" PIL_FIXTURES)
string(JSON PIL_COUNT LENGTH "${PIL_FIXTURES}" vectors)
math(EXPR PIL_LAST "${PIL_COUNT} - 1")
foreach(index RANGE 0 ${PIL_LAST})
    foreach(field name accepted decoded_hex wire_hex)
        string(JSON PIL_${field} GET "${PIL_FIXTURES}" vectors ${index} ${field})
    endforeach()
    if(PIL_accepted)
        set(mode accept)
    else()
        set(mode reject)
    endif()
    add_test(NAME "test_pil_wire_${PIL_name}" COMMAND test_pil_wire
        "${mode}" "${PIL_decoded_hex}" "${PIL_wire_hex}")
endforeach()
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME test_pil_wire_reference COMMAND "${Python3_EXECUTABLE}"
    "${REPO}/tests/generate_pil_fixtures.py" --check)

add_executable(test_pil_skeletons "${REPO}/tests/test_pil_skeletons.c"
    "${REPO}/app_program/subsystem2/drv_motor_sim.c"
    "${REPO}/app_program/subsystem3/drv_line_sim.c"
    "${REPO}/app_program/subsystem3/barcode_sim.c"
    "${REPO}/app_program/subsystem4/imu_sim.c"
    "${REPO}/app_program/subsystem5/drv_ultrasonic_sim.c"
    "${REPO}/app_program/platform/drv_gpio_irq_sim.c")
set_property(TARGET test_pil_skeletons PROPERTY LINK_LIBRARIES "")
target_include_directories(test_pil_skeletons PRIVATE
    "${REPO}/tests/pil_stubs" "${REPO}/app_program")
target_compile_definitions(test_pil_skeletons PRIVATE
    APP_SIM=1 APP_BENCH=0 APP_BARCODE_CAPTURE=1)
add_test(NAME test_pil_skeletons COMMAND test_pil_skeletons)
add_test(NAME test_pil_make COMMAND "${Python3_EXECUTABLE}"
    "${REPO}/tests/test_pil_make.py" "${CMAKE_CURRENT_BINARY_DIR}")

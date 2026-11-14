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

foreach(unit metrics dump stream session)
    add_executable(test_pil_gate1_${unit}
        "${REPO}/tests/test_pil_gate1_${unit}.c"
        "${REPO}/app_program/sim/pil_gate1_${unit}.c"
        "${REPO}/app_program/sim/pil_wire.c")
    if(unit STREQUAL "session")
        target_sources(test_pil_gate1_${unit} PRIVATE
            "${REPO}/app_program/sim/pil_gate1_metrics.c")
    endif()
    set_property(TARGET test_pil_gate1_${unit} PROPERTY LINK_LIBRARIES "")
    target_include_directories(test_pil_gate1_${unit} PRIVATE
        "${REPO}/app_program/sim")
    add_test(NAME test_pil_gate1_${unit} COMMAND test_pil_gate1_${unit})
endforeach()
file(READ "${REPO}/tests/fixtures/pil_gate1_d1.json" D1_FIXTURES)
string(JSON D1_COUNT LENGTH "${D1_FIXTURES}" vectors)
math(EXPR D1_LAST "${D1_COUNT} - 1")
foreach(index RANGE 0 ${D1_LAST})
    foreach(field name accepted_envelope wire_hex)
        string(JSON D1_${field} GET "${D1_FIXTURES}" vectors ${index} ${field})
    endforeach()
    if(D1_accepted_envelope)
        set(mode accept)
    else()
        set(mode reject)
    endif()
    add_test(NAME "test_pil_gate1_d1_${D1_name}" COMMAND test_pil_gate1_dump
        "${mode}" "${D1_wire_hex}")
endforeach()
add_test(NAME test_pil_gate1_d1_reference COMMAND "${Python3_EXECUTABLE}"
    "${REPO}/tests/generate_pil_gate1_fixtures.py" --check)
add_executable(pil_gate1_fixture "${REPO}/tests/pil_gate1_fixture.c"
    "${REPO}/app_program/sim/pil_gate1_dump.c"
    "${REPO}/app_program/sim/pil_gate1_metrics.c"
    "${REPO}/app_program/sim/pil_wire.c")
set_property(TARGET pil_gate1_fixture PROPERTY LINK_LIBRARIES "")
target_include_directories(pil_gate1_fixture PRIVATE "${REPO}/app_program/sim")
add_test(NAME test_pil_gate1_c_fixture COMMAND pil_gate1_fixture
    "${CMAKE_CURRENT_BINARY_DIR}/pil_gate1_c_fixture.bin")

add_executable(test_pil_gate1_hw "${REPO}/tests/test_pil_gate1_hw.c"
    "${REPO}/app_program/sim/pil_gate1_uart_hw.c"
    "${REPO}/app_program/sim/pil_gate1_line_time.c")
set_property(TARGET test_pil_gate1_hw PROPERTY LINK_LIBRARIES "")
target_include_directories(test_pil_gate1_hw PRIVATE
    "${REPO}/tests/pil_stubs" "${REPO}/app_program/sim")
add_test(NAME test_pil_gate1_hw COMMAND test_pil_gate1_hw)
add_test(NAME test_pil_gate1_make COMMAND "${Python3_EXECUTABLE}"
    "${REPO}/tests/test_pil_gate1_make.py")
add_executable(test_pil_gate1_rxlog "${REPO}/tests/test_pil_gate1_rxlog.c"
    "${REPO}/app_program/sim/pil_gate1_rxlog.c"
    "${REPO}/app_program/sim/pil_wire.c")
set_property(TARGET test_pil_gate1_rxlog PROPERTY LINK_LIBRARIES "")
target_include_directories(test_pil_gate1_rxlog PRIVATE
    "${REPO}/tests/pil_stubs" "${REPO}/app_program/sim")
add_test(NAME test_pil_gate1_rxlog COMMAND test_pil_gate1_rxlog)
set(pil_critical "${CMAKE_CURRENT_BINARY_DIR}/pil_gate1_critical.h")
add_custom_command(OUTPUT "${pil_critical}"
    COMMAND "${Python3_EXECUTABLE}" "${REPO}/tests/generate_pil_critical.py"
        "${REPO}/kernel/sysdepend/cpu/core/armv6m/cpu_status.h"
        "${pil_critical}"
    DEPENDS "${REPO}/tests/generate_pil_critical.py"
        "${REPO}/kernel/sysdepend/cpu/core/armv6m/cpu_status.h"
    VERBATIM)
add_custom_target(pil_gate1_critical DEPENDS "${pil_critical}")
foreach(profile up smp control)
    set(target test_pil_gate1_runtime_${profile})
    add_executable(${target} "${REPO}/tests/test_pil_gate1_runtime.c"
        "${REPO}/app_program/sim/pil_gate1_task.c"
        "${REPO}/app_program/sim/pil_gate1_rxlog.c"
        "${REPO}/app_program/sim/pil_gate1_session.c"
        "${REPO}/app_program/sim/pil_gate1_stream.c"
        "${REPO}/app_program/sim/pil_gate1_metrics.c"
        "${REPO}/app_program/sim/pil_gate1_dump.c"
        "${REPO}/app_program/sim/pil_wire.c")
    set_property(TARGET ${target} PROPERTY LINK_LIBRARIES "")
    add_dependencies(${target} pil_gate1_critical)
    target_include_directories(${target} PRIVATE
        "${REPO}/tests/pil_stubs" "${REPO}/app_program/sim"
        "${REPO}/kernel/knlinc" "${CMAKE_CURRENT_BINARY_DIR}")
    target_compile_definitions(${target} PRIVATE
        APP_SIM=1 APP_SIM_GATE1=1 PIL_GATE1_RUNTIME_TEST=1
        TK_SUPPORT_SMP=$<IF:$<STREQUAL:${profile},up>,0,1>
        APP_FORCE_CORE0=$<IF:$<STREQUAL:${profile},control>,1,0>)
    foreach(mode wake wakelease wakenative priority clean lease overrun
                 lateprogress queuefull missing)
        add_test(NAME ${target}_${mode} COMMAND ${target} ${mode})
    endforeach()
endforeach()

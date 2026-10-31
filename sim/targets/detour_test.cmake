# Read firmware sources. Never add these options to the firmware build.
set(SIM_ENTRY "${REPO}/src/detour_test.c")
set(SIM_FIRMWARE_SOURCES
    "${SIM_ENTRY}"
    "${REPO}/src/obstacle_detour.c"
    "${REPO}/src/obstacle.c"
    "${REPO}/src/obstacle_profile.c"
    "${REPO}/src/obstacle_scan.c"
    "${REPO}/src/line_steer.c"
    "${REPO}/src/line_position.c"
    "${REPO}/src/line_spin.c"
    "${REPO}/src/steer_log.c")
set(SIM_HOOKS dt_init dt_update dt_reading obstacle_update
    obstacle_scan_start obstacle_scan_service lp_update steer_update
    steer_straight steer_reset obstacle_init)
set_property(SOURCE "${SIM_ENTRY}" APPEND PROPERTY COMPILE_DEFINITIONS main=firmware_main)
foreach(hook IN LISTS SIM_HOOKS)
    set_property(SOURCE "${SIM_ENTRY}" APPEND PROPERTY
        COMPILE_DEFINITIONS "${hook}=sim_hook_${hook}")
endforeach()
foreach(source IN LISTS SIM_FIRMWARE_SOURCES)
    set_property(SOURCE "${source}" APPEND PROPERTY COMPILE_DEFINITIONS printf=sim_printf)
    set_property(SOURCE "${source}" APPEND PROPERTY COMPILE_OPTIONS -fno-builtin-printf -fno-builtin-puts)
endforeach()
set(SIM_TARGET_SOURCES "${CMAKE_CURRENT_LIST_DIR}/detour_hooks.c")

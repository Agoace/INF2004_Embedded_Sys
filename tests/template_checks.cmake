# Shared-template checks use isolated host doubles, not SDK headers.
host_test(test_wifi_stack_contention tests/test_wifi_stack_contention.c)
target_include_directories(test_wifi_stack_contention BEFORE PRIVATE
    "${REPO}/tests/template_stubs" "${REPO}/tests/mtk_stubs")
target_compile_definitions(test_wifi_stack_contention PRIVATE
    TM_WIFI_CYW43=1 TM_NET_LWIP=1 TM_MQTT=0)
host_test(test_template_tick tests/test_template_tick.c)
foreach(bad_clock 0 512)
    host_test(test_template_tick_${bad_clock} tests/test_template_tick.c)
    target_compile_definitions(test_template_tick_${bad_clock} PRIVATE TEST_XOSC_MHZ=${bad_clock})
endforeach()
foreach(smp 0 1)
    foreach(net 0 1)
        set(name test_template_wifi_smp${smp}_net${net})
        host_test(${name} tests/test_template_wifi.c
            lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c
            app_program/platform/app_task.c
            app_program/subsystem1/robot_health.c
            tests/wifi_stack_health_checks.c)
        target_include_directories(${name} BEFORE PRIVATE
            "${REPO}/tests/template_stubs" "${REPO}/tests/mtk_stubs")
        target_compile_definitions(${name} PRIVATE TK_SUPPORT_SMP=${smp}
            TM_NET_LWIP=${net} TM_WIFI_CYW43=1 TM_MQTT=${net})
        add_test(NAME ${name}_wrap COMMAND ${name} wrap)
        add_test(NAME ${name}_cleanup_failure COMMAND ${name} cleanup_failure)
        add_test(NAME ${name}_config COMMAND ${name} config)
    endforeach()
endforeach()

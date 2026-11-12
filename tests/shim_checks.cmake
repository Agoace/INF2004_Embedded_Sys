# Compile the actual UP shim, never the M1 public-API double.
add_executable(test_smp_single_core
    "${REPO}/tests/test_smp_single_core.c"
    "${REPO}/app_program/platform/smp_single_core.c")
set_property(TARGET test_smp_single_core PROPERTY LINK_LIBRARIES "")
target_include_directories(test_smp_single_core BEFORE PRIVATE
    "${REPO}/tests/smp_stubs" "${REPO}/tests/mtk_stubs")
target_compile_definitions(test_smp_single_core PRIVATE
    TK_SUPPORT_SMP=0 APP_SMP_SHIM_HOST_TEST=1)
foreach(scenario init plain_claim plain_try plain_busy plain_unlock irq_claim
        irq_try irq_busy irq_unlock nested wait inc_isr dec_isr add_isr sub_isr
        old_values wrap null barrier)
    add_test(NAME test_real_up_shim_${scenario}
        COMMAND test_smp_single_core ${scenario})
    set_tests_properties(test_real_up_shim_${scenario} PROPERTIES TIMEOUT 5)
endforeach()

# On-chip kernel bench (BENCH_KERNEL=1 firmware only; host checks always).
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME test_bench_kernel_selector COMMAND "${Python3_EXECUTABLE}"
    "${REPO}/tests/test_bench_kernel_selector.py")
host_test(test_bench_kernel_stats tests/test_bench_kernel_stats.c
    app_program/bench_kernel/bench_stats.c)
host_test(test_bench_kernel_stats_threads tests/test_bench_kernel_stats.c
    app_program/bench_kernel/bench_stats.c)
host_test(test_bench_report tests/test_bench_report.c
    app_program/bench_kernel/bench_report.c)
host_test(test_bench_kernel_gpio tests/test_bench_kernel_gpio.c
    app_program/bench_kernel/bench_gpio.c
    app_program/bench_kernel/bench_gpio_hw.c)
host_test(test_bench_kernel_encoder tests/test_bench_kernel_encoder.c
    app_program/bench_kernel/bench_gpio.c
    app_program/bench_kernel/bench_gpio_hw.c
    app_program/bench_kernel/bench_motor_hw.c
    app_program/platform/drv_gpio_irq.c
    app_program/subsystem2/motion.c app_program/subsystem2/motor_ctl.c
    app_program/subsystem2/motion_pack.c app_program/subsystem2/subsystem2.c)
if(TARGET test_bench_kernel_encoder)
    target_include_directories(test_bench_kernel_encoder BEFORE PRIVATE
        "${REPO}/tests/mtk_stubs")
endif()
host_test(test_bench_kernel_generator tests/test_bench_kernel_generator.c
    app_program/bench_kernel/bench_generator.c
    app_program/bench_kernel/bench_gpio.c
    app_program/bench_kernel/bench_stats.c)
host_test(test_bench_kernel_runtime tests/test_bench_kernel_runtime.c
    app_program/bench_kernel/bench_runtime.c
    app_program/bench_kernel/bench_generator.c
    app_program/bench_kernel/bench_gpio.c
    app_program/bench_kernel/bench_stats.c)
if(TARGET test_bench_kernel_runtime)
    target_include_directories(test_bench_kernel_runtime BEFORE PRIVATE
        "${REPO}/tests/mtk_stubs")
    add_test(NAME test_bench_kernel_runtime_past
        COMMAND test_bench_kernel_runtime past)
    add_test(NAME test_bench_kernel_runtime_late_poll
        COMMAND test_bench_kernel_runtime late_poll)
endif()
host_test(test_bench_kernel_inputs tests/test_bench_kernel_inputs.c
    app_program/bench_kernel/bench_inputs_hw.c
    app_program/bench_kernel/bench_motor_hw.c
    app_program/bench_kernel/bench_gpio.c)
foreach(t test_bench_kernel_stats test_bench_kernel_stats_threads
        test_bench_report test_bench_kernel_gpio test_bench_kernel_encoder
        test_bench_kernel_generator test_bench_kernel_runtime
        test_bench_kernel_inputs)
    if(TARGET ${t})
        target_compile_definitions(${t} PRIVATE BENCH_KERNEL=1)
    endif()
endforeach()
if(TARGET test_bench_kernel_stats_threads)
    find_package(Threads REQUIRED)
    target_compile_definitions(test_bench_kernel_stats_threads PRIVATE
        BENCH_STATS_THREADS=1)
    target_link_libraries(test_bench_kernel_stats_threads Threads::Threads)
endif()
if(TARGET test_bench_report)
    target_include_directories(test_bench_report BEFORE PRIVATE
        "${REPO}/tests/mtk_stubs")
endif()

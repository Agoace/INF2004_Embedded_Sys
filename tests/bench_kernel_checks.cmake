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
foreach(t test_bench_kernel_stats test_bench_kernel_stats_threads
        test_bench_report)
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

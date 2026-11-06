#include "subsystem2.h"
#include "motion.h"

EXPORT ER subsystem2_init(void) { return (ER)motion_init(); }
EXPORT ER subsystem2_start(void) {
    const int er = motion_start();
    return er < E_OK ? (ER)er : (ER)motion_bench_start();
}

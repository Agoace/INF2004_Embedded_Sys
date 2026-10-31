#include "sim_world.h"
#include "sim_runtime.h"
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

uint16_t sim_floor[3];
_Noreturn void sim_error(const char *message) { fprintf(stderr, "%s\n", message); exit(2); }
void sim_event_at(uint64_t us, const char *kind, const char *fields, ...) {
    (void)us;
    printf("{\"type\":\"%s\",", kind);
    va_list args; va_start(args, fields); vprintf(fields, args); va_end(args);
    puts("}");
}
static void near(double actual, double expected, double tolerance) {
    if (fabs(actual - expected) > tolerance) {
        fprintf(stderr, "actual %.12f expected %.12f tolerance %.12f\n", actual, expected, tolerance);
        exit(1);
    }
}
static world_config_t config(void) {
    return (world_config_t){.loaded = true, .symmetric_reverse = true, .zero_pct = 12,
        .tau_ms = {50, 50}, .front = 123, .rear = -67, .width = 160, .track = 111, .diameter = 65,
        .ir = {{119,22},{119,0},{119,-22}}, .floor = {202,185,191}, .line = {1645,2615,3424},
        .footprint = 3, .line_width = 18};
}
int main(int argc, char **argv) {
    if (argc == 2) {
        world_read(stdin);
        if (strcmp(argv[1], "ir") == 0) {
            uint16_t raw[3]; world_ir(raw);
            printf("{\"raw\":[%u,%u,%u]}\n", (unsigned int)raw[0], (unsigned int)raw[1], (unsigned int)raw[2]);
        } else {
            world_duty(0u,100); world_duty(1u,100);
            world_advance(0u,1000000u);
            assert(world_state()->contact);
            const world_state_t stopped = *world_state();
            world_advance(1000000u,2000000u);
            near(world_state()->x, stopped.x, 0);
            near(world_state()->y, stopped.y, 0);
            near(world_state()->counts[0], stopped.counts[0], 0);
            puts("{\"contact_latched\":true}");
        }
        world_destroy();
        return 0;
    }
    const double pi = acos(-1.0), circumference = 204.204;
    world_config_t c = config();
    world_init(&c);
    /* Independent hand interpolation of the 20% loaded target. */
    const double mapped[2] = {6.0 + 8.0*94.0/88.0, 7.0 + 8.0*93.0/88.0};
    const double rpm[2] = {26.253 + (53.881-26.253)*(mapped[0]-10)/10,
                           26.556 + (54.352-26.556)*(mapped[1]-10)/10};
    const double corrected[2] = {rpm[0]*2540/2535, rpm[1]*2539/2535};
    double speed[2], distance[2], counts[2];
    for (unsigned int i = 0; i < 2u; ++i) {
        speed[i] = corrected[i]*circumference/60;
        distance[i] = speed[i]*(1-0.05*(1-exp(-20)));
        counts[i] = distance[i]*2535/circumference;
        near(world_target_speed(i,20),speed[i],1e-10);
        world_duty(i,20);
    }
    world_advance(0u,1000000u);
    for (size_t i = 0; i < 2u; ++i) { near(world_state()->counts[i],counts[i],1e-7); }
    int32_t left, right; world_counts(&left, &right);
    assert(left == 1560 && right == 1677);
    const double turn = (distance[1]-distance[0])/111;
    const double arc = (distance[0]+distance[1])/2;
    near(world_state()->x, arc*sin(turn)/turn, 1e-7);
    near(world_state()->y, arc*(1-cos(turn))/turn, 1e-7);
    printf("{\"straight_20pct_1s\":{\"wheel_mm\":[%.9f,%.9f],\"counts\":[%.9f,%.9f],\"x_mm\":%.9f,\"y_mm\":%.9f}}\n",
           distance[0],distance[1],counts[0],counts[1],world_state()->x,world_state()->y);
    const double before[2] = {world_state()->counts[0],world_state()->counts[1]};
    world_duty(0u,0); world_duty(1u,0);
    world_advance(1000000u,2000000u);
    for (size_t i = 0; i < 2u; ++i) {
        near(world_state()->counts[i]-before[i],speed[i]*(1-exp(-20))*0.05*(1-exp(-20))*2535/circumference,1e-7);
    }
    printf("{\"loaded_coast_counts\":[%.9f,%.9f]}\n",world_state()->counts[0]-before[0],world_state()->counts[1]-before[1]);
    world_init(&c); world_duty(0u,20); world_duty(1u,-20);
    world_advance(0u,1000000u);
    const double difference = world_state()->counts[1]-world_state()->counts[0];
    const double degrees = world_state()->heading*180/pi;
    const double counts_per_degree = 2535*111*pi/180/circumference;
    near(degrees,difference/counts_per_degree,1e-7);
    near(degrees,difference/24.05,0.001);
    printf("{\"spin_degrees\":%.9f,\"count_difference\":%.9f,\"counts_per_degree\":%.9f}\n",degrees,difference,counts_per_degree);
    world_init(&c); world_duty(0u,-20); world_duty(1u,-20); world_advance(0u,1000000u);
    near(world_state()->counts[0],-counts[0],1e-7); near(world_state()->counts[1],-counts[1],1e-7);
    world_counts(&left, &right); assert(left == -1560 && right == -1677);
    printf("{\"reverse_counts\":[%.9f,%.9f]}\n",world_state()->counts[0],world_state()->counts[1]);
    const double reverse_x = world_state()->x;
    world_reset_counts(); world_counts(&left, &right);
    assert(left == 0 && right == 0 && world_state()->x == reverse_x);
    c.loaded = false;
    const double count_rate[2] = {26.253*2540/60,26.556*2539/60};
    c.tau_ms[0] = 27.5/count_rate[0]*1000; c.tau_ms[1] = 23/count_rate[1]*1000;
    world_init(&c); world_duty(0u,10); world_duty(1u,10); world_advance(0u,5000000u);
    const double start[2] = {world_state()->counts[0],world_state()->counts[1]};
    world_duty(0u,0); world_duty(1u,0); world_advance(5000000u,7000000u);
    near(world_state()->counts[0]-start[0],27.5,1e-6); near(world_state()->counts[1]-start[1],23,1e-6);
    printf("{\"reference_tau_ms\":[%.9f,%.9f],\"reference_coast_counts\":[%.9f,%.9f]}\n",
        c.tau_ms[0],c.tau_ms[1],world_state()->counts[0]-start[0],world_state()->counts[1]-start[1]);
    c = config(); world_init(&c); assert(world_gap_consistent());
    near(world_line_fraction(11,18,3),0,0);
    c.ir[0].y = 14; world_init(&c); assert(!world_gap_consistent());
    near(world_line_fraction(7,18,3),1,0);
    world_destroy();
    return 0;
}

#ifndef SIM_WORLD_H
#define SIM_WORLD_H
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef struct { double x, y; } world_point_t;
typedef struct {
    bool loaded, symmetric_reverse;
    double zero_pct, tau_ms[2];
    double x, y, heading_deg;
    double front, rear, width, track, diameter, servo_x, face_offset;
    world_point_t ir[3];
    uint16_t floor[3], line[3];
    double footprint, line_width, tile_size, origin_x, origin_y, curve_radius;
} world_config_t;
typedef struct {
    double x, y, heading, speed[2], counts[2];
    int duty[2];
    bool contact;
    uint64_t contact_us;
    size_t obstacle;
} world_state_t;
typedef struct {
    bool hit;
    size_t obstacle;
    double distance, angle_deg;
    world_point_t origin, point;
} world_echo_t;
void world_servo(uint16_t pulse);
world_echo_t world_echo(void);
void world_init(const world_config_t *config);
void world_read(FILE *input);
void world_destroy(void);
void world_advance(uint64_t start_us, uint64_t end_us);
void world_snapshot(uint64_t time_us);
void world_duty(unsigned int wheel, int duty);
void world_counts(int32_t *left, int32_t *right);
void world_reset_counts(void);
void world_ir(uint16_t raw[3]);
double world_line_fraction(double distance, double line_width, double footprint);
double world_target_speed(unsigned int wheel, int duty);
const world_state_t *world_state(void);
bool world_is_active(void);
bool world_gap_consistent(void);
#endif

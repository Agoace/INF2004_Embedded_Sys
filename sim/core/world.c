#include "sim_world.h"
#include "sim_runtime.h"
#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.14159265358979323846
#define COUNTS_PER_TURN 2535.0

typedef struct { int col, row, kind, rotation; } tile_t;
typedef struct { int kind; world_point_t a, b; double radius, start, sweep, width; } stroke_t;
typedef struct {
    int kind, specular;
    world_point_t centre;
    double angle, width, height, max_incidence, corner_range;
} obstacle_t;
static world_config_t cfg;
static world_state_t state;
static tile_t *tiles;
static stroke_t *strokes;
static obstacle_t *obstacles;
static size_t tile_count, stroke_count, obstacle_count;
static bool active;
static double servo_deg;

static const double rpm[2][10] = {
    {26.253, 53.881, 81.486, 109.006, 136.648, 163.417, 190.842, 217.029, 242.664, 260.829},
    {26.556, 54.352, 81.948, 109.328, 136.527, 163.136, 188.965, 214.298, 239.560, 257.860},
};
static const double old_counts[2] = {2540.0, 2539.0};
static const double unloaded_zero[2] = {6.0, 7.0};

void world_destroy(void) {
    free(tiles); free(strokes); free(obstacles);
    tiles = NULL; strokes = NULL; obstacles = NULL;
    tile_count = stroke_count = obstacle_count = 0;
    active = false;
}
void world_init(const world_config_t *config) {
    world_destroy();
    cfg = *config;
    servo_deg = 0;
    state = (world_state_t){.x = cfg.x, .y = cfg.y, .heading = cfg.heading_deg * PI / 180.0};
    active = true;
}
const world_state_t *world_state(void) { return &state; }
bool world_is_active(void) { return active; }

static void *allocate(size_t count, size_t size) {
    void *p = calloc(count == 0u ? 1u : count, size);
    if (p == NULL) { sim_error("world allocation failed"); }
    return p;
}
static void read_count(FILE *input, size_t *count) {
    if (fscanf(input, "%zu", count) != 1 || *count > 100000u) { sim_error("invalid world count"); }
}
void world_read(FILE *input) {
    int enabled, loaded, reverse;
    if (fscanf(input, "%d", &enabled) != 1) { sim_error("missing world configuration"); }
    if (enabled == 0) { active = false; return; } /* Clock/device contract probe only. */
    world_config_t c = {0};
    if (fscanf(input, "%d %d %lf %lf %lf %lf %lf %lf", &loaded, &reverse,
        &c.zero_pct, &c.tau_ms[0], &c.tau_ms[1], &c.x, &c.y, &c.heading_deg) != 8 ||
        fscanf(input, "%lf %lf %lf %lf %lf %lf %lf", &c.front, &c.rear, &c.width,
               &c.track, &c.diameter, &c.servo_x, &c.face_offset) != 7) { sim_error("invalid motion configuration"); }
    c.loaded = loaded != 0; c.symmetric_reverse = reverse != 0;
    for (size_t i = 0; i < 3u; ++i) {
        unsigned int floor, line;
        if (fscanf(input, "%lf %lf %u %u", &c.ir[i].x, &c.ir[i].y, &floor, &line) != 4 ||
            floor > 4095u || line > 4095u) { sim_error("invalid IR configuration"); }
        c.floor[i] = (uint16_t)floor; c.line[i] = (uint16_t)line;
    }
    if (fscanf(input, "%lf %lf %lf %lf %lf %lf", &c.footprint, &c.line_width,
        &c.tile_size, &c.origin_x, &c.origin_y, &c.curve_radius) != 6) { sim_error("invalid floor configuration"); }
    world_init(&c);
    read_count(input, &tile_count);
    tiles = allocate(tile_count, sizeof *tiles);
    for (size_t i = 0; i < tile_count; ++i) {
        if (fscanf(input, "%d %d %d %d", &tiles[i].col, &tiles[i].row,
            &tiles[i].kind, &tiles[i].rotation) != 4) { sim_error("invalid tile"); }
    }
    read_count(input, &stroke_count);
    strokes = allocate(stroke_count, sizeof *strokes);
    for (size_t i = 0; i < stroke_count; ++i) {
        stroke_t *s = &strokes[i];
        if (fscanf(input, "%d %lf %lf %lf %lf %lf %lf %lf %lf", &s->kind,
            &s->a.x, &s->a.y, &s->b.x, &s->b.y, &s->radius, &s->start, &s->sweep, &s->width) != 9) {
            sim_error("invalid stroke");
        }
        s->start *= PI / 180.0; s->sweep *= PI / 180.0;
    }
    read_count(input, &obstacle_count);
    obstacles = allocate(obstacle_count, sizeof *obstacles);
    for (size_t i = 0; i < obstacle_count; ++i) {
        obstacle_t *o = &obstacles[i];
        if (fscanf(input, "%d %lf %lf %lf %lf %lf %d %lf %lf", &o->kind, &o->centre.x,
            &o->centre.y, &o->angle, &o->width, &o->height, &o->specular,
            &o->max_incidence, &o->corner_range) != 9) { sim_error("invalid obstacle"); }
        o->angle *= PI / 180.0;
        o->max_incidence *= PI / 180.0;
    }
    if (!world_gap_consistent()) { sim_error("real IR model failed C-L gap consistency"); }
}

static world_point_t transform(world_point_t p, double x, double y, double h) {
    return (world_point_t){x + p.x * cos(h) - p.y * sin(h), y + p.x * sin(h) + p.y * cos(h)};
}
static double segment_distance(world_point_t p, world_point_t a, world_point_t b) {
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double square = dx * dx + dy * dy;
    double t = square == 0.0 ? 0.0 : ((p.x - a.x) * dx + (p.y - a.y) * dy) / square;
    t = fmax(0.0, fmin(1.0, t));
    return hypot(p.x - a.x - t * dx, p.y - a.y - t * dy);
}
static double arc_distance(world_point_t p, world_point_t centre, double radius, double start, double sweep) {
    const double angle = atan2(p.y - centre.y, p.x - centre.x);
    double relative = fmod((sweep < 0.0 ? start - angle : angle - start) + 4.0 * PI, 2.0 * PI);
    if (relative < 0.0) { relative += 2.0 * PI; }
    if (relative <= fabs(sweep)) { return fabs(hypot(p.x - centre.x, p.y - centre.y) - radius); }
    const world_point_t a = {centre.x + radius * cos(start), centre.y + radius * sin(start)};
    const world_point_t b = {centre.x + radius * cos(start + sweep), centre.y + radius * sin(start + sweep)};
    return fmin(hypot(p.x - a.x, p.y - a.y), hypot(p.x - b.x, p.y - b.y));
}
double world_line_fraction(double distance, double line_width, double footprint) {
    const double lo = fmax(-line_width / 2.0, distance - footprint / 2.0);
    const double hi = fmin(line_width / 2.0, distance + footprint / 2.0);
    return fmax(0.0, fmin(1.0, (hi - lo) / footprint));
}
bool world_gap_consistent(void) {
    /* Exercise production tile/stroke -> footprint -> ADC code, not a second model. */
    const world_state_t saved = state;
    const size_t saved_tiles = tile_count, saved_strokes = stroke_count;
    stroke_t *saved_pointer = strokes;
    stroke_t line = {.kind = 0, .a = {-1000000,0}, .b = {1000000,0}, .width = cfg.line_width};
    tile_count = 0; stroke_count = 1; strokes = &line;
    state.x = 0; state.y = -(cfg.ir[0].y + cfg.ir[1].y) / 2.0; state.heading = 0;
    uint16_t raw[3]; world_ir(raw);
    bool consistent = true;
    for (size_t i = 0; i < 3u; ++i) {
        const double rise = fabs((double)raw[i] - cfg.floor[i]) / fabs((double)cfg.line[i] - cfg.floor[i]);
        if (rise >= 0.01) { consistent = false; }
    }
    state = saved; tile_count = saved_tiles; stroke_count = saved_strokes; strokes = saved_pointer;
    return consistent;
}
static double floor_fraction(world_point_t p) {
    double fraction = 0.0;
    for (size_t i = 0; i < tile_count; ++i) {
        const tile_t t = tiles[i];
        const double half = cfg.tile_size / 2.0;
        const world_point_t q = transform((world_point_t){
            p.x - cfg.origin_x - ((double)t.col + 0.5) * cfg.tile_size,
            p.y - cfg.origin_y - ((double)t.row + 0.5) * cfg.tile_size}, 0, 0, -(double)t.rotation * PI / 180.0);
        if (fabs(q.x) > half + cfg.footprint || fabs(q.y) > half + cfg.footprint) { continue; }
        double distance = 1e30;
        if (t.kind == 0 || t.kind == 2) {
            distance = segment_distance(q, (world_point_t){-half, 0}, (world_point_t){half, 0});
        }
        if (t.kind == 2) {
            distance = fmin(distance, segment_distance(q, (world_point_t){0, -half}, (world_point_t){0, half}));
        }
        if (t.kind == 1) { distance = arc_distance(q, (world_point_t){-half, -half}, cfg.curve_radius, 0, PI / 2.0); }
        fraction = fmax(fraction, world_line_fraction(distance, cfg.line_width, cfg.footprint));
    }
    for (size_t i = 0; i < stroke_count; ++i) {
        const stroke_t s = strokes[i];
        const double distance = s.kind == 0 ? segment_distance(p, s.a, s.b) :
            arc_distance(p, s.a, s.radius, s.start, s.sweep);
        fraction = fmax(fraction, world_line_fraction(distance, s.width, cfg.footprint));
    }
    return fraction;
}
void world_ir(uint16_t raw[3]) {
    for (size_t i = 0; i < 3u; ++i) {
        if (!active) { raw[i] = sim_floor[i]; continue; }
        const double f = floor_fraction(transform(cfg.ir[i], state.x, state.y, state.heading));
        const double adc = (double)cfg.floor[i] + f * ((double)cfg.line[i] - (double)cfg.floor[i]);
        raw[i] = (uint16_t)lround(fmax(0.0, fmin(4095.0, adc)));
    }
}

double world_target_speed(unsigned int wheel, int duty) {
    if (!active) { return 0.0; }
    double pct = fmin(100.0, fabs((double)duty));
    const double zero = cfg.loaded ? cfg.zero_pct : unloaded_zero[wheel];
    if (pct <= zero) { return 0.0; }
    if (duty < 0 && !cfg.symmetric_reverse) { sim_error("asymmetric reverse needs a measured curve"); }
    if (cfg.loaded) { pct = unloaded_zero[wheel] + (pct - zero) * (100.0 - unloaded_zero[wheel]) / (100.0 - zero); }
    double speed_rpm;
    if (pct < 10.0) {
        speed_rpm = rpm[wheel][0] * (pct - unloaded_zero[wheel]) / (10.0 - unloaded_zero[wheel]);
    } else if (pct >= 100.0) { speed_rpm = rpm[wheel][9]; }
    else {
        const unsigned int index = (unsigned int)(pct / 10.0) - 1u;
        const double share = (pct - ((double)index + 1.0) * 10.0) / 10.0;
        speed_rpm = rpm[wheel][index] + share * (rpm[wheel][index + 1u] - rpm[wheel][index]);
    }
    const double corrected = speed_rpm * old_counts[wheel] / COUNTS_PER_TURN;
    return (duty < 0 ? -1.0 : 1.0) * corrected * (cfg.diameter * 204.204 / 65.0) / 60.0;
}
void world_duty(unsigned int wheel, int duty) { state.duty[wheel] = duty > 100 ? 100 : duty < -100 ? -100 : duty; }
void world_reset_counts(void) { state.counts[0] = state.counts[1] = 0.0; }
void world_counts(int32_t *left, int32_t *right) {
    *left = (int32_t)state.counts[0]; *right = (int32_t)state.counts[1];
}
static void rectangle(world_point_t out[4], double x, double y, double h, double front, double rear, double width) {
    out[0] = transform((world_point_t){front, width / 2.0}, x, y, h);
    out[1] = transform((world_point_t){rear, width / 2.0}, x, y, h);
    out[2] = transform((world_point_t){rear, -width / 2.0}, x, y, h);
    out[3] = transform((world_point_t){front, -width / 2.0}, x, y, h);
}
static bool inside(world_point_t p, const world_point_t polygon[4]) {
    for (size_t i = 0; i < 4u; ++i) {
        const world_point_t a = polygon[i], b = polygon[(i + 1u) % 4u];
        if ((b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x) < 0.0) { return false; }
    }
    return true;
}
static bool separated(const world_point_t a[4], const world_point_t b[4]) {
    for (size_t i = 0; i < 4u; ++i) {
        const world_point_t p = a[i], q = a[(i + 1u) % 4u];
        const double nx = p.y - q.y, ny = q.x - p.x;
        double min_a = 1e30, max_a = -1e30, min_b = 1e30, max_b = -1e30;
        for (size_t j = 0; j < 4u; ++j) {
            const double da = a[j].x * nx + a[j].y * ny, db = b[j].x * nx + b[j].y * ny;
            min_a = fmin(min_a, da); max_a = fmax(max_a, da);
            min_b = fmin(min_b, db); max_b = fmax(max_b, db);
        }
        if (max_a < min_b || max_b < min_a) { return true; }
    }
    return false;
}
static double clearance(size_t *nearest) {
    world_point_t robot[4];
    rectangle(robot, state.x, state.y, state.heading, cfg.front, cfg.rear, cfg.width);
    double best = 1e30;
    for (size_t i = 0; i < obstacle_count; ++i) {
        const obstacle_t o = obstacles[i];
        double distance = 1e30;
        if (o.kind == 1) {
            distance = inside(o.centre, robot) ? 0.0 : 1e30;
            for (size_t j = 0; j < 4u; ++j) {
                distance = fmin(distance, segment_distance(o.centre, robot[j], robot[(j + 1u) % 4u]));
            }
            distance = fmax(0.0, distance - o.width / 2.0);
        } else {
            world_point_t box[4];
            rectangle(box, o.centre.x, o.centre.y, o.angle, o.width / 2.0, -o.width / 2.0, o.height);
            if (!separated(robot, box) && !separated(box, robot)) { distance = 0.0; }
            else {
                for (size_t j = 0; j < 4u; ++j) {
                    for (size_t k = 0; k < 4u; ++k) {
                        distance = fmin(distance, segment_distance(robot[j], box[k], box[(k + 1u) % 4u]));
                        distance = fmin(distance, segment_distance(box[j], robot[k], robot[(k + 1u) % 4u]));
                    }
                }
            }
        }
        if (distance < best) { best = distance; *nearest = i; }
    }
    return best;
}
void world_advance(uint64_t start_us, uint64_t end_us) {
    if (!active || state.contact) { return; }
    uint64_t time = start_us;
    while (time < end_us) {
        uint64_t step = end_us - time;
        if (step > 100u) { step = 100u; }
        const double target[2] = {world_target_speed(0u, state.duty[0]), world_target_speed(1u, state.duty[1])};
        const double bound = fmax(fabs(state.speed[0]), fabs(target[0])) + fmax(fabs(state.speed[1]), fabs(target[1]));
        const double radius = hypot(fmax(fabs(cfg.front), fabs(cfg.rear)), cfg.width / 2.0);
        const double max_vertex_speed = bound * (0.5 + radius / cfg.track);
        size_t nearest = 0;
        const double gap = clearance(&nearest);
        if (gap == 0.0 || (obstacle_count != 0u && max_vertex_speed > 0.0 && gap < max_vertex_speed * 0.000002)) {
            state.contact = true; state.contact_us = time; state.obstacle = nearest;
            state.speed[0] = state.speed[1] = 0.0;
            sim_event_at(time, "contact", "\"obstacle_index\":%zu,\"x_mm\":%.9f,\"y_mm\":%.9f,\"heading_deg\":%.9f,\"clearance_mm\":%.9f",
                nearest, state.x, state.y, state.heading * 180.0 / PI, gap);
            return;
        }
        if (max_vertex_speed > 0.0 && obstacle_count != 0u) {
            const double safe_us = floor(gap / max_vertex_speed * 500000.0);
            if (safe_us < (double)step) { step = (uint64_t)fmax(1.0, safe_us); }
        }
        const double dt = (double)step / 1000000.0;
        double ds[2];
        for (size_t i = 0; i < 2u; ++i) {
            const double tau = cfg.tau_ms[i] / 1000.0;
            const double decay = exp(-dt / tau);
            ds[i] = target[i] * dt + (state.speed[i] - target[i]) * tau * (1.0 - decay);
            state.speed[i] = target[i] + (state.speed[i] - target[i]) * decay;
            state.counts[i] += ds[i] * COUNTS_PER_TURN / (cfg.diameter * 204.204 / 65.0);
        }
        const double angle = (ds[1] - ds[0]) / cfg.track;
        const double half = angle / 2.0;
        const double distance = (ds[0] + ds[1]) / 2.0 * (fabs(half) < 1e-12 ? 1.0 : sin(half) / half);
        state.x += distance * cos(state.heading + half);
        state.y += distance * sin(state.heading + half);
        state.heading += angle;
        time += step;
    }
}
void world_servo(uint16_t pulse) { servo_deg = ((double)pulse - 1500.0) * 0.09; }
static double cross(world_point_t a, world_point_t b) { return a.x*b.y - a.y*b.x; }
static void echo_candidate(world_echo_t *best, world_point_t p, size_t index, double heading,
                           world_point_t normal, bool corner) {
    const double x=p.x-best->origin.x, y=p.y-best->origin.y;
    const double distance=hypot(x,y);
    const double offset=atan2(sin(atan2(y,x)-heading),cos(atan2(y,x)-heading));
    if (distance>=1e-9 && fabs(offset)>PI/12+1e-12) { return; }
    if (distance<best->surface_distance) {
        best->surface_hit=true; best->surface_distance=distance;
        best->surface_point=p; best->surface_obstacle=index;
    }
    const obstacle_t o=obstacles[index];
    /* Outward normal dotted with P->sensor: 1 is normal incidence. */
    const double cosine=distance<1e-9 ? 1.0 : -(normal.x*x+normal.y*y)/distance;
    const bool corner_echo=corner && o.corner_range>0 && distance<=o.corner_range+1e-9;
    if (o.specular && !corner_echo && cosine+1e-12<cos(o.max_incidence)) { return; }
    if (distance<best->distance) {
        best->hit=true; best->distance=distance; best->point=p; best->obstacle=index;
    }
}
world_echo_t world_echo(void) {
    const double heading=state.heading+servo_deg*PI/180;
    const world_point_t axis=transform((world_point_t){cfg.servo_x,0},state.x,state.y,state.heading);
    world_echo_t best={.distance=1e30,.surface_distance=1e30,.angle_deg=heading*180/PI,
        .origin=transform((world_point_t){cfg.face_offset,0},axis.x,axis.y,heading)};
    if (!active) { return best; }
    for (size_t i=0;i<obstacle_count;++i) {
        const obstacle_t o=obstacles[i];
        if (o.kind==0) {
            world_point_t box[4];
            rectangle(box,o.centre.x,o.centre.y,o.angle,o.width/2,-o.width/2,o.height);
            /* On a visible planar face, incidence increases with range.
             * The exact geometric minimum in the sector is therefore also
             * the best incidence; no angular sampling or centre-ray proxy. */
            for (size_t j=0;j<4u;++j) {
                const world_point_t a=box[j],b=box[(j+1u)%4u];
                const world_point_t edge={b.x-a.x,b.y-a.y};
                const world_point_t delta={a.x-best.origin.x,a.y-best.origin.y};
                const double square=edge.x*edge.x+edge.y*edge.y;
                const double t=fmax(0,fmin(1,-(delta.x*edge.x+delta.y*edge.y)/square));
                const double length=sqrt(square);
                const world_point_t normal={edge.y/length,-edge.x/length};
                echo_candidate(&best,a,i,heading,normal,true);
                echo_candidate(&best,(world_point_t){a.x+t*edge.x,a.y+t*edge.y},i,heading,normal,false);
                for (int side=-1;side<=1;side+=2) {
                    const double h=heading+(double)side*PI/12;
                    const world_point_t ray={cos(h),sin(h)};
                    const double denominator=cross(ray,edge);
                    if (fabs(denominator)<1e-12) { continue; }
                    const double range=cross(delta,edge)/denominator;
                    const double along=cross(delta,ray)/denominator;
                    if (range>=0 && along>=0 && along<=1) {
                        echo_candidate(&best,(world_point_t){best.origin.x+range*ray.x,best.origin.y+range*ray.y},i,heading,normal,false);
                    }
                }
            }
        } else {
            const double dx=o.centre.x-best.origin.x,dy=o.centre.y-best.origin.y;
            const double distance=hypot(dx,dy),radius=o.width/2;
            if (distance>0) {
                echo_candidate(&best,(world_point_t){o.centre.x-radius*dx/distance,o.centre.y-radius*dy/distance},i,heading,
                    (world_point_t){-dx/distance,-dy/distance},false);
            } else {
                echo_candidate(&best,(world_point_t){best.origin.x+radius*cos(heading),best.origin.y+radius*sin(heading)},i,heading,
                    (world_point_t){cos(heading),sin(heading)},false);
            }
            for (int side=-1;side<=1;side+=2) {
                const double h=heading+(double)side*PI/12;
                const double projection=dx*cos(h)+dy*sin(h);
                const double discriminant=radius*radius-distance*distance+projection*projection;
                if (discriminant<0) { continue; }
                for (int root=-1;root<=1;root+=2) {
                    const double range=projection+(double)root*sqrt(discriminant);
                    if (range>=0) {
                        const world_point_t p={best.origin.x+range*cos(h),best.origin.y+range*sin(h)};
                        echo_candidate(&best,p,i,heading,(world_point_t){(p.x-o.centre.x)/radius,(p.y-o.centre.y)/radius},false);
                    }
                }
            }
        }
    }
    return best;
}
void world_snapshot(uint64_t time_us) {
    if (!active) { return; }
    uint16_t raw[3]; world_ir(raw);
    int32_t left, right; world_counts(&left, &right);
    world_point_t ir[3];
    for (size_t i = 0; i < 3u; ++i) { ir[i] = transform(cfg.ir[i], state.x, state.y, state.heading); }
    sim_event_at(time_us, "world", "\"x_mm\":%.9f,\"y_mm\":%.9f,\"heading_deg\":%.9f,"
        "\"servo_deg\":%.9f,\"speed_mm_s\":[%.9f,%.9f],\"counts\":[%.9f,%.9f],\"encoder_counts\":[%" PRId32 ",%" PRId32 "],\"duty\":[%d,%d],"
        "\"ir_raw\":[%u,%u,%u],\"ir_points_mm\":[[%.9f,%.9f],[%.9f,%.9f],[%.9f,%.9f]],\"contact\":%s",
        state.x, state.y, state.heading * 180.0 / PI, servo_deg, state.speed[0], state.speed[1], state.counts[0], state.counts[1], left, right,
        state.duty[0], state.duty[1], (unsigned int)raw[0], (unsigned int)raw[1], (unsigned int)raw[2],
        ir[0].x, ir[0].y, ir[1].x, ir[1].y, ir[2].x, ir[2].y, state.contact ? "true" : "false");
}

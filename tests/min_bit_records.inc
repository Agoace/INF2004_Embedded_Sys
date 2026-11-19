/* Record tables for test_min_bit_proof.c. One block per stored record.
 * Offsets are bit numbers inside one stored element, least significant
 * bit of the first byte first (the layout comments in each pack header).
 */

/* ---- value lists built at start ------------------------------------------ */

static int64_t pulse_list[73];      /* servo pulses for -36..36 deg */
static int64_t deg_list[8];         /* 0,5,..,30 and 255 (blocked) */
static int64_t geometry_list[1024]; /* 0..1022 and NONE */
static int64_t letter_list[45];     /* 0 and the Code39 alphabet plus '*' */
static int64_t reason_list[19];     /* BW0..10 and BP100..107 */
static int64_t cpu_list[1002];      /* 0..1000 permille and UNKNOWN */

static void build_lists(void)
{
    for (int a = -36; a <= 36; ++a) {
        const int32_t off = a >= 0 ? (a * 111 + 5) / 10
                                   : -((-a * 111 + 5) / 10);
        pulse_list[a + 36] = 1500 + off;
    }
    for (unsigned int i = 0u; i < 7u; ++i) {
        deg_list[i] = (int64_t)(i * OBSTACLE_PACK_DEG_STEP);
    }
    deg_list[7] = OBSTACLE_PACK_DEG_BLOCKED;
    for (unsigned int i = 0u; i <= OBSTACLE_PACK_GEOMETRY_MAX; ++i) {
        geometry_list[i] = i;
    }
    geometry_list[1023] = RM_OBSTACLE_GEOMETRY_NONE;
    const char alphabet[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*";
    letter_list[0] = 0;
    for (unsigned int i = 0u; i < 44u; ++i) {
        letter_list[i + 1u] = (uint8_t)alphabet[i];
    }
    for (unsigned int i = 0u; i <= 10u; ++i) {
        reason_list[i] = i;
    }
    for (unsigned int i = 0u; i < 8u; ++i) {
        reason_list[11u + i] = 100 + (int64_t)i;
    }
    for (unsigned int i = 0u; i <= 1000u; ++i) {
        cpu_list[i] = i;
    }
    cpu_list[1001] = RM_CPU_LOAD_UNKNOWN;
}

static const int64_t bad_angle30[] = {-31, 31};
static const int64_t bad_angle36[] = {-37, 37};

/* ---- motion A1 (subsystem2/motion_pack.h) -------------------------------- */

typedef struct {
    motion_status_t s;
    uint32_t us;
} motion_native_t;

static bool motion_pack_w(uint8_t *out, const void *n)
{
    const motion_native_t *m = n;
    motion_packed_t p;
    if (!motion_pack(&p, &m->s, m->us)) {
        return false;
    }
    memcpy(out, &p, sizeof(p));
    return true;
}
static bool motion_unpack_w(void *n, const uint8_t *in)
{
    motion_native_t *m = n;
    motion_packed_t p;
    memcpy(&p, in, sizeof(p));
    return motion_unpack(&m->s, &m->us, &p);
}
FIELD_IO(motion_native_t, m_counts, v->s.counts[i])
FIELD_IO(motion_native_t, m_travel, v->s.travel_mm[i])
FIELD_IO(motion_native_t, m_heading, v->s.heading_cdeg)
FIELD_IO(motion_native_t, m_ticks, v->s.ticks)
FIELD_IO(motion_native_t, m_tick_max, v->s.tick_max_us)
FIELD_IO(motion_native_t, m_gap_min, v->s.gap_min_us)
FIELD_IO(motion_native_t, m_gap_max, v->s.gap_max_us)
FIELD_IO(motion_native_t, m_late, v->s.late_wakes)
FIELD_IO(motion_native_t, m_missed, v->s.missed_wakes)
FIELD_IO(motion_native_t, m_us, v->us)
FIELD_IO(motion_native_t, m_speed, v->s.speed_mm_s[i])
FIELD_IO(motion_native_t, m_duty, v->s.duty_permille[i])
FIELD_IO(motion_native_t, m_servo, v->s.servo_deg)
FIELD_IO(motion_native_t, m_pulse, v->s.servo_us)
FIELD_IO(motion_native_t, m_settled, v->s.servo_settled)
FIELD_IO(motion_native_t, m_faults, v->s.faults)
FIELD_IO(motion_native_t, m_armed, v->s.watchdog_armed)
static const int64_t bad_duty[] = {-801, 801};
static const int64_t bad_pulse[] = {1501, 1099};
static const int64_t bad_faults[] = {16};
static const field_t motion_fields[] = {
    {"counts", 0, 32, 2, I32, .range = "int32 (wrapping count)", .set = IO(m_counts)},
    {"travel_mm", 64, 32, 2, I32, .range = "int32", .set = IO(m_travel)},
    {"heading_cdeg", 128, 32, 1, I32, .range = "int32", .set = IO(m_heading)},
    {"ticks", 160, 32, 1, U32, .range = "u32 (wrapping counter)", .set = IO(m_ticks)},
    {"tick_max_us", 192, 32, 1, U32, .range = "u32", .set = IO(m_tick_max)},
    {"gap_min_us", 224, 32, 1, U32, .range = "u32", .set = IO(m_gap_min)},
    {"gap_max_us", 256, 32, 1, U32, .range = "u32", .set = IO(m_gap_max)},
    {"late_wakes", 288, 32, 1, U32, .range = "u32 (wrapping counter)", .set = IO(m_late)},
    {"missed_wakes", 320, 32, 1, U32, .range = "u32 (wrapping counter)", .set = IO(m_missed)},
    {"publication us", 352, 32, 1, U32, .range = "u32 (wrapping time)", .set = IO(m_us)},
    {"speed_mm_s", 384, 16, 2, I16, .range = "int16 (code clamp INT16_MIN..MAX)", .set = IO(m_speed)},
    {"duty_permille", 416, 11, 2, RANGE(-800, 800), BAD(bad_duty), .range = "-800..800", .set = IO(m_duty)},
    {"servo_deg", 438, 7, 1, RANGE(-36, 36), BAD(bad_angle36), .range = "-36..36", .set = IO(m_servo)},
    {"servo_us (pulse code)", 448, 7, 1, LIST(pulse_list), BAD(bad_pulse), .range = "73 pulses of -36..36 deg", .set = IO(m_pulse)},
    {"servo_settled", 455, 1, 1, RANGE(0, 1), .range = "bool", .set = IO(m_settled)},
    {"faults", 456, 4, 1, .flags = 0xf, BAD(bad_faults), .range = "4 independent bits", .set = IO(m_faults)},
    {"watchdog_armed", 460, 1, 1, RANGE(0, 1), .range = "bool", .set = IO(m_armed)},
};
static const record_t motion_record = {
    "Motion A1 observation", "app_program/subsystem2/motion_pack.h",
    sizeof(motion_native_t), 60, 480, 1, 32, L_SEPARATE,
    motion_pack_w, motion_unpack_w, motion_fields,
    sizeof(motion_fields) / sizeof(motion_fields[0]), 22, 60, 60, 60,
    "The pulse code repeats the servo angle; removing it is category B5 "
    "(not approved)."};

/* ---- sonar board and history (subsystem5/sonar_pack.h) ------------------- */

static bool sonar_pack_w(uint8_t *out, const void *n)
{
    uint32_t w[SONAR_PACK_WORDS];
    if (!sonar_pack(n, w)) {
        return false;
    }
    memcpy(out, w, sizeof(w));
    return true;
}
static bool sonar_unpack_w(void *n, const uint8_t *in)
{
    uint32_t w[SONAR_PACK_WORDS];
    memcpy(w, in, sizeof(w));
    sonar_unpack(w, n);
    return true;
}
FIELD_IO(sonar_snapshot_t, s_generation, v->generation)
FIELD_IO(sonar_snapshot_t, s_id, v->id)
FIELD_IO(sonar_snapshot_t, s_ping_sequence, v->ping_sequence)
FIELD_IO(sonar_snapshot_t, s_result_sequence, v->result_sequence)
FIELD_IO(sonar_snapshot_t, s_trigger, v->trigger_ms)
FIELD_IO(sonar_snapshot_t, s_timestamp, v->timestamp_ms)
FIELD_IO(sonar_snapshot_t, s_deadline, v->deadline_ms)
FIELD_IO(sonar_snapshot_t, s_ping_generation, v->ping_generation)
FIELD_IO(sonar_snapshot_t, s_ping_id, v->ping_id)
FIELD_IO(sonar_snapshot_t, s_echo, v->echo)
FIELD_IO(sonar_snapshot_t, s_angle, v->angle)
FIELD_IO(sonar_snapshot_t, s_ping_angle, v->ping_angle)
FIELD_IO(sonar_snapshot_t, s_status, v->status)
FIELD_IO(sonar_snapshot_t, s_pending, v->pending)
FIELD_IO(sonar_snapshot_t, s_valid, v->valid)
static const int64_t bad_status3[] = {4};
#define SONAR_FIELDS(echo_reason)                                              \
    {"generation", 0, 32, 1, U32, .range = "u32 (wrapping)", .set = IO(s_generation)}, \
    {"id", 32, 32, 1, U32, .range = "u32 (wrapping)", .set = IO(s_id)},               \
    {"ping_sequence", 64, 32, 1, U32, .range = "u32 (wrapping)", .set = IO(s_ping_sequence)}, \
    {"result_sequence", 96, 32, 1, U32, .range = "u32 (wrapping)", .set = IO(s_result_sequence)}, \
    {"trigger_ms", 128, 32, 1, U32, .range = "u32 (wrapping time)", .set = IO(s_trigger)}, \
    {"timestamp_ms", 160, 32, 1, U32, .range = "u32 (wrapping time)", .set = IO(s_timestamp)}, \
    {"deadline_ms", 192, 32, 1, U32, .range = "u32 (wrapping time)", .set = IO(s_deadline)}, \
    {"ping_generation", 224, 32, 1, U32, .range = "u32 (wrapping)", .set = IO(s_ping_generation)}, \
    {"ping_id", 256, 32, 1, U32, .range = "u32 (wrapping)", .set = IO(s_ping_id)},     \
    {"echo", 288, 16, 1, RANGE(0, 0x3fff), .reason = (echo_reason),                   \
     .range = "ultrasonic word: distance 9, status 2, streak 3 bits", .set = IO(s_echo)}, \
    {"angle", 304, 6, 1, RANGE(-30, 30), BAD(bad_angle30), .range = "-30..30", .set = IO(s_angle)}, \
    {"ping_angle", 310, 6, 1, RANGE(-30, 30), BAD(bad_angle30), .range = "-30..30", .set = IO(s_ping_angle)}, \
    {"status", 316, 2, 1, RANGE(0, 3), BAD(bad_status3), .range = "0..3", .set = IO(s_status)}, \
    {"pending", 318, 1, 1, RANGE(0, 1), .range = "bool", .set = IO(s_pending)},       \
    {"valid", 319, 1, 1, RANGE(0, 1), .range = "bool", .set = IO(s_valid)}
static const field_t sonar_board_fields[] = {SONAR_FIELDS(R_WORD_SLACK)};
static const field_t sonar_history_fields[] = {SONAR_FIELDS(R_GAP)};
static const record_t sonar_board_record = {
    "Sonar board", "app_program/subsystem5/sonar_pack.h",
    sizeof(sonar_snapshot_t), 40, 320, 1, 32, L_SEPARATE,
    sonar_pack_w, sonar_unpack_w, sonar_board_fields,
    sizeof(sonar_board_fields) / sizeof(sonar_board_fields[0]), 0, 40, 40,
    40, "Echo keeps all 16 bits of the native word; the top 2 bits are always "
    "zero. Word 9 stays one word, so 14 bits save 0 B."};
static const record_t sonar_history_record = {
    "Sonar history", "app_program/subsystem5/sonar_pack.h",
    sizeof(sonar_snapshot_t), 40, 320, SONAR_HISTORY_CAPACITY, 32, L_ARRAY,
    sonar_pack_w, sonar_unpack_w, sonar_history_fields,
    sizeof(sonar_history_fields) / sizeof(sonar_history_fields[0]), 0, 640,
    640, 636, "Same layout as the board. Only a bit stream over all 16 "
    "entries can use the 2 echo bits (4 B)."};

/* ---- obstacle board (vehicle/obstacle_pack.h) ----------------------------- */

static bool obstacle_pack_w(uint8_t *out, const void *n)
{
    uint32_t w[OBSTACLE_PACK_WORDS];
    if (!obstacle_pack(n, w)) {
        return false;
    }
    memcpy(out, w, sizeof(w));
    return true;
}
static bool obstacle_unpack_w(void *n, const uint8_t *in)
{
    uint32_t w[OBSTACLE_PACK_WORDS];
    memcpy(w, in, sizeof(w));
    obstacle_unpack(w, n);
    return true;
}
FIELD_IO(rm_obstacle_t, o_scan, v->scan_id)
FIELD_IO(rm_obstacle_t, o_uptime, v->uptime_ms)
FIELD_IO(rm_obstacle_t, o_boot, v->boot_id)
FIELD_IO(rm_obstacle_t, o_flags, v->flags)
FIELD_IO(rm_obstacle_t, o_decision, v->decision)
FIELD_IO(rm_obstacle_t, o_angle, v->closest_angle_deg)
FIELD_IO(rm_obstacle_t, o_closest, v->closest_mm)
FIELD_IO(rm_obstacle_t, o_geo, v->geometry_flags)
FIELD_IO(rm_obstacle_t, o_profile, v->profile_word)
FIELD_IO(rm_obstacle_t, o_rstep, v->right_free_step)
FIELD_IO(rm_obstacle_t, o_lstep, v->left_free_step)
FIELD_IO(rm_obstacle_t, o_rdeg, v->right_clear_deg)
FIELD_IO(rm_obstacle_t, o_ldeg, v->left_clear_deg)
FIELD_IO(rm_obstacle_t, o_width, v->width_mm)
FIELD_IO(rm_obstacle_t, o_lclear, v->left_clear_mm)
FIELD_IO(rm_obstacle_t, o_rclear, v->right_clear_mm)
static const int64_t bad_decision[] = {3};
static const int64_t bad_closest[] = {512};
static const int64_t bad_geo[] = {32};
static const int64_t bad_profile[] = {0x8000};
static const int64_t bad_step[] = {8};
static const int64_t bad_deg[] = {1, 35};
static const int64_t bad_geometry[] = {1023};
static const field_t obstacle_fields[] = {
    {"scan_id", 0, 32, 1, U32, .range = "u32 (wrapping)", .set = IO(o_scan)},
    {"uptime_ms", 32, 32, 1, U32, .range = "u32 (wrapping time)", .set = IO(o_uptime)},
    {"boot_id", 64, 32, 1, U32, .range = "u32 (random)", .set = IO(o_boot)},
    {"flags", 96, 8, 1, .flags = 0xff, .range = "8 independent bits", .set = IO(o_flags)},
    {"decision", 104, 2, 1, RANGE(0, 2), BAD(bad_decision), .range = "0..2", .set = IO(o_decision)},
    {"closest_angle_deg", 106, 6, 1, RANGE(-30, 30), BAD(bad_angle30), .range = "-30..30", .set = IO(o_angle)},
    {"closest_mm", 112, 9, 1, RANGE(0, 511), BAD(bad_closest), .range = "0..511", .set = IO(o_closest)},
    {"geometry_flags", 121, 5, 1, .flags = 0x1f, BAD(bad_geo), .range = "5 independent bits", .set = IO(o_geo)},
    {"profile_word", 128, 15, 1, RANGE(0, 0x7fff), BAD(bad_profile), .range = "0..0x7fff", .set = IO(o_profile)},
    {"right_free_step", 143, 3, 1, RANGE(0, 7), BAD(bad_step), .range = "0..7", .set = IO(o_rstep)},
    {"left_free_step", 146, 3, 1, RANGE(0, 7), BAD(bad_step), .range = "0..7", .set = IO(o_lstep)},
    {"right_clear_deg", 149, 3, 1, LIST(deg_list), BAD(bad_deg), .range = "0,5,..,30 or 255", .set = IO(o_rdeg)},
    {"left_clear_deg", 152, 3, 1, LIST(deg_list), BAD(bad_deg), .range = "0,5,..,30 or 255", .set = IO(o_ldeg)},
    {"width_mm", 160, 10, 1, LIST(geometry_list), BAD(bad_geometry), .range = "0..1022 or NONE", .set = IO(o_width)},
    {"left_clear_mm", 170, 10, 1, LIST(geometry_list), BAD(bad_geometry), .range = "0..1022 or NONE", .set = IO(o_lclear)},
    {"right_clear_mm", 180, 10, 1, LIST(geometry_list), BAD(bad_geometry), .range = "0..1022 or NONE", .set = IO(o_rclear)},
};
static const record_t obstacle_record = {
    "Obstacle board", "app_program/vehicle/obstacle_pack.h",
    sizeof(rm_obstacle_t), 24, 192, 1, 32, L_SEPARATE,
    obstacle_pack_w, obstacle_unpack_w, obstacle_fields,
    sizeof(obstacle_fields) / sizeof(obstacle_fields[0]), 9, 24, 24, 24, 0};

/* ---- vehicle A1 (vehicle/vehicle_pack.h) ---------------------------------- */

static bool vehicle_pack_w(uint8_t *out, const void *n)
{
    vehicle_packed_t p;
    if (!vehicle_pack(&p, n)) {
        return false;
    }
    memcpy(out, &p, sizeof(p));
    return true;
}
static bool vehicle_unpack_w(void *n, const uint8_t *in)
{
    vehicle_packed_t p;
    memcpy(&p, in, sizeof(p));
    return vehicle_unpack(n, &p);
}
FIELD_IO(vehicle_snapshot_t, v_timestamp, v->timestamp_us)
FIELD_IO(vehicle_snapshot_t, v_sequence, v->sequence)
FIELD_IO(vehicle_snapshot_t, v_wake_lost, v->wake_lost)
FIELD_IO(vehicle_snapshot_t, v_overruns, v->overruns)
FIELD_IO(vehicle_snapshot_t, v_response, v->response_max_us)
FIELD_IO(vehicle_snapshot_t, v_handoffs, v->handoffs)
FIELD_IO(vehicle_snapshot_t, v_command_id, v->command_id)
FIELD_IO(vehicle_snapshot_t, v_left, v->left_mm_s)
FIELD_IO(vehicle_snapshot_t, v_right, v->right_mm_s)
FIELD_IO(vehicle_snapshot_t, v_servo, v->servo_deg)
FIELD_IO(vehicle_snapshot_t, v_state, v->state)
FIELD_IO(vehicle_snapshot_t, v_module, v->module)
FIELD_IO(vehicle_snapshot_t, v_phase, v->phase)
FIELD_IO(vehicle_snapshot_t, v_opcode, v->command_opcode)
FIELD_IO(vehicle_snapshot_t, v_status, v->command_status)
FIELD_IO(vehicle_snapshot_t, v_active, v->active)
FIELD_IO(vehicle_snapshot_t, v_calibrated, v->calibrated)
static const int64_t bad_state2[] = {3};
static const int64_t bad_module4[] = {5};
static const int64_t bad_status5[] = {6};
static const field_t vehicle_fields[] = {
    {"timestamp_us", 0, 32, 1, U32, .range = "u32 (wrapping time)", .set = IO(v_timestamp)},
    {"sequence", 32, 32, 1, U32, .range = "u32 (wrapping)", .set = IO(v_sequence)},
    {"wake_lost", 64, 32, 1, U32, .range = "u32 (wrapping counter)", .set = IO(v_wake_lost)},
    {"overruns", 96, 32, 1, U32, .range = "u32 (wrapping counter)", .set = IO(v_overruns)},
    {"response_max_us", 128, 32, 1, U32, .range = "u32", .set = IO(v_response)},
    {"handoffs", 160, 32, 1, U32, .range = "u32 (wrapping counter)", .set = IO(v_handoffs)},
    {"command_id", 192, 32, 1, U32, .range = "u32 (remote id)", .set = IO(v_command_id)},
    {"left_mm_s", 224, 16, 1, I16, .range = "int16", .set = IO(v_left)},
    {"right_mm_s", 240, 16, 1, I16, .range = "int16", .set = IO(v_right)},
    {"servo_deg", 256, 7, 1, RANGE(-36, 36), BAD(bad_angle36), .range = "-36..36", .set = IO(v_servo)},
    {"state", 263, 2, 1, RANGE(0, 2), BAD(bad_state2), .range = "0..2", .set = IO(v_state)},
    {"module", 265, 3, 1, RANGE(0, 4), BAD(bad_module4), .range = "0..4", .set = IO(v_module)},
    {"phase", 268, 2, 1, RANGE(0, 3), .range = "0..3", .set = IO(v_phase)},
    {"command_opcode", 270, 3, 1, RANGE(0, 4), BAD(bad_module4), .range = "0..4", .set = IO(v_opcode)},
    {"command_status", 273, 3, 1, RANGE(0, 5), BAD(bad_status5), .range = "0..5", .set = IO(v_status)},
    {"active", 276, 1, 1, RANGE(0, 1), .range = "bool", .set = IO(v_active)},
    {"calibrated", 277, 1, 1, RANGE(0, 1), .range = "bool", .set = IO(v_calibrated)},
};
static const record_t vehicle_record = {
    "Vehicle A1 snapshot", "app_program/vehicle/vehicle_pack.h",
    sizeof(vehicle_snapshot_t), 36, 288, 1, 32, L_SEPARATE,
    vehicle_pack_w, vehicle_unpack_w, vehicle_fields,
    sizeof(vehicle_fields) / sizeof(vehicle_fields[0]), 10, 36, 36, 36, 0};

/* ---- KR1 RAM-log row (vehicle/vehicle_log.h) ------------------------------ */

typedef struct {
    vr_values_t v;
    uint8_t flags; /* decoded only; vr_pack computes it from the clamp */
} kr1_native_t;

static bool kr1_pack_w(uint8_t *out, const void *n)
{
    vr_row_t row;
    vr_pack(&row, &((const kr1_native_t *)n)->v);
    memcpy(out, row.bytes, VR_BYTES);
    return true;
}
static uint32_t kr1_bits(const uint8_t *b, unsigned int off, unsigned int w)
{
    uint32_t v = 0u;
    for (unsigned int i = 0u; i < w; ++i) {
        v |= (uint32_t)bit_at(b, off + i) << i;
    }
    return v;
}
static int32_t kr1_signed(uint32_t v, unsigned int w)
{
    return (v >> (w - 1u)) ? (int32_t)v - (int32_t)(1u << w) : (int32_t)v;
}
/* Independent decode from the VR_* offsets (as scripts/robot_log_decode.py). */
static bool kr1_unpack_w(void *n, const uint8_t *in)
{
    kr1_native_t *k = n;
    k->v.elapsed_ms = kr1_bits(in, VR_TIME, 16u) * 20u;
    k->v.state = (uint8_t)kr1_bits(in, VR_STATE, 2u);
    k->v.module = (uint8_t)kr1_bits(in, VR_MODULE, 3u);
    k->v.raw[0] = (uint16_t)kr1_bits(in, VR_L, 12u);
    k->v.raw[1] = (uint16_t)kr1_bits(in, VR_C, 12u);
    k->v.position = (int8_t)kr1_signed(kr1_bits(in, VR_POS, 8u), 8u);
    k->v.asked[0] = (int16_t)kr1_signed(kr1_bits(in, VR_ASK_L, 10u), 10u);
    k->v.asked[1] = (int16_t)kr1_signed(kr1_bits(in, VR_ASK_R, 10u), 10u);
    k->v.measured[0] = (int16_t)kr1_signed(kr1_bits(in, VR_MEAS_L, 10u), 10u);
    k->v.measured[1] = (int16_t)kr1_signed(kr1_bits(in, VR_MEAS_R, 10u), 10u);
    k->v.echo = (uint16_t)kr1_bits(in, VR_ECHO, 16u);
    k->v.servo = (int8_t)kr1_signed(kr1_bits(in, VR_SERVO, 7u), 7u);
    k->v.events = (uint8_t)kr1_bits(in, VR_EVENTS, 8u);
    k->flags = (uint8_t)kr1_bits(in, VR_FLAGS, 4u);
    return true;
}
static void set_k_time(void *p, unsigned int i, int64_t x)
{
    (void)i;
    ((kr1_native_t *)p)->v.elapsed_ms = (uint32_t)x * 20u;
}
static int64_t get_k_time(const void *p, unsigned int i)
{
    (void)i;
    return ((const kr1_native_t *)p)->v.elapsed_ms / 20u;
}
/* Flag bit 0 is the only flag vr_pack sets: an asked/measured speed outside
 * -512..511 was clamped. 512 and 511 store the same speed code. */
static void set_k_flags(void *p, unsigned int i, int64_t x)
{
    (void)i;
    ((kr1_native_t *)p)->v.asked[0] = x ? 512 : 511;
}
static int64_t get_k_flags(const void *p, unsigned int i)
{
    (void)i;
    return ((const kr1_native_t *)p)->flags;
}
FIELD_IO(kr1_native_t, k_state, v->v.state)
FIELD_IO(kr1_native_t, k_module, v->v.module)
FIELD_IO(kr1_native_t, k_raw, v->v.raw[i])
FIELD_IO(kr1_native_t, k_position, v->v.position)
FIELD_IO(kr1_native_t, k_asked, v->v.asked[i])
FIELD_IO(kr1_native_t, k_measured, v->v.measured[i])
FIELD_IO(kr1_native_t, k_echo, v->v.echo)
FIELD_IO(kr1_native_t, k_servo, v->v.servo)
FIELD_IO(kr1_native_t, k_events, v->v.events)
static const field_t kr1_fields[] = {
    {"time (20 ms ticks)", VR_TIME, 16, 1, U16, .range = "0..65535 (wraps; no run time limit in code)", .set = IO(k_time)},
    {"state", VR_STATE, 2, 1, RANGE(0, 2), .range = "0..2", .set = IO(k_state)},
    {"module", VR_MODULE, 3, 1, RANGE(0, 5), .range = "0..5 (5 = scan reverse)", .set = IO(k_module)},
    {"raw L, C", VR_L, 12, 2, RANGE(0, 4095), .range = "ADC 0..4095", .set = IO(k_raw)},
    {"position", VR_POS, 8, 1, RANGE(-100, 100), .range = "-100..100", .set = IO(k_position)},
    {"asked L, R", VR_ASK_L, 10, 2, RANGE(-512, 511), .range = "-512..511 (clamped)", .set = IO(k_asked)},
    {"measured L, R", VR_MEAS_L, 10, 2, RANGE(-512, 511), .range = "-512..511 (clamped)", .set = IO(k_measured)},
    {"echo", VR_ECHO, 16, 1, RANGE(0, 0x3fff), .reason = R_GAP, .range = "ultrasonic word: distance 9, status 2, streak 3 bits", .set = IO(k_echo)},
    {"servo", VR_SERVO, 7, 1, RANGE(-36, 36), .range = "-36..36", .set = IO(k_servo)},
    {"events", VR_EVENTS, 8, 1, .flags = 0xff, .range = "8 defined bits (lost, found, echo, ping, outcome 3, fault)", .set = IO(k_events)},
    {"flags", VR_FLAGS, 4, 1, RANGE(0, 1), .reason = R_GAP, .range = "bit 0 only (speed clamp)", .set = IO(k_flags)},
};
static const record_t kr1_record = {
    "KR1 RAM-log row", "app_program/vehicle/vehicle_log.h",
    sizeof(kr1_native_t), VR_BYTES, 128, VR_ROWS, 8, L_ARRAY,
    kr1_pack_w, kr1_unpack_w, kr1_fields,
    sizeof(kr1_fields) / sizeof(kr1_fields[0]), 0, 72016, 72016, 69203,
    "Slack: echo 2 bits and flags 3 bits. A row stays 16 B (123 bits) "
    "unless the time also drops 3 bits (120 bits), which needs a decode rule "
    "that the code does not have."};

/* ---- barcode frame and edge (subsystem3/barcode_evidence_pack.h) ---------- */

static bool frame_pack_w(uint8_t *out, const void *n)
{
    bcep_frame_t p;
    if (!bcep_frame_pack(&p, n)) {
        return false;
    }
    memcpy(out, &p, sizeof(p));
    return true;
}
static bool frame_unpack_w(void *n, const uint8_t *in)
{
    bcep_frame_t p;
    memcpy(&p, in, sizeof(p));
    return bcep_frame_unpack(n, &p);
}
#define FLOAT_IO(type, id, lval)                                            \
    static void set_##id(void *p, unsigned int i, int64_t x)              \
    {                                                                       \
        type *v = (type *)p;                                                \
        const uint32_t b = (uint32_t)x;                                     \
        (void)i;                                                            \
        memcpy(&lval, &b, 4u);                                              \
    }                                                                       \
    static int64_t get_##id(const void *p, unsigned int i)                 \
    {                                                                       \
        const type *v = (const type *)p;                                    \
        uint32_t b;                                                         \
        (void)i;                                                            \
        memcpy(&b, &lval, 4u);                                              \
        return b;                                                           \
    }
FIELD_IO(barcode_live_frame_t, f_epoch, v->record.epoch)
FIELD_IO(barcode_live_frame_t, f_sequence, v->record.sequence)
FIELD_IO(barcode_live_frame_t, f_read_ms, v->record.read_ms)
FIELD_IO(barcode_live_frame_t, f_read_counts, v->record.read_counts[i])
FIELD_IO(barcode_live_frame_t, f_end_counts, v->record.frame_end_counts[i])
FIELD_IO(barcode_live_frame_t, f_revision, v->record.eligibility_revision)
FIELD_IO(barcode_live_frame_t, f_command, v->record.command)
FIELD_IO(barcode_live_frame_t, f_valid, v->record.valid)
FIELD_IO(barcode_live_frame_t, f_source, v->record.source)
FIELD_IO(barcode_live_frame_t, f_letter, v->record.letter)
FIELD_IO(barcode_live_frame_t, f_direction, v->record.direction)
FIELD_IO(barcode_live_frame_t, f_flags, v->record.flags)
FIELD_IO(barcode_live_frame_t, f_version, v->record.version)
FIELD_IO(barcode_live_frame_t, f_index, v->index)
FIELD_IO(barcode_live_frame_t, f_reason, v->reason)
FIELD_IO(barcode_live_frame_t, f_start, v->start_us)
FIELD_IO(barcode_live_frame_t, f_edge, v->edge_us)
FIELD_IO(barcode_live_frame_t, f_frame, v->frame_us)
FIELD_IO(barcode_live_frame_t, f_eligible, v->eligible_us)
FIELD_IO(barcode_live_frame_t, f_terminal, v->terminal_us)
FIELD_IO(barcode_live_frame_t, f_control_age, v->control_age_ms)
FIELD_IO(barcode_live_frame_t, f_record_age, v->record_age_ms)
FIELD_IO(barcode_live_frame_t, f_spin_white, v->spin_white)
FIELD_IO(barcode_live_frame_t, f_leading_white, v->leading_white)
FLOAT_IO(barcode_live_frame_t, f_narrow, v->narrow_mm)
FLOAT_IO(barcode_live_frame_t, f_wide, v->wide_mm)
FLOAT_IO(barcode_live_frame_t, f_leading_mm, v->leading_mm)
FIELD_IO(barcode_live_frame_t, f_rise, v->rise)
FIELD_IO(barcode_live_frame_t, f_fall, v->fall)
static const int64_t bad_command[] = {5};
static const int64_t bad_bool[] = {2};
static const int64_t bad_source[] = {3};
static const int64_t bad_letter[] = {'a'};
static const int64_t bad_reason[] = {11, 108};
static const int64_t bad_leading[] = {221};
static const int64_t bad_rise[] = {351};
static const int64_t bad_fall[] = {276};
static const field_t frame_fields[] = {
    {"epoch", 0, 32, 1, U32, .range = "u32", .set = IO(f_epoch)},
    {"sequence", 32, 32, 1, U32, .range = "u32 (wrapping)", .set = IO(f_sequence)},
    {"read_ms", 64, 32, 1, U32, .range = "u32 (wrapping time)", .set = IO(f_read_ms)},
    {"read_counts", 96, 32, 2, I32, .range = "int32", .set = IO(f_read_counts)},
    {"frame_end_counts", 160, 32, 2, I32, .range = "int32", .set = IO(f_end_counts)},
    {"eligibility_revision", 224, 32, 1, U32, .range = "u32 (wrapping)", .set = IO(f_revision)},
    {"command", 256, 3, 1, RANGE(0, 4), BAD(bad_command), .range = "0..4", .set = IO(f_command)},
    {"valid", 259, 1, 1, RANGE(0, 1), BAD(bad_bool), .range = "0..1", .set = IO(f_valid)},
    {"source", 260, 2, 1, RANGE(0, 2), BAD(bad_source), .range = "0..2", .set = IO(f_source)},
    {"letter", 262, 6, 1, LIST(letter_list), BAD(bad_letter), .range = "NUL or 44 Code39 letters", .set = IO(f_letter)},
    {"direction", 268, 2, 1, RANGE(0, 2), BAD(bad_source), .range = "0..2", .set = IO(f_direction)},
    {"flags", 270, 8, 1, .flags = 0xff, .range = "8 bits", .set = IO(f_flags)},
    {"version", 278, 1, 1, RANGE(0, 1), BAD(bad_bool), .range = "0..1", .set = IO(f_version)},
    {"index", 279, 32, 1, U32, .range = "u32", .set = IO(f_index)},
    {"reason", 311, 5, 1, LIST(reason_list), BAD(bad_reason), .range = "0..10 or 100..107", .set = IO(f_reason)},
    {"start_us", 316, 32, 1, U32, .range = "u32 (wrapping time)", .set = IO(f_start)},
    {"edge_us", 348, 32, 1, U32, .range = "u32 (wrapping time)", .set = IO(f_edge)},
    {"frame_us", 380, 32, 1, U32, .range = "u32 (wrapping time)", .set = IO(f_frame)},
    {"eligible_us", 412, 32, 1, U32, .range = "u32 (wrapping time)", .set = IO(f_eligible)},
    {"terminal_us", 444, 32, 1, U32, .range = "u32 (wrapping time)", .set = IO(f_terminal)},
    {"control_age_ms", 476, 32, 1, U32, .range = "u32", .set = IO(f_control_age)},
    {"record_age_ms", 508, 32, 1, U32, .range = "u32", .set = IO(f_record_age)},
    {"spin_white", 540, 32, 1, U32, .range = "u32", .set = IO(f_spin_white)},
    {"leading_white", 572, 8, 1, RANGE(0, 220), BAD(bad_leading), .range = "0..220", .set = IO(f_leading_white)},
    {"narrow_mm", 580, 32, 1, U32, .range = "binary32 bits (exact)", .set = IO(f_narrow)},
    {"wide_mm", 612, 32, 1, U32, .range = "binary32 bits (exact)", .set = IO(f_wide)},
    {"leading_mm", 644, 32, 1, U32, .range = "binary32 bits (exact)", .set = IO(f_leading_mm)},
    {"rise", 676, 9, 1, RANGE(0, 350), BAD(bad_rise), .range = "0..350", .set = IO(f_rise)},
    {"fall", 685, 9, 1, RANGE(0, 275), BAD(bad_fall), .range = "0..275", .set = IO(f_fall)},
};
static const record_t frame_board_record = {
    "Barcode frame boards", "app_program/subsystem3/barcode_evidence_pack.h",
    sizeof(barcode_live_frame_t), 88, 704, 8, 32, L_SEPARATE,
    frame_pack_w, frame_unpack_w, frame_fields,
    sizeof(frame_fields) / sizeof(frame_fields[0]), 10, 704, 704, 704,
    "Eight separate boards (barcode_live_source.c FRAME_ROWS)."};
static const record_t frame_slot_record = {
    "Barcode failed-capture frames", "app_program/subsystem3/barcode_evidence_pack.h",
    sizeof(barcode_live_frame_t), 88, 704, 3, 32, L_SEPARATE,
    frame_pack_w, frame_unpack_w, frame_fields,
    sizeof(frame_fields) / sizeof(frame_fields[0]), 10, 264, 264, 264,
    "One frame in each of the three capture slots."};

static bool edge_pack_w(uint8_t *out, const void *n)
{
    uint32_t w[6] = {0};
    if (!bcep_edge_pack(w, 0u, n)) {
        return false;
    }
    memcpy(out, w, sizeof(w));
    return true;
}
static bool edge_unpack_w(void *n, const uint8_t *in)
{
    uint32_t w[6];
    memcpy(w, in, sizeof(w));
    return bcep_edge_unpack(n, w, 0u);
}
FIELD_IO(bcep_edge_t, e_us, v->us)
FIELD_IO(bcep_edge_t, e_motor, v->motor_us)
FIELD_IO(bcep_edge_t, e_left, v->left)
FIELD_IO(bcep_edge_t, e_right, v->right)
FLOAT_IO(bcep_edge_t, e_mm, v->mm)
FIELD_IO(bcep_edge_t, e_raw, v->raw)
FIELD_IO(bcep_edge_t, e_black, v->black)
FIELD_IO(bcep_edge_t, e_phase, v->phase)
static const int64_t bad_raw12[] = {4096};
static const int64_t bad_phase[] = {1, 4};
static const field_t edge_fields[] = {
    {"us", 0, 32, 1, U32, .range = "u32 (wrapping time)", .set = IO(e_us)},
    {"motor_us", 32, 32, 1, U32, .range = "u32 (wrapping time)", .set = IO(e_motor)},
    {"left", 64, 32, 1, U32, .range = "u32 count", .set = IO(e_left)},
    {"right", 96, 32, 1, U32, .range = "u32 count", .set = IO(e_right)},
    {"mm", 128, 32, 1, U32, .range = "binary32 bits (exact)", .set = IO(e_mm)},
    {"raw", 160, 12, 1, RANGE(0, 4095), BAD(bad_raw12), .range = "ADC 0..4095", .set = IO(e_raw)},
    {"black", 172, 1, 1, RANGE(0, 1), BAD(bad_bool), .range = "0..1", .set = IO(e_black)},
    {"phase", 173, 1, 1, RANGE(2, 3), BAD(bad_phase), .range = "WINDOW 2 or TAIL 3", .set = IO(e_phase)},
};
static const record_t edge_record = {
    "Barcode failed-capture edges", "app_program/subsystem3/barcode_evidence_pack.h",
    sizeof(bcep_edge_t), 24, 174, BCEP_EDGE_CAPACITY, 32, L_STREAMED,
    edge_pack_w, edge_unpack_w, edge_fields,
    sizeof(edge_fields) / sizeof(edge_fields[0]), 0, 1392, 1392, 1392,
    "Edges are packed back to back (174 bits each). There is one array in "
    "each of the three capture slots."};

/* ---- telemetry events, scan rows, INFO, index (subsystem1/telemetry_pack.h) */

static bool event_pack_w(uint8_t *out, const void *n)
{
    telemetry_event_packed_t p;
    if (!telemetry_event_pack(&p, n)) {
        return false;
    }
    memcpy(out, &p, sizeof(p));
    return true;
}
static bool event_unpack_w(void *n, const uint8_t *in)
{
    telemetry_event_packed_t p;
    memcpy(&p, in, sizeof(p));
    return telemetry_event_unpack(n, &p);
}
FIELD_IO(telemetry_event_t, ev_ms, v->ms)
FIELD_IO(telemetry_event_t, ev_word, v->word)
FIELD_IO(telemetry_event_t, ev_detail, v->detail)
FIELD_IO(telemetry_event_t, ev_kind, v->kind)
FIELD_IO(telemetry_event_t, ev_from, v->from)
FIELD_IO(telemetry_event_t, ev_to, v->to)
static const int64_t bad_kind[] = {0, 11};
static const int64_t bad_state5[] = {6};
static const field_t event_fields[] = {
    {"ms", 0, 32, 1, U32, .range = "u32 (wrapping time)", .set = IO(ev_ms)},
    {"word", 32, 16, 1, U16, .range = "u16 payload", .set = IO(ev_word)},
    {"detail", 48, 8, 1, RANGE(0, 255), .range = "u8 payload", .set = IO(ev_detail)},
    {"kind", 56, 4, 1, RANGE(1, 10), BAD(bad_kind), .range = "1..10", .set = IO(ev_kind)},
    {"from", 60, 3, 1, RANGE(0, 5), BAD(bad_state5), .range = "0..5", .set = IO(ev_from)},
    {"to", 63, 3, 1, RANGE(0, 5), BAD(bad_state5), .range = "0..5", .set = IO(ev_to)},
};
static const record_t event_record = {
    "Telemetry events", "app_program/subsystem1/telemetry_pack.h",
    sizeof(telemetry_event_t), 9, 72, TELEMETRY_EVENTS, 8, L_ARRAY,
    event_pack_w, event_unpack_w, event_fields,
    sizeof(event_fields) / sizeof(event_fields[0]), 6, 4608, 4224, 4224,
    "Each event starts on a byte, so 6 zero bits per event."};

static bool scan_pack_w(uint8_t *out, const void *n)
{
    telemetry_scan_packed_t p;
    if (!telemetry_scan_pack(&p, n)) {
        return false;
    }
    memcpy(out, &p, sizeof(p));
    return true;
}
static bool scan_unpack_w(void *n, const uint8_t *in)
{
    telemetry_scan_packed_t p;
    memcpy(&p, in, sizeof(p));
    return telemetry_scan_unpack(n, &p);
}
FIELD_IO(telemetry_scan_t, sc_id, v->scan_id)
FIELD_IO(telemetry_scan_t, sc_duration, v->duration_ms)
FIELD_IO(telemetry_scan_t, sc_fine, v->fine_mask)
FIELD_IO(telemetry_scan_t, sc_hits, v->coarse_hits)
FIELD_IO(telemetry_scan_t, sc_attempt, v->attempt)
static const int64_t bad_fine[] = {8192};
static const int64_t bad_hits[] = {6};
static const int64_t bad_attempt[] = {0, 3};
static const field_t scan_fields[] = {
    {"scan_id", 0, 32, 1, U32, .range = "u32 (wrapping)", .set = IO(sc_id)},
    {"duration_ms", 32, 32, 1, U32, .range = "u32", .set = IO(sc_duration)},
    {"fine_mask", 64, 13, 1, .flags = 0x1fff, BAD(bad_fine), .range = "13 independent bits", .set = IO(sc_fine)},
    {"coarse_hits", 77, 3, 1, RANGE(0, 5), BAD(bad_hits), .range = "0..5", .set = IO(sc_hits)},
    {"attempt", 80, 1, 1, RANGE(1, 2), BAD(bad_attempt), .range = "1..2", .set = IO(sc_attempt)},
};
static const record_t scan_record = {
    "Stopped scan rows", "app_program/subsystem1/telemetry_pack.h",
    sizeof(telemetry_scan_t), 11, 88, TELEMETRY_SCAN_ROWS, 8, L_ARRAY,
    scan_pack_w, scan_unpack_w, scan_fields,
    sizeof(scan_fields) / sizeof(scan_fields[0]), 7, 528, 486, 486,
    "Each row starts on a byte, so 7 zero bits per row."};

static bool info_pack_w(uint8_t *out, const void *n)
{
    telemetry_info_packed_t p;
    if (!telemetry_info_pack(&p, n)) {
        return false;
    }
    memcpy(out, &p, sizeof(p));
    return true;
}
static bool info_unpack_w(void *n, const uint8_t *in)
{
    telemetry_info_packed_t p;
    memcpy(&p, in, sizeof(p));
    return telemetry_info_unpack(n, &p);
}
FIELD_IO(telemetry_info_t, in_reason, *(uint8_t *)&v->reason[i])
FIELD_IO(telemetry_info_t, in_wake_lost, v->wake_lost)
FIELD_IO(telemetry_info_t, in_overruns, v->overruns)
FIELD_IO(telemetry_info_t, in_response, v->response_max_us)
FIELD_IO(telemetry_info_t, in_gap_min, v->gap_min_us)
FIELD_IO(telemetry_info_t, in_gap_max, v->gap_max_us)
FIELD_IO(telemetry_info_t, in_sampled, v->sampled)
FIELD_IO(telemetry_info_t, in_missed, v->missed)
FIELD_IO(telemetry_info_t, in_dropped, v->dropped)
FIELD_IO(telemetry_info_t, in_isr_max, v->isr_max_us)
FIELD_IO(telemetry_info_t, in_line_max, v->line_max_us)
FIELD_IO(telemetry_info_t, in_counts, v->counts[i])
FIELD_IO(telemetry_info_t, in_floor, v->floor[i])
FIELD_IO(telemetry_info_t, in_line, v->line[i])
FIELD_IO(telemetry_info_t, in_raw_bc, v->raw_bc)
FIELD_IO(telemetry_info_t, in_faults, v->faults)
static const field_t info_fields[] = {
    {"reason", 0, 8, 48, RANGE(0, 255), .range = "text bytes", .set = IO(in_reason)},
    {"wake_lost", 384, 32, 1, U32, .range = "u32 (wrapping counter)", .set = IO(in_wake_lost)},
    {"overruns", 416, 32, 1, U32, .range = "u32 (wrapping counter)", .set = IO(in_overruns)},
    {"response_max_us", 448, 32, 1, U32, .range = "u32", .set = IO(in_response)},
    {"gap_min_us", 480, 32, 1, U32, .range = "u32", .set = IO(in_gap_min)},
    {"gap_max_us", 512, 32, 1, U32, .range = "u32", .set = IO(in_gap_max)},
    {"sampled", 544, 32, 1, U32, .range = "u32 (wrapping counter)", .set = IO(in_sampled)},
    {"missed", 576, 32, 1, U32, .range = "u32 (wrapping counter)", .set = IO(in_missed)},
    {"dropped", 608, 32, 1, U32, .range = "u32 (wrapping counter)", .set = IO(in_dropped)},
    {"isr_max_us", 640, 32, 1, U32, .range = "u32", .set = IO(in_isr_max)},
    {"line_max_us", 672, 32, 1, U32, .range = "u32", .set = IO(in_line_max)},
    {"counts", 704, 32, 2, I32, .range = "int32", .set = IO(in_counts)},
    {"floor", 768, 12, 2, RANGE(0, 4095), BAD(bad_raw12), .range = "ADC 0..4095", .set = IO(in_floor)},
    {"line", 792, 12, 2, RANGE(0, 4095), BAD(bad_raw12), .range = "ADC 0..4095", .set = IO(in_line)},
    {"raw_bc", 816, 12, 1, RANGE(0, 4095), BAD(bad_raw12), .range = "ADC 0..4095", .set = IO(in_raw_bc)},
    {"faults", 828, 4, 1, .flags = 0xf, BAD(bad_faults), .range = "4 independent bits", .set = IO(in_faults)},
};
static const record_t info_record = {
    "Retained INFO", "app_program/subsystem1/telemetry_pack.h",
    sizeof(telemetry_info_t), 104, 832, 1, 8, L_SEPARATE,
    info_pack_w, info_unpack_w, info_fields,
    sizeof(info_fields) / sizeof(info_fields[0]), 0, 104, 104, 104, 0};

static bool index_pack_w(uint8_t *out, const void *n)
{
    telemetry_index_packed_t p;
    if (!telemetry_index_pack(&p, n)) {
        return false;
    }
    memcpy(out, &p, sizeof(p));
    return true;
}
static bool index_unpack_w(void *n, const uint8_t *in)
{
    telemetry_index_packed_t p;
    memcpy(&p, in, sizeof(p));
    return telemetry_index_unpack(n, &p);
}
FIELD_IO(telemetry_log_index_t, ix_generation, v->generation)
FIELD_IO(telemetry_log_index_t, ix_lost, v->lost)
FIELD_IO(telemetry_log_index_t, ix_newest, v->newest_ms)
FIELD_IO(telemetry_log_index_t, ix_rows, v->rows)
FIELD_IO(telemetry_log_index_t, ix_events, v->events)
static const int64_t bad_rows[] = {TELEMETRY_ROWS + 1};
static const int64_t bad_events[] = {TELEMETRY_EVENTS + 1};
static const field_t index_fields[] = {
    {"generation", 0, 32, 1, U32, .range = "u32 (wrapping)", .set = IO(ix_generation)},
    {"lost", 32, 32, 1, U32, .range = "u32 (wrapping counter)", .set = IO(ix_lost)},
    {"newest_ms", 64, 32, 1, U32, .range = "u32 (wrapping time)", .set = IO(ix_newest)},
    {"rows", 96, 13, 1, RANGE(0, TELEMETRY_ROWS), BAD(bad_rows), .range = "0..4501", .set = IO(ix_rows)},
    {"events", 109, 10, 1, RANGE(0, TELEMETRY_EVENTS), BAD(bad_events), .range = "0..512", .set = IO(ix_events)},
};
static const record_t index_record = {
    "Telemetry log index", "app_program/subsystem1/telemetry_pack.h",
    sizeof(telemetry_log_index_t), 16, 128, 1, 32, L_SEPARATE,
    index_pack_w, index_unpack_w, index_fields,
    sizeof(index_fields) / sizeof(index_fields[0]), 9, 16, 16, 16, 0};

/* ---- Health payload and S1 sonar copy (subsystem1/network_pack.h) --------- */

static bool health_pack_w(uint8_t *out, const void *n)
{
    network_health_packed_t p;
    if (!network_health_pack(&p, n)) {
        return false;
    }
    memcpy(out, &p, sizeof(p));
    return true;
}
static bool health_unpack_w(void *n, const uint8_t *in)
{
    network_health_packed_t p;
    memcpy(&p, in, sizeof(p));
    return network_health_unpack(n, &p);
}
#define HEALTH_IO(field) FIELD_IO(rm_heartbeat_t, h_##field, v->field)
NETWORK_HEALTH_U32_FIELDS(HEALTH_IO)
HEALTH_IO(wifi_start_status)
HEALTH_IO(stack_min_free)
HEALTH_IO(health_age_ms)
HEALTH_IO(cpu_load_permille)
HEALTH_IO(flags)
HEALTH_IO(reset_reason)
HEALTH_IO(wifi_link_status)
HEALTH_IO(mqtt_state)
HEALTH_IO(wifi_requested_priority)
HEALTH_IO(wifi_observed_priority)
HEALTH_IO(board_unavailable_mask)
#define HU32(name, off) \
    {#name, off, 32, 1, U32, .range = "u32", .set = IO(h_##name)}
static const int64_t bad_cpu[] = {1001};
static const int64_t bad_flags7[] = {128};
static const int64_t bad_reset[] = {3};
static const int64_t bad_link[] = {-4, 4};
static const int64_t bad_prio[] = {33};
static const int64_t bad_mask5[] = {32};
static const field_t health_fields[] = {
    HU32(uptime_ms, 0), HU32(connect_attempts, 32), HU32(telemetry_lost, 64),
    HU32(event_lost, 96), HU32(command_lost, 128),
    HU32(sonar_history_missed, 160), HU32(vehicle_overruns, 192),
    HU32(motion_late, 224), HU32(motion_missed, 256),
    HU32(mqtt_hook_max_us, 288), HU32(publish_ok, 320),
    HU32(publish_failed, 352), HU32(line_busy, 384), HU32(sonar_busy, 416),
    HU32(motion_busy, 448), HU32(vehicle_busy, 480), HU32(health_busy, 512),
    HU32(latest_sonar_history_sequence, 544),
    HU32(wifi_poll_gap_max_us, 576), HU32(wifi_poll_gap_samples, 608),
    HU32(boot_id, 640), HU32(board_unavailable, 672),
    {"wifi_start_status", 704, 32, 1, I32, .range = "int32 status", .set = IO(h_wifi_start_status)},
    {"stack_min_free", 736, 16, 1, U16, .range = "u16 bytes", .set = IO(h_stack_min_free)},
    {"health_age_ms", 752, 16, 1, U16, .range = "u16 ms", .set = IO(h_health_age_ms)},
    {"cpu_load_permille", 768, 10, 1, LIST(cpu_list), BAD(bad_cpu), .range = "0..1000 or UNKNOWN", .set = IO(h_cpu_load_permille)},
    {"flags", 778, 7, 1, .flags = 0x7f, BAD(bad_flags7), .range = "7 independent bits", .set = IO(h_flags)},
    {"reset_reason", 785, 2, 1, RANGE(0, 2), BAD(bad_reset), .range = "0..2", .set = IO(h_reset_reason)},
    {"wifi_link_status", 787, 3, 1, RANGE(-3, 3), BAD(bad_link), .range = "-3..3", .set = IO(h_wifi_link_status)},
    {"mqtt_state", 790, 2, 1, RANGE(0, 3), BAD(bad_status3), .range = "0..3", .set = IO(h_mqtt_state)},
    {"wifi_requested_priority", 792, 6, 1, RANGE(0, 32), BAD(bad_prio), .range = "0..32", .set = IO(h_wifi_requested_priority)},
    {"wifi_observed_priority", 798, 6, 1, RANGE(0, 32), BAD(bad_prio), .range = "0..32", .set = IO(h_wifi_observed_priority)},
    {"board_unavailable_mask", 804, 5, 1, .flags = 0x1f, BAD(bad_mask5), .range = "5 independent bits", .set = IO(h_board_unavailable_mask)},
};
static const record_t health_record = {
    "Health payload", "app_program/subsystem1/network_pack.h",
    sizeof(rm_heartbeat_t), 104, 832, 1, 32, L_SEPARATE,
    health_pack_w, health_unpack_w, health_fields,
    sizeof(health_fields) / sizeof(health_fields[0]), 23, 104, 104, 104,
    "The Health board also holds a u32 time, the fault snapshot below and a "
    "u32 phase (132 B with its sequence)."};

static bool ssc_pack_w(uint8_t *out, const void *n)
{
    network_sonar_packed_t p;
    if (!network_sonar_pack(&p, n)) {
        return false;
    }
    memcpy(out, &p, sizeof(p));
    return true;
}
static bool ssc_unpack_w(void *n, const uint8_t *in)
{
    network_sonar_packed_t p;
    memcpy(&p, in, sizeof(p));
    return network_sonar_unpack(n, &p);
}
#define SSC_IO(field) FIELD_IO(rm_sonar_t, c_##field, v->field)
NETWORK_SONAR_U32_FIELDS(SSC_IO)
SSC_IO(status)
SSC_IO(valid)
SSC_IO(pending)
SSC_IO(angle)
SSC_IO(ping_angle)
SSC_IO(echo)
#define CU32(name, off) \
    {#name, off, 32, 1, U32, .range = "u32", .set = IO(c_##name)}
static const int64_t bad_echo14[] = {0x4000};
static const field_t ssc_fields[] = {
    CU32(history_sequence, 0), CU32(generation, 32), CU32(request_id, 64),
    CU32(trigger_ms, 96), CU32(timestamp_ms, 128), CU32(ping_sequence, 160),
    CU32(boot_id, 192),
    {"status", 224, 2, 1, RANGE(0, 3), BAD(bad_status3), .range = "0..3", .set = IO(c_status)},
    {"valid", 226, 1, 1, RANGE(0, 1), .range = "bool", .set = IO(c_valid)},
    {"pending", 227, 1, 1, RANGE(0, 1), .range = "bool", .set = IO(c_pending)},
    {"angle", 228, 6, 1, RANGE(-30, 30), BAD(bad_angle30), .range = "-30..30", .set = IO(c_angle)},
    {"ping_angle", 234, 6, 1, RANGE(-30, 30), BAD(bad_angle30), .range = "-30..30", .set = IO(c_ping_angle)},
    {"echo", 240, 14, 1, RANGE(0, 0x3fff), BAD(bad_echo14), .range = "ultrasonic word, 14 bits", .set = IO(c_echo)},
};
static const record_t ssc_record = {
    "S1 pending sonar copy", "app_program/subsystem1/network_pack.h",
    sizeof(rm_sonar_t), 32, 256, 1, 32, L_SEPARATE,
    ssc_pack_w, ssc_unpack_w, ssc_fields,
    sizeof(ssc_fields) / sizeof(ssc_fields[0]), 2, 32, 32, 32, 0};

/* ---- fault mask F (platform/fault_mask.h, subsystem1/robot_health.c) ------ */

#define OWNER_RECORD(id, title, mask, rng)                                  \
    static bool id##_pack(uint8_t *out, const void *n)                    \
    {                                                                       \
        fault_mask_board_t b = {0};                                         \
        if (!fault_mask_publish(&b, (mask), *(const uint32_t *)n)) {        \
            return false;                                                   \
        }                                                                   \
        memcpy(out, &b.word, 4u);                                           \
        return true;                                                        \
    }                                                                       \
    static bool id##_unpack(void *n, const uint8_t *in)                   \
    {                                                                       \
        memcpy(n, in, 4u);                                                  \
        return true;                                                        \
    }                                                                       \
    FIELD_IO(uint32_t, id##_word, *v)                                       \
    static const int64_t id##_bad[] = {(int64_t)(~(mask)&FAULT_MASK_ALL)}; \
    static const field_t id##_fields[] = {                                  \
        {"owner word", 0, 32, 1, .flags = (mask), BAD(id##_bad),            \
         .reason = R_WORD_SLACK, .range = (rng), .set = IO(id##_word)}};    \
    static const record_t id##_record = {                                   \
        title, "app_program/platform/fault_mask.h", 4, 4, 32, 1, 32,       \
        L_SEPARATE, id##_pack, id##_unpack, id##_fields, 1, 0, 4, 4, 4,          \
        "One writer stores the whole word atomically; owners cannot share " \
        "a word without a shared read-modify-write."};
OWNER_RECORD(own_vehicle, "F owner word: Vehicle", FAULT_OWNER_VEHICLE,
             "bits 4, 6")
OWNER_RECORD(own_sonar, "F owner word: Sonar", FAULT_OWNER_SONAR, "bit 5")
OWNER_RECORD(own_telemetry, "F owner word: Telemetry", FAULT_OWNER_TELEMETRY,
             "bit 7")
OWNER_RECORD(own_network, "F owner word: Wi-Fi", FAULT_OWNER_NETWORK,
             "bits 8, 9")

static bool copy16_pack(uint8_t *out, const void *n)
{
    memcpy(out, n, 16u);
    return true;
}
static bool copy16_unpack(void *n, const uint8_t *in)
{
    memcpy(n, in, 16u);
    return true;
}
FIELD_IO(fault_mask_snapshot_t, fs_now, v->now)
FIELD_IO(fault_mask_snapshot_t, fs_run, v->run)
FIELD_IO(fault_mask_snapshot_t, fs_valid, v->valid)
FIELD_IO(fault_mask_snapshot_t, fs_epoch, v->epoch)
static const field_t fault_snapshot_fields[] = {
    {"now", 0, 32, 1, .flags = FAULT_MASK_ALL, .reason = R_GAP, .range = "10 fault bits", .set = IO(fs_now)},
    {"run", 32, 32, 1, .flags = FAULT_MASK_ALL, .reason = R_GAP, .range = "10 fault bits", .set = IO(fs_run)},
    {"valid", 64, 32, 1, .flags = FAULT_MASK_ALL, .reason = R_GAP, .range = "10 fault bits", .set = IO(fs_valid)},
    {"epoch", 96, 32, 1, U32, .range = "u32 run epoch", .set = IO(fs_epoch)},
};
static const record_t fault_snapshot_record = {
    "F Health fault snapshot", "app_program/platform/fault_mask.h",
    sizeof(fault_mask_snapshot_t), 16, 128, 2, 32, L_SEPARATE,
    copy16_pack, copy16_unpack, fault_snapshot_fields,
    sizeof(fault_snapshot_fields) / sizeof(fault_snapshot_fields[0]), 0,
    32, 32, 16,
    "Two copies: in the Health board and in the Health working views. The "
    "producer limits the masks (fault_mask_combine); the store does not."};

/* Mirror of the private fault_stop_capture_t (robot_health.c). */
typedef struct {
    uint32_t owner[5];
    uint32_t valid;
    uint32_t epoch;
    uint32_t us;
} stop_capture_t;
_Static_assert(sizeof(stop_capture_t) == 32u, "STOP capture payload");
static bool copy32_pack(uint8_t *out, const void *n)
{
    memcpy(out, n, 32u);
    return true;
}
static bool copy32_unpack(void *n, const uint8_t *in)
{
    memcpy(n, in, 32u);
    return true;
}
FIELD_IO(stop_capture_t, sc_motion, v->owner[0])
FIELD_IO(stop_capture_t, sc_vehicle, v->owner[1])
FIELD_IO(stop_capture_t, sc_sonar, v->owner[2])
FIELD_IO(stop_capture_t, sc_telemetry, v->owner[3])
FIELD_IO(stop_capture_t, sc_network, v->owner[4])
FIELD_IO(stop_capture_t, sc_valid, v->valid)
FIELD_IO(stop_capture_t, sc_epoch, v->epoch)
FIELD_IO(stop_capture_t, sc_us, v->us)
static const field_t stop_fields[] = {
    {"owner Motion", 0, 32, 1, .flags = FAULT_OWNER_MOTION, .reason = R_GAP, .range = "bits 0..3", .set = IO(sc_motion)},
    {"owner Vehicle", 32, 32, 1, .flags = FAULT_OWNER_VEHICLE, .reason = R_GAP, .range = "bits 4, 6", .set = IO(sc_vehicle)},
    {"owner Sonar", 64, 32, 1, .flags = FAULT_OWNER_SONAR, .reason = R_GAP, .range = "bit 5", .set = IO(sc_sonar)},
    {"owner Telemetry", 96, 32, 1, .flags = FAULT_OWNER_TELEMETRY, .reason = R_GAP, .range = "bit 7", .set = IO(sc_telemetry)},
    {"owner Wi-Fi", 128, 32, 1, .flags = FAULT_OWNER_NETWORK, .reason = R_GAP, .range = "bits 8, 9", .set = IO(sc_network)},
    {"valid", 160, 32, 1, .flags = FAULT_MASK_ALL, .reason = R_GAP, .range = "10 fault bits", .set = IO(sc_valid)},
    {"epoch", 192, 32, 1, U32, .range = "u32 run epoch", .set = IO(sc_epoch)},
    {"us", 224, 32, 1, U32, .range = "u32 (wrapping time)", .set = IO(sc_us)},
};
static const record_t stop_record = {
    "F STOP capture", "app_program/subsystem1/robot_health.c",
    sizeof(stop_capture_t), 32, 256, 1, 32, L_SEPARATE,
    copy32_pack, copy32_unpack, stop_fields,
    sizeof(stop_fields) / sizeof(stop_fields[0]), 0, 32, 32, 12,
    "Plain u32 words (36 B board with its sequence). The producer limits "
    "each owner word (capture_owner_values)."};

/* ---- all records ---------------------------------------------------------- */

static const record_t *const records[] = {
    &motion_record, &sonar_board_record, &sonar_history_record,
    &obstacle_record, &vehicle_record, &kr1_record, &frame_board_record,
    &frame_slot_record, &edge_record, &event_record, &index_record,
    &scan_record, &info_record, &health_record, &ssc_record,
    &own_vehicle_record, &own_sonar_record, &own_telemetry_record,
    &own_network_record, &fault_snapshot_record, &stop_record,
};

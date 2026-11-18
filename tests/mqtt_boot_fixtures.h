static uint32_t fixture_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8u) | ((uint32_t)p[2] << 16u) |
           ((uint32_t)p[3] << 24u);
}

/* Both boots intentionally restart generation, sequence and run IDs. */
static void test_boot_fixtures(void)
{
    const uint32_t ids[2] = {0x11223344u, 0xaabbccddu};
    const char *batch_names[2] = {"boot_a", "boot_b"};
    const char *begin_names[2] = {"log_begin_boot_a", "log_begin_boot_b"};
    const char *state_names[2] = {"state_boot_a", "state_boot_b"};
    const char *beat_names[2] = {"heartbeat_boot_a", "heartbeat_boot_b"};
    const char *status_names[2] = {"status_boot_a", "status_boot_b"};
    const char *offline_names[2] = {"status_offline_boot_a",
                                    "status_offline_boot_b"};
    const char *summary_names[2] = {"summary_boot_a", "summary_boot_b"};
    const char *detour_names[2] = {"detour_boot_a", "detour_boot_b"};
    const char *event_names[2] = {"event_boot_a", "event_boot_b"};
    const char *sonar_names[2] = {"sonar_boot_a", "sonar_boot_b"};
    const char *obstacle_names[2] = {"obstacle_boot_a", "obstacle_boot_b"};
    uint8_t bytes[RM_HEARTBEAT_SIZE];

    for (unsigned int boot = 0u; boot < 2u; ++boot) {
        uint8_t row[RM_LOG_RECORD_SIZE] = {0};
        rm_log_batch_t batch = {.kind = RM_LOG_ROWS,
                                .count = 1u,
                                .generation = 2u,
                                .first_index = 0u,
                                .publish_ms = 5000u,
                                .newest_ms = 4990u,
                                .log_rows = 1u,
                                .boot_id = ids[boot]};
        row[0] = (uint8_t)(10u + boot);
        emit_batch(batch_names[boot], &batch, row);
        const size_t n = rm_pack_log_batch(bytes, sizeof(bytes), &batch, row);
        assert(n == RM_LOG_BATCH_HEADER_SIZE + RM_LOG_RECORD_SIZE);
        assert(bytes[0] == RM_LOG_VERSION &&
               fixture_u32(bytes + 32u) == ids[boot]);
        assert(bytes[RM_LOG_BATCH_HEADER_SIZE] == (uint8_t)(10u + boot));
        rm_log_t begin = {.kind = RM_LOG_BEGIN,
                          .dump_id = 7u,
                          .generation = 2u,
                          .boot_id = ids[boot]};
        begin.body[0] = 1u;
        rm_pack_log(bytes, &begin);
        assert(bytes[0] == RM_LOG_VERSION &&
               fixture_u32(bytes + 32u) == ids[boot]);
        emit_named(begin_names[boot], bytes, RM_LOG_SIZE);
        rm_state_t state = {.uptime_ms = 5000u,
                            .sequence = 1u,
                            .flags = 0x0fu,
                            .boot_id = ids[boot]};
        rm_pack_state(bytes, &state);
        assert(bytes[0] == RM_STATE_VERSION &&
               fixture_u32(bytes + 72u) == ids[boot]);
        emit_named(state_names[boot], bytes, RM_STATE_SIZE);
        rm_heartbeat_t beat = {.uptime_ms = 5000u,
                               .cpu_load_permille = 117u,
                               .boot_id = ids[boot],
                               .board_unavailable_mask = 8u,
                               .board_unavailable = 233u};
        rm_pack_heartbeat(bytes, &beat);
        assert(bytes[0] == RM_HEARTBEAT_VERSION && bytes[3] == 8u);
        assert(fixture_u32(bytes + 100u) == ids[boot]);
        assert(fixture_u32(bytes + 104u) == 233u &&
               fixture_u32(bytes + 68u) == 0u);
        emit_named(beat_names[boot], bytes, RM_HEARTBEAT_SIZE);
        rm_pack_status(bytes, RM_STATUS_ONLINE, ids[boot]);
        assert(bytes[0] == RM_STATUS_VERSION &&
               fixture_u32(bytes + 2u) == ids[boot]);
        emit_named(status_names[boot], bytes, RM_STATUS_SIZE);
        rm_pack_status(bytes, RM_STATUS_OFFLINE, ids[boot]);
        emit_named(offline_names[boot], bytes, RM_STATUS_SIZE);
        rm_summary_t summary = {.run_id = 1u,
                                .duration_ms = 1000u,
                                .stop_source = RM_STOP_SOURCE_GP21,
                                .stop_reason = RM_STOP_GP21,
                                .max_speed_error_mm_s = RM_SUMMARY_NONE_U16,
                                .hump_peak = RM_SUMMARY_NONE_U16,
                                .boot_id = ids[boot]};
        rm_pack_summary(bytes, &summary);
        assert(bytes[0] == RM_SUMMARY_VERSION &&
               fixture_u32(bytes + 44u) == ids[boot]);
        emit_named(summary_names[boot], bytes, RM_SUMMARY_SIZE);
        rm_detour_t detour = {.sequence = 1u,
                              .outcome = RM_DETOUR_DONE_LINE,
                              .side = 2u,
                              .start_ms = 100u,
                              .end_ms = 200u,
                              .duration_ms = 100u,
                              .scan_id = 1u,
                              .boot_id = ids[boot]};
        rm_pack_detour(bytes, &detour);
        assert(bytes[0] == RM_DETOUR_VERSION &&
               fixture_u32(bytes + 32u) == ids[boot]);
        emit_named(detour_names[boot], bytes, RM_DETOUR_SIZE);
        rm_event_t event = {.kind = 5u,
                            .sequence = 1u,
                            .uptime_ms = 5000u,
                            .boot_id = ids[boot]};
        rm_pack_event(bytes, &event);
        assert(bytes[0] == RM_EVENT_VERSION &&
               fixture_u32(bytes + 16u) == ids[boot]);
        emit_named(event_names[boot], bytes, RM_EVENT_SIZE);
        memset(bytes, 0xa5, sizeof(bytes));
        rm_pack_log_event(bytes, &event);
        assert(bytes[0] == RM_VERSION &&
               bytes[16] == 0xa5u); /* no record growth */
        rm_sonar_t sonar = {.status = 1u,
                            .valid = true,
                            .echo = 200u,
                            .history_sequence = 1u,
                            .timestamp_ms = 5000u,
                            .boot_id = ids[boot]};
        rm_pack_sonar(bytes, &sonar);
        assert(bytes[0] == RM_SONAR_VERSION &&
               fixture_u32(bytes + 32u) == ids[boot]);
        emit_named(sonar_names[boot], bytes, RM_SONAR_SIZE);
        rm_obstacle_t obstacle = {.scan_id = 1u,
                                  .uptime_ms = 5000u,
                                  .closest_mm = 200u,
                                  .flags = RM_OBSTACLE_CLOSEST_VALID,
                                  .decision = 2u,
                                  .boot_id = ids[boot]};
        rm_pack_obstacle(bytes, &obstacle);
        assert(bytes[0] == RM_OBSTACLE_VERSION &&
               fixture_u32(bytes + 20u) == ids[boot]);
        emit_named(obstacle_names[boot], bytes, RM_OBSTACLE_SIZE);
    }
}

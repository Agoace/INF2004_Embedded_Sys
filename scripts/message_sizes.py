#!/usr/bin/env python3
"""Measure ARM application message/value sizes without running firmware.

Compile only a generated probe. Symbol lengths give sizeof values with the
firmware ABI, not host alignment. No private headers or firmware C included.
The manifest covers each current public board/queue value and MQTT topic.
Update ENTRIES when an interface is added; native MQTT value sizes are NOT
wire sizes. Output is a Markdown table with the source of each expression.
"""
import argparse
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
# header, sizeof types, wire-size macros
ENTRIES = [
    ("subsystem2/motion.h", ["motion_status_t"], []),
    ("subsystem2/motion_pack.h", ["motion_packed_t"], []),
    ("subsystem2/motor_ctl.h",
     ["mc_config_t", "mc_wheel_t", "mc_t", "mc_output_t"], []),
    ("subsystem3/drv_line.h", ["drv_line_sample_t"], []),
    ("subsystem3/line_service.h", ["line_snapshot_t"], []),
    ("subsystem3/barcode_calibration.h", ["barcode_cal_report_t"], []),
    ("subsystem3/barcode_live_source.h",
     ["barcode_live_record_t", "barcode_live_control_t"], []),
    ("subsystem4/imu_service.h", ["imu_snapshot_t"], []),
    ("subsystem4/imu_hump.h",
     ["imu_hump_report_t", "imu_hump_summary_t"], []),
    ("subsystem4/imu_hump_diag.h", ["imu_hump_diag_report_t"], []),
    ("subsystem5/sonar_service.h",
     ["sonar_request_t", "sonar_snapshot_t", "sonar_history_entry_t"], []),
    ("subsystem1/telemetry.h",
     ["telemetry_event_t", "telemetry_info_t", "telemetry_message_t",
      "telemetry_log_index_t", "telemetry_stats_t"],
     ["TELEMETRY_BYTES"]),
    ("subsystem1/robot_network.h", ["robot_command_t"], []),
    ("vehicle/vehicle_board.h", ["vehicle_snapshot_t"], []),
    ("subsystem1/mqtt_wire.h",
     ["rm_state_t", "rm_sonar_t", "rm_obstacle_t", "rm_detour_t",
      "rm_event_t", "rm_summary_t", "rm_heartbeat_t", "rm_command_t",
      "rm_log_t", "rm_log_batch_t"],
     ["RM_STATE_SIZE", "RM_SONAR_SIZE", "RM_OBSTACLE_SIZE",
      "RM_DETOUR_SIZE", "RM_STATUS_SIZE", "RM_EVENT_SIZE",
      "RM_EVENT_RECORD_SIZE", "RM_SUMMARY_SIZE", "RM_HEARTBEAT_SIZE",
      "RM_COMMAND_SIZE", "RM_LOG_SIZE", "RM_LOG_BATCH_HEADER_SIZE",
      "RM_LOG_RECORD_SIZE", "RM_LOG_BATCH_MAX_SIZE"]),
]


def measure(cc, nm):
    lines = ["#define TM_MQTT 1", "#define APP_TEST 0"]
    expressions = []
    for header, types, macros in ENTRIES:
        lines.append(f'#include "app_program/{header}"')
        for name in types + macros:
            expr = f"sizeof({name})" if name in types else name
            expressions.append((expr, "app_program/" + header))
    # Health has one value and one u32 timestamp. Motion's compact payload
    # already contains time. APP_BOARD adds a sequence and word padding.
    expressions.append(("sizeof(rm_heartbeat_t) + sizeof(uint32_t)",
                        "app_program/subsystem1/robot_health.c"))
    board_types = ["motion_packed_t", "line_snapshot_t", "imu_snapshot_t",
                   "barcode_cal_report_t", "barcode_live_record_t",
                   "barcode_live_control_t",
                   "imu_hump_report_t", "imu_hump_diag_report_t",
                   "imu_hump_summary_t", "vehicle_snapshot_t",
                   "rm_obstacle_t", "rm_detour_t", "rm_summary_t",
                   "telemetry_log_index_t"]
    for name in board_types:
        expressions.append((f"4 + ((sizeof({name}) + 3) / 4) * 4",
                            "app_program/platform/app_board.h"))
    expressions.append(("4 + sizeof(rm_heartbeat_t) + 4",
                        "app_program/subsystem1/robot_health.c"))
    for i, (expr, _) in enumerate(expressions):
        lines.append(f"unsigned char cr1_size_{i}[{expr}];")
    with tempfile.TemporaryDirectory(prefix="cr1-size-") as tmp:
        source, obj = Path(tmp) / "probe.c", Path(tmp) / "probe.o"
        source.write_text("\n".join(lines) + "\n")
        subprocess.run([cc, "-mcpu=cortex-m0plus", "-mthumb", "-std=c11",
                        "-I", str(ROOT), "-I", str(ROOT / "tests/mtk_stubs"),
                        "-c", str(source), "-o", str(obj)], check=True)
        output = subprocess.check_output([nm, "-S", str(obj)], text=True)
    sizes = {}
    for line in output.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[3].startswith("cr1_size_"):
            sizes[int(parts[3].removeprefix("cr1_size_"))] = int(parts[1], 16)
    return [(expr, sizes[i], source) for i, (expr, source) in
            enumerate(expressions)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default="arm-none-eabi-gcc")
    parser.add_argument("--nm", default="arm-none-eabi-nm")
    args = parser.parse_args()
    print("Expression | Bytes | Source")
    print("--- | ---: | ---")
    for expr, size, source in measure(args.cc, args.nm):
        print(f"`{expr}` | {size} | `{source}`")


if __name__ == "__main__":
    main()

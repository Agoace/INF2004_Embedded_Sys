#!/usr/bin/env python3
"""Source ownership guard: only the Vehicle task publishes its fault word."""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[1]
source = root / "app_program/vehicle/vehicle_task.c"
text = source.read_text()
assert text.count("static fault_mask_board_t vehicle_fault_board;") == 1
assert text.count("fault_mask_publish(") == 1
body = re.search(r"static void publish_vehicle_faults\(void\)\n\{(.*?)\n\}",
                 text, re.S).group(1)
assert "vehicle_fault_value(imu_gate.latched, vsm.state," in body
assert "vsm.fault)" in body
assert "fault_mask_publish(&vehicle_fault_board, FAULT_OWNER_VEHICLE, value)" in body
getter = re.search(r"bool vehicle_fault_mask_try_read\(uint32_t \*out\)\n\{(.*?)\n\}",
                   text, re.S).group(1)
assert "fault_mask_try_read(&vehicle_fault_board, FAULT_OWNER_VEHICLE, out)" in getter
assert "publish" not in getter and "__atomic_store" not in getter
assert "return false;" in getter  # TEST/BENCH are unavailable, not all-clear.
# Board name and private publication helper cannot be accessed by other owners.
for path in (root / "app_program").rglob("*.[ch]"):
    if path != source:
        other = path.read_text()
        assert "vehicle_fault_board" not in other, path
        assert "publish_vehicle_faults" not in other, path
print("Vehicle fault owner guard: PASS")

"""Cross-language log round trip and exact copied settings contract."""
import importlib.util
from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("decoder", root / "scripts/robot_log_decode.py")
decoder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(decoder)
payload = subprocess.check_output([sys.argv[1]], text=True).strip()
assert decoder.decode(payload) == [89980, 2, 4, "detour", 4095, 1234, -100, -378, 378,
                                   -511, 511, 400, 0, 400, 0, -36, 165, 0]
reverse_word = int.from_bytes(bytes.fromhex(payload), "little")
reverse_word = (reverse_word & ~(7 << 18)) | (5 << 18)
reverse_payload = reverse_word.to_bytes(16, "little").hex()
assert decoder.decode(reverse_payload)[2:4] == [5, "scan_reverse"]
# Descriptive reset metadata must not alter either CSV stream.
for reason in ("power/RUN", "watchdog", "other"):
    capture = f"[robot] KR1 reset {reason}; idle rows 1 events 1\nKR 0 {payload}\nVE 10 1 0 2 4\n"
    csv_rows = subprocess.check_output(
        [sys.executable, str(root / "scripts/robot_log_decode.py")], input=capture, text=True).splitlines()
    assert len(csv_rows) == 2 and csv_rows[1].split(",") == ["0", *map(str, decoder.decode(payload))]
    csv_events = subprocess.check_output(
        [sys.executable, str(root / "scripts/robot_log_decode.py"), "--events"],
        input=capture, text=True).splitlines()
    assert len(csv_events) == 2 and csv_events[1] == "10,1,0,2,4"
# Hump lines: run end, unavailable, GP20 DUMP and STREAM drive formats.
capture = ("[robot] hump highest_mm 10.8 side right count 2 flags 05 gated 1\n"
           "[robot] hump unavailable flags 03\n"
           "[robot] hump peak_0.1mm 250 count 3 flags 21 gated 0\n"
           "[robot] hump peak_0.1mm 65535 count 0 flags 03 gated 0\n"
           "[S4_HUMP] peak_0.1mm 0 count 0 flags 01 gated 2 long 1\n"
           f"KR 0 {payload}\n")
csv_hump = subprocess.check_output(
    [sys.executable, str(root / "scripts/robot_log_decode.py"), "--hump"],
    input=capture, text=True).splitlines()
assert csv_hump == ["source,peak_mm,side,count,flags,gated",
                    "run_end,10.8,right,2,05,1", "run_end,unavailable,,,03,",
                    "dump,25.0,left,3,21,0", "dump,unavailable,,,03,",
                    "stream,0.0,none,0,01,2"], csv_hump
ref = (root / "src/vc_line_test.c").read_text()
first = ref.index("static const lt_config_t LINE_CFG")
last = ref.index("typedef enum {\n    STOP_BUTTON", first)
config = ref[first:last].replace(".back_off_fn = NULL", ".back_off_fn = 0")
assert config in (root / "app_program/vehicle/vehicle_robot_config.h").read_text()
# The unchanged robot motor configuration supplies KI=128.
assert ".ki_q8 = 128u" in (root / "app_program/subsystem2/motor_ctl_robot.h").read_text()
print("KR1 C/Python round trip and reference settings: PASS")

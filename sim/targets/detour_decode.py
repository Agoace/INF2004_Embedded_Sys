"""Observation only. Never feeds firmware, time, devices, or world state."""
import re

STEPS = ("BACK", "TURN_AWAY", "STRAIGHT", "TURN_TOWARD", "ON_LINE", "ALIGN", "DONE", "FAILED")
FAILS = ("NONE", "NO_SIDE", "TURN_LIMIT", "TIME", "SENSOR", "TOO_NEAR", "NO_LINE")


def decode(events):
    phase = "NOT_RUNNING"
    confirmed = phase
    phases = [{"t_us": 0, "phase": phase, "evidence": "process start", "inferred": True}]
    timeline, prints, mismatches = [], [], []
    last_step, latest_dt = None, None
    pending_text = ""
    running = False

    def transition(name, event, evidence, pending=True):
        nonlocal phase, confirmed
        if not pending:
            confirmed = name
        if phase != name:
            phase = name
            phases.append({"t_us": event["t_us"], "phase": name, "evidence": evidence,
                           "inferred": True, "caller_assignment_pending": pending,
                           "last_confirmed_phase": confirmed})

    for event in events:
        if event["type"] == "text":
            pending_text += event["text"]
            while "\n" in pending_text:
                line, pending_text = pending_text.split("\n", 1)
                if line.startswith("[run] start,"):
                    running, last_step = True, None
                    transition("RUN_START_UNRESOLVED", event, "console run start", False)
                if re.match(r"\[run\] \d+ ms, stopped by", line):
                    running = False
                    transition("NOT_RUNNING", event, "console run summary; final drive stop", False)
                match = re.match(r"\[detour\] (\w+) at (\d+) ms:", line)
                if match:
                    printed = {"t_us": event["t_us"], "step": match[1], "run_ms": int(match[2])}
                    prints.append(printed)
                    pose = re.search(r"pose \((-?\d+), (-?\d+)\) heading ([+-]\d+)", line)
                    values = [int(v) for v in pose.groups()] if pose else None
                    expected = [latest_dt[k] for k in ("x_mm", "y_mm", "heading")] if latest_dt else None
                    if latest_dt is None or STEPS[latest_dt["step"]] != match[1] or values != expected:
                        mismatches.append({"printed": printed, "latest_hook": latest_dt})
            continue
        if event["type"] != "hook":
            continue
        call = event["call"]
        if running:
            if call in ("steer_reset", "steer_update", "steer_straight"):
                transition("FOLLOW", event, call, False)
            elif call == "lp_update" and phase == "FOLLOW" and event["result"] & 31 == 5:
                transition("SEARCH", event, "lp_update LINE_LOST")
            elif call == "obstacle_update" and (event["result"] >> 9) & 7 == 3:
                transition("STOP", event, "obstacle_update STOP")
            elif call == "obstacle_scan_start" and event["result"]:
                confirmed = "STOP"
                transition("SCAN", event, "successful scan start")
            elif call == "dt_init":
                transition("DETOUR", event, "dt_init after scan completion")
        if running and ((call == "obstacle_scan_service" and phase == "SCAN") or
                        (call in ("dt_update", "dt_reading") and phase == "DETOUR")):
            confirmed = phase
        event["phase_inferred"] = phase
        event["phase_last_confirmed"] = confirmed
        if call == "lp_update" and phase == "SEARCH" and event["result"] & 31 == 6:
            event["transition_pending"] = "FOLLOW reset"
        if call == "obstacle_scan_service" and event["result"] == 2:
            event["transition_pending"] = "dt_init / DETOUR"
        if call in ("dt_init", "dt_update", "dt_reading"):
            event["step_name"] = STEPS[event["step"]]
            event["fail_name"] = FAILS[event["fail"]]
            if event["step"] in (6, 7):
                event["transition_pending"] = "FOLLOW reset" if event["step"] == 6 else "run end"
            latest_dt = event
            if call == "dt_init" or event["step"] != last_step:
                timeline.append({key: event[key] for key in (
                    "t_us", "call", "step", "step_name", "fail", "fail_name", "heading", "x_mm", "y_mm", "world_pose")})
                last_step = event["step"]
    # Each printed state must also appear in order in the hooked transitions.
    cursor = 0
    omitted = []
    for printed in prints:
        while cursor < len(timeline) and timeline[cursor]["step_name"] != printed["step"]:
            omitted.append(timeline[cursor])
            cursor += 1
        if cursor == len(timeline):
            mismatches.append({"out_of_order": printed})
        else:
            cursor += 1
    omitted.extend(timeline[cursor:])
    return {"phase_timeline": phases, "detour_timeline": timeline,
            "module_pose_frame": "S frame relative to detour start, start-of-step pose/heading; not live world pose",
            "world_pose_frame": "rear axle x/y mm and heading degrees at exact hook completion",
            "step_crosscheck": {"status": "mismatch" if mismatches else "match", "printed": prints,
                                "mismatches": mismatches, "unprinted_hook_states": omitted}}

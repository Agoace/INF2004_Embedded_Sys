"""Private numeric protocol from the validated scenario to the native world."""

def normalise(s, active=True):
    floor = s["robot"]["ir"]["floor_adc"]
    buttons = [e for e in s["events"] if "pin" in e]
    faults = [e for e in s["events"] if "ultrasonic_fault" in e]
    rows = [f'{s["end_us"]} {s["clock_quantum_us"]} {floor[0]} {floor[1]} {floor[2]} {len(buttons)}']
    rows += [f'{e["at_us"]} {e["pin"]} {int(e["down"])}' for e in buttons]
    rows.append(str(int(active)))
    if not active:
        return "\n".join(rows + ["0"]) + "\n"
    robot, track = s["robot"], s["track"]
    motor, pose, geometry, ir = (robot[key] for key in ("motors", "start_pose", "geometry", "ir"))
    rows.append(" ".join(map(str, [int(motor["profile"] == "provisional-loaded"),
        int(motor["symmetric_reverse"]), motor["loaded_zero_pct"], *motor["coast_tau_ms"],
        pose["x_mm"], pose["y_mm"], pose["heading_deg"]])))
    rows.append(" ".join(str(geometry[key]) for key in
        ("front_x_mm", "rear_x_mm", "width_mm", "wheel_track_mm", "wheel_diameter_mm", "servo_axis_x_mm", "sensor_face_offset_mm")))
    for point, floor, line in zip(ir["points_mm"], ir["floor_adc"], ir["line_adc"]):
        rows.append(f"{point[0]} {point[1]} {floor} {line}")
    rows.append(" ".join(map(str, [ir["footprint_mm"], track["line_width_mm"], track["tile_size_mm"],
                                   *track["origin_mm"], track["curve_radius_mm"]])))
    rows.append(str(len(track["tiles"])))
    for tile in track["tiles"]:
        kind = ["straight", "curve", "cross", "blank"].index(tile["type"])
        rows.append(f'{tile["col"]} {tile["row"]} {kind} {tile["rotation_deg"]}')
    strokes = []
    for primitive in track["primitives"]:
        if primitive["type"] == "arc":
            strokes.append([1, *primitive["centre_mm"], 0, 0, primitive["radius_mm"],
                            primitive["start_deg"], primitive["sweep_deg"], primitive["width_mm"]])
        else:
            points = primitive["points_mm"]
            ends = points[1:] + ([points[0]] if primitive["closed"] else [])
            for a, b in zip(points, ends):
                strokes.append([0, *a, *b, 0, 0, 0, primitive["width_mm"]])
    rows.append(str(len(strokes)))
    rows += [" ".join(map(str, stroke)) for stroke in strokes]
    rows.append(str(len(s["obstacles"])))
    for obstacle in s["obstacles"]:
        pose = obstacle["pose"]
        kind = int(obstacle["type"] == "bottle")
        size = [obstacle["diameter_mm"], 0] if kind else obstacle["size_mm"]
        echo = obstacle["echo"]
        settings = [0, 90, 0] if isinstance(echo, str) else [
            int(echo["material"] == "specular"), echo["max_incidence_deg"], echo["corner_echo_max_mm"]]
        rows.append(" ".join(map(str, [kind, pose["x_mm"], pose["y_mm"], pose["heading_deg"], *size, *settings])))
    rows.append(str(len(faults)))
    rows += [f'{e["at_us"]} ' + str(["NORMAL", "NO_RISE", "STUCK_HIGH", "NO_RETURN"].index(e["ultrasonic_fault"])) for e in faults]
    return "\n".join(rows) + "\n"

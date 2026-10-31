"""Strict version 1 scenario reader; standard library only, no silent defaults."""
import json
import math
from pathlib import Path


class ScenarioError(ValueError):
    pass


def fail(path, message):
    raise ScenarioError(f"{path}: {message}")


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            fail(key, "duplicate JSON key")
        result[key] = value
    return result


def unwrap(node, path, assumptions):
    if isinstance(node, dict):
        if "assumed" in node:
            if set(node) != {"value", "assumed", "note"}:
                fail(path, "assumption requires exactly value, assumed, note")
            if node["assumed"] is not True or not isinstance(node["note"], str) or not node["note"].strip():
                fail(path, "assumed must be true and note must be non-empty text")
            # A wrapper describes one value; nested assumptions remain explicit.
            assumptions.append({"path": path, **node})
            return unwrap(node["value"], path, assumptions)
        return {key: unwrap(value, f"{path}.{key}", assumptions) for key, value in node.items()}
    if isinstance(node, list):
        return [unwrap(value, f"{path}[{i}]", assumptions) for i, value in enumerate(node)]
    return node


def obj(value, path, keys):
    if not isinstance(value, dict):
        fail(path, "expected object")
    expected = set(keys.split())
    if set(value) != expected:
        fail(path, f"missing {sorted(expected - set(value))}; unknown {sorted(set(value) - expected)}")


def number(value, path, low=-1e9, high=1e9, integer=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or (isinstance(value, float) and not math.isfinite(value))):
        fail(path, "expected finite number")
    if integer and not isinstance(value, int):
        fail(path, "expected integer")
    if not low <= value <= high:
        fail(path, f"expected value in [{low}, {high}]")


def text(value, path):
    if not isinstance(value, str) or not value.strip():
        fail(path, "expected non-empty text")


def choice(value, path, values):
    if not any(type(value) is type(option) and value == option for option in values):
        fail(path, f"expected one of {values}")


def array(value, path, length=None):
    if not isinstance(value, list) or (length is not None and len(value) != length):
        fail(path, f"expected array of length {length}" if length is not None else "expected array")


def pose(value, path):
    obj(value, path, "x_mm y_mm heading_deg")
    for key in value:
        number(value[key], f"{path}.{key}")


def validate(s):
    obj(s, "$", "schema_version name firmware_target end_us clock_quantum_us seed provenance track robot obstacles events noise")
    number(s["schema_version"], "schema_version", 1, 1, integer=True)
    text(s["name"], "name")
    choice(s["firmware_target"], "firmware_target", ["detour_test"])
    number(s["end_us"], "end_us", 1, 3600000000, integer=True)
    number(s["clock_quantum_us"], "clock_quantum_us", 1, 1000, integer=True)
    number(s["seed"], "seed", 0, 2**32 - 1, integer=True)
    obj(s["provenance"], "provenance", "track geometry adc")
    for key, value in s["provenance"].items():
        text(value, f"provenance.{key}")
    track = s["track"]
    obj(track, "track", "tile_size_mm thickness_mm line_width_mm material origin_mm tiles curve_radius_mm primitives")
    for key in ("tile_size_mm", "thickness_mm", "line_width_mm", "curve_radius_mm"):
        number(track[key], f"track.{key}", 0.001, 10000)
    text(track["material"], "track.material")
    array(track["origin_mm"], "track.origin_mm", 2)
    for value in track["origin_mm"]:
        number(value, "track.origin_mm")
    array(track["tiles"], "track.tiles")
    occupied = set()
    for i, tile in enumerate(track["tiles"]):
        p = f"track.tiles[{i}]"
        obj(tile, p, "col row type rotation_deg")
        for key in ("col", "row"):
            number(tile[key], f"{p}.{key}", -10000, 10000, integer=True)
        choice(tile["type"], f"{p}.type", ["straight", "curve", "cross", "blank"])
        choice(tile["rotation_deg"], f"{p}.rotation_deg", [0, 90, 180, 270])
        cell = (tile["col"], tile["row"])
        if cell in occupied:
            fail(p, "duplicate tile cell")
        occupied.add(cell)
    array(track["primitives"], "track.primitives")
    for i, primitive in enumerate(track["primitives"]):
        p = f"track.primitives[{i}]"
        if not isinstance(primitive, dict):
            fail(p, "expected primitive object")
        kind = primitive.get("type")
        if kind == "polyline":
            obj(primitive, p, "type points_mm closed width_mm")
            array(primitive["points_mm"], p + ".points_mm")
            if len(primitive["points_mm"]) < 2 or not isinstance(primitive["closed"], bool):
                fail(p, "polyline requires two points and boolean closed")
            for point in primitive["points_mm"]:
                array(point, p, 2)
                for value in point:
                    number(value, p)
        elif kind == "arc":
            obj(primitive, p, "type centre_mm radius_mm start_deg sweep_deg width_mm")
            array(primitive["centre_mm"], p, 2)
            for value in primitive["centre_mm"]:
                number(value, p)
            number(primitive["radius_mm"], p + ".radius_mm", 0.001)
            number(primitive["start_deg"], p + ".start_deg")
            number(primitive["sweep_deg"], p + ".sweep_deg", -360, 360)
        else:
            fail(p, "unknown primitive type")
        number(primitive["width_mm"], p + ".width_mm", 0.001)
    if not track["tiles"] and not track["primitives"]:
        fail("track", "no tiles or primitives")
    robot = s["robot"]
    obj(robot, "robot", "start_pose geometry ir motors servo ultrasonic")
    pose(robot["start_pose"], "robot.start_pose")
    geometry = robot["geometry"]
    obj(geometry, "robot.geometry", "width_mm length_mm front_x_mm rear_x_mm wheel_track_mm wheel_diameter_mm servo_axis_x_mm sensor_face_offset_mm")
    for key, value in geometry.items():
        number(value, "robot.geometry." + key)
    for key in ("width_mm", "length_mm", "wheel_track_mm", "wheel_diameter_mm"):
        number(geometry[key], "robot.geometry." + key, 0.001)
    if not math.isclose(geometry["front_x_mm"] - geometry["rear_x_mm"], geometry["length_mm"]):
        fail("robot.geometry", "front minus rear must equal length")
    ir = robot["ir"]
    obj(ir, "robot.ir", "points_mm footprint_mm floor_adc line_adc")
    array(ir["points_mm"], "robot.ir.points_mm", 3)
    for point in ir["points_mm"]:
        array(point, "robot.ir.points_mm", 2)
        for value in point:
            number(value, "robot.ir.points_mm")
    ys = [point[1] for point in ir["points_mm"]]
    if not (ys[0] > ys[1] == 0 > ys[2]):
        fail("robot.ir.points_mm", "expected physical left/centre/right, centre y=0")
    number(ir["footprint_mm"], "robot.ir.footprint_mm", 0.001)
    for key in ("floor_adc", "line_adc"):
        array(ir[key], "robot.ir." + key, 3)
        for value in ir[key]:
            number(value, "robot.ir." + key, 0, 4095, integer=True)
    for floor, line in zip(ir["floor_adc"], ir["line_adc"]):
        if floor == line:
            fail("robot.ir", "floor and line ADC must differ")
    # Geometric consistency check, not a substitute for a sensor model.
    gap = ys[0] - ys[1]
    width = track["line_width_mm"]
    footprint = ir["footprint_mm"]
    distance = gap / 2
    overlap = max(0.0, min(width / 2, distance + footprint / 2) - max(-width / 2, distance - footprint / 2))
    if overlap / footprint >= 0.01:
        fail("robot.ir", "C-L gap consistency: gap-centred line gives >=1% rise")
    # The native world repeats this check with its production IR overlap function.
    motors = robot["motors"]
    obj(motors, "robot.motors", "profile loaded_zero_pct symmetric_reverse coast_tau_ms")
    choice(motors["profile"], "robot.motors.profile", ["unloaded-reference", "provisional-loaded"])
    number(motors["loaded_zero_pct"], "robot.motors.loaded_zero_pct", 0, 99)
    if motors["symmetric_reverse"] is not True:
        fail("robot.motors.symmetric_reverse", "only the approved symmetric reverse model is implemented")
    array(motors["coast_tau_ms"], "robot.motors.coast_tau_ms", 2)
    for value in motors["coast_tau_ms"]:
        number(value, "robot.motors.coast_tau_ms", 0.001, 10000)
    obj(robot["servo"], "robot.servo", "instant_angle")
    if not isinstance(robot["servo"]["instant_angle"], bool):
        fail("robot.servo.instant_angle", "expected boolean")
    ultra = robot["ultrasonic"]
    obj(ultra, "robot.ultrasonic", "beam_deg rise_latency_us echo_timeout_us trigger_gap_us")
    for key, value in ultra.items():
        number(value, "robot.ultrasonic." + key, 1, 1000000, integer=True)
    # M1 has one timing implementation. Reject rather than ignore timing changes.
    if ultra != {"beam_deg": 30, "rise_latency_us": 215, "echo_timeout_us": 3000, "trigger_gap_us": 60000}:
        fail("robot.ultrasonic", "placeholder ultrasonic requires the documented timing and beam")
    array(s["obstacles"], "obstacles")
    names = set()
    for i, obstacle in enumerate(s["obstacles"]):
        p = f"obstacles[{i}]"
        if not isinstance(obstacle, dict):
            fail(p, "expected object")
        kind = obstacle.get("type")
        if kind == "box":
            obj(obstacle, p, "id type pose size_mm contact echo")
            array(obstacle["size_mm"], p, 2)
            for value in obstacle["size_mm"]:
                number(value, p, 0.001)
        elif kind == "bottle":
            obj(obstacle, p, "id type pose diameter_mm contact echo")
            number(obstacle["diameter_mm"], p + ".diameter_mm", 0.001)
        else:
            fail(p, "unknown obstacle type")
        text(obstacle["id"], p + ".id")
        if obstacle["id"] in names:
            fail(p, "duplicate obstacle id")
        names.add(obstacle["id"])
        pose(obstacle["pose"], p + ".pose")
        choice(obstacle["contact"], p + ".contact", ["freeze-both"])
        choice(obstacle["echo"], p + ".echo", ["ideal-sector"])
    array(s["events"], "events")
    if len(s["events"]) > 100000:
        fail("events", "at most 100000 events are supported")
    previous = -1
    levels = {20: False, 21: False}
    for i, event in enumerate(s["events"]):
        p = f"events[{i}]"
        obj(event, p, "at_us pin down")
        number(event["at_us"], p + ".at_us", 0, s["end_us"] - 1, integer=True)
        number(event["pin"], p + ".pin", 20, 21, integer=True)
        if event["at_us"] < previous:
            fail(p, "events must be ordered")
        previous = event["at_us"]
        if not isinstance(event["down"], bool) or event["down"] == levels[event["pin"]]:
            fail(p, "button events must change the level")
        levels[event["pin"]] = event["down"]
    obj(s["noise"], "noise", "enabled")
    if s["noise"]["enabled"] is not False:
        fail("noise.enabled", "noise is not implemented yet")


def load(path):
    raw = json.loads(Path(path).read_text(), object_pairs_hook=unique_object,
                     parse_constant=lambda value: fail("$", "non-finite JSON number " + value))
    assumptions = []
    scenario = unwrap(raw, "$", assumptions)
    validate(scenario)
    return raw, scenario, assumptions

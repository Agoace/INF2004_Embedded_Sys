"""Explicit robot parameters. Assumptions remain visible, never calibration."""

from dataclasses import dataclass
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class Parameter:
    value: float
    assumed: bool
    source: str

    @classmethod
    def read(cls, item):
        if not isinstance(item, dict) or set(item) != {"value", "assumed", "source"}:
            raise ValueError("parameter requires value, assumed and source")
        if type(item["value"]) not in (int, float) or not math.isfinite(item["value"]):
            raise ValueError("parameter must be finite")
        if type(item["assumed"]) is not bool or not isinstance(item["source"], str):
            raise ValueError("parameter needs explicit assumption and source")
        if not item["source"].strip():
            raise ValueError("parameter source is empty")
        return cls(float(item["value"]), item["assumed"], item["source"])


REQUIRED = frozenset((
    "track_mm", "wheel_diameter_mm", "counts_per_revolution", "firmware_counts_per_m",
    "total_mass_kg", "wheel_mass_kg", "caster_mass_kg", "head_mass_kg",
    "caster_forward_mm", "caster_radius_mm", "body_width_mm", "body_length_mm",
    "body_front_mm", "body_height_mm", "body_centre_height_mm", "com_forward_mm",
    "com_axle_height_mm", "contact_friction", "wheel_inertia_kg_m2", "drive_tau_ms",
    "coast_tau_ms", "stall_torque_nm", "start_duty_fraction", "stopped_speed_rad_s",
    "max_assumed_duty_fraction",
    "servo_axis_forward_mm", "servo_axis_axle_height_mm", "sonar_face_offset_mm",
    "servo_slew_deg_s", "line_forward_mm", "line_left_mm", "line_clearance_mm",
    "barcode_forward_mm", "barcode_left_mm", "barcode_clearance_mm",
    "imu_forward_mm", "imu_axle_height_mm", "field_east_gauss", "field_north_gauss",
    "field_up_gauss",
))
SIGNED = frozenset(("com_forward_mm", "com_axle_height_mm", "sonar_face_offset_mm",
                    "servo_axis_axle_height_mm", "imu_axle_height_mm",
                    "field_east_gauss", "field_north_gauss", "field_up_gauss"))


@dataclass(frozen=True)
class RobotProfile:
    parameters: tuple
    profile_id: str

    @classmethod
    def load(cls, path):
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

    @classmethod
    def from_dict(cls, data):
        if set(data) != {"schema", "profile_id", "parameters"} or data["schema"] != 1:
            raise ValueError("unsupported robot profile schema")
        if not isinstance(data["profile_id"], str) or not data["profile_id"].strip():
            raise ValueError("profile ID required")
        if set(data["parameters"]) != REQUIRED:
            raise ValueError("unknown or missing robot parameter")
        items = tuple((key, Parameter.read(data["parameters"][key])) for key in sorted(REQUIRED))
        result = cls(items, data["profile_id"])
        for key, param in items:
            if key not in SIGNED and param.value <= 0:
                raise ValueError(f"{key} must be positive")
        if result["counts_per_revolution"] != int(result["counts_per_revolution"]):
            raise ValueError("encoder counts must be integral")
        if result["start_duty_fraction"] > 1:
            raise ValueError("start duty exceeds one")
        if not 0.7 <= result["max_assumed_duty_fraction"] <= 1:
            raise ValueError("assumed duty limit must be0.7..1")
        components = 2 * result["wheel_mass_kg"] + result["caster_mass_kg"]
        if components + result["head_mass_kg"] >= result["total_mass_kg"]:
            raise ValueError("component masses exceed total")
        return result

    def __getitem__(self, name):
        return dict(self.parameters)[name].value

    def assumption_manifest(self):
        return {key: {"value": p.value, "assumed": p.assumed, "source": p.source}
                for key, p in self.parameters}

    def mount_mm(self, name):
        """Axle-centred coordinates: flat clearance converted exactly once."""
        radius = self["wheel_diameter_mm"] / 2
        if name in ("line_c", "line_l"):
            return (self["line_forward_mm"],
                    self["line_left_mm"] if name == "line_l" else 0.0,
                    self["line_clearance_mm"] - radius)
        if name == "barcode":
            return (self["barcode_forward_mm"], self["barcode_left_mm"],
                    self["barcode_clearance_mm"] - radius)
        if name == "imu":
            return (self["imu_forward_mm"], 0.0, self["imu_axle_height_mm"])
        raise ValueError("unknown sensor mount")

"""Temporary 16-byte bench codec. Not the final robot telemetry specification."""
import argparse
import json
from pathlib import Path
import struct

QUALITY = ("unavailable", "valid", "stale", "fault")
FIELDS = {"version", "session", "sequence", "timestamp_ms", "line_quality",
          "barcode_quality", "adc", "pattern", "barcode"}


def decode_status(data):
    """Decode provisional status/LWT; online reports do not prove execution."""
    if not isinstance(data, (bytes, bytearray)) or len(data) != 5 or data[0] not in (0x40, 0x60, 0x70):
        raise ValueError("invalid bench status/LWT")
    return dict(online=bool(data[0] & 0x20), ready=bool(data[0] & 0x10),
                session=int.from_bytes(data[1:], "big"))


def _integer(value, maximum, name):
    if type(value) is not int or not 0 <= value <= maximum:
        raise ValueError(f"{name} must be an integer from 0 to {maximum}")


def _validate(frame):
    if not isinstance(frame, dict) or frame.keys() != FIELDS:
        raise ValueError("frame must contain exactly the documented bench fields")
    for name, maximum in (("version", 1), ("session", 0xFFFFFFFF), ("sequence", 65535),
                          ("timestamp_ms", 0xFFFFFFFF), ("line_quality", 3),
                          ("barcode_quality", 3), ("pattern", 7), ("barcode", 1)):
        _integer(frame[name], maximum, name)
    if frame["version"] != 1:
        raise ValueError("unsupported bench telemetry version")
    adc = frame["adc"]
    if not isinstance(adc, (list, tuple)) or len(adc) != 3:
        raise ValueError("adc must contain three readings")
    for value in adc:
        _integer(value, 4095, "ADC reading")
    if frame["line_quality"] in (0, 3) and (any(adc) or frame["pattern"]):
        raise ValueError("unavailable/fault line values must be zero")
    if frame["barcode_quality"] in (0, 3) and frame["barcode"]:
        raise ValueError("unavailable/fault barcode value must be zero")


def encode(frame):
    """Encode supplied values exactly; caller owns freshness and session rules."""
    _validate(frame)
    header = 0x40 | (frame["line_quality"] << 4) | (frame["barcode_quality"] << 2)
    left, centre, right = frame["adc"]
    payload = (left << 28) | (centre << 16) | (right << 4) | (frame["pattern"] << 1) | frame["barcode"]
    return (struct.pack(">BIHI", header, frame["session"], frame["sequence"], frame["timestamp_ms"])
            + payload.to_bytes(5, "big"))


def decode(payload):
    """Decode a complete bench frame; reject malformed and noncanonical values."""
    if not isinstance(payload, (bytes, bytearray)) or len(payload) != 16:
        raise ValueError("payload must contain exactly 16 bytes")
    header, session, sequence, timestamp = struct.unpack(">BIHI", payload[:11])
    if header >> 6 != 1:
        raise ValueError("unsupported bench telemetry version")
    if header & 3:
        raise ValueError("padding bits must be zero")
    bits = int.from_bytes(payload[11:], "big")
    frame = dict(version=1, session=session, sequence=sequence, timestamp_ms=timestamp,
                 line_quality=(header >> 4) & 3, barcode_quality=(header >> 2) & 3,
                 adc=[(bits >> 28) & 4095, (bits >> 16) & 4095, (bits >> 4) & 4095],
                 pattern=(bits >> 1) & 7, barcode=bits & 1)
    _validate(frame)
    return frame


def readable(frame):
    """Expose quality labels; do not display missing/fault zeros as measurements."""
    _validate(frame)
    line_values = frame["line_quality"] in (1, 2)
    barcode_value = frame["barcode_quality"] in (1, 2)
    return {"version": 1, "session": frame["session"], "sequence": frame["sequence"],
            "timestamp_ms": frame["timestamp_ms"],
            "line": {"quality": QUALITY[frame["line_quality"]],
                     "adc": list(frame["adc"]) if line_values else None,
                     "pattern": frame["pattern"] if line_values else None},
            "barcode": {"quality": QUALITY[frame["barcode_quality"]],
                        "sample": frame["barcode"] if barcode_value else None}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="operation", required=True)
    for operation in ("encode", "compare"):
        command = sub.add_parser(operation)
        command.add_argument("json_frame", help="JSON with all documented bench fields")
    decoder = sub.add_parser("decode")
    source = decoder.add_mutually_exclusive_group(required=True)
    source.add_argument("--hex", dest="hex_payload")
    source.add_argument("--file", type=Path, help="Binary file containing one frame")
    args = parser.parse_args()
    try:
        if args.operation == "decode":
            payload = bytes.fromhex(args.hex_payload) if args.hex_payload is not None else args.file.read_bytes()
            print(json.dumps(readable(decode(payload)), indent=2))
        else:
            frame = json.loads(args.json_frame)
            wire = encode(frame)
            if args.operation == "encode":
                print(wire.hex())
            else:
                text = json.dumps(decode(wire), separators=(",", ":")).encode("utf-8")
                print(json.dumps({"binary_bytes": len(wire), "equivalent_json_bytes": len(text)}))
    except (ValueError, OSError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()

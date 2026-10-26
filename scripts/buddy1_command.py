"""Provisional seven-byte bench codec. No MQTT connection or command execution."""
import argparse
import json
import struct

COMMANDS = ("START", "STOP", "ESTOP", "RESET")


def encode(opcode, sequence, session):
    """Return wire bytes; reject out-of-range values instead of truncating."""
    for name, value, maximum in (("opcode", opcode, 3),
                                 ("sequence", sequence, 65535),
                                 ("session", session, 0xFFFFFFFF)):
        if type(value) is not int or not 0 <= value <= maximum:
            raise ValueError(f"{name} must be an integer from 0 to {maximum}")
    return struct.pack(">BHI", 0x40 | (opcode << 4), sequence, session)


def decode(payload):
    """Return (opcode, sequence, session). This does not record acceptance."""
    if not isinstance(payload, (bytes, bytearray)) or len(payload) != 7:
        raise ValueError("payload must contain exactly seven bytes")
    header, sequence, session = struct.unpack(">BHI", payload)
    if header >> 6 != 1:
        raise ValueError("unsupported provisional protocol version")
    if header & 0x0F:
        raise ValueError("padding bits must be zero")
    return (header >> 4) & 3, sequence, session


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="operation", required=True)
    encoder = sub.add_parser("encode", help="Print hexadecimal bytes only; send nothing")
    encoder.add_argument("command", choices=COMMANDS)
    encoder.add_argument("--sequence", type=lambda value: int(value, 0), required=True)
    encoder.add_argument("--session", type=lambda value: int(value, 0), required=True)
    decoder = sub.add_parser("decode", help="Inspect hexadecimal bytes only")
    decoder.add_argument("hex_payload")
    args = parser.parse_args()
    try:
        if args.operation == "encode":
            print(encode(COMMANDS.index(args.command), args.sequence, args.session).hex())
        else:
            opcode, sequence, session = decode(bytes.fromhex(args.hex_payload))
            print(json.dumps({"command": COMMANDS[opcode], "sequence": sequence,
                              "session": session, "decoded_only": True}))
    except ValueError as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()

"""Compact binary MQTT frames shared with the car firmware.

Python mirror of app_program/telemetry_frame.h; keep the two in step (the shared test vectors in test_app.py are
the bytes printed by testing/system/test_telemetry_frame.c).

Telemetry frame (car -> dashboard), 5 to 15 bytes:
  byte 0: bit 7 = 1 (binary), bits 6..4 source, bits 3..2 size class of value 1, bits 1..0 size class of value 2
          (0: value is 0, no bytes; 1: int8; 2: int16; 3: int32)
  byte 1: bits 5..0 type, bit 6 = a 16-bit flags field follows the sequence, bit 7 reserved (0)
  bytes 2..4: sequence, low 24 bits, little-endian
  [flags: 2 bytes] [value 1] [value 2], little-endian two's complement
Command frame (dashboard -> car): byte 0 = 0x80 | opcode; MAP is followed by 10 bytes (26 actions x 3 bits).
"""
from typing import NamedTuple, Optional

BINARY_MARK = 0x80
CMD_START, CMD_STOP, CMD_PAUSE, CMD_RESUME, CMD_MAP, CMD_MAPRESET = 1, 2, 3, 4, 5, 6
MAP_LETTERS = 26
MAP_ACTION_BITS = 3
MAP_BYTES = (MAP_LETTERS * MAP_ACTION_BITS + 7) // 8
_MAP_VALUES = {"N": 0, "L": 1, "S": 2, "R": 3, "U": 4}
_CLASS_BYTES = (0, 1, 2, 4)


class Telemetry(NamedTuple):
    source: int
    type: int
    flags: int
    value1: int
    value2: int
    sequence: int  # low 24 bits only


def is_binary(raw: bytes) -> bool:
    return len(raw) > 0 and (raw[0] & BINARY_MARK) != 0


def _size_class(value: int) -> int:
    if value == 0:
        return 0
    if -128 <= value <= 127:
        return 1
    if -32768 <= value <= 32767:
        return 2
    return 3


def encode_telemetry(source: int, type_: int, flags: int, value1: int, value2: int, sequence: int) -> bytes:
    """Encode like the firmware does (used by the tests and by simulators)."""
    c1, c2 = _size_class(value1), _size_class(value2)
    out = bytearray([
        BINARY_MARK | ((source & 7) << 4) | (c1 << 2) | c2,
        (type_ & 63) | (0x40 if flags else 0),
        sequence & 0xFF, (sequence >> 8) & 0xFF, (sequence >> 16) & 0xFF,
    ])
    if flags:
        out += (flags & 0xFFFF).to_bytes(2, "little")
    for value, cls in ((value1, c1), (value2, c2)):
        out += (value & 0xFFFFFFFF).to_bytes(4, "little")[: _CLASS_BYTES[cls]]
    return bytes(out)


def decode_telemetry(raw: bytes) -> Optional[Telemetry]:
    """Decode a telemetry frame; None if it is not a well-formed frame."""
    if len(raw) < 5 or not (raw[0] & BINARY_MARK) or (raw[1] & 0x80):
        return None
    c1, c2 = (raw[0] >> 2) & 3, raw[0] & 3
    has_flags = bool(raw[1] & 0x40)
    index = 5 + (2 if has_flags else 0)
    if len(raw) != index + _CLASS_BYTES[c1] + _CLASS_BYTES[c2]:
        return None
    flags = int.from_bytes(raw[5:7], "little") if has_flags else 0
    values = []
    for cls in (c1, c2):
        width = _CLASS_BYTES[cls]
        values.append(int.from_bytes(raw[index:index + width], "little", signed=True) if width else 0)
        index += width
    return Telemetry((raw[0] >> 4) & 7, raw[1] & 63, flags, values[0], values[1],
                     raw[2] | (raw[3] << 8) | (raw[4] << 16))


def canonical_text(frame: Telemetry) -> str:
    """The text form the dashboard page already understands (generic value1 / value2 names)."""
    return (f"src={frame.source} type={frame.type} seq={frame.sequence} "
            f"value1={frame.value1} value2={frame.value2} flags={frame.flags}")


def encode_command(opcode: int) -> bytes:
    return bytes([BINARY_MARK | (opcode & 0x7F)])


def encode_map(codes: str) -> bytes:
    """26 letters of L, S, R, U or N (A..Z) as an 11-byte MAP command."""
    if len(codes) != MAP_LETTERS or any(letter not in _MAP_VALUES for letter in codes):
        raise ValueError("map must be 26 letters of L, S, R, U or N")
    bits = 0
    for index, letter in enumerate(codes):
        bits |= _MAP_VALUES[letter] << (index * MAP_ACTION_BITS)
    return encode_command(CMD_MAP) + bits.to_bytes(MAP_BYTES, "little")

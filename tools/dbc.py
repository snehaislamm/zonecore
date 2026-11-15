"""Minimal DBC parser + codec (Intel byte order) and AUTOSAR-E2E-P1-style helpers.

Used three ways:
  * tools/dbcgen.py   -> generates the embedded C pack/unpack layer
  * sim/ (SIL)        -> Python-side encode/decode of frames sent to/from the C VCU
  * tests             -> golden-vector cross-check that C and Python agree bit-for-bit
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

_BO = re.compile(r"^BO_\s+(\d+)\s+(\w+)\s*:\s*(\d+)\s+(\w+)")
_SG = re.compile(
    r"^\s*SG_\s+(\w+)\s*:\s*(\d+)\|(\d+)@([01])([+-])\s*\(([^,]+),([^)]+)\)\s*"
    r"\[([^|]+)\|([^\]]+)\]\s*\"([^\"]*)\"\s*(.*)$"
)


@dataclass
class Signal:
    name: str
    start: int
    length: int
    signed: bool
    factor: float
    offset: float
    minimum: float
    maximum: float
    unit: str

    @property
    def is_integer(self) -> bool:
        return self.factor == 1.0 and self.offset == 0.0 and not self.signed and self.length <= 16

    def raw_limits(self) -> tuple[int, int]:
        if self.signed:
            return -(1 << (self.length - 1)), (1 << (self.length - 1)) - 1
        return 0, (1 << self.length) - 1


@dataclass
class Message:
    frame_id: int
    name: str
    dlc: int
    sender: str
    signals: list[Signal] = field(default_factory=list)

    def signal(self, name: str) -> Signal:
        return next(s for s in self.signals if s.name == name)


def parse(path: str | Path) -> dict[str, Message]:
    msgs: dict[str, Message] = {}
    cur: Message | None = None
    for line in Path(path).read_text().splitlines():
        if m := _BO.match(line):
            cur = Message(int(m[1]), m[2], int(m[3]), m[4])
            msgs[cur.name] = cur
        elif (m := _SG.match(line)) and cur is not None:
            if m[4] != "1":
                raise ValueError(f"{m[1]}: only Intel (@1) byte order supported")
            cur.signals.append(
                Signal(m[1], int(m[2]), int(m[3]), m[5] == "-", float(m[6]), float(m[7]),
                       float(m[8]), float(m[9]), m[10])
            )
        elif not line.strip():
            cur = None
    _check_overlap(msgs)
    return msgs


def _check_overlap(msgs: dict[str, Message]) -> None:
    for msg in msgs.values():
        used = 0
        for s in msg.signals:
            mask = ((1 << s.length) - 1) << s.start
            if used & mask:
                raise ValueError(f"{msg.name}.{s.name} overlaps another signal")
            if s.start + s.length > msg.dlc * 8:
                raise ValueError(f"{msg.name}.{s.name} exceeds DLC")
            used |= mask


def encode(msg: Message, values: dict[str, float]) -> bytes:
    word = 0
    for s in msg.signals:
        phys = values.get(s.name, 0.0)
        raw = int(round((phys - s.offset) / s.factor))
        lo, hi = s.raw_limits()
        raw = max(lo, min(hi, raw))
        word |= (raw & ((1 << s.length) - 1)) << s.start
    return word.to_bytes(msg.dlc, "little")


def decode(msg: Message, data: bytes) -> dict[str, float]:
    word = int.from_bytes(bytes(data[: msg.dlc]), "little")
    out: dict[str, float] = {}
    for s in msg.signals:
        raw = (word >> s.start) & ((1 << s.length) - 1)
        if s.signed and raw & (1 << (s.length - 1)):
            raw -= 1 << s.length
        out[s.name] = raw * s.factor + s.offset
    return out


# ---------------- E2E (mirror of src/com/zc_e2e.c) ----------------
def crc8_j1850(data: bytes, init: int = 0xFF, final_xor: bool = True) -> int:
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc ^ 0xFF if final_xor else crc


def e2e_protect(frame: bytes, data_id: int, counter: int) -> bytes:
    f = bytearray(frame)
    f[1] = (f[1] & 0xF0) | (counter & 0x0F)
    f[0] = crc8_j1850(bytes([data_id & 0xFF, data_id >> 8]) + bytes(f[1:8]))
    return bytes(f)


def e2e_crc_ok(frame: bytes, data_id: int) -> bool:
    return frame[0] == crc8_j1850(bytes([data_id & 0xFF, data_id >> 8]) + bytes(frame[1:8]))

"""Cyclic I/O layout of the Mowito robot PC (mw_profinet_bridge), PLC's view.

Same offsets as the bridge's IoMap (io_image.hpp) and the GSDML module
IDM_IO_6_6. "out" is what the PLC writes (PLC -> PC), "in" what it reads
(PC -> PLC). Bits are (byte, bit); REALs are 4-byte big-endian IEEE754.
Override any of them with a JSON file whose keys match the field names here
(the same names as the bridge's *_bit / *_byte registry params, minus the
suffix), e.g. {"stop_cycle": "0.6", "actual_torque": 2}.
"""
import json
import struct
from dataclasses import dataclass, field, fields


def parse_bit(text):
    byte, _, bit = str(text).partition(".")
    if not byte.isdigit() or bit not in list("01234567"):
        raise ValueError(f"{text!r} is not a byte.bit address (e.g. 0.3)")
    return int(byte), int(bit)


@dataclass
class Layout:
    # PLC -> PC (PLC outputs)
    start_cycle: tuple = (0, 0)
    nutrunner_ack_toggle: tuple = (0, 1)
    torque_ok: tuple = (0, 2)
    fault_reset: tuple = (0, 3)
    stop_cycle: tuple = (0, 4)
    actual_torque: int = 2
    # PC -> PLC (PLC inputs)
    servo_ready: tuple = (0, 0)
    servo_busy: tuple = (0, 1)
    cycle_done: tuple = (0, 2)
    servo_fault: tuple = (0, 3)
    nutrunner_req: tuple = (0, 4)
    nutrunner_req_toggle: tuple = (0, 5)
    system_status: int = 1
    target_torque: int = 2
    # fixed by the GSDML module
    out_bytes: int = 6
    in_bytes: int = 6

    @classmethod
    def load(cls, path=None):
        layout = cls()
        if not path:
            return layout
        with open(path) as fh:
            overrides = json.load(fh)
        known = {f.name: f for f in fields(cls)}
        for key, value in overrides.items():
            if key not in known:
                raise ValueError(f"{path}: unknown layout key {key!r}")
            setattr(layout, key, parse_bit(value) if known[key].type is tuple else int(value))
        return layout


def get_bit(image, addr):
    byte, bit = addr
    return bool((image[byte] >> bit) & 1)


def set_bit(image, addr, value):
    byte, bit = addr
    if value:
        image[byte] |= 1 << bit
    else:
        image[byte] &= ~(1 << bit) & 0xFF


def get_real(image, offset):
    return struct.unpack(">f", bytes(image[offset:offset + 4]))[0]


def set_real(image, offset, value):
    image[offset:offset + 4] = struct.pack(">f", value)


@dataclass
class RobotStatus:
    """The PC -> PLC image, decoded."""
    servo_ready: bool = False
    servo_busy: bool = False
    cycle_done: bool = False
    servo_fault: bool = False
    nutrunner_req: bool = False
    nutrunner_req_toggle: bool = False
    system_status: int = 0
    target_torque: float = 0.0

    @classmethod
    def decode(cls, layout, image):
        return cls(
            servo_ready=get_bit(image, layout.servo_ready),
            servo_busy=get_bit(image, layout.servo_busy),
            cycle_done=get_bit(image, layout.cycle_done),
            servo_fault=get_bit(image, layout.servo_fault),
            nutrunner_req=get_bit(image, layout.nutrunner_req),
            nutrunner_req_toggle=get_bit(image, layout.nutrunner_req_toggle),
            system_status=image[layout.system_status],
            target_torque=get_real(image, layout.target_torque),
        )

    def short(self):
        flags = [n for n in ("servo_ready", "servo_busy", "cycle_done", "servo_fault", "nutrunner_req")
                 if getattr(self, n)]
        return (f"[{' '.join(flags) or '-'}] toggle={int(self.nutrunner_req_toggle)} "
                f"status={self.system_status} target={self.target_torque:.3f}")

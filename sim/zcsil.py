"""ctypes bridge to libzonecore_sil.so (the production C code, built for host).

The struct layouts below mirror include/zc/zc_vcu.h field-for-field; a size
check against zc_vcu_abi_sizes() runs at load time so ABI drift fails loudly.
"""
from __future__ import annotations

import ctypes as C
import os
from pathlib import Path

N_CELLS = 96
N_TEMPS = 16
N_WHEELS = 4

f32, u8, i8, u16, u32 = C.c_float, C.c_uint8, C.c_int8, C.c_uint16, C.c_uint32


class VcuIn(C.Structure):
    _fields_ = [
        ("cell_v", f32 * N_CELLS), ("cell_t", f32 * N_TEMPS),
        ("pack_i", f32), ("pack_v", f32), ("bus_v", f32), ("coolant_t", f32),
        ("apps1", f32), ("apps2", f32), ("brake_sw", u8), ("drive_request", u8),
        ("wheel_w", f32 * N_WHEELS), ("steer_rad", f32), ("yaw_rate", f32), ("ax", f32),
        ("plug_present", u8), ("target_soc", f32),
        ("stalk_left", u8), ("stalk_right", u8), ("hazard_btn", u8), ("crash", u8),
        ("ambient_lux", f32),
        ("win_up", u8), ("win_down", u8), ("win_auto_up", u8),
        ("win_current", f32), ("win_pos_mm", f32),
        ("tcs_enable", u8), ("tv_enable", u8),
    ]


class VcuOut(C.Structure):
    _fields_ = [
        ("trq_cmd", f32 * N_WHEELS), ("trq_demand", f32),
        ("inv_enable", u8), ("safe_state", u8), ("apps_fault", u8), ("bto_active", u8),
        ("tqmon_fault", u8), ("wdgm_fault", u8),
        ("v_est", f32), ("yaw_ref", f32), ("mz_cmd", f32), ("slip", f32 * N_WHEELS),
        ("hv_state", u8), ("k_neg", u8), ("k_pos", u8), ("k_pre", u8), ("k_dcc", u8),
        ("bms_level", u8), ("bal_count", u8), ("bms_faults", u32),
        ("soc", f32), ("soc_cc", f32), ("chg_i_lim", f32), ("dch_i_lim", f32),
        ("vmax", f32), ("vmin", f32), ("tmax", f32),
        ("chg_state", u8), ("chg_end", u8), ("chg_i_req", f32), ("chg_v_req", f32),
        ("th_mode", u8), ("chiller_w", f32), ("heater_w", f32), ("pump", f32),
        ("turn_l", u8), ("turn_r", u8), ("stop_lamp", u8), ("headlamp", u8),
        ("locked", u8), ("hazard", u8), ("ess", u8), ("win_motor", i8), ("win_pinch", u8),
        ("win_force_est", f32),
        ("dtc_confirmed", u16), ("sched_overruns", u32), ("time_ms", u32),
    ]


# enums mirrored from headers
HV_OPEN, HV_PRECHARGE, HV_CLOSED, HV_OPENING, HV_FAULT = range(5)
SAFE_NORMAL, SAFE_DERATED, SAFE_STOP, SAFE_HV_OFF = range(4)
(CHG_IDLE, CHG_CONNECTED, CHG_CABLE_CHECK, CHG_PRECHARGE, CHG_CHARGING,
 CHG_STOPPING, CHG_COMPLETE, CHG_FAULT) = range(8)
(END_NONE, END_TARGET_SOC, END_FULL, END_UNPLUG, END_EVSE_TIMEOUT, END_EVSE_FAULT,
 END_ESTOP, END_BMS_FAULT, END_STEP_TIMEOUT) = range(9)
TH_OFF, TH_COOL, TH_HEAT = range(3)
INJ_NONE, INJ_STALL_DRIVE_TASK, INJ_TORQUE_OFFSET, INJ_EVSE_CRC = range(4)


def _find_lib() -> Path:
    env = os.environ.get("ZC_SIL_LIB")
    if env:
        return Path(env)
    root = Path(__file__).resolve().parents[1]
    for cand in (root / "build" / "libzonecore_sil.so", root / "build" / "libzonecore_sil.dylib"):
        if cand.exists():
            return cand
    raise FileNotFoundError("libzonecore_sil not found - run `cmake -B build && cmake --build build`")


class Vcu:
    """Thin object wrapper; one instance per process (the C side is a singleton)."""

    def __init__(self, soc_init: float = 0.5, soc_init_var: float = 1e-4):
        lib = C.CDLL(str(_find_lib()))
        lib.zc_vcu_init.argtypes = [f32, f32]
        lib.zc_vcu_step.argtypes = [C.POINTER(VcuIn), C.POINTER(VcuOut), u32]
        lib.zc_vcu_can_rx.argtypes = [u32, C.POINTER(u8), u8]
        lib.zc_vcu_can_tx_pop.argtypes = [C.POINTER(u32), C.POINTER(u8), C.POINTER(u8)]
        lib.zc_vcu_can_tx_pop.restype = u8
        lib.zc_vcu_inject.argtypes = [u8, f32]
        lib.zc_vcu_dtc.argtypes = [u8, C.POINTER(u32), C.POINTER(u8), C.POINTER(u16)]
        lib.zc_vcu_dtc.restype = u8
        lib.zc_vcu_cell_balancing.argtypes = [u8]
        lib.zc_vcu_cell_balancing.restype = u8
        lib.zc_vcu_abi_sizes.argtypes = [C.POINTER(u32), C.POINTER(u32)]
        lib.zc_vcu_sched_util_pct.restype = f32
        lib.zc_vcu_task_max_exec_us.argtypes = [u8]
        lib.zc_vcu_task_max_exec_us.restype = u32
        self.lib = lib

        a, b = u32(), u32()
        lib.zc_vcu_abi_sizes(C.byref(a), C.byref(b))
        if (a.value, b.value) != (C.sizeof(VcuIn), C.sizeof(VcuOut)):
            raise RuntimeError(
                f"ABI mismatch: C in/out={a.value}/{b.value}, py={C.sizeof(VcuIn)}/{C.sizeof(VcuOut)}")

        lib.zc_vcu_init(soc_init, soc_init_var)
        self.inp = VcuIn()
        self.out = VcuOut()
        self._buf = (u8 * 8)()

    def step(self, n_ms: int) -> VcuOut:
        self.lib.zc_vcu_step(C.byref(self.inp), C.byref(self.out), n_ms)
        return self.out

    def can_rx(self, frame_id: int, data: bytes) -> None:
        buf = (u8 * 8).from_buffer_copy(bytes(data).ljust(8, b"\0"))
        self.lib.zc_vcu_can_rx(frame_id, buf, len(data))

    def can_tx(self):
        """Yield (id, bytes) for every frame the VCU transmitted since last call."""
        fid, dlc = u32(), u8()
        while self.lib.zc_vcu_can_tx_pop(C.byref(fid), self._buf, C.byref(dlc)):
            yield fid.value, bytes(self._buf)[: dlc.value]

    def inject(self, kind: int, value: float = 0.0) -> None:
        self.lib.zc_vcu_inject(kind, value)

    def dtcs(self) -> list[tuple[int, int, int]]:
        out, i = [], 0
        code, st, occ = u32(), u8(), u16()
        while self.lib.zc_vcu_dtc(i, C.byref(code), C.byref(st), C.byref(occ)):
            out.append((code.value, st.value, occ.value))
            i += 1
        return out

    def balancing(self, cell: int) -> bool:
        return bool(self.lib.zc_vcu_cell_balancing(cell))

    def task_max_exec_us(self) -> list[int]:
        return [self.lib.zc_vcu_task_max_exec_us(i) for i in range(4)]

    def sched_util_pct(self) -> float:
        return self.lib.zc_vcu_sched_util_pct()

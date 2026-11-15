"""SIL test bench: closes the loop between the C VCU and the Python plants.

All charger traffic and the inverter torque command go over *encoded CAN
frames* (DBC + E2E), so the bench also exercises the generated codec and the
E2E layer from the receiving node's point of view.
"""
from __future__ import annotations

import ctypes as C
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import dbc  # noqa: E402

from plant import BatteryPlant, EvsePlant, VehiclePlant, WindowPlant  # noqa: E402
import zcsil  # noqa: E402

DBC = dbc.parse(Path(__file__).resolve().parents[1] / "can" / "zonecore.dbc")
M_TRQ, M_CHG, M_EVSE = DBC["VCU_TorqueCmd"], DBC["VCU_ChargeReq"], DBC["EVSE_Status"]
M_BMS = DBC["BMS_Status"]
AUX_W = 800.0
CHILLER_COP = 2.5


class Bench:
    def __init__(self, soc=0.5, soc_est=None, soc_var=1e-4, temp=25.0, mu=0.9, vx0=0.0, seed=1,
                 evse: EvsePlant | None = None, **batt_kw):
        self.vcu = zcsil.Vcu(soc if soc_est is None else soc_est, soc_var)
        self.bat = BatteryPlant(soc0=soc, t0=temp, seed=seed, t_amb=temp, **batt_kw)
        self.veh = VehiclePlant(mu=mu, vx=vx0)
        self.evse = evse
        self.win = WindowPlant()
        self.t = 0.0
        self.log = defaultdict(list)
        self.trq = [0.0] * 4
        self.brake_nm = 0.0
        self.extra_current = 0.0         # forced pack current (battery-only tests)
        self.e2e_errors = 0
        self.evse_tx_acc = 0.0
        i = self.vcu.inp
        i.target_soc = 0.8
        i.ambient_lux = 20000.0
        i.tcs_enable = 1
        i.tv_enable = 1
        self._sync_inputs()

    # ---------------------------------------------------------------- I/O
    def _sync_inputs(self) -> None:
        i, b, v = self.vcu.inp, self.bat, self.veh
        C.memmove(i.cell_v, np.ascontiguousarray(b.sense_v(), dtype=np.float32).ctypes.data, 4 * zcsil.N_CELLS)
        C.memmove(i.cell_t, np.ascontiguousarray(b.sense_t(), dtype=np.float32).ctypes.data, 4 * zcsil.N_TEMPS)
        i.pack_i = b.current
        i.pack_v = b.pack_v
        i.bus_v = b.bus_v
        i.coolant_t = b.t_coolant
        for k in range(4):
            i.wheel_w[k] = v.w[k]
        i.yaw_rate = v.r
        i.ax = v.ax
        i.win_current = self.win.current
        i.win_pos_mm = self.win.pos_mm

    def _send_evse(self, dt: float) -> None:
        if self.evse is None or not self.vcu.inp.plug_present:
            return
        self.evse_tx_acc += dt
        while self.evse_tx_acc >= 0.05:            # 20 Hz
            self.evse_tx_acc -= 0.05
            if not self.evse.comm_alive():
                continue
            raw = dbc.encode(M_EVSE, self.evse.status())
            frame = dbc.e2e_protect(raw, M_EVSE.frame_id, self.evse.counter)
            self.evse.counter = (self.evse.counter + 1) & 0xF
            self.vcu.can_rx(M_EVSE.frame_id, frame)

    def _read_can(self) -> None:
        for fid, data in self.vcu.can_tx():
            if fid == M_TRQ.frame_id:
                if not dbc.e2e_crc_ok(data, fid):
                    self.e2e_errors += 1
                    continue
                d = dbc.decode(M_TRQ, data)
                self.trq = [d["TrqFL"], d["TrqFR"], d["TrqRL"], d["TrqRR"]] if d["InvEnable"] else [0.0] * 4
            elif fid == M_CHG.frame_id and self.evse is not None:
                if dbc.e2e_crc_ok(data, fid):
                    self.evse.on_charge_req(dbc.decode(M_CHG, data))
                else:
                    self.e2e_errors += 1
            elif fid == M_BMS.frame_id and not dbc.e2e_crc_ok(data, fid):
                self.e2e_errors += 1

    # ---------------------------------------------------------------- step
    def step(self, dt_ms: int = 10, substep_ms: int = 1, drive: bool = True, record: tuple = ()) -> zcsil.VcuOut:
        dt = dt_ms / 1000.0
        self._sync_inputs()
        self._send_evse(dt)
        o = self.vcu.step(dt_ms)
        self._read_can()

        bal = np.array([self.vcu.balancing(c) for c in range(zcsil.N_CELLS)], dtype=float) \
            if o.bal_count else np.zeros(zcsil.N_CELLS)
        n_sub = max(1, dt_ms // substep_ms)
        h = dt / n_sub
        for _ in range(n_sub):
            if drive:
                self.veh.step(h, self.trq, self.vcu.inp.steer_rad, self.brake_nm)
            p_elec = self.veh.elec_power() if (o.k_pos and o.k_neg) else 0.0
            p_th = o.chiller_w / CHILLER_COP + o.heater_w
            i_chg = 0.0
            if self.evse is not None:
                self.evse.step(h, self.bat.pack_v, bool(o.k_dcc and o.k_pos))
                i_chg = self.evse.i_out if (o.k_dcc and o.k_pos) else 0.0
            v = max(self.bat.pack_v, 100.0)
            hv_on = bool(o.k_pos and o.k_neg)
            i_pack = ((p_elec + AUX_W + p_th) / v if hv_on else 0.0) - i_chg + self.extra_current
            self.bat.step(h, i_pack, (bool(o.k_neg), bool(o.k_pos), bool(o.k_pre)),
                          o.chiller_w, o.heater_w, o.pump, bal)
            self.win.step(h, o.win_motor)
        self.t += dt
        for name in record:
            self.log[name].append(self.value(name))
        if record:
            self.log["t"].append(self.t)
        return o

    def value(self, name: str) -> float:
        o, b, v = self.vcu.out, self.bat, self.veh
        table = {
            "true_soc": lambda: float(np.mean(b.soc)), "est_soc": lambda: o.soc, "cc_soc": lambda: o.soc_cc,
            "pack_i": lambda: b.current, "pack_v": lambda: b.pack_v, "vmax": lambda: float(np.max(b.cell_v())),
            "tmax": lambda: float(np.max(b.temp)), "coolant": lambda: b.t_coolant,
            "chg_i_req": lambda: o.chg_i_req, "chg_lim": lambda: o.chg_i_lim, "chg_state": lambda: o.chg_state,
            "chiller": lambda: o.chiller_w, "heater": lambda: o.heater_w,
            "evse_i": lambda: self.evse.i_out if self.evse else 0.0,
            "vx": lambda: v.vx, "v_est": lambda: o.v_est, "yaw": lambda: v.r, "yaw_ref": lambda: o.yaw_ref,
            "slip_mean": lambda: float(np.mean(v.slip())), "slip_max": lambda: float(np.max(v.slip())),
            "trq_sum": lambda: sum(self.trq), "trq_demand": lambda: o.trq_demand, "mz": lambda: o.mz_cmd,
            "stop_lamp": lambda: o.stop_lamp, "turn_l": lambda: o.turn_l, "ax": lambda: v.ax,
            "win_pos": lambda: self.win.pos_mm, "win_force": lambda: self.win.force,
        }
        return table[name]()

    def run(self, seconds: float, dt_ms: int = 10, **kw) -> zcsil.VcuOut:
        o = self.vcu.out
        for _ in range(int(round(seconds * 1000 / dt_ms))):
            o = self.step(dt_ms, **kw)
        return o

    def power_up(self) -> zcsil.VcuOut:
        """Key on, request HV for driving, wait for contactors."""
        self.vcu.inp.drive_request = 1
        for _ in range(100):
            o = self.step(10)
            if o.hv_state == zcsil.HV_CLOSED:
                return o
        raise AssertionError(f"HV did not close: state={o.hv_state} faults=0x{o.bms_faults:x}")

    def plug_in(self, target_soc: float = 0.8) -> None:
        self.vcu.inp.drive_request = 0
        self.vcu.inp.plug_present = 1
        self.vcu.inp.target_soc = target_soc
        if self.evse is None:
            self.evse = EvsePlant()

    def set_pedal(self, pct: float, apps2: float | None = None) -> None:
        self.vcu.inp.apps1 = pct
        self.vcu.inp.apps2 = pct if apps2 is None else apps2

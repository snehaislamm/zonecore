"""Physical plant models for the SIL bench.

Deliberately *not* identical to the controller's internal models: cells have
manufacturing spread in capacity/resistance/initial SOC, resistance depends on
temperature, tyres follow a Pacejka-style curve, and the EVSE is a separate
state machine talking only via CAN. This is what makes SIL meaningful.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

N_CELLS, N_TEMPS = 96, 16

OCV_SOC = np.linspace(0.0, 1.0, 11)
OCV_V = np.array([3.00, 3.45, 3.55, 3.61, 3.66, 3.72, 3.81, 3.91, 4.00, 4.09, 4.19])


def ocv(soc):
    return np.interp(soc, OCV_SOC, OCV_V)


# ============================================================ HV battery
@dataclass
class BatteryPlant:
    soc0: float = 0.5
    t0: float = 25.0
    seed: int = 1
    cap_ah: float = 150.0
    r0: float = 0.0008
    r1: float = 0.0005
    c1: float = 40000.0
    cell_heat_cap: float = 2800.0      # J/K per cell
    h_cell: float = 6.0                # W/K cell->coolant at full pump
    coolant_heat_cap: float = 60000.0  # J/K
    h_amb: float = 25.0                # W/K coolant->ambient
    t_amb: float = 25.0
    bleed_a: float = 0.1               # passive balancing current
    dclink_c: float = 1.0e-3
    precharge_r: float = 50.0
    precharge_broken: bool = False

    def __post_init__(self):
        rng = np.random.default_rng(self.seed)
        self.q = self.cap_ah * (1.0 + 0.01 * rng.standard_normal(N_CELLS))
        self.r0_i = self.r0 * (1.0 + 0.05 * rng.standard_normal(N_CELLS))
        self.soc = np.clip(self.soc0 + 0.004 * rng.standard_normal(N_CELLS), 0.0, 1.0)
        self.vrc = np.zeros(N_CELLS)
        self.temp = np.full(N_CELLS, float(self.t0)) + 0.3 * rng.standard_normal(N_CELLS)
        self.t_coolant = float(self.t0)
        self.current = 0.0
        self.bus_v = 0.0
        self.v_offset = np.zeros(N_CELLS)   # sensor fault injection
        self.t_offset = np.zeros(N_TEMPS)
        self.heat_w = 0.0

    def r0_t(self):
        # resistance rises ~2 %/K below 25 degC, falls (bounded) above
        return self.r0_i * np.clip(1.0 + 0.02 * (25.0 - self.temp), 0.6, 3.0)

    def cell_v(self):
        return ocv(self.soc) - self.vrc - self.r0_t() * self.current

    @property
    def pack_v(self) -> float:
        return float(np.sum(self.cell_v()))

    def sense_v(self):
        return self.cell_v() + self.v_offset

    def sense_t(self):
        return self.temp[:: N_CELLS // N_TEMPS][:N_TEMPS] + self.t_offset

    def step(self, dt: float, current: float, contactors: tuple[bool, bool, bool],
             chiller_w: float, heater_w: float, pump: float, balancing) -> None:
        self.current = current
        i_cell = current + self.bleed_a * balancing
        a = math.exp(-dt / (self.r1 * self.c1))
        self.soc = np.clip(self.soc - i_cell * dt / (3600.0 * self.q), 0.0, 1.0)
        self.vrc = a * self.vrc + self.r1 * (1.0 - a) * current

        # thermal: joule + polarisation heat, conduction to coolant loop
        q_gen = current**2 * self.r0_t() + self.vrc**2 / self.r1
        self.heat_w = float(np.sum(q_gen))
        h = self.h_cell * (0.25 + 0.75 * pump)
        q_cool = h * (self.temp - self.t_coolant)
        self.temp += dt * (q_gen - q_cool) / self.cell_heat_cap
        q_chiller = chiller_w if self.t_coolant > 2.0 else 0.0
        dq = np.sum(q_cool) - q_chiller + heater_w - self.h_amb * (self.t_coolant - self.t_amb)
        self.t_coolant += dt * dq / self.coolant_heat_cap

        # DC link
        neg, pos, pre = contactors
        v = self.pack_v
        if neg and pos:
            self.bus_v = v
        elif neg and pre and not self.precharge_broken:
            tau = self.precharge_r * self.dclink_c
            self.bus_v += (v - self.bus_v) * (1.0 - math.exp(-dt / tau))
        else:
            self.bus_v *= math.exp(-dt / 0.5)  # active discharge


# ============================================================ vehicle
@dataclass
class VehiclePlant:
    mass: float = 3200.0
    iz: float = 5200.0
    wheelbase: float = 3.08
    track: float = 1.70
    r_w: float = 0.38
    gear: float = 9.0
    j_wheel: float = 5.0            # wheel + reflected rotor inertia
    mu: float = 0.9
    c_alpha_f: float = 110000.0     # axle cornering stiffness [N/rad] (understeering SUV)
    c_alpha_r: float = 160000.0
    cd_a: float = 0.95
    crr: float = 0.012
    eta: float = 0.92
    motor_tau: float = 0.005
    vx: float = 0.0
    vy: float = 0.0
    r: float = 0.0
    ax: float = 0.0
    w: list = field(default_factory=lambda: [0.0] * 4)
    t_motor: list = field(default_factory=lambda: [0.0] * 4)

    def __post_init__(self):
        self.w = [self.vx / self.r_w] * 4

    @staticmethod
    def _tyre(lam: float, fz: float, mu: float) -> float:
        return mu * fz * math.sin(1.65 * math.atan(10.0 * lam))

    def elec_power(self) -> float:
        p = 0.0
        for i in range(4):
            pm = self.t_motor[i] * self.w[i] * self.gear
            p += pm / self.eta if pm > 0 else pm * self.eta
        return p

    def step(self, dt: float, trq_cmd, steer: float, brake_nm: float = 0.0) -> None:
        g = 9.81
        a = b = self.wheelbase / 2.0
        fz = self.mass * g / 4.0
        k = 1.0 - math.exp(-dt / self.motor_tau)
        fx = [0.0] * 4
        for i in range(4):
            self.t_motor[i] += (trq_cmd[i] - self.t_motor[i]) * k
            v_wheel = self.vx
            lam = (self.w[i] * self.r_w - v_wheel) / max(abs(v_wheel), 3.0)
            fx[i] = self._tyre(lam, fz, self.mu)
            t_brake = math.copysign(brake_nm, self.w[i]) if abs(self.w[i]) > 0.05 else 0.0
            dw = (self.t_motor[i] * self.gear - fx[i] * self.r_w - t_brake) / self.j_wheel
            self.w[i] = max(self.w[i] + dw * dt, 0.0) if self.vx >= -0.01 else self.w[i] + dw * dt

        # lateral single-track with friction-limited axle forces
        vx = max(self.vx, 0.5)
        alpha_f = steer - (self.vy + a * self.r) / vx
        alpha_r = -(self.vy - b * self.r) / vx
        fyf = max(-self.mu * 2 * fz, min(self.mu * 2 * fz, self.c_alpha_f * alpha_f))
        fyr = max(-self.mu * 2 * fz, min(self.mu * 2 * fz, self.c_alpha_r * alpha_r))
        if self.vx < 1.0:
            fyf *= self.vx
            fyr *= self.vx

        f_long = sum(fx) - 0.5 * 1.2 * self.cd_a * self.vx**2 - (self.crr * self.mass * g if self.vx > 0.05 else 0.0)
        mz = (fx[1] + fx[3] - fx[0] - fx[2]) * self.track / 2.0 + a * fyf * math.cos(steer) - b * fyr

        dvx = f_long / self.mass + self.vy * self.r
        dvy = (fyf * math.cos(steer) + fyr) / self.mass - self.vx * self.r
        self.ax = f_long / self.mass
        self.vx = max(self.vx + dvx * dt, 0.0)
        self.vy += dvy * dt
        self.r += (mz / self.iz) * dt
        if self.vx < 0.05:
            self.vy *= 0.9
            self.r *= 0.9

    def slip(self):
        return [(w * self.r_w - self.vx) / max(self.vx, 2.0) for w in self.w]


# ============================================================ DC EVSE
NOT_READY, READY, CABLE_CHECK_OK, PRECHARGE, CHARGING, STOPPED, FAULT = range(7)


@dataclass
class EvsePlant:
    max_current: float = 350.0
    max_power: float = 150_000.0
    ramp_aps: float = 100.0
    isolation_ok: bool = True
    comm_drop_at_s: float | None = None
    estop_at_s: float | None = None
    t: float = 0.0
    state: int = NOT_READY
    v_out: float = 0.0
    i_out: float = 0.0
    timer: float = 0.0
    counter: int = 0
    req: dict = field(default_factory=dict)

    def on_charge_req(self, msg: dict) -> None:
        self.req = msg

    def i_max(self, v: float) -> float:
        return min(self.max_current, self.max_power / max(v, 50.0))

    def step(self, dt: float, pack_v: float, dc_closed: bool) -> None:
        self.t += dt
        self.timer += dt
        vcu_state = int(self.req.get("ChargeState", 0))
        if self.estop_at_s is not None and self.t >= self.estop_at_s:
            self.state = FAULT
        if self.state == NOT_READY and self.timer > 0.5:
            self.state, self.timer = READY, 0.0
        elif self.state == READY and vcu_state == 2:          # CABLE_CHECK
            if self.timer > 2.0 and self.isolation_ok:
                self.state, self.timer = CABLE_CHECK_OK, 0.0
        elif self.state == CABLE_CHECK_OK and vcu_state == 3:  # PRECHARGE
            self.state = PRECHARGE
        elif self.state in (PRECHARGE, CHARGING, STOPPED):
            target_v = float(self.req.get("TargetVoltage", 0.0))
            if self.state == PRECHARGE:
                self.v_out += max(-200 * dt, min(200 * dt, target_v - self.v_out))
                if vcu_state == 4:
                    self.state = CHARGING
            if self.state == CHARGING:
                self.v_out = pack_v if dc_closed else self.v_out
                tgt = min(float(self.req.get("TargetCurrent", 0.0)), self.i_max(self.v_out))
                if not dc_closed or not int(self.req.get("ChargeEnable", 0)):
                    tgt = 0.0
                self.i_out += max(-400 * dt, min(self.ramp_aps * dt, tgt - self.i_out))
                if vcu_state in (5, 6, 7):
                    self.state = STOPPED
            if self.state == STOPPED:
                self.i_out = max(0.0, self.i_out - 400 * dt)
        if self.state == FAULT:
            self.i_out = 0.0

    def comm_alive(self) -> bool:
        return self.comm_drop_at_s is None or self.t < self.comm_drop_at_s

    def status(self) -> dict:
        return {
            "EvseState": self.state, "PresentVoltage": self.v_out, "PresentCurrent": self.i_out,
            "MaxCurrent": round(self.i_max(max(self.v_out, 300.0))), "IsolationOk": int(self.isolation_ok),
            "EStop": int(self.state == FAULT and self.estop_at_s is not None),
        }


# ============================================================ power window
@dataclass
class WindowPlant:
    pos_mm: float = 450.0
    v_free: float = 110.0         # mm/s
    f_stall: float = 450.0        # N
    i_free: float = 3.0           # A
    n_per_a: float = 40.0
    obstacle_mm: float | None = None
    k_obstacle: float = 10.0      # N/mm compliant test object
    force: float = 0.0
    peak_force: float = 0.0
    current: float = 3.0

    def step(self, dt: float, motor: int) -> None:
        f_obs = 0.0
        if self.obstacle_mm is not None and self.pos_mm < self.obstacle_mm:
            f_obs = self.k_obstacle * (self.obstacle_mm - self.pos_mm)
        self.force = f_obs
        self.peak_force = max(self.peak_force, f_obs)
        if motor < 0:
            v = -self.v_free * max(0.0, 1.0 - f_obs / self.f_stall)
            self.current = self.i_free + f_obs / self.n_per_a
        elif motor > 0:
            v = self.v_free
            self.current = self.i_free
        else:
            v, self.current = 0.0, 0.0
        self.pos_mm = min(450.0, max(0.0, self.pos_mm + v * dt))

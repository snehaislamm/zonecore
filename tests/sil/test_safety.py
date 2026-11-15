"""SIL: fault injection against the safety mechanisms (ISO 26262 style).

Each test injects a fault, then checks (a) detection, (b) the reaction /
safe state, and (c) that it happened inside the fault-tolerant time interval.
"""
import zcsil as Z
from bench import Bench


def driving(v0=15.0, pedal=30.0) -> Bench:
    b = Bench(soc=0.6, vx0=v0)
    b.power_up()
    b.set_pedal(pedal)
    b.run(0.5)
    return b


def time_until(b: Bench, cond, limit_s: float, dt_ms: int = 10) -> float:
    t0 = b.t
    while b.t - t0 < limit_s:
        o = b.step(dt_ms)
        if cond(o):
            return b.t - t0
    raise AssertionError("condition never met")


def dtc_confirmed(b: Bench, code: int) -> bool:
    return any(c == code and (st & 0x08) for c, st, _ in b.vcu.dtcs())


def test_apps_disagreement_removes_torque():
    """@verifies REQ-PROP-001 REQ-SAF-003 REQ-SAF-004"""
    b = driving(pedal=40)
    assert sum(b.trq) > 100
    b.set_pedal(40, apps2=20)  # track 2 sticks / drifts
    dt = time_until(b, lambda o: sum(b.trq) <= 0.0, 0.5)
    assert dt <= 0.12
    b.set_pedal(40)            # sensor "recovers": fault stays latched
    o = b.run(0.3)
    assert o.apps_fault and o.safe_state == Z.SAFE_STOP and sum(b.trq) <= 0.0
    assert dtc_confirmed(b, 0x0A2001)


def test_torque_monitor_catches_corrupted_command():
    """@verifies REQ-SAF-002"""
    b = driving(v0=20.0, pedal=0.0)   # coasting / regen: driver wants no torque
    b.vcu.inject(Z.INJ_TORQUE_OFFSET, 150.0)  # +600 Nm total unintended
    dt = time_until(b, lambda o: o.tqmon_fault, 0.5)
    assert dt <= 0.10                  # FTTI
    b.run(0.02)
    assert all(t == 0.0 for t in b.trq)
    o = b.run(0.2)
    assert o.safe_state == Z.SAFE_STOP and dtc_confirmed(b, 0x0A2002)


def test_alive_supervision_detects_task_stall():
    """@verifies REQ-SAF-001"""
    b = driving()
    b.vcu.inject(Z.INJ_STALL_DRIVE_TASK)
    dt = time_until(b, lambda o: o.wdgm_fault, 1.0)
    assert dt <= 0.31                  # 2 missed 100 ms supervision cycles (+1 phase)
    o = b.run(0.1)
    assert o.safe_state == Z.SAFE_STOP and dtc_confirmed(b, 0x0A3001)


def test_cell_overvoltage_critical_opens_contactors():
    """@verifies REQ-BMS-003 REQ-SAF-004"""
    b = driving()
    b.bat.v_offset[42] = 0.6           # cell reads ~4.3+ V
    dt = time_until(b, lambda o: o.k_pos == 0, 0.5)
    assert dt <= 0.05
    o = b.run(0.2)
    assert o.hv_state == Z.HV_FAULT and o.safe_state == Z.SAFE_HV_OFF
    assert all(t == 0.0 for t in b.trq)


def test_sense_wire_open_controlled_shutdown():
    """@verifies REQ-BMS-005 REQ-BMS-002"""
    b = driving(pedal=10)
    b.bat.v_offset[7] = -10.0          # reads negative: open sense wire
    dt = time_until(b, lambda o: o.hv_state in (Z.HV_OPENING, Z.HV_FAULT), 0.5)
    assert dt <= 0.08
    o = b.run(1.0)
    assert o.hv_state == Z.HV_FAULT and o.k_pos == 0


def test_overtemperature_severe():
    """@verifies REQ-BMS-004"""
    b = driving()
    b.bat.t_offset[3] = 40.0           # sensor sees 65 degC
    dt = time_until(b, lambda o: o.bms_level >= 2, 2.0)
    assert 0.95 <= dt <= 1.1


def test_precharge_failure_latched():
    """@verifies REQ-BMS-006"""
    b = Bench(soc=0.6, precharge_broken=True)
    b.vcu.inp.drive_request = 1
    dt = time_until(b, lambda o: o.hv_state == Z.HV_FAULT, 2.0)
    assert 0.95 <= dt <= 1.1
    o = b.run(0.2)
    assert o.k_neg == 0 and o.k_pre == 0 and o.k_pos == 0
    assert dtc_confirmed(b, 0x0A1020)


def test_crash_isolates_hv_unlocks_and_hazards():
    """@verifies REQ-BODY-007"""
    b = driving(v0=12.0)
    assert b.vcu.out.locked
    b.vcu.inp.crash = 1
    o = b.run(0.2)
    assert o.safe_state == Z.SAFE_HV_OFF and o.k_pos == 0
    assert not o.locked and o.hazard

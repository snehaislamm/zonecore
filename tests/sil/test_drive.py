"""SIL: propulsion, vehicle dynamics, bus-level E2E."""
import numpy as np
import pytest

import dbc
import zcsil as Z
from bench import Bench, M_TRQ


def launch(mu: float, tcs: bool, seconds: float = 3.0) -> Bench:
    b = Bench(soc=0.7, mu=mu)
    b.vcu.inp.tcs_enable = int(tcs)
    b.power_up()
    b.set_pedal(100)
    for _ in range(int(seconds * 100)):
        b.step(10, record=("slip_mean", "vx"))
    return b


@pytest.mark.parametrize("mu", [0.3, 0.1])
def test_tcs_low_mu_launch(mu):
    """@verifies REQ-DYN-001"""
    on, off = launch(mu, True), launch(mu, False)
    slip_on = np.array(on.log["slip_mean"])[50:]
    slip_off = np.array(off.log["slip_mean"])[50:]
    assert slip_on.mean() <= 0.25
    assert slip_off.mean() > 2.0          # uncontrolled: wheels flare
    assert on.veh.vx > off.veh.vx         # and TCS is faster, not just safer


def step_steer(tv: bool) -> float:
    b = Bench(soc=0.7, vx0=20.0)
    b.vcu.inp.tv_enable = int(tv)
    b.power_up()
    b.set_pedal(30)
    b.run(1.0)
    b.vcu.inp.steer_rad = 0.04
    for _ in range(300):
        b.step(10, record=("yaw", "yaw_ref"))
    e = np.array(b.log["yaw"]) - np.array(b.log["yaw_ref"])
    return float(np.sqrt(np.mean(e**2)))


def test_torque_vectoring_step_steer():
    """@verifies REQ-DYN-002"""
    rms_on, rms_off = step_steer(True), step_steer(False)
    assert rms_on <= 0.7 * rms_off, f"TV {rms_on:.4f} vs off {rms_off:.4f}"


def test_power_limited_acceleration():
    """@verifies REQ-PROP-003"""
    b = Bench(soc=0.7)
    b.power_up()
    b.set_pedal(100)
    worst = 0.0
    for _ in range(600):
        o = b.step(10)
        if b.veh.vx > 5.0:
            worst = max(worst, b.bat.current / max(o.dch_i_lim, 1.0))
    assert worst <= 1.05  # within 5 %: aux loads + motor efficiency model error


def test_one_pedal_regen_and_stop_lamp():
    """@verifies REQ-PROP-004 REQ-BODY-004"""
    b = Bench(soc=0.6, vx0=25.0)
    b.power_up()
    b.set_pedal(0)
    decel, lamp = [], []
    for _ in range(300):
        o = b.step(10)
        decel.append(-b.veh.ax)
        lamp.append(o.stop_lamp)
    d = np.array(decel[100:])
    assert 1.2 < d.mean() < 1.9            # 1.5 m/s^2 target + drag
    assert all(lamp[150:])                 # > 1.3 m/s^2 lights stop lamps
    assert b.bat.current < 0               # energy recovered into the pack


def test_brake_throttle_override_sil():
    """@verifies REQ-PROP-002"""
    b = Bench(soc=0.6, vx0=15.0)
    b.power_up()
    b.set_pedal(60)
    b.run(0.5)
    b.vcu.inp.brake_sw = 1
    b.brake_nm = 800.0
    o = b.run(0.05)
    assert o.bto_active and sum(b.trq) <= 0.0


def test_inverter_frames_e2e_and_codec():
    """@verifies REQ-COM-001 REQ-COM-004"""
    b = Bench(soc=0.6)
    b.power_up()
    b.set_pedal(40)
    counters, mismatches = [], 0
    for _ in range(200):
        b._sync_inputs()
        o = b.vcu.step(10)
        for fid, data in b.vcu.can_tx():
            if fid == M_TRQ.frame_id:
                assert dbc.e2e_crc_ok(data, fid)
                d = dbc.decode(M_TRQ, data)
                counters.append(int(d["Counter"]))
                ref = list(o.trq_cmd)
                mismatches += sum(abs(d[k] - r) > 0.25 for k, r in zip(("TrqFL", "TrqFR", "TrqRL", "TrqRR"), ref))
                # Python re-encode must be bit-identical to what C sent
                assert dbc.e2e_protect(dbc.encode(M_TRQ, d), fid, int(d["Counter"])) == data
    assert mismatches == 0
    diffs = np.diff(counters) % 16
    assert np.all(diffs == 1)

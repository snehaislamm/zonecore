"""SIL: body controls, lighting homologation behaviour, anti-pinch window."""
import numpy as np

import zcsil as Z
from bench import Bench


def test_emergency_stop_signal_and_auto_hazard():
    """@verifies REQ-BODY-005"""
    b = Bench(soc=0.6, vx0=27.0)   # ~97 km/h
    b.power_up()
    b.set_pedal(0)
    b.vcu.inp.brake_sw = 1
    b.brake_nm = 2600.0
    lamp, ess = [], []
    while b.veh.vx > 0.05 and b.t < 10:
        o = b.step(10)
        lamp.append(o.stop_lamp)
        ess.append(o.ess)
    lamp = np.array(lamp)
    assert any(ess)
    i = int(np.argmax(ess))
    window = lamp[i:i + 100]                       # first second of ESS
    edges = int(np.sum(np.diff(window.astype(int)) == 1))
    assert 3 <= edges <= 5                         # 4 Hz +/- 1
    o = b.run(0.2)
    assert o.hazard                                 # auto hazards at standstill


def test_auto_headlamps_tunnel_entry():
    """@verifies REQ-BODY-006"""
    b = Bench(soc=0.6)
    b.vcu.inp.ambient_lux = 200.0
    t = 0.0
    while not b.vcu.out.headlamp and t < 3.0:
        b.step(10, drive=False)
        t += 0.01
    assert t <= 2.0


def test_autolock_above_8kmh():
    """@verifies REQ-BODY-008"""
    b = Bench(soc=0.6)
    b.power_up()
    b.set_pedal(30)
    unlocked_until = 0.0
    while b.t < 5.0 and not b.vcu.out.locked:
        b.step(10)
        unlocked_until = b.veh.vx
    assert b.vcu.out.locked and unlocked_until * 3.6 <= 9.0


def run_window(obstacle_mm):
    b = Bench(soc=0.6)
    b.win.obstacle_mm = obstacle_mm
    b.vcu.inp.win_auto_up = 1
    b.step(10, drive=False)
    b.vcu.inp.win_auto_up = 0
    for _ in range(800):
        b.step(10, drive=False)
    return b


def test_window_anti_pinch():
    """@verifies REQ-MECH-001"""
    for obstacle in (150.0, 60.0, 25.0):
        b = run_window(obstacle)
        assert b.win.peak_force < 100.0, f"{obstacle} mm: {b.win.peak_force:.1f} N"
        assert b.vcu.out.win_pinch
        assert b.win.pos_mm > obstacle + 100.0    # reversed well clear


def test_window_auto_up_unobstructed():
    """@verifies REQ-MECH-002"""
    b = run_window(None)
    assert b.win.pos_mm <= 0.5 and b.vcu.out.win_motor == 0 and not b.vcu.out.win_pinch

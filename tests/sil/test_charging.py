"""SIL: DC fast charging, battery thermal, and charger-communication faults."""
import numpy as np
import pytest

import zcsil as Z
from bench import Bench
from plant import EvsePlant


def charge(b: Bench, max_s: float, dt_ms: int = 100, until=(Z.CHG_COMPLETE, Z.CHG_FAULT), record=()):
    while b.t < max_s:
        o = b.step(dt_ms, substep_ms=dt_ms, drive=False, record=record)
        if o.chg_state in until:
            break
    return b.vcu.out


@pytest.fixture(scope="module")
def dcfc_run():
    b = Bench(soc=0.10, temp=25.0)
    b.plug_in(target_soc=0.80)
    o = charge(b, 45 * 60, record=("chg_i_req", "chg_lim", "evse_i", "vmax", "tmax", "true_soc",
                                   "est_soc", "chiller", "coolant", "chg_state"))
    return b, o


def test_dcfc_10_to_80_completes(dcfc_run):
    """@verifies REQ-CHG-007 REQ-CHG-006"""
    b, o = dcfc_run
    assert o.chg_state == Z.CHG_COMPLETE
    assert o.chg_end == Z.END_TARGET_SOC
    assert b.t < 40 * 60, f"took {b.t / 60:.1f} min"
    assert max(b.log["tmax"]) < 50.0
    assert abs(b.value("true_soc") - 0.80) < 0.03
    assert o.k_dcc == 0 and o.hv_state == Z.HV_OPEN
    assert b.evse.i_out < 2.0
    assert b.e2e_errors == 0


def test_dcfc_cv_holds_cell_voltage(dcfc_run):
    """@verifies REQ-CHG-003"""
    b, _ = dcfc_run
    assert max(b.log["vmax"]) <= 4.20


def test_dcfc_current_never_exceeds_limits(dcfc_run):
    """@verifies REQ-CHG-002"""
    b, _ = dcfc_run
    req = np.array(b.log["chg_i_req"])
    lim = np.array(b.log["chg_lim"])
    assert np.all(req <= lim + 0.5)
    assert np.all(req <= b.evse.max_current + 0.5)
    assert req.max() > 250.0  # and it actually fast-charges


def test_dcfc_thermal_management_engages(dcfc_run):
    """@verifies REQ-THM-001"""
    b, _ = dcfc_run
    tmax, ch = np.array(b.log["tmax"]), np.array(b.log["chiller"])
    first_cool = np.argmax(ch > 0)
    assert ch.max() > 0
    assert tmax[first_cool] <= 33.0  # chiller came on near the 32 degC DCFC threshold


def test_dcfc_soc_estimate_tracks_truth(dcfc_run):
    """@verifies REQ-BMS-001"""
    b, _ = dcfc_run
    err = np.abs(np.array(b.log["est_soc"]) - np.array(b.log["true_soc"]))
    assert err.max() < 0.03


def test_cold_soak_preconditioning():
    """@verifies REQ-THM-003 REQ-BMS-008"""
    b = Bench(soc=0.20, temp=5.0)
    b.plug_in()
    lim0 = None
    while b.t < 300:
        o = b.step(100, substep_ms=100, drive=False)
        if o.chg_state == Z.CHG_CHARGING and lim0 is None:
            lim0 = o.chg_i_lim
    assert o.heater_w > 0 and o.th_mode == Z.TH_HEAT
    assert lim0 is not None and lim0 < 150.0           # cold pack: charge power derated
    assert float(np.min(b.bat.temp)) > 6.0              # pack is warming
    assert o.chg_i_lim > lim0                           # and the limit rises with it


def _to_charging(b: Bench, t_end: float):
    charge(b, t_end, dt_ms=100, until=())


def test_evse_comm_loss_aborts_session():
    """@verifies REQ-CHG-004 REQ-COM-003 REQ-SAF-004"""
    b = Bench(soc=0.30)
    b.plug_in()
    b.evse = EvsePlant(comm_drop_at_s=60.0)
    _to_charging(b, 59.0)
    assert b.vcu.out.chg_state == Z.CHG_CHARGING and b.vcu.out.chg_i_req > 100
    last_valid = None
    while b.t < 62.0:
        if last_valid is None and not b.evse.comm_alive():
            last_valid = b.t
        o = b.step(10, substep_ms=10, drive=False)
        if o.chg_state == Z.CHG_FAULT:
            break
    assert o.chg_state == Z.CHG_FAULT and o.chg_end == Z.END_EVSE_TIMEOUT
    assert o.k_dcc == 0 and o.chg_i_req == 0.0
    assert b.t - last_valid <= 0.30
    b.run(0.2, dt_ms=100, substep_ms=100, drive=False)
    codes = {c: st for c, st, _ in b.vcu.dtcs()}
    assert codes[0x0A4001] & 0x08  # EVSE comm DTC confirmed


def test_evse_estop_aborts_fast():
    """@verifies REQ-CHG-005"""
    b = Bench(soc=0.30)
    b.plug_in()
    b.evse = EvsePlant(estop_at_s=40.0)
    _to_charging(b, 39.9)
    while b.t < 41.0:
        o = b.step(10, substep_ms=10, drive=False)
        if o.chg_state == Z.CHG_FAULT:
            break
    assert o.chg_end == Z.END_ESTOP
    # frame period 50 ms + 50 ms reaction budget
    assert b.t - 40.0 <= 0.10 + 1e-6


def test_unplug_during_charge():
    """@verifies REQ-CHG-005"""
    b = Bench(soc=0.30)
    b.plug_in()
    _to_charging(b, 30.0)
    b.vcu.inp.plug_present = 0
    o = b.step(10, substep_ms=10, drive=False)
    assert o.chg_state == Z.CHG_FAULT and o.chg_end == Z.END_UNPLUG and o.k_dcc == 0


def test_corrupted_evse_frames_are_rejected():
    """@verifies REQ-COM-002"""
    b = Bench(soc=0.30)
    b.plug_in()
    _to_charging(b, 30.0)
    b.vcu.inject(Z.INJ_EVSE_CRC)
    o = charge(b, 31.0, dt_ms=10)
    assert o.chg_state == Z.CHG_FAULT and o.chg_end == Z.END_EVSE_TIMEOUT

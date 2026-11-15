"""SIL: BMS estimation and balancing against the spread-cell battery plant."""
import numpy as np

from bench import Bench

# repeating synthetic urban/highway current profile [A], 100 ms resolution
PROFILE = [0] * 20 + [150] * 100 + [40] * 200 + [-80] * 60 + [250] * 30 + [0] * 90


def test_soc_ekf_recovers_from_20pct_initial_error():
    """@verifies REQ-BMS-001"""
    b = Bench(soc=0.60, soc_est=0.40, soc_var=0.04)
    err_cc, err_ekf = [], []
    for k in range(6000):  # 600 s
        b.extra_current = PROFILE[k % len(PROFILE)]
        o = b.step(100, substep_ms=100, drive=False)
        true = b.value("true_soc")
        err_ekf.append(abs(o.soc - true))
        err_cc.append(abs(o.soc_cc - true))
    assert err_ekf[-1] < 0.03
    assert max(err_ekf[3000:]) < 0.03      # stays converged under load
    assert err_cc[-1] > 0.15               # pure coulomb counting never recovers


def test_passive_balancing_bleeds_high_cells():
    """@verifies REQ-BMS-009"""
    b = Bench(soc=0.85, seed=7)
    high = np.arange(0, 96, 7)
    b.bat.soc[high] += 0.02                # inject imbalance
    lowest = int(np.argmin(b.bat.soc))
    gap0 = b.bat.soc[high].mean() - b.bat.soc[lowest]
    o = b.run(1.0, dt_ms=100, substep_ms=100, drive=False)
    assert o.bal_count >= len(high)
    assert all(b.vcu.balancing(int(c)) for c in high)
    assert not b.vcu.balancing(lowest)
    hours = 2.0
    b.run(hours * 3600, dt_ms=1000, substep_ms=1000, drive=False)   # parked
    gap1 = b.bat.soc[high].mean() - b.bat.soc[lowest]
    expected = b.bat.bleed_a * hours / b.bat.cap_ah    # passive bleed is slow: ~0.13 % SOC
    assert gap0 - gap1 > 0.75 * expected

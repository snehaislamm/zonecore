#!/usr/bin/env python3
"""Run headline SIL scenarios and render the figures used in the README.

usage: python3 sim/demo.py   (writes docs/img/*.png)
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bench import Bench  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "docs" / "img"
BLUE, ORANGE, INK, MUTED, GRID, SURF = "#2a78d6", "#eb6834", "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"

plt.rcParams.update({
    "figure.facecolor": SURF, "axes.facecolor": SURF, "axes.edgecolor": GRID, "axes.labelcolor": MUTED,
    "xtick.color": MUTED, "ytick.color": MUTED, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False, "lines.linewidth": 2.0, "font.size": 9,
    "axes.titlesize": 10, "axes.titlecolor": INK, "axes.titleweight": "bold", "legend.frameon": False,
})


def dcfc():
    b = Bench(soc=0.10)
    b.plug_in(0.80)
    rec = ("evse_i", "chg_lim", "vmax", "tmax", "coolant", "true_soc", "est_soc")
    while b.t < 45 * 60:
        o = b.step(100, substep_ms=100, drive=False, record=rec)
        if o.chg_state in (6, 7):
            break
    t = np.array(b.log["t"]) / 60
    fig, ax = plt.subplots(4, 1, figsize=(7, 8), sharex=True)
    ax[0].plot(t, b.log["evse_i"], color=BLUE, label="delivered")
    ax[0].plot(t, b.log["chg_lim"], color=ORANGE, lw=1.2, ls="--", label="BMS limit")
    ax[0].set_ylabel("A"); ax[0].set_title("Charge current"); ax[0].legend(loc="upper right")
    ax[1].plot(t, b.log["vmax"], color=BLUE); ax[1].axhline(4.20, color=MUTED, lw=1, ls=":")
    ax[1].text(t[0], 4.205, "4.20 V cell limit", color=MUTED, fontsize=8, va="bottom")
    ax[1].set_ylabel("V"); ax[1].set_title("Highest cell voltage (CV loop)")
    ax[2].plot(t, b.log["tmax"], color=BLUE, label="hottest cell")
    ax[2].plot(t, b.log["coolant"], color=ORANGE, label="coolant")
    ax[2].set_ylabel("degC"); ax[2].set_title("Battery thermal"); ax[2].legend(loc="upper right")
    ax[3].plot(t, 100 * np.array(b.log["true_soc"]), color=BLUE, label="true (plant)")
    ax[3].plot(t, 100 * np.array(b.log["est_soc"]), color=ORANGE, lw=1.2, ls="--", label="EKF estimate")
    ax[3].set_ylabel("%"); ax[3].set_title("State of charge"); ax[3].legend(loc="lower right")
    ax[3].set_xlabel("minutes")
    fig.suptitle(f"DC fast charge 10 -> 80 % in {t[-1]:.1f} min (SIL)", color=INK, fontweight="bold")
    fig.tight_layout()
    fig.savefig(OUT / "dcfc.png", dpi=140)
    return t[-1]


def tcs():
    fig, ax = plt.subplots(1, 2, figsize=(8, 3.2))
    for on, col, lab in ((True, BLUE, "TCS on"), (False, ORANGE, "TCS off")):
        b = Bench(soc=0.7, mu=0.3)
        b.vcu.inp.tcs_enable = int(on)
        b.power_up()
        b.set_pedal(100)
        for _ in range(300):
            b.step(10, record=("slip_mean", "vx"))
        t = np.array(b.log["t"]) - b.log["t"][0]
        ax[0].plot(t, b.log["slip_mean"], color=col, label=lab)
        ax[1].plot(t, b.log["vx"], color=col, label=lab)
    ax[0].set_yscale("symlog", linthresh=0.5); ax[0].set_title("Mean wheel slip"); ax[0].set_xlabel("s")
    ax[1].set_title("Vehicle speed [m/s]"); ax[1].set_xlabel("s"); ax[1].legend()
    fig.suptitle("Full-pedal launch on ice-like surface (mu = 0.3)", color=INK, fontweight="bold")
    fig.tight_layout()
    fig.savefig(OUT / "tcs.png", dpi=140)


def tv():
    fig, ax = plt.subplots(figsize=(7, 3.2))
    ref = None
    for on, col, lab in ((True, BLUE, "TV on"), (False, ORANGE, "TV off")):
        b = Bench(soc=0.7, vx0=20.0)
        b.vcu.inp.tv_enable = int(on)
        b.power_up()
        b.set_pedal(30)
        b.run(1.0)
        b.vcu.inp.steer_rad = 0.04
        for _ in range(300):
            b.step(10, record=("yaw", "yaw_ref"))
        t = np.array(b.log["t"]) - b.log["t"][0]
        ax.plot(t, b.log["yaw"], color=col, label=lab)
        ref = b.log["yaw_ref"]
    ax.plot(t, ref, color=MUTED, lw=1.2, ls=":", label="reference")
    ax.set_title("Yaw rate after 0.04 rad step steer at 72 km/h"); ax.set_ylabel("rad/s"); ax.set_xlabel("s")
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUT / "tv.png", dpi=140)


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    mins = dcfc()
    tcs()
    tv()
    print(f"figures written to {OUT} (DCFC {mins:.1f} min)")

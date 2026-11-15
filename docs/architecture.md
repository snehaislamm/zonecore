# Architecture

## Context

```
                 ┌──────────────────────────── zonecore VCU (C99) ─────────────────────────────┐
 AFE: 96 cell V  │  ┌────────┐   SOC, limits   ┌────────────┐  T_total  ┌───────────┐  4×T    │  CAN 0x101
 16 cell T, I, V ├─▶│  BMS   ├────────────────▶│ Propulsion ├──────────▶│ Dynamics  ├─────────┼──▶ 4 inverters
                 │  │EKF·SOP │                 │APPS·BTO·map│           │TCS·TV·v̂  │  E2E    │  (E2E: CRC+ctr)
 APPS1/2, brake ─┼─▶│HV SM   │◀── hv_request ──┤            │           └───────────┘         │
 wheels, IMU,    │  └───┬────┘                 └─────┬──────┘                ▲                │
 steering ───────┼──────┼──────────────────────────────┼───────────────────────┘                │
                 │      │ tmax/tmin            ┌──────▼──────┐  torque sum  ┌──────────────┐   │
 EVSE ◀──CAN────▶│ ┌────▼─────┐                │  Safety     │◀─────────────┤ L2 torque    │   │
 0x300/0x301 E2E │ │ Charging │──hv_request──▶ │ SSM·WdgM·DTC│              │ monitor      │   │
                 │ │ session  │                └─────────────┘              └──────────────┘   │
 coolant T ──────┼▶│ Thermal  │─▶ chiller/heater/pump                                          │
 switches, lux,  │ ┌──────────┐                                                               │
 window I/pos ───┼▶│ Body     │─▶ indicators, stop, headlamps, locks, window motor            │
                 │ └──────────┘                                                               │
                 └─────────────────────────────────────────────────────────────────────────────┘
```

In a zonal E/E architecture these functions would be split between a central
compute node (propulsion, dynamics, charging, safety) and zone controllers
(body, window, lamp drivers, AFE gateway). Here they share one image so the
whole loop can be tested together; module boundaries follow the zonal split so
each `src/<domain>` folder could move to its own ECU with only the
`zc_vcu.c` wiring changing to CAN/Ethernet signals.

## Runtime: static cyclic executive

| Task | Period | Offset | Budget | Contents |
|---|---|---|---|---|
| `bms` | 10 ms | 0 | 150 µs | cell stats, SOC EKF, fault qualification, contactor SM |
| `drive` | 10 ms | 1 | 120 µs | APPS, pedal map, SOP limit, TCS, TV, L2 torque monitor, inverter frame |
| `body` | 10 ms | 2 | 80 µs | charge session, E2E rx timers, lighting, locks, window |
| `slow` | 100 ms | 5 | 400 µs | SOP tables, balancing, thermal PID, WdgM, safe-state, DTCs, BMS frames |

Offsets spread the 10 ms tasks across different 1 ms slots so no tick carries
two heavy tasks. Measured host execution (Release, x86) is single-digit µs per
task; utilisation from budgets is ~4 %, leaving headroom for an MCU that is
~50× slower. `zc_sched_t` records max execution time and budget overruns per
task — the same data you would read from an OS timing-protection hook.

## Design rules

* C99, no heap, no recursion, no VLAs, fixed-width types, single exit style,
  every `switch` has `default`, every `if/else if` chain ends in `else`.
* Compiled with `-Wall -Wextra -Werror -pedantic -Wshadow -Wdouble-promotion
  -Wcast-qual -Wmissing-prototypes -Wfloat-equal -Wswitch-enum`, and
  `clang-tidy` (bugprone, CERT, clang-analyzer) with warnings as errors.
* Calibration constants isolated in `zc_params.h` and module-local tables
  (would be an A2L-described calibration segment on target).
* CAN layer generated from `can/zonecore.dbc` by `tools/dbcgen.py`; CI fails if
  generated files are stale.
* Test hooks (`ZC_TEST_HOOKS`) compiled only into the SIL/unit builds.

## SIL bench

`sim/bench.py` closes the loop at **bus level**: the C library is loaded via
ctypes (ABI size-checked at load), the Python EVSE talks to it only through
DBC-encoded, E2E-protected frames, and the inverters receive torque by decoding
the VCU's own CAN frames. The plants deliberately differ from the controller's
internal models (cell-to-cell spread, temperature-dependent resistance,
Pacejka-style tyres, understeering chassis) so tests catch model-mismatch
problems — e.g. tuning the SOC EKF's current-dependent measurement noise came
directly out of a failing SIL charge run.

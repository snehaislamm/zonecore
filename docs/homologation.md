# Regulation-driven requirements (homologation view)

Several features are implemented to behave the way type-approval regulations
expect, and each has an automated test so compliance evidence regenerates on
every commit. **These are engineering interpretations for a portfolio project;
the current consolidated regulation text is the authority** — thresholds below
should be re-checked against the applicable series of amendments before being
quoted as compliance.

| Feature | Regulation (as modelled) | Behaviour implemented | Evidence |
|---|---|---|---|
| Direction-indicator flash rate | UN R48 / FMVSS 108 | ~90 flashes/min (window 60–120), ~50 % duty, first flash on activation | `test_indicator_frequency_duty` |
| Hazard warning priority | UN R48 | Hazard overrides direction indication | `test_hazard_priority` |
| Stop lamps under regenerative braking | UN R13-H (regen decel thresholds) | Lit above 1.3 m/s², never lit below 0.7 m/s², hysteresis between | `test_regen_stop_lamp_thresholds`, `test_one_pedal_regen_and_stop_lamp` |
| Emergency stop signal | UN R48 | ≥ 6 m/s² above 50 km/h → stop lamps flash at 4 Hz; auto hazards at standstill | `test_ess_and_auto_hazard`, `test_emergency_stop_signal_and_auto_hazard` |
| Automatic headlamp switching | UN R48 | ON within 2 s below 1000 lux; OFF only after sustained > 7000 lux | `test_auto_headlamp`, `test_auto_headlamps_tunnel_entry` |
| Power-window anti-pinch | FMVSS 118 / UN R21 | Reverse before 100 N on an obstruction in the 4–200 mm zone | `test_window_anti_pinch` |
| Post-crash HV safety | UN R94/R95 HV provisions, UN R100 | Crash signal → contactors open, doors unlock, hazards on | `test_crash_isolates_hv_unlocks_and_hazards` |
| REESS protection | UN R100 (REESS over-charge / over-temp / short-circuit) | Cell OV/UV/OT two-tier protection, isolation on critical | `test_cell_overvoltage_critical_opens_contactors`, unit BMS suite |
| DC charging conformance | IEC 61851-23 / DIN SPEC 70121 / ISO 15118-2 (abstracted) | Cable check → precharge voltage match → current demand → controlled stop | `test_charging.py` |

## How a homologation engineer would use this

1. Each row maps to a requirement ID in `requirements/requirements.csv`.
2. `tools/trace.py` produces the evidence index (`docs/traceability.md`).
3. SIL results are the "virtual test report"; the physical test (lab lamp
   timing, FMVSS 118 force rig, R100 abuse tests) would then confirm the same
   numbers on hardware.

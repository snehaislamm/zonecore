# How zonecore maps to the Vehicle Controls internship streams

The posting is a general application matched to one of 17 teams. This table is
the "pick the team, point at the code" index — use it in the application and
in interviews.

| Team / role | Where it lives in zonecore | Talking point |
|---|---|---|
| **Engineering Intern – Electrical Systems** | `src/bms/zc_bms.c` contactor/precharge SM, `sim/plant.py` DC-link RC model | Sized precharge (50 Ω · 1 mF, τ = 50 ms), 95 % threshold, weld detection at precharge start, open-under-load counting |
| **SWE – Charging** | `src/charge/zc_charge.c`, `can/zonecore.dbc` EVSE msgs | DIN 70121/ISO 15118-style session: cable check → precharge V-match → CC/CV → controlled stop; 10→80 % in ~28 min SIL |
| **SWE – Controls Test and Automation** | `tests/sil/*`, `sim/bench.py` | Closed-loop SIL with fault injection; 32 scenario tests in ~20 s on every commit |
| **SWE – Embedded Software** | `src/platform/zc_sched.c`, all `src/` | Rate-monotonic cyclic executive with WCET/overrun tracking; no heap; `-Werror` + clang-tidy clean |
| **SWE – Embedded Software Programming** | `src/com/zc_e2e.c`, generated `zc_can_db.c` | Bit-level Intel signal packing, two's-complement sign extension, CRC-8/SAE-J1850 (verified with the catalogue check value 0x4B) |
| **SWE – Energy Storage and Distribution** | `src/bms/zc_bms.c` | 2-state EKF SOC (OCV + 1RC Thevenin), state-of-power 2-D tables, passive balancing, two-tier OV/UV/OT protection |
| **SWE – Functional Safety** | `docs/safety/safety_concept.md`, `src/safety/` | HARA → safety goals → FSRs; E-Gas L2 torque monitor, alive supervision, latching safe-state manager, UDS DTCs, FTTI-timed fault-injection tests |
| **SWE – High Voltage Charging** | charging + BMS + thermal interplay | HV isolation sequencing, DC contactor gating on |V_EVSE − V_pack| < 20 V, abort on E2E timeout/E-stop/unplug within budget |
| **SWE – Mechanisms** | `zc_win_*` in `src/body/zc_body.c`, `WindowPlant` | Current-based anti-pinch with baseline learning; < 100 N in the 4–200 mm zone, reversal; one-touch close |
| **SWE – Systems Architecture** | `docs/architecture.md`, `zc_vcu.c` | Zonal partitioning rationale, task/offset design, signal ownership, CAN matrix |
| **SWE – Systems Engineering, Propulsion** | `src/propulsion/`, `requirements.csv` | Requirements written first (46, ASIL-tagged), pedal map + SOP torque envelope + jerk limiting + BTO |
| **SWE – Test Automation** | `tools/trace.py`, `.github/workflows/ci.yml` | CI gates: build `-Werror`, unit, SIL, DBC-staleness check, traceability `--strict` |
| **SWE – Test Automation and Lighting** | lighting in `src/body/zc_body.c`, `test_body.*` | Indicator timing, comfort flash, R13-H regen stop-lamp thresholds, ESS 4 Hz, auto headlamps — all auto-verified |
| **SWE – Thermal Controls** | `src/thermal/zc_thermal.c` | Mode arbitration with hysteresis, separate DCFC setpoints, PID + back-calculation anti-windup, cold-soak preconditioning |
| **SWE – Unit Testing and Automation** | `tests/unit/` (47 tests, 255 checks) | Dependency-free xUnit harness; boundary-value tests on every threshold; requirement tags |
| **SWE – Vehicle Dynamics Controls** | `src/dynamics/zc_dynamics.c` | Speed estimation under all-wheel spin, per-wheel slip PI, single-track yaw reference, direct yaw-moment TV (−82 % RMS yaw error) |
| **Systems Engineering Intern – Homologation** | `docs/homologation.md` | Regulation → requirement → automated evidence mapping (R48, R13-H, FMVSS 118, R100) |

## Qualifications coverage

| Posting asks for | Evidence |
|---|---|
| C or C++ proficiency | ~3.1 k lines of MISRA-flavoured C99 |
| Python exposure | ~1.5 k lines: plants, SIL bench, DBC codegen, traceability tool |
| Embedded systems / control theory / RTOS | EKF, PID w/ anti-windup, slip & yaw control, cyclic executive, WCET tracking |
| CAN / LIN / Automotive Ethernet | Hand-written DBC, generated codec, E2E profile, bus-level SIL |
| MATLAB / Simulink | Plants written in Python/NumPy; same structure as a Simulink plant model (a port is the obvious next step) |
| ISO 26262 | HARA, safety goals, ASIL-tagged requirements, FTTI-verified mechanisms |
| SIL / HIL / bench testing | SIL done; `zc_vcu_step` + CAN API is the seam a HIL rig would replace |

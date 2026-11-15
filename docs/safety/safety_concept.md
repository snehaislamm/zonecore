# Functional safety concept (ISO 26262, concept phase — portfolio scope)

> This is a learning-grade work product that follows the *structure* of ISO 26262-3
> (item definition → HARA → safety goals → functional safety concept). Ratings
> are the author's engineering judgement for a hypothetical vehicle, not a
> certified assessment.

## 1. Item definition

**Item:** zonecore VCU — propulsion torque arbitration for a quad-motor BEV,
HV battery management (contactors, cell protection), DC fast-charge session
control, battery thermal control, exterior lighting and power-window control.

**Boundaries:** inputs from AFE (cell V/T), current sensor, dual-track APPS,
brake switch, wheel speeds, IMU, steering angle, EVSE over CAN. Outputs to four
inverters (CAN, E2E-protected), contactor drivers, chiller/heater/pump,
lamp drivers, window motor. Friction brakes, steering and the inverters' own
monitoring are outside the item.

## 2. Hazard analysis and risk assessment (excerpt)

S = severity, E = exposure, C = controllability (ISO 26262-3 Table 4 classes).

| ID | Malfunction | Operational situation | Hazard | S | E | C | ASIL |
|---|---|---|---|---|---|---|---|
| H1 | Unintended positive torque | Urban driving, pedestrians near | Unintended acceleration, collision | S3 | E4 | C3 | **D** |
| H2 | Unintended torque from stuck/drifted pedal sensor | Any driving | Unintended acceleration | S3 | E4 | C2 | **C** |
| H3 | Cell overcharge not stopped | DC fast charging | Thermal runaway / fire, occupants nearby | S3 | E3 | C3 | **C** |
| H4 | Contactors closed into an uncharged DC link | Key-on | Contactor weld, later loss of HV isolation | S2 | E4 | C3 | **B** |
| H5 | Charge current continues after EVSE comm loss | DC fast charging | Overcurrent at inlet / cable heating | S2 | E3 | C3 | **B** |
| H6 | Excessive wheel slip on low µ | Launch on snow/ice | Loss of lateral stability | S2 | E3 | C2 | **B** |
| H7 | Stop lamps not lit during strong regen | Highway, following traffic | Rear-end collision | S2 | E4 | C1 | **A** |
| H8 | Control task hangs, outputs frozen | Any driving | Frozen (possibly high) torque | S3 | E4 | C2 | **C** |

## 3. Safety goals

| SG | Safety goal | ASIL | Safe state | FTTI |
|---|---|---|---|---|
| SG1 | Prevent propulsion torque exceeding driver demand | D | Zero torque (`SAFE_STOP`) | 100 ms |
| SG2 | Prevent torque from implausible pedal input | C | Zero propulsion torque, regen allowed | 200 ms |
| SG3 | Prevent cell voltage/temperature beyond safe limits | C | HV isolated (`HV_OFF`) | 100 ms |
| SG4 | Prevent contactor closure without precharge | B | All contactors open | 1 s |
| SG5 | Prevent uncontrolled DC charge current | B | Zero request, inlet isolated | 300 ms |
| SG6 | Detect loss of control-software liveness | C | `SAFE_STOP` | 300 ms |

## 4. Functional safety concept → implementation → verification

| FSR | Mechanism | Code | Requirement / tests |
|---|---|---|---|
| SG1 | **E-Gas-style level-2 torque monitor**: diverse recomputation of the admissible torque from the raw pedal; Σ wheel commands > envelope + 10 % for 50 ms ⇒ immediate torque cut, latched | `src/safety/zc_safety.c` `zc_tqmon_*`, applied in `zc_vcu.c` `task_drive` | REQ-SAF-002 |
| SG1 | E2E protection of the inverter command (CRC-8 J1850 + alive counter + Data ID) | `src/com/zc_e2e.c` | REQ-COM-001..003 |
| SG2 | Dual-track APPS plausibility (Δ > 10 %, range) debounced 100 ms, latched; brake-throttle override | `src/propulsion/zc_propulsion.c` | REQ-PROP-001/002 |
| SG3 | Two-tier cell limits: SEVERE (controlled ramp-down, then open) and CRITICAL (open immediately); sensor plausibility | `src/bms/zc_bms.c` `monitor()`, `hv_sm()` | REQ-BMS-002..005 |
| SG4 | Precharge supervision (95 % / 1 s) + weld diagnosis at precharge start | `src/bms/zc_bms.c` `hv_sm()` | REQ-BMS-006/007 |
| SG5 | Charge-session abort on E2E timeout, E-stop, unplug, BMS fault; CV loop + BMS voltage-window derate as independent layers | `src/charge/zc_charge.c` | REQ-CHG-002..005 |
| SG6 | Alive supervision of every periodic task (watchdog-manager pattern), tolerance one cycle | `src/safety/zc_safety.c` `zc_wdgm_*` | REQ-SAF-001 |
| all | Latching safe-state manager (no de-escalation within the drive cycle); DTCs with UDS status byte + freeze frame | `zc_ssm_arbitrate`, `zc_dtc_*` | REQ-SAF-003/004 |

## 5. Verification approach

* **Unit level** (`tests/unit`, host-compiled with `-Werror -pedantic` and a
  MISRA-flavoured warning set): every safety mechanism exercised at its
  threshold and one step either side (boundary-value analysis).
* **SIL level** (`tests/sil`): fault injection on the closed-loop system —
  APPS drift, corrupted torque command, task stall, cell over-voltage, open
  sense wire, over-temperature, precharge failure, corrupted/lost EVSE frames,
  crash signal — each asserting *detection*, *reaction* and *timing vs FTTI*.
* **Traceability**: `tools/trace.py --strict` fails CI if any requirement has
  no verifying test. See `docs/traceability.md`.

## 6. Known gaps (deliberate, would be next steps)

* No hardware metrics (SPFM/LFM/PMHF) — needs an HW architecture.
* Level-3 monitoring (controller question/answer with an independent MCU) not
  modelled; level-2 runs on the same core.
* No freedom-from-interference argument (memory partitioning, MPU) — host build.
* Inverter-side reaction to torque-command timeout is out of item scope.

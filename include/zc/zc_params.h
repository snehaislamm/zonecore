/**
 * @file zc_params.h
 * @brief Calibration constants for the reference vehicle: a ~3.2 t quad-motor
 *        SUV with a 96s 150 Ah NMC pack (~53 kWh, ~350 V nominal).
 *        Placeholder values chosen to be physically plausible; in a production
 *        program these live in a calibration (A2L) segment, not in code.
 */
#ifndef ZC_PARAMS_H
#define ZC_PARAMS_H

/* ---- Pack ---- */
#define ZC_BMS_N_CELLS        96U
#define ZC_BMS_N_TEMPS        16U
#define ZC_CELL_CAP_AH        150.0f
#define ZC_CELL_R0_OHM        0.0008f
#define ZC_CELL_R1_OHM        0.0005f
#define ZC_CELL_C1_F          40000.0f

#define ZC_CELL_V_MAX         4.20f
#define ZC_CELL_OV_WARN_V     4.22f
#define ZC_CELL_OV_CRIT_V     4.30f
#define ZC_CELL_UV_WARN_V     2.80f
#define ZC_CELL_UV_CRIT_V     2.50f
#define ZC_CELL_OT_C          60.0f
#define ZC_CELL_OT_CRIT_C     70.0f
#define ZC_CELL_SENSOR_MIN_V  0.5f
#define ZC_CELL_SENSOR_MAX_V  5.0f
#define ZC_TEMP_SENSOR_MIN_C  (-45.0f)
#define ZC_TEMP_SENSOR_MAX_C  125.0f

#define ZC_PRECHARGE_RATIO    0.95f
#define ZC_PRECHARGE_TMO_MS   1000U
#define ZC_OPEN_CURRENT_A     5.0f
#define ZC_OPEN_TMO_MS        500U

/* ---- Charging (DC) ---- */
#define ZC_CHG_CV_CELL_V      4.17f
#define ZC_CHG_TAPER_CUTOFF_A 7.5f      /* 0.05 C */
#define ZC_CHG_VMATCH_V       20.0f
#define ZC_CHG_RAMP_UP_APS    50.0f
#define ZC_CHG_RAMP_DN_APS    400.0f

/* ---- Driveline ---- */
#define ZC_N_WHEELS           4U        /* FL, FR, RL, RR */
#define ZC_MOTOR_T_MAX_NM     250.0f
#define ZC_GEAR_RATIO         9.0f
#define ZC_WHEEL_R_M          0.38f
#define ZC_TRACK_M            1.70f
#define ZC_WHEELBASE_M        3.08f
#define ZC_VEH_MASS_KG        3200.0f
#define ZC_FRONT_SPLIT        0.40f
#define ZC_TCS_SLIP_TARGET    0.10f
#define ZC_APPS_DELTA_PCT     10.0f
#define ZC_APPS_FAULT_MS      100U
#define ZC_BTO_ENTER_PCT      25.0f
#define ZC_BTO_EXIT_PCT       5.0f

#define ZC_GRAVITY            9.81f

#endif

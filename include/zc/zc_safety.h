/**
 * @file zc_safety.h
 * @brief Functional-safety mechanisms (ISO 26262 concept level):
 *   - degradation / safe-state manager
 *   - E-Gas-style level-2 torque monitor (independent plausibility of the
 *     commanded torque against the driver's pedal)
 *   - alive supervision of the periodic tasks (watchdog-manager pattern)
 *   - diagnostic trouble code store with UDS (ISO 14229) status bits
 */
#ifndef ZC_SAFETY_H
#define ZC_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

#include "zc/zc_util.h"

typedef enum {
    ZC_SAFE_NORMAL = 0,
    ZC_SAFE_DERATED = 1,   /**< limp home: torque capped */
    ZC_SAFE_STOP = 2,      /**< no propulsion torque, regen/steer OK */
    ZC_SAFE_HV_OFF = 3     /**< HV isolated */
} zc_safe_state_t;

/* ---------------- Torque monitor ---------------- */
typedef struct {
    zc_debounce_t db;
    bool          fault;
    float         allowed;
    float         margin_nm;
} zc_tqmon_t;

void zc_tqmon_init(zc_tqmon_t *m);
/** pedal_min_pct: independently read min(APPS1, APPS2). cmd_sum: sum of
 *  per-wheel commands actually sent to the inverters. */
bool zc_tqmon_step_10ms(zc_tqmon_t *m, float pedal_min_pct, float speed_mps, float cmd_sum_nm);

/* ---------------- Alive supervision ---------------- */
#define ZC_WDGM_MAX_SE 4U
typedef struct {
    uint16_t count[ZC_WDGM_MAX_SE];
    uint16_t exp_min[ZC_WDGM_MAX_SE];
    uint16_t exp_max[ZC_WDGM_MAX_SE];
    uint8_t  fail_cycles[ZC_WDGM_MAX_SE];
    uint8_t  n;
    bool     fault;
} zc_wdgm_t;

void zc_wdgm_init(zc_wdgm_t *w);
uint8_t zc_wdgm_add(zc_wdgm_t *w, uint16_t exp_min, uint16_t exp_max);
void zc_wdgm_checkpoint(zc_wdgm_t *w, uint8_t se);
/** Call once per supervision cycle (100 ms). */
void zc_wdgm_main(zc_wdgm_t *w);

/* ---------------- DTC store ---------------- */
#define ZC_DTC_ST_TEST_FAILED      0x01U
#define ZC_DTC_ST_FAILED_THIS_OC   0x02U
#define ZC_DTC_ST_PENDING          0x04U
#define ZC_DTC_ST_CONFIRMED        0x08U
#define ZC_DTC_ST_FAILED_SINCE_CLR 0x20U

typedef enum {
    ZC_DTC_CELL_OV = 0,
    ZC_DTC_CELL_UV,
    ZC_DTC_CELL_OT,
    ZC_DTC_CELL_VSENSE,
    ZC_DTC_CELL_TSENSE,
    ZC_DTC_PRECHARGE,
    ZC_DTC_CONTACTOR_WELD,
    ZC_DTC_APPS_PLAUS,
    ZC_DTC_TORQUE_MON,
    ZC_DTC_ALIVE_SUP,
    ZC_DTC_EVSE_COMM,
    ZC_DTC_CHG_ABORT,
    ZC_DTC_COUNT
} zc_dtc_id_t;

typedef struct {
    uint32_t code;            /**< 3-byte UDS DTC */
    uint8_t  status;
    uint16_t occurrences;
    uint32_t first_fail_ms;
    float    snap_soc;
    float    snap_speed;
} zc_dtc_entry_t;

typedef struct {
    zc_dtc_entry_t e[ZC_DTC_COUNT];
} zc_dtc_t;

void zc_dtc_init(zc_dtc_t *d);
void zc_dtc_report(zc_dtc_t *d, zc_dtc_id_t id, bool failed, uint32_t now_ms, float soc, float speed);
void zc_dtc_clear_all(zc_dtc_t *d);
uint16_t zc_dtc_confirmed_count(const zc_dtc_t *d);

/* ---------------- Safe-state manager ---------------- */
typedef struct {
    uint8_t bms_level;
    bool    apps_fault;
    bool    tqmon_fault;
    bool    wdgm_fault;
    bool    crash;
    bool    thermal_derate;  /**< pack approaching OT: limp home */
} zc_ssm_in_t;

/** Latching max-of-requests arbitration. */
zc_safe_state_t zc_ssm_arbitrate(zc_safe_state_t current, const zc_ssm_in_t *in);

#endif

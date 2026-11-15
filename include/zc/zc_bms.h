/**
 * @file zc_bms.h
 * @brief Battery management: EKF state-of-charge, cell monitoring with fault
 *        qualification, state-of-power current limits, passive balancing and
 *        the HV contactor / precharge state machine.
 */
#ifndef ZC_BMS_H
#define ZC_BMS_H

#include <stdbool.h>
#include <stdint.h>

#include "zc/zc_params.h"
#include "zc/zc_util.h"

typedef enum {
    ZC_HV_OPEN = 0,
    ZC_HV_PRECHARGE,
    ZC_HV_CLOSED,
    ZC_HV_OPENING,
    ZC_HV_FAULT
} zc_hv_state_t;

typedef enum {
    ZC_BMS_FLT_NONE = 0,     /**< normal */
    ZC_BMS_FLT_WARN = 1,     /**< derate */
    ZC_BMS_FLT_SEVERE = 2,   /**< controlled shutdown: ramp current, then open */
    ZC_BMS_FLT_CRITICAL = 3  /**< open contactors immediately */
} zc_bms_flt_t;

/* individual fault bits */
#define ZC_BMS_F_OV        (1UL << 0)
#define ZC_BMS_F_UV        (1UL << 1)
#define ZC_BMS_F_OT        (1UL << 2)
#define ZC_BMS_F_OV_CRIT   (1UL << 3)
#define ZC_BMS_F_UV_CRIT   (1UL << 4)
#define ZC_BMS_F_OT_CRIT   (1UL << 5)
#define ZC_BMS_F_VSENSE    (1UL << 6)
#define ZC_BMS_F_TSENSE    (1UL << 7)
#define ZC_BMS_F_PRECHARGE (1UL << 8)
#define ZC_BMS_F_WELD      (1UL << 9)
#define ZC_BMS_F_UT_CHG    (1UL << 10)

typedef struct {
    float cell_v[ZC_BMS_N_CELLS];
    float cell_t[ZC_BMS_N_TEMPS];
    float pack_i;   /**< [A] positive = discharge */
    float pack_v;   /**< [V] pack side of contactors */
    float bus_v;    /**< [V] DC-link side of contactors */
} zc_bms_meas_t;

typedef struct {
    bool neg;
    bool pos;
    bool pre;
} zc_contactors_t;

typedef struct {
    /* --- SOC estimation --- */
    float soc;         /**< EKF estimate [0..1] */
    float vrc;         /**< RC-branch polarisation [V] */
    float P[2][2];     /**< EKF covariance */
    float soc_cc;      /**< pure coulomb count, for comparison */
    float ekf_innov;   /**< last innovation [V] */

    /* --- cell statistics --- */
    float vmax, vmin, vavg, tmax, tmin;
    uint8_t vmax_idx, vmin_idx;

    /* --- fault qualification --- */
    zc_debounce_t db_ov, db_uv, db_ot, db_ovc, db_uvc, db_otc, db_vs, db_ts, db_utc;
    uint32_t faults;           /**< ZC_BMS_F_* bitmask, latched */
    zc_bms_flt_t level;

    /* --- state of power --- */
    float chg_i_lim;           /**< [A] */
    float dch_i_lim;           /**< [A] */

    /* --- balancing --- */
    uint8_t bal_mask[(ZC_BMS_N_CELLS + 7U) / 8U];
    uint8_t bal_count;

    /* --- HV state machine --- */
    zc_hv_state_t hv;
    uint32_t hv_timer_ms;
    zc_contactors_t k;
    uint32_t open_under_load;
} zc_bms_t;

void zc_bms_init(zc_bms_t *b, float soc_init, float soc_init_var);
/** 10 ms: SOC EKF, monitoring, HV state machine. */
void zc_bms_step_10ms(zc_bms_t *b, const zc_bms_meas_t *m, bool hv_request, bool charging);
/** 100 ms: state-of-power limits and balancing decisions. */
void zc_bms_step_100ms(zc_bms_t *b, const zc_bms_meas_t *m, bool charging);
bool zc_bms_cell_balancing(const zc_bms_t *b, uint8_t cell);

/* exposed for unit tests */
float zc_bms_ocv(float soc);
float zc_bms_docv_dsoc(float soc);

#endif

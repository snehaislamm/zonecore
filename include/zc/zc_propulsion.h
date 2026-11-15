/**
 * @file zc_propulsion.h
 * @brief Driver torque interpretation: redundant accelerator pedal (APPS)
 *        plausibility, brake-throttle override, one-pedal pedal map with
 *        regen, battery state-of-power torque limiting, degradation-mode
 *        caps and jerk limiting. Output is total motor torque demand.
 */
#ifndef ZC_PROPULSION_H
#define ZC_PROPULSION_H

#include <stdbool.h>
#include <stdint.h>

#include "zc/zc_util.h"

typedef struct {
    float   apps1_pct;
    float   apps2_pct;
    bool    brake;
    float   veh_speed_mps;
    float   motor_speed_avg_rads;
    float   pack_v;
    float   dch_i_lim;
    float   chg_i_lim;
    uint8_t safe_state;   /**< zc_safe_state_t */
    bool    hv_ready;
} zc_prop_in_t;

typedef struct {
    zc_debounce_t db_apps;
    bool          apps_fault;   /**< latched */
    bool          bto_active;
    float         pedal_pct;
    zc_ratelim_t  jerk;
    float         t_driver;     /**< unlimited driver demand, total motor Nm */
    float         t_max_dch;    /**< power-limited, total motor Nm */
    float         t_max_chg;
    float         t_demand;     /**< final total motor torque demand */
} zc_prop_t;

void  zc_prop_init(zc_prop_t *p);
void  zc_prop_step_10ms(zc_prop_t *p, const zc_prop_in_t *in);
/** Pedal map: total motor torque [Nm] for pedal % and speed (no limits). */
float zc_prop_pedal_map(float pedal_pct, float speed_mps);

#endif

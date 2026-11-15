/**
 * @file zc_thermal.h
 * @brief Battery thermal management: mode arbitration (cool / heat / off)
 *        with hysteresis and separate setpoints for driving and DC fast
 *        charging (preconditioning), PID control of chiller and PTC heater
 *        power with anti-windup, and coolant pump scheduling.
 */
#ifndef ZC_THERMAL_H
#define ZC_THERMAL_H

#include <stdbool.h>
#include <stdint.h>

#include "zc/zc_util.h"

typedef enum { ZC_TH_OFF = 0, ZC_TH_COOL, ZC_TH_HEAT } zc_th_mode_t;

typedef struct {
    float tmax;
    float tmin;
    float coolant_t;
    bool  fast_charging;   /**< DC session active or about to start */
    bool  hv_available;
} zc_th_in_t;

typedef struct {
    zc_th_mode_t mode;
    zc_pid_t     pid_cool;
    zc_pid_t     pid_heat;
    float        chiller_w;
    float        heater_w;
    float        pump;     /**< 0..1 */
} zc_thermal_t;

#define ZC_TH_CHILLER_MAX_W 8000.0f
#define ZC_TH_HEATER_MAX_W  6000.0f

void zc_thermal_init(zc_thermal_t *t);
void zc_thermal_step_100ms(zc_thermal_t *t, const zc_th_in_t *in);

#endif

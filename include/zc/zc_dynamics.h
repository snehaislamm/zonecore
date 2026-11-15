/**
 * @file zc_dynamics.h
 * @brief Quad-motor vehicle dynamics control: vehicle speed estimation,
 *        per-wheel traction control (slip PI), yaw-rate reference from a
 *        linear single-track model, and torque vectoring (direct yaw
 *        moment control by left/right torque differential).
 *        Wheel order: 0=FL 1=FR 2=RL 3=RR.
 */
#ifndef ZC_DYNAMICS_H
#define ZC_DYNAMICS_H

#include <stdbool.h>
#include <stdint.h>

#include "zc/zc_params.h"
#include "zc/zc_util.h"

typedef struct {
    float t_total;                 /**< total motor torque demand [Nm] */
    float wheel_w[ZC_N_WHEELS];    /**< wheel speeds [rad/s] */
    float steer_rad;               /**< road-wheel angle */
    float yaw_rate;                /**< [rad/s] */
    float ax;                      /**< longitudinal accel [m/s^2] */
    bool  tcs_enable;
    bool  tv_enable;
} zc_dyn_in_t;

typedef struct {
    float    v_est;
    bool     primed;
    bool     spin;     /**< wheel speed above inertial prediction under drive torque */
    float    yaw_ref;
    float    mz_cmd;
    float    slip[ZC_N_WHEELS];
    float    t_cmd[ZC_N_WHEELS];   /**< per-motor torque [Nm] */
    bool     tcs_active[ZC_N_WHEELS];
    zc_pid_t tcs[ZC_N_WHEELS];
    zc_pid_t yaw;
    zc_lpf_t yaw_ref_lpf;
} zc_dyn_t;

void  zc_dyn_init(zc_dyn_t *d);
void  zc_dyn_step_10ms(zc_dyn_t *d, const zc_dyn_in_t *in);
float zc_dyn_yaw_reference(float v_mps, float steer_rad);

#endif

#include "zc/zc_dynamics.h"

#include <string.h>

#define DT_S         0.01f
#define K_US         0.0015f   /* [rad/(m/s^2)]-ish understeer gradient of reference model */
#define MU_REF       0.9f
#define V_SLIP_MIN   2.0f      /* [m/s] slip denominator floor near standstill */
#define V_TV_MIN     3.0f
#define MZ_MAX       6000.0f   /* [Nm] yaw moment authority */
#define K_SPEED_CORR 0.05f     /* complementary filter wheel-correction gain per step */

void zc_dyn_init(zc_dyn_t *d)
{
    (void)memset(d, 0, sizeof(*d));
    for (uint8_t i = 0U; i < ZC_N_WHEELS; i++) {
        /* output = torque reduction, negative; err = target - slip */
        zc_pid_init(&d->tcs[i], 1200.0f, 6000.0f, 0.0f, -ZC_MOTOR_T_MAX_NM, 0.0f);
    }
    zc_pid_init(&d->yaw, 40000.0f, 120000.0f, 0.0f, -MZ_MAX, MZ_MAX);
    zc_lpf_init(&d->yaw_ref_lpf, 2.5f, DT_S); /* desired yaw response bandwidth */
}

float zc_dyn_yaw_reference(float v_mps, float steer_rad)
{
    const float v = zc_maxf(v_mps, 0.0f);
    float r = (v * steer_rad) / (ZC_WHEELBASE_M + (K_US * v * v));
    /* friction-limited: |r| <= mu*g/v */
    if (v > 1.0f) {
        const float rmax = (MU_REF * ZC_GRAVITY) / v;
        r = zc_clampf(r, -rmax, rmax);
    }
    return r;
}

static void speed_estimate(zc_dyn_t *d, const zc_dyn_in_t *in)
{
    /* In drive, the slowest wheel is the best ground-speed proxy; in
     * braking, the fastest. Blend with integrated accelerometer so that
     * all-wheel spin (AWD launch on ice) does not pull the estimate up. */
    float vmin = 1.0e9f;
    float vmax = -1.0e9f;
    for (uint8_t i = 0U; i < ZC_N_WHEELS; i++) {
        const float v = in->wheel_w[i] * ZC_WHEEL_R_M;
        vmin = zc_minf(vmin, v);
        vmax = zc_maxf(vmax, v);
    }
    const float v_ref = (in->t_total >= 0.0f) ? vmin : vmax;
    if (!d->primed) {
        d->v_est = v_ref;
        d->primed = true;
    }
    const float v_pred = d->v_est + (in->ax * DT_S);
    /* Under drive torque, wheels reading faster than the inertial prediction
     * are probably spinning: correct upward only very slowly. When coasting
     * (no torque) the wheels free-roll and are trusted fully, which also
     * bounds long-term accelerometer drift. */
    float k = K_SPEED_CORR;
    d->spin = (v_ref > v_pred) && (in->t_total > 50.0f);
    if (d->spin) {
        k *= 0.02f;
    }
    d->v_est = zc_maxf(v_pred + (k * (v_ref - v_pred)), 0.0f);
}

void zc_dyn_step_10ms(zc_dyn_t *d, const zc_dyn_in_t *in)
{
    speed_estimate(d, in);

    /* ---- base split ---- */
    const float tf = 0.5f * ZC_FRONT_SPLIT * in->t_total;
    const float tr = 0.5f * (1.0f - ZC_FRONT_SPLIT) * in->t_total;
    float t[ZC_N_WHEELS] = {tf, tf, tr, tr};

    /* ---- torque vectoring (direct yaw moment control) ---- */
    d->yaw_ref = zc_lpf_step(&d->yaw_ref_lpf, zc_dyn_yaw_reference(d->v_est, in->steer_rad));
    if (in->tv_enable && (d->v_est > V_TV_MIN)) {
        d->mz_cmd = zc_pid_step(&d->yaw, d->yaw_ref, in->yaw_rate, DT_S);
    } else {
        zc_pid_reset(&d->yaw);
        d->mz_cmd = 0.0f;
    }
    /* Mz = 2 axles * 2 wheels * dF * track/2  ->  dF = Mz / (2*track) */
    const float dt_motor = ((d->mz_cmd / (2.0f * ZC_TRACK_M)) * ZC_WHEEL_R_M) / ZC_GEAR_RATIO;
    t[0] -= dt_motor;
    t[2] -= dt_motor;
    t[1] += dt_motor;
    t[3] += dt_motor;

    /* ---- per-wheel traction control ---- */
    const float vden = zc_maxf(d->v_est, V_SLIP_MIN);
    for (uint8_t i = 0U; i < ZC_N_WHEELS; i++) {
        d->slip[i] = ((in->wheel_w[i] * ZC_WHEEL_R_M) - d->v_est) / vden;
        t[i] = zc_clampf(t[i], -ZC_MOTOR_T_MAX_NM, ZC_MOTOR_T_MAX_NM);
        if (in->tcs_enable && (t[i] > 0.0f)) {
            const float red = zc_pid_step(&d->tcs[i], ZC_TCS_SLIP_TARGET, d->slip[i], DT_S);
            d->tcs_active[i] = red < -1.0f;
            t[i] = zc_maxf(t[i] + red, 0.0f);
        } else {
            zc_pid_reset(&d->tcs[i]);
            d->tcs_active[i] = false;
        }
        d->t_cmd[i] = t[i];
    }
}

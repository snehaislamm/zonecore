#include "zc/zc_propulsion.h"

#include <string.h>

#include "zc/zc_params.h"
#include "zc/zc_safety.h"

#define DT_S         0.01f
#define T_TOTAL_MAX  (ZC_MOTOR_T_MAX_NM * (float)ZC_N_WHEELS)
#define OMEGA_MIN    60.0f     /* [rad/s] motor; avoids P/omega singularity */
#define REGEN_DECEL  1.5f      /* [m/s^2] one-pedal lift-off decel */
#define REGEN_PEDAL  15.0f     /* [%] pedal at which regen fades to zero */
#define ETA_DRIVE    0.92f

void zc_prop_init(zc_prop_t *p)
{
    (void)memset(p, 0, sizeof(*p));
    zc_debounce_init(&p->db_apps, (uint16_t)(ZC_APPS_FAULT_MS / 10U), 0xFFFFU);
    zc_ratelim_init(&p->jerk, 2500.0f, 5000.0f, 0.0f); /* total Nm/s */
}

float zc_prop_pedal_map(float pedal_pct, float speed_mps)
{
    const float pedal = zc_clampf(pedal_pct, 0.0f, 100.0f);
    /* lift-off regen torque (total, motor side) for REGEN_DECEL, faded out
     * below 2 m/s so the vehicle creeps to a stop smoothly */
    const float regen_full = (REGEN_DECEL * ZC_VEH_MASS_KG * ZC_WHEEL_R_M) / ZC_GEAR_RATIO;
    const float fade = zc_clampf((speed_mps - 0.5f) / 1.5f, 0.0f, 1.0f);
    float t;
    if (pedal < REGEN_PEDAL) {
        t = -regen_full * fade * (1.0f - (pedal / REGEN_PEDAL));
    } else {
        const float x = (pedal - REGEN_PEDAL) / (100.0f - REGEN_PEDAL);
        t = T_TOTAL_MAX * x * (0.4f + (0.6f * x)); /* progressive tip-in */
    }
    return t;
}

void zc_prop_step_10ms(zc_prop_t *p, const zc_prop_in_t *in)
{
    /* ---- APPS plausibility (dual-track) ---- */
    const bool range_bad = (in->apps1_pct < -5.0f) || (in->apps1_pct > 105.0f) ||
                           (in->apps2_pct < -5.0f) || (in->apps2_pct > 105.0f);
    const bool delta_bad = zc_absf(in->apps1_pct - in->apps2_pct) > ZC_APPS_DELTA_PCT;
    if (zc_debounce_step(&p->db_apps, range_bad || delta_bad)) {
        p->apps_fault = true;
    }
    p->pedal_pct = zc_clampf(zc_minf(in->apps1_pct, in->apps2_pct), 0.0f, 100.0f);

    /* ---- brake-throttle override ---- */
    if (in->brake && (p->pedal_pct > ZC_BTO_ENTER_PCT)) {
        p->bto_active = true;
    } else if (p->pedal_pct < ZC_BTO_EXIT_PCT) {
        p->bto_active = false;
    } else {
        /* hold */
    }

    float pedal = p->pedal_pct;
    if (p->apps_fault || p->bto_active) {
        pedal = 0.0f; /* fall back to lift-off behaviour (regen only) */
    }
    p->t_driver = zc_prop_pedal_map(pedal, in->veh_speed_mps);

    /* ---- battery state of power -> torque envelope ---- */
    const float omega = zc_maxf(zc_absf(in->motor_speed_avg_rads), OMEGA_MIN);
    const float p_dch = in->dch_i_lim * in->pack_v * ETA_DRIVE;
    const float p_chg = in->chg_i_lim * in->pack_v / ETA_DRIVE;
    p->t_max_dch = zc_minf(T_TOTAL_MAX, p_dch / omega);
    p->t_max_chg = zc_minf(T_TOTAL_MAX, p_chg / omega);

    float t = zc_clampf(p->t_driver, -p->t_max_chg, p->t_max_dch);

    /* ---- degradation modes ---- */
    switch ((zc_safe_state_t)in->safe_state) {
    case ZC_SAFE_DERATED:
        t = zc_minf(t, 0.3f * T_TOTAL_MAX);
        break;
    case ZC_SAFE_STOP:
        t = zc_minf(t, 0.0f); /* no propulsion, regen still allowed */
        break;
    case ZC_SAFE_HV_OFF:
        t = 0.0f;
        break;
    case ZC_SAFE_NORMAL:
    default:
        break;
    }
    if (!in->hv_ready) {
        t = 0.0f;
    }

    /* torque cuts are immediate, increases are jerk limited */
    if (t < p->jerk.y) {
        p->jerk.y = zc_maxf(t, p->jerk.y - (p->jerk.fall_per_s * DT_S));
        if ((t <= 0.0f) && (p->jerk.y > 0.0f) &&
            (p->apps_fault || p->bto_active || (in->safe_state >= (uint8_t)ZC_SAFE_STOP))) {
            p->jerk.y = zc_minf(t, 0.0f); /* safety cut: no ramp */
        }
    } else {
        (void)zc_ratelim_step(&p->jerk, t, DT_S);
    }
    p->t_demand = p->jerk.y;
}

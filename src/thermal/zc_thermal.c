#include "zc/zc_thermal.h"

#include <string.h>

#define DT_S 0.1f

typedef struct {
    float cool_on;    /**< enter COOL when tmax above */
    float cool_sp;    /**< regulate tmax to */
    float heat_on;    /**< enter HEAT when tmin below */
    float heat_sp;    /**< regulate tmin to */
} th_cal_t;

/* Fast charge: run the pack warmer (lower resistance, higher charge C-rate)
 * but cool earlier and harder, since I^2R heat is highest. */
static const th_cal_t k_drive = {35.0f, 30.0f, 5.0f, 12.0f};
static const th_cal_t k_dcfc  = {32.0f, 28.0f, 20.0f, 25.0f};
#define HYST_C 3.0f

void zc_thermal_init(zc_thermal_t *t)
{
    (void)memset(t, 0, sizeof(*t));
    /* output = power [W]; error is (setpoint - meas): for cooling invert sign */
    zc_pid_init(&t->pid_cool, 1500.0f, 60.0f, 0.0f, 0.0f, ZC_TH_CHILLER_MAX_W);
    zc_pid_init(&t->pid_heat, 1200.0f, 40.0f, 0.0f, 0.0f, ZC_TH_HEATER_MAX_W);
    t->mode = ZC_TH_OFF;
}

void zc_thermal_step_100ms(zc_thermal_t *t, const zc_th_in_t *in)
{
    const th_cal_t *cal = in->fast_charging ? &k_dcfc : &k_drive;
    const zc_th_mode_t prev = t->mode;

    if (!in->hv_available) {
        t->mode = ZC_TH_OFF;
    } else {
        switch (t->mode) {
        case ZC_TH_OFF:
            if (in->tmax > cal->cool_on) {
                t->mode = ZC_TH_COOL;
            } else if (in->tmin < cal->heat_on) {
                t->mode = ZC_TH_HEAT;
            } else {
                /* stay off */
            }
            break;
        case ZC_TH_COOL:
            if (in->tmax < (cal->cool_sp - HYST_C)) {
                t->mode = ZC_TH_OFF;
            }
            break;
        case ZC_TH_HEAT:
        default:
            if ((in->tmin > (cal->heat_sp + HYST_C)) || (in->tmax > cal->cool_on)) {
                t->mode = ZC_TH_OFF;
            }
            break;
        }
    }
    if (t->mode != prev) {
        zc_pid_reset(&t->pid_cool);
        zc_pid_reset(&t->pid_heat);
    }

    t->chiller_w = 0.0f;
    t->heater_w = 0.0f;
    t->pump = in->hv_available ? 0.2f : 0.0f; /* base circulation for sensing */
    if (t->mode == ZC_TH_COOL) {
        /* reverse-acting: more power when tmax above setpoint */
        t->chiller_w = zc_pid_step(&t->pid_cool, -cal->cool_sp, -in->tmax, DT_S);
        t->pump = zc_clampf(0.4f + (t->chiller_w / ZC_TH_CHILLER_MAX_W), 0.0f, 1.0f);
    } else if (t->mode == ZC_TH_HEAT) {
        t->heater_w = zc_pid_step(&t->pid_heat, cal->heat_sp, in->tmin, DT_S);
        t->pump = zc_clampf(0.4f + (t->heater_w / ZC_TH_HEATER_MAX_W), 0.0f, 1.0f);
    } else {
        /* off */
    }
}

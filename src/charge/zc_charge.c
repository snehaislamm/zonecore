#include "zc/zc_charge.h"

#include <string.h>

#include "zc/zc_bms.h"
#include "zc/zc_params.h"

#define DT_S              0.01f
#define CABLE_CHECK_TMO   10000U
#define PRECHARGE_TMO     8000U
#define STOP_TMO          2000U
#define V_LIMIT           ((ZC_CELL_V_MAX * (float)ZC_BMS_N_CELLS) + 2.0f)
#define I_ABS_MAX         1000.0f

void zc_chg_init(zc_chg_t *c)
{
    (void)memset(c, 0, sizeof(*c));
    c->state = ZC_CHG_IDLE;
    /* CV loop: current allowance [A] regulated on the hottest-voltage cell. */
    zc_pid_init(&c->cv_pid, 2000.0f, 5000.0f, 0.0f, 0.0f, I_ABS_MAX);
    zc_ratelim_init(&c->i_ramp, ZC_CHG_RAMP_UP_APS, ZC_CHG_RAMP_DN_APS, 0.0f);
}

bool zc_chg_active(const zc_chg_t *c)
{
    return (c->state == ZC_CHG_PRECHARGE) || (c->state == ZC_CHG_CHARGING) ||
           (c->state == ZC_CHG_STOPPING);
}

static void enter(zc_chg_t *c, zc_chg_state_t s)
{
    c->state = s;
    c->timer_ms = 0U;
}

static void fault(zc_chg_t *c, zc_chg_end_t why)
{
    c->end_reason = why;
    enter(c, ZC_CHG_FAULT);
}

/* Abort conditions evaluated every cycle once a session has started. */
static bool check_abort(zc_chg_t *c, const zc_chg_in_t *in)
{
    bool abort = true;
    if (!in->plug_present) {
        fault(c, ZC_CHG_END_UNPLUG);
    } else if (!in->evse_valid) {
        fault(c, ZC_CHG_END_EVSE_TIMEOUT);
    } else if (in->evse_estop) {
        fault(c, ZC_CHG_END_ESTOP);
    } else if (in->evse_state == (uint8_t)ZC_EVSE_FAULT) {
        fault(c, ZC_CHG_END_EVSE_FAULT);
    } else if (in->bms_level >= 2U) {
        fault(c, ZC_CHG_END_BMS_FAULT);
    } else {
        abort = false;
    }
    return abort;
}

void zc_chg_step_10ms(zc_chg_t *c, const zc_chg_in_t *in)
{
    c->timer_ms += 10U;
    float i_target = 0.0f;
    c->v_req = in->pack_v;

    switch (c->state) {
    case ZC_CHG_IDLE:
        c->end_reason = ZC_CHG_END_NONE;
        if (in->plug_present) {
            enter(c, ZC_CHG_CONNECTED);
        }
        break;

    case ZC_CHG_CONNECTED:
        if (!in->plug_present) {
            enter(c, ZC_CHG_IDLE);
        } else if (in->evse_valid && (in->evse_state >= (uint8_t)ZC_EVSE_READY) &&
                   (in->evse_state != (uint8_t)ZC_EVSE_FAULT) && (in->soc < in->target_soc) &&
                   (in->bms_level == 0U)) {
            enter(c, ZC_CHG_CABLE_CHECK);
        } else {
            /* wait for EVSE */
        }
        break;

    case ZC_CHG_CABLE_CHECK:
        if (!check_abort(c, in)) {
            if ((in->evse_state == (uint8_t)ZC_EVSE_CABLE_CHECK_OK) && in->evse_isolation_ok) {
                enter(c, ZC_CHG_PRECHARGE);
            } else if (c->timer_ms > CABLE_CHECK_TMO) {
                fault(c, ZC_CHG_END_STEP_TIMEOUT);
            } else {
                /* waiting */
            }
        }
        break;

    case ZC_CHG_PRECHARGE:
        c->hv_request = true;
        if (!check_abort(c, in)) {
            const float dv = zc_absf(in->evse_v - in->pack_v);
            if (in->hv_closed && (dv < ZC_CHG_VMATCH_V)) {
                c->dc_contactors = true;
                c->cv_pid.integ = I_ABS_MAX; /* start with full allowance */
                c->i_ramp.y = 0.0f;
                enter(c, ZC_CHG_CHARGING);
            } else if (c->timer_ms > PRECHARGE_TMO) {
                fault(c, ZC_CHG_END_STEP_TIMEOUT);
            } else {
                /* waiting */
            }
        }
        break;

    case ZC_CHG_CHARGING:
        if (!check_abort(c, in)) {
            const float i_cv = zc_pid_step(&c->cv_pid, ZC_CHG_CV_CELL_V, in->vmax_cell, DT_S);
            i_target = zc_minf(zc_minf(in->bms_chg_i_lim, in->evse_i_max), i_cv);
            c->v_req = V_LIMIT;
            if (in->soc >= in->target_soc) {
                c->end_reason = ZC_CHG_END_TARGET_SOC;
                enter(c, ZC_CHG_STOPPING);
            } else if ((c->timer_ms > 5000U) && (c->i_ramp.y < ZC_CHG_TAPER_CUTOFF_A) &&
                       (in->vmax_cell > (ZC_CHG_CV_CELL_V - 0.02f))) {
                c->end_reason = ZC_CHG_END_FULL;
                enter(c, ZC_CHG_STOPPING);
            } else {
                /* charging */
            }
        }
        break;

    case ZC_CHG_STOPPING:
        if (!in->plug_present) {
            fault(c, ZC_CHG_END_UNPLUG);
        } else if ((in->evse_i < 2.0f) || (c->timer_ms > STOP_TMO)) {
            c->dc_contactors = false;
            c->hv_request = false;
            enter(c, ZC_CHG_COMPLETE);
        } else {
            /* ramping down */
        }
        break;

    case ZC_CHG_COMPLETE:
        c->hv_request = false;
        c->dc_contactors = false;
        if (!in->plug_present) {
            enter(c, ZC_CHG_IDLE);
        }
        break;

    case ZC_CHG_FAULT:
    default:
        c->state = ZC_CHG_FAULT;
        if (!in->plug_present && (c->timer_ms > 1000U)) {
            enter(c, ZC_CHG_IDLE);
        }
        break;
    }

    if (c->state == ZC_CHG_FAULT) {
        /* safe state: zero demand, isolate inlet immediately */
        c->i_ramp.y = 0.0f;
        c->dc_contactors = false;
        c->hv_request = false;
    }
    c->i_req = zc_ratelim_step(&c->i_ramp, i_target, DT_S);
    c->enable = (c->state == ZC_CHG_CHARGING);
    c->complete = (c->state == ZC_CHG_COMPLETE);
}

#include "zc/zc_safety.h"

#include <string.h>

#include "zc/zc_params.h"
#include "zc/zc_propulsion.h"

/* ================= Torque monitor (E-Gas level 2) =================
 * Recomputes the maximum admissible total torque from the raw pedal using
 * the same pedal map (diverse execution path, separate RAM), plus margin.
 * TCS/TV/limits may only *reduce* or redistribute torque, so the sum of
 * commanded wheel torques must never exceed this envelope. */
void zc_tqmon_init(zc_tqmon_t *m)
{
    (void)memset(m, 0, sizeof(*m));
    zc_debounce_init(&m->db, 5U, 0xFFFFU); /* 50 ms fault tolerant time slice */
    m->margin_nm = 0.10f * ZC_MOTOR_T_MAX_NM * (float)ZC_N_WHEELS;
}

bool zc_tqmon_step_10ms(zc_tqmon_t *m, float pedal_min_pct, float speed_mps, float cmd_sum_nm)
{
    m->allowed = zc_maxf(zc_prop_pedal_map(pedal_min_pct, speed_mps), 0.0f) + m->margin_nm;
    if (zc_debounce_step(&m->db, cmd_sum_nm > m->allowed)) {
        m->fault = true;
    }
    return m->fault;
}

/* ================= Alive supervision ================= */
void zc_wdgm_init(zc_wdgm_t *w) { (void)memset(w, 0, sizeof(*w)); }

uint8_t zc_wdgm_add(zc_wdgm_t *w, uint16_t exp_min, uint16_t exp_max)
{
    uint8_t id = 0xFFU;
    if (w->n < ZC_WDGM_MAX_SE) {
        id = w->n;
        w->exp_min[id] = exp_min;
        w->exp_max[id] = exp_max;
        w->n++;
    }
    return id;
}

void zc_wdgm_checkpoint(zc_wdgm_t *w, uint8_t se)
{
    if ((se < w->n) && (w->count[se] < 0xFFFFU)) {
        w->count[se]++;
    }
}

void zc_wdgm_main(zc_wdgm_t *w)
{
    for (uint8_t i = 0U; i < w->n; i++) {
        const bool ok = (w->count[i] >= w->exp_min[i]) && (w->count[i] <= w->exp_max[i]);
        if (ok) {
            w->fail_cycles[i] = 0U;
        } else if (w->fail_cycles[i] < 255U) {
            w->fail_cycles[i]++;
        } else {
            /* saturated */
        }
        if (w->fail_cycles[i] >= 2U) { /* tolerate one late cycle */
            w->fault = true;
        }
        w->count[i] = 0U;
    }
}

/* ================= DTC store ================= */
static const uint32_t k_dtc_codes[ZC_DTC_COUNT] = {
    0x0A1001U, 0x0A1002U, 0x0A1003U, 0x0A1010U, 0x0A1011U, 0x0A1020U,
    0x0A1021U, 0x0A2001U, 0x0A2002U, 0x0A3001U, 0x0A4001U, 0x0A4002U};

void zc_dtc_init(zc_dtc_t *d)
{
    (void)memset(d, 0, sizeof(*d));
    for (uint8_t i = 0U; i < (uint8_t)ZC_DTC_COUNT; i++) {
        d->e[i].code = k_dtc_codes[i];
    }
}

void zc_dtc_report(zc_dtc_t *d, zc_dtc_id_t id, bool failed, uint32_t now_ms, float soc, float speed)
{
    if ((uint32_t)id < (uint32_t)ZC_DTC_COUNT) {
        zc_dtc_entry_t *e = &d->e[id];
        if (failed) {
            if ((e->status & ZC_DTC_ST_TEST_FAILED) == 0U) {
                e->occurrences++;
                if ((e->status & ZC_DTC_ST_CONFIRMED) == 0U) {
                    e->first_fail_ms = now_ms; /* freeze frame on first confirmation */
                    e->snap_soc = soc;
                    e->snap_speed = speed;
                }
            }
            /* callers report already-qualified (debounced) faults, so confirm directly */
            e->status |= (uint8_t)(ZC_DTC_ST_TEST_FAILED | ZC_DTC_ST_FAILED_THIS_OC | ZC_DTC_ST_PENDING |
                                   ZC_DTC_ST_CONFIRMED | ZC_DTC_ST_FAILED_SINCE_CLR);
        } else {
            e->status &= (uint8_t)~ZC_DTC_ST_TEST_FAILED;
        }
    }
}

void zc_dtc_clear_all(zc_dtc_t *d) { zc_dtc_init(d); }

uint16_t zc_dtc_confirmed_count(const zc_dtc_t *d)
{
    uint16_t n = 0U;
    for (uint8_t i = 0U; i < (uint8_t)ZC_DTC_COUNT; i++) {
        if ((d->e[i].status & ZC_DTC_ST_CONFIRMED) != 0U) {
            n++;
        }
    }
    return n;
}

/* ================= Safe-state manager ================= */
zc_safe_state_t zc_ssm_arbitrate(zc_safe_state_t current, const zc_ssm_in_t *in)
{
    zc_safe_state_t req = ZC_SAFE_NORMAL;
    if (in->thermal_derate) { req = ZC_SAFE_DERATED; }
    if (in->apps_fault || in->tqmon_fault || in->wdgm_fault) { req = ZC_SAFE_STOP; }
    if ((in->bms_level >= 2U) || in->crash) { req = ZC_SAFE_HV_OFF; }
    /* latch: never de-escalate within an operation cycle */
    return (req > current) ? req : current;
}

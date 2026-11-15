#include "zc/zc_bms.h"

#include <math.h>
#include <string.h>

#define DT_FAST_S 0.01f

/* ---------------- Cell OCV curve (NMC, 25 degC) ---------------- */
static const float k_ocv_soc[] = {0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f};
static const float k_ocv_v[]   = {3.00f, 3.45f, 3.55f, 3.61f, 3.66f, 3.72f, 3.81f, 3.91f, 4.00f, 4.09f, 4.19f};
static const zc_lut1d_t k_ocv = {k_ocv_soc, k_ocv_v, (uint8_t)ZC_ARRAY_LEN(k_ocv_soc)};

float zc_bms_ocv(float soc) { return zc_lut1d(&k_ocv, soc); }
float zc_bms_docv_dsoc(float soc) { return zc_lut1d_slope(&k_ocv, soc); }

/* ---------------- State-of-power tables (C-rate vs T, SOC) ---------------- */
static const float k_dch_t[]   = {-20.0f, 0.0f, 10.0f, 25.0f, 45.0f, 55.0f, 60.0f};
static const float k_dch_soc[] = {0.0f, 0.1f, 0.2f, 0.5f, 1.0f};
static const float k_dch_c[] = {
    /* -20 */ 0.0f, 0.5f, 0.8f, 1.0f, 1.0f,
    /*   0 */ 0.0f, 1.5f, 2.0f, 2.5f, 2.5f,
    /*  10 */ 0.0f, 2.5f, 3.0f, 3.5f, 3.5f,
    /*  25 */ 0.0f, 3.0f, 4.0f, 4.0f, 4.0f,
    /*  45 */ 0.0f, 3.0f, 4.0f, 4.0f, 4.0f,
    /*  55 */ 0.0f, 1.5f, 2.0f, 2.0f, 2.0f,
    /*  60 */ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static const zc_lut2d_t k_dch = {k_dch_t, k_dch_soc, k_dch_c, 7U, 5U};

static const float k_chg_t[]   = {-10.0f, 0.0f, 10.0f, 20.0f, 25.0f, 45.0f, 50.0f, 55.0f};
static const float k_chg_soc[] = {0.0f, 0.2f, 0.5f, 0.6f, 0.8f, 0.9f, 1.0f};
static const float k_chg_c[] = {
    /* -10 */ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    /*   0 */ 0.2f, 0.2f, 0.2f, 0.2f, 0.1f, 0.05f, 0.0f,
    /*  10 */ 0.8f, 0.8f, 0.7f, 0.6f, 0.4f, 0.2f, 0.0f,
    /*  20 */ 1.8f, 1.8f, 1.6f, 1.3f, 0.8f, 0.3f, 0.0f,
    /*  25 */ 2.0f, 2.0f, 1.8f, 1.5f, 0.9f, 0.35f, 0.0f,
    /*  45 */ 2.0f, 2.0f, 1.8f, 1.5f, 0.9f, 0.35f, 0.0f,
    /*  50 */ 1.0f, 1.0f, 0.9f, 0.7f, 0.5f, 0.2f, 0.0f,
    /*  55 */ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static const zc_lut2d_t k_chg = {k_chg_t, k_chg_soc, k_chg_c, 8U, 7U};

/* ---------------- EKF tuning ---------------- */
#define EKF_Q_SOC  1.0e-12f
#define EKF_Q_VRC  1.0e-10f
#define EKF_R_V    4.0e-6f   /* (2 mV)^2 averaged-cell measurement noise */
#define EKF_R0_UNCERT 0.50f  /* relative 1-sigma uncertainty of R0 */

void zc_bms_init(zc_bms_t *b, float soc_init, float soc_init_var)
{
    (void)memset(b, 0, sizeof(*b));
    b->soc = zc_clampf(soc_init, 0.0f, 1.0f);
    b->soc_cc = b->soc;
    b->P[0][0] = soc_init_var;
    b->P[1][1] = 1.0e-6f; /* polarisation ~0 after key-off rest */

    /* 10 ms samples */
    zc_debounce_init(&b->db_ov, 10U, 100U);   /* 100 ms qualify */
    zc_debounce_init(&b->db_uv, 50U, 100U);   /* 500 ms: tolerate load dips */
    zc_debounce_init(&b->db_ot, 100U, 300U);  /* 1 s */
    zc_debounce_init(&b->db_ovc, 3U, 1000U);
    zc_debounce_init(&b->db_uvc, 10U, 1000U);
    zc_debounce_init(&b->db_otc, 20U, 1000U);
    zc_debounce_init(&b->db_vs, 5U, 100U);
    zc_debounce_init(&b->db_ts, 10U, 100U);
    zc_debounce_init(&b->db_utc, 50U, 100U);
    b->hv = ZC_HV_OPEN;
}

static void cell_stats(zc_bms_t *b, const zc_bms_meas_t *m, bool *v_implausible, bool *t_implausible)
{
    float sum = 0.0f;
    b->vmax = -1.0e9f;
    b->vmin = 1.0e9f;
    *v_implausible = false;
    *t_implausible = false;
    for (uint8_t i = 0U; i < ZC_BMS_N_CELLS; i++) {
        const float v = m->cell_v[i];
        if ((v < ZC_CELL_SENSOR_MIN_V) || (v > ZC_CELL_SENSOR_MAX_V)) {
            *v_implausible = true;
        }
        sum += v;
        if (v > b->vmax) { b->vmax = v; b->vmax_idx = i; }
        if (v < b->vmin) { b->vmin = v; b->vmin_idx = i; }
    }
    b->vavg = sum / (float)ZC_BMS_N_CELLS;
    b->tmax = -1.0e9f;
    b->tmin = 1.0e9f;
    for (uint8_t i = 0U; i < ZC_BMS_N_TEMPS; i++) {
        const float t = m->cell_t[i];
        if ((t < ZC_TEMP_SENSOR_MIN_C) || (t > ZC_TEMP_SENSOR_MAX_C)) {
            *t_implausible = true;
        }
        b->tmax = zc_maxf(b->tmax, t);
        b->tmin = zc_minf(b->tmin, t);
    }
}

/* 2-state EKF on a first-order Thevenin cell model:
 *   soc[k+1] = soc[k] - I*dt/(3600*Q)
 *   vrc[k+1] = a*vrc[k] + R1*(1-a)*I,   a = exp(-dt/(R1*C1))
 *   v        = OCV(soc) - vrc - R0*I                                   */
static void soc_ekf(zc_bms_t *b, float i_a, float v_meas)
{
    const float a = expf(-DT_FAST_S / (ZC_CELL_R1_OHM * ZC_CELL_C1_F));
    const float dsoc = (i_a * DT_FAST_S) / (3600.0f * ZC_CELL_CAP_AH);

    /* predict */
    b->soc -= dsoc;
    b->soc_cc = zc_clampf(b->soc_cc - dsoc, 0.0f, 1.0f);
    b->vrc = (a * b->vrc) + (ZC_CELL_R1_OHM * (1.0f - a) * i_a);
    const float p00 = b->P[0][0] + EKF_Q_SOC;
    const float p01 = a * b->P[0][1];
    const float p11 = (a * a * b->P[1][1]) + EKF_Q_VRC;

    /* update */
    const float h0 = zc_bms_docv_dsoc(b->soc);
    const float h1 = -1.0f;
    const float v_pred = zc_bms_ocv(b->soc) - b->vrc - (ZC_CELL_R0_OHM * i_a);
    const float innov = v_meas - v_pred;
    const float ph0 = (p00 * h0) + (p01 * h1);
    const float ph1 = (p01 * h0) + (p11 * h1);
    /* measurement noise grows with |I|: R0 is uncertain (temperature, ageing,
     * cell spread), so under heavy load trust coulomb counting more */
    const float sig_r0 = EKF_R0_UNCERT * ZC_CELL_R0_OHM * i_a;
    const float s = (h0 * ph0) + (h1 * ph1) + EKF_R_V + (sig_r0 * sig_r0);
    const float k0 = ph0 / s;
    const float k1 = ph1 / s;

    b->soc = zc_clampf(b->soc + (k0 * innov), 0.0f, 1.0f);
    b->vrc += k1 * innov;
    b->ekf_innov = innov;
    /* Joseph-free simplified form P = (I - K H) P, kept symmetric */
    b->P[0][0] = p00 - (k0 * ph0);
    b->P[0][1] = p01 - (k0 * ph1);
    b->P[1][0] = b->P[0][1];
    b->P[1][1] = p11 - (k1 * ph1);
    if (b->P[0][0] < 1.0e-9f) { b->P[0][0] = 1.0e-9f; }
    if (b->P[1][1] < 1.0e-9f) { b->P[1][1] = 1.0e-9f; }
}

static void monitor(zc_bms_t *b, bool v_impl, bool t_impl, bool charging, float i_a)
{
    struct { zc_debounce_t *db; bool raw; uint32_t bit; } chk[] = {
        {&b->db_ov,  b->vmax > ZC_CELL_OV_WARN_V, ZC_BMS_F_OV},
        {&b->db_uv,  b->vmin < ZC_CELL_UV_WARN_V, ZC_BMS_F_UV},
        {&b->db_ot,  b->tmax > ZC_CELL_OT_C,      ZC_BMS_F_OT},
        {&b->db_ovc, b->vmax > ZC_CELL_OV_CRIT_V, ZC_BMS_F_OV_CRIT},
        {&b->db_uvc, b->vmin < ZC_CELL_UV_CRIT_V, ZC_BMS_F_UV_CRIT},
        {&b->db_otc, b->tmax > ZC_CELL_OT_CRIT_C, ZC_BMS_F_OT_CRIT},
        {&b->db_vs,  v_impl,                      ZC_BMS_F_VSENSE},
        {&b->db_ts,  t_impl,                      ZC_BMS_F_TSENSE},
        {&b->db_utc, charging && (b->tmin < 0.0f) && (i_a < -5.0f), ZC_BMS_F_UT_CHG},
    };
    for (uint8_t i = 0U; i < (uint8_t)ZC_ARRAY_LEN(chk); i++) {
        if (zc_debounce_step(chk[i].db, chk[i].raw)) {
            b->faults |= chk[i].bit; /* latched until power cycle */
        }
    }

    zc_bms_flt_t lvl = ZC_BMS_FLT_NONE;
    if ((b->faults & ZC_BMS_F_UT_CHG) != 0UL) { lvl = ZC_BMS_FLT_WARN; }
    if ((b->faults & (ZC_BMS_F_OV | ZC_BMS_F_UV | ZC_BMS_F_OT | ZC_BMS_F_VSENSE | ZC_BMS_F_TSENSE |
                      ZC_BMS_F_PRECHARGE)) != 0UL) {
        lvl = ZC_BMS_FLT_SEVERE;
    }
    if ((b->faults & (ZC_BMS_F_OV_CRIT | ZC_BMS_F_UV_CRIT | ZC_BMS_F_OT_CRIT | ZC_BMS_F_WELD)) != 0UL) {
        lvl = ZC_BMS_FLT_CRITICAL;
    }
    b->level = lvl;
}

static void all_open(zc_bms_t *b)
{
    b->k.neg = false;
    b->k.pos = false;
    b->k.pre = false;
}

static void hv_sm(zc_bms_t *b, const zc_bms_meas_t *m, bool hv_request)
{
    b->hv_timer_ms += 10U;
    const bool want = hv_request && (b->level < ZC_BMS_FLT_SEVERE);

    switch (b->hv) {
    case ZC_HV_OPEN:
        all_open(b);
        if (want) {
            b->hv = ZC_HV_PRECHARGE;
            b->hv_timer_ms = 0U;
            b->k.neg = true;
            b->k.pre = true;
        }
        break;

    case ZC_HV_PRECHARGE:
        if ((b->hv_timer_ms <= 10U) && (m->bus_v > (0.9f * m->pack_v)) && (m->pack_v > 50.0f)) {
            /* DC link already at pack voltage before precharge could act:
             * main positive contactor suspected welded. */
            b->faults |= ZC_BMS_F_WELD;
            b->hv = ZC_HV_FAULT;
            all_open(b);
        } else if (!want) {
            b->hv = ZC_HV_OPEN;
            all_open(b);
        } else if ((m->bus_v >= (ZC_PRECHARGE_RATIO * m->pack_v)) && (b->hv_timer_ms >= 50U)) {
            b->k.pos = true;
            b->hv = ZC_HV_CLOSED;
            b->hv_timer_ms = 0U;
        } else if (b->hv_timer_ms > ZC_PRECHARGE_TMO_MS) {
            b->faults |= ZC_BMS_F_PRECHARGE;
            b->hv = ZC_HV_FAULT;
            all_open(b);
        } else {
            /* keep precharging */
        }
        break;

    case ZC_HV_CLOSED:
        if (b->hv_timer_ms >= 20U) {
            b->k.pre = false; /* overlap 20 ms, then drop precharge relay */
        }
        if (b->level == ZC_BMS_FLT_CRITICAL) {
            all_open(b);
            b->open_under_load++;
            b->hv = ZC_HV_FAULT;
        } else if (!want) {
            b->hv = ZC_HV_OPENING;
            b->hv_timer_ms = 0U;
        } else {
            /* stay closed */
        }
        break;

    case ZC_HV_OPENING:
        if ((zc_absf(m->pack_i) < ZC_OPEN_CURRENT_A) || (b->hv_timer_ms > ZC_OPEN_TMO_MS) ||
            (b->level == ZC_BMS_FLT_CRITICAL)) {
            if (zc_absf(m->pack_i) >= ZC_OPEN_CURRENT_A) {
                b->open_under_load++;
            }
            all_open(b);
            b->hv = (b->level >= ZC_BMS_FLT_SEVERE) ? ZC_HV_FAULT : ZC_HV_OPEN;
        }
        break;

    case ZC_HV_FAULT:
    default:
        all_open(b); /* latched until next power cycle */
        b->hv = ZC_HV_FAULT;
        break;
    }
}

void zc_bms_step_10ms(zc_bms_t *b, const zc_bms_meas_t *m, bool hv_request, bool charging)
{
    bool v_impl;
    bool t_impl;
    cell_stats(b, m, &v_impl, &t_impl);
    if (!v_impl) {
        soc_ekf(b, m->pack_i, b->vavg);
    }
    monitor(b, v_impl, t_impl, charging, m->pack_i);
    hv_sm(b, m, hv_request);
}

void zc_bms_step_100ms(zc_bms_t *b, const zc_bms_meas_t *m, bool charging)
{
    /* ---- state of power ---- */
    float chg = zc_lut2d(&k_chg, b->tmin, b->soc) * ZC_CELL_CAP_AH;
    float dch = zc_lut2d(&k_dch, b->tmin, b->soc) * ZC_CELL_CAP_AH;
    /* hot-side derate uses the hottest sensor */
    chg = zc_minf(chg, zc_lut2d(&k_chg, b->tmax, b->soc) * ZC_CELL_CAP_AH);
    dch = zc_minf(dch, zc_lut2d(&k_dch, b->tmax, b->soc) * ZC_CELL_CAP_AH);
    /* voltage-window derates (protective, independent of the charger CV loop) */
    chg *= zc_clampf((4.21f - b->vmax) / (4.21f - 4.18f), 0.0f, 1.0f);
    dch *= zc_clampf((b->vmin - 2.90f) / (3.10f - 2.90f), 0.0f, 1.0f);
    if (b->level >= ZC_BMS_FLT_SEVERE) {
        chg = 0.0f;
        dch = 0.0f;
    } else if (b->level == ZC_BMS_FLT_WARN) {
        chg = 0.0f; /* UT charge warning: inhibit charging */
    } else {
        /* normal */
    }
    b->chg_i_lim = chg;
    b->dch_i_lim = dch;

    /* ---- passive balancing: bleed cells > vmin + 10 mV at low current ---- */
    (void)memset(b->bal_mask, 0, sizeof(b->bal_mask));
    b->bal_count = 0U;
    const bool quiet = (zc_absf(m->pack_i) < 20.0f) || charging;
    if (quiet && (b->level == ZC_BMS_FLT_NONE) && (b->vmin > 3.40f) && ((b->vmax - b->vmin) > 0.015f)) {
        for (uint8_t i = 0U; i < ZC_BMS_N_CELLS; i++) {
            if (m->cell_v[i] > (b->vmin + 0.010f)) {
                b->bal_mask[i / 8U] |= (uint8_t)(1U << (i % 8U));
                b->bal_count++;
            }
        }
    }
}

bool zc_bms_cell_balancing(const zc_bms_t *b, uint8_t cell)
{
    return (cell < ZC_BMS_N_CELLS) && ((b->bal_mask[cell / 8U] & (uint8_t)(1U << (cell % 8U))) != 0U);
}

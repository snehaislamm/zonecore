#include <string.h>

#include "utest.h"
#include "zc/zc_bms.h"

static zc_bms_meas_t uniform(float v, float t, float i)
{
    zc_bms_meas_t m;
    memset(&m, 0, sizeof(m));
    for (unsigned k = 0U; k < ZC_BMS_N_CELLS; k++) { m.cell_v[k] = v; }
    for (unsigned k = 0U; k < ZC_BMS_N_TEMPS; k++) { m.cell_t[k] = t; }
    m.pack_i = i;
    m.pack_v = v * (float)ZC_BMS_N_CELLS;
    m.bus_v = 0.0f;
    return m;
}

/* drive the contactor SM to CLOSED with an ideal precharge */
static void close_hv(zc_bms_t *b, zc_bms_meas_t *m)
{
    for (int i = 0; i < 20; i++) {
        zc_bms_step_10ms(b, m, true, false);
        if (b->k.pre && (b->hv_timer_ms >= 20U)) { m->bus_v = m->pack_v; }
    }
}

/* @verifies REQ-BMS-006 */
static void test_precharge_sequence(void)
{
    zc_bms_t b;
    zc_bms_init(&b, 0.5f, 0.01f);
    zc_bms_meas_t m = uniform(3.7f, 25.0f, 0.0f);
    zc_bms_step_10ms(&b, &m, true, false);
    UT_ASSERT(b.hv == ZC_HV_PRECHARGE);
    UT_ASSERT(b.k.neg && b.k.pre && !b.k.pos);
    m.bus_v = 0.5f * m.pack_v;
    for (int i = 0; i < 10; i++) { zc_bms_step_10ms(&b, &m, true, false); }
    UT_ASSERT(!b.k.pos); /* not yet 95 % */
    m.bus_v = 0.96f * m.pack_v;
    zc_bms_step_10ms(&b, &m, true, false);
    UT_ASSERT(b.hv == ZC_HV_CLOSED && b.k.pos);
    for (int i = 0; i < 3; i++) { zc_bms_step_10ms(&b, &m, true, false); }
    UT_ASSERT(!b.k.pre);
}

/* @verifies REQ-BMS-006 */
static void test_precharge_timeout(void)
{
    zc_bms_t b;
    zc_bms_init(&b, 0.5f, 0.01f);
    zc_bms_meas_t m = uniform(3.7f, 25.0f, 0.0f);
    for (int i = 0; i < 120; i++) { zc_bms_step_10ms(&b, &m, true, false); }
    UT_ASSERT(b.hv == ZC_HV_FAULT);
    UT_ASSERT((b.faults & ZC_BMS_F_PRECHARGE) != 0UL);
    UT_ASSERT(!b.k.neg && !b.k.pos && !b.k.pre);
}

/* @verifies REQ-BMS-007 */
static void test_weld_detection(void)
{
    zc_bms_t b;
    zc_bms_init(&b, 0.5f, 0.01f);
    zc_bms_meas_t m = uniform(3.7f, 25.0f, 0.0f);
    m.bus_v = m.pack_v; /* link already live with everything open */
    zc_bms_step_10ms(&b, &m, true, false);
    zc_bms_step_10ms(&b, &m, true, false);
    UT_ASSERT(b.hv == ZC_HV_FAULT);
    UT_ASSERT((b.faults & ZC_BMS_F_WELD) != 0UL);
}

/* @verifies REQ-BMS-002 */
static void test_ov_severe_controlled_open(void)
{
    zc_bms_t b;
    zc_bms_init(&b, 0.9f, 0.01f);
    zc_bms_meas_t m = uniform(4.1f, 25.0f, 50.0f);
    close_hv(&b, &m);
    UT_ASSERT(b.hv == ZC_HV_CLOSED);
    m.cell_v[17] = 4.24f;
    for (int i = 0; i < 9; i++) { zc_bms_step_10ms(&b, &m, true, false); }
    UT_ASSERT(b.level == ZC_BMS_FLT_NONE); /* not yet qualified at 90 ms */
    zc_bms_step_10ms(&b, &m, true, false);
    UT_ASSERT(b.level == ZC_BMS_FLT_SEVERE);
    zc_bms_step_10ms(&b, &m, true, false);
    UT_ASSERT(b.hv == ZC_HV_OPENING); /* waits for current decay */
    zc_bms_step_100ms(&b, &m, false);
    UT_ASSERT((b.chg_i_lim <= 0.0f) && (b.dch_i_lim <= 0.0f));
    m.pack_i = 1.0f;
    zc_bms_step_10ms(&b, &m, true, false);
    UT_ASSERT(b.hv == ZC_HV_FAULT && !b.k.pos);
    UT_ASSERT(b.open_under_load == 0U);
}

/* @verifies REQ-BMS-003 */
static void test_ov_critical_immediate_open(void)
{
    zc_bms_t b;
    zc_bms_init(&b, 0.9f, 0.01f);
    zc_bms_meas_t m = uniform(4.1f, 25.0f, 200.0f);
    close_hv(&b, &m);
    m.cell_v[3] = 4.35f;
    int ms = 0;
    while (b.k.pos && (ms < 200)) {
        zc_bms_step_10ms(&b, &m, true, false);
        ms += 10;
    }
    UT_ASSERT(ms <= 50);
    UT_ASSERT(b.level == ZC_BMS_FLT_CRITICAL);
}

/* @verifies REQ-BMS-004 */
static void test_overtemp(void)
{
    zc_bms_t b;
    zc_bms_init(&b, 0.5f, 0.01f);
    zc_bms_meas_t m = uniform(3.7f, 25.0f, 0.0f);
    m.cell_t[5] = 62.0f;
    for (int i = 0; i < 99; i++) { zc_bms_step_10ms(&b, &m, false, false); }
    UT_ASSERT(b.level == ZC_BMS_FLT_NONE);
    zc_bms_step_10ms(&b, &m, false, false);
    UT_ASSERT(b.level == ZC_BMS_FLT_SEVERE);
}

/* @verifies REQ-BMS-005 */
static void test_sensor_plausibility(void)
{
    zc_bms_t b;
    zc_bms_init(&b, 0.5f, 0.01f);
    zc_bms_meas_t m = uniform(3.7f, 25.0f, 0.0f);
    m.cell_v[40] = 0.0f; /* open sense wire */
    for (int i = 0; i < 6; i++) { zc_bms_step_10ms(&b, &m, false, false); }
    UT_ASSERT((b.faults & ZC_BMS_F_VSENSE) != 0UL);
    UT_ASSERT(b.level == ZC_BMS_FLT_SEVERE);
}

/* @verifies REQ-BMS-008 */
static void test_sop_derates(void)
{
    zc_bms_t b;
    zc_bms_init(&b, 0.5f, 0.01f);
    zc_bms_meas_t m = uniform(3.72f, 25.0f, 0.0f);
    zc_bms_step_10ms(&b, &m, false, false);
    zc_bms_step_100ms(&b, &m, false);
    const float chg25 = b.chg_i_lim;
    const float dch25 = b.dch_i_lim;
    UT_ASSERT(chg25 > 200.0f && dch25 > 500.0f);

    m = uniform(3.72f, -5.0f, 0.0f);
    zc_bms_step_10ms(&b, &m, false, false);
    zc_bms_step_100ms(&b, &m, false);
    UT_ASSERT(b.chg_i_lim < 0.25f * chg25);

    m = uniform(3.72f, 58.0f, 0.0f);
    zc_bms_step_10ms(&b, &m, false, false);
    zc_bms_step_100ms(&b, &m, false);
    UT_ASSERT(b.dch_i_lim < 0.5f * dch25);

    m = uniform(3.72f, 25.0f, 0.0f);
    m.cell_v[0] = 4.21f; /* top of voltage window */
    zc_bms_step_10ms(&b, &m, false, false);
    zc_bms_step_100ms(&b, &m, false);
    UT_ASSERT(b.chg_i_lim < 1.0f);
}

/* @verifies REQ-BMS-009 */
static void test_balancing_rules(void)
{
    zc_bms_t b;
    zc_bms_init(&b, 0.8f, 0.01f);
    zc_bms_meas_t m = uniform(3.95f, 25.0f, 2.0f);
    m.cell_v[10] = 3.97f;
    m.cell_v[11] = 3.955f; /* within 10 mV of min: not bled */
    zc_bms_step_10ms(&b, &m, false, false);
    zc_bms_step_100ms(&b, &m, false);
    UT_ASSERT(zc_bms_cell_balancing(&b, 10U));
    UT_ASSERT(!zc_bms_cell_balancing(&b, 11U));
    UT_ASSERT(b.bal_count == 1U);
    m.pack_i = 150.0f; /* heavy discharge: inhibit */
    zc_bms_step_10ms(&b, &m, false, false);
    zc_bms_step_100ms(&b, &m, false);
    UT_ASSERT(b.bal_count == 0U);
}

/* @verifies REQ-BMS-001 */
static void test_ekf_converges_at_rest(void)
{
    /* true SOC 0.70, estimator starts at 0.50: OCV observability alone
     * must pull it in. Full drive-cycle case is covered in SIL. */
    zc_bms_t b;
    zc_bms_init(&b, 0.50f, 0.04f);
    const float v = zc_bms_ocv(0.70f);
    zc_bms_meas_t m = uniform(v, 25.0f, 0.0f);
    for (int i = 0; i < 6000; i++) { zc_bms_step_10ms(&b, &m, false, false); }
    UT_NEAR(b.soc, 0.70f, 0.03f);
}

void suite_bms(void)
{
    UT_RUN(test_precharge_sequence);
    UT_RUN(test_precharge_timeout);
    UT_RUN(test_weld_detection);
    UT_RUN(test_ov_severe_controlled_open);
    UT_RUN(test_ov_critical_immediate_open);
    UT_RUN(test_overtemp);
    UT_RUN(test_sensor_plausibility);
    UT_RUN(test_sop_derates);
    UT_RUN(test_balancing_rules);
    UT_RUN(test_ekf_converges_at_rest);
}

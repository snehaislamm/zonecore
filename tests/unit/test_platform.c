#include <string.h>

#include "utest.h"
#include "zc/zc_can_db.h"
#include "zc/zc_e2e.h"
#include "zc/zc_sched.h"
#include "zc/zc_util.h"

/* ============================ util ============================ */
static void test_lut1d_interp_and_clamp(void)
{
    static const float x[] = {0.0f, 1.0f, 2.0f};
    static const float y[] = {0.0f, 10.0f, 30.0f};
    const zc_lut1d_t t = {x, y, 3U};
    UT_NEAR(zc_lut1d(&t, 0.5f), 5.0f, 1e-5f);
    UT_NEAR(zc_lut1d(&t, 1.5f), 20.0f, 1e-5f);
    UT_NEAR(zc_lut1d(&t, -3.0f), 0.0f, 1e-6f);
    UT_NEAR(zc_lut1d(&t, 9.0f), 30.0f, 1e-6f);
    UT_NEAR(zc_lut1d_slope(&t, 1.5f), 20.0f, 1e-5f);
}

static void test_lut2d_bilinear(void)
{
    static const float x[] = {0.0f, 1.0f};
    static const float y[] = {0.0f, 1.0f};
    static const float z[] = {0.0f, 1.0f, 2.0f, 3.0f};
    const zc_lut2d_t t = {x, y, z, 2U, 2U};
    UT_NEAR(zc_lut2d(&t, 0.5f, 0.5f), 1.5f, 1e-5f);
    UT_NEAR(zc_lut2d(&t, 1.0f, 0.0f), 2.0f, 1e-5f);
    UT_NEAR(zc_lut2d(&t, 5.0f, 5.0f), 3.0f, 1e-5f);
}

static void test_debounce_qualify_and_heal(void)
{
    zc_debounce_t d;
    zc_debounce_init(&d, 3U, 2U);
    UT_ASSERT(!zc_debounce_step(&d, true));
    UT_ASSERT(!zc_debounce_step(&d, true));
    UT_ASSERT(zc_debounce_step(&d, true));
    UT_ASSERT(zc_debounce_step(&d, false));
    UT_ASSERT(!zc_debounce_step(&d, false));
}

static void test_ratelim(void)
{
    zc_ratelim_t r;
    zc_ratelim_init(&r, 10.0f, 20.0f, 0.0f);
    UT_NEAR(zc_ratelim_step(&r, 100.0f, 0.1f), 1.0f, 1e-5f);
    UT_NEAR(zc_ratelim_step(&r, -100.0f, 0.1f), -1.0f, 1e-5f);
}

/* @verifies REQ-THM-002 */
static void test_pid_antiwindup_bounded(void)
{
    zc_pid_t p;
    zc_pid_init(&p, 1.0f, 5.0f, 0.0f, -1.0f, 1.0f);
    for (int i = 0; i < 10000; i++) {
        (void)zc_pid_step(&p, 100.0f, 0.0f, 0.01f); /* hard saturation */
    }
    UT_ASSERT(p.integ <= 1.0f);
    /* error reverses: output must leave saturation within a few steps */
    float u = 1.0f;
    int steps = 0;
    while ((u > 0.0f) && (steps < 1000)) {
        u = zc_pid_step(&p, -1.0f, 0.0f, 0.01f);
        steps++;
    }
    UT_ASSERT(steps < 50);
}

void suite_util(void)
{
    UT_RUN(test_lut1d_interp_and_clamp);
    UT_RUN(test_lut2d_bilinear);
    UT_RUN(test_debounce_qualify_and_heal);
    UT_RUN(test_ratelim);
    UT_RUN(test_pid_antiwindup_bounded);
}

/* ============================ E2E ============================ */
/* @verifies REQ-COM-001 */
static void test_crc8_j1850_check_value(void)
{
    const uint8_t s[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    UT_ASSERT(zc_crc8_j1850(s, 9U, 0xFFU, true) == 0x4BU); /* CRC-8/SAE-J1850 catalogue check */
}

/* @verifies REQ-COM-001 REQ-COM-002 */
static void test_e2e_sequence(void)
{
    zc_e2e_tx_t tx;
    zc_e2e_rx_t rx;
    zc_e2e_tx_init(&tx, 0x300U);
    zc_e2e_rx_init(&rx, 0x300U, 3U, 250U);
    uint8_t f[8] = {0, 0, 1, 2, 3, 4, 5, 6};

    zc_e2e_protect(&tx, f);
    UT_ASSERT(zc_e2e_check(&rx, f) == ZC_E2E_OK);
    UT_ASSERT(zc_e2e_check(&rx, f) == ZC_E2E_REPEATED);
    zc_e2e_protect(&tx, f);
    UT_ASSERT(zc_e2e_check(&rx, f) == ZC_E2E_OK);
    zc_e2e_protect(&tx, f);
    zc_e2e_protect(&tx, f); /* one lost */
    UT_ASSERT(zc_e2e_check(&rx, f) == ZC_E2E_OK_SOME_LOST);
    for (int i = 0; i < 5; i++) {
        zc_e2e_protect(&tx, f);
    }
    UT_ASSERT(zc_e2e_check(&rx, f) == ZC_E2E_WRONG_SEQ);
    zc_e2e_protect(&tx, f);
    f[4] ^= 0x01U; /* bit flip */
    UT_ASSERT(zc_e2e_check(&rx, f) == ZC_E2E_CRC_ERR);
}

/* @verifies REQ-COM-002 */
static void test_e2e_masquerade_wrong_data_id(void)
{
    zc_e2e_tx_t tx;
    zc_e2e_rx_t rx;
    zc_e2e_tx_init(&tx, 0x101U);
    zc_e2e_rx_init(&rx, 0x300U, 3U, 250U);
    uint8_t f[8] = {0};
    zc_e2e_protect(&tx, f);
    UT_ASSERT(zc_e2e_check(&rx, f) == ZC_E2E_CRC_ERR);
    UT_ASSERT(!zc_e2e_usable(&rx));
}

/* @verifies REQ-COM-003 */
static void test_e2e_timeout(void)
{
    zc_e2e_tx_t tx;
    zc_e2e_rx_t rx;
    zc_e2e_tx_init(&tx, 0x300U);
    zc_e2e_rx_init(&rx, 0x300U, 3U, 250U);
    uint8_t f[8] = {0};
    zc_e2e_protect(&tx, f);
    (void)zc_e2e_check(&rx, f);
    for (int i = 0; i < 25; i++) { zc_e2e_rx_tick(&rx, 10U); }
    UT_ASSERT(zc_e2e_usable(&rx));
    zc_e2e_rx_tick(&rx, 10U);
    UT_ASSERT(!zc_e2e_usable(&rx));
    UT_ASSERT(rx.status == ZC_E2E_TIMEOUT);
}

void suite_e2e(void)
{
    UT_RUN(test_crc8_j1850_check_value);
    UT_RUN(test_e2e_sequence);
    UT_RUN(test_e2e_masquerade_wrong_data_id);
    UT_RUN(test_e2e_timeout);
}

/* ============================ CAN ============================ */
/* @verifies REQ-COM-004 */
static void test_can_roundtrip_signed_and_scaled(void)
{
    zc_can_vcu_torquecmd_t a;
    zc_can_vcu_torquecmd_t b;
    memset(&a, 0, sizeof(a));
    a.Counter = 7U;
    a.TrqFL = -123.5f;
    a.TrqFR = 250.0f;
    a.TrqRL = 0.5f;
    a.TrqRR = -1024.0f;
    a.InvEnable = 1U;
    a.SafeState = 2U;
    uint8_t d[8];
    zc_can_pack_vcu_torquecmd(&a, d);
    zc_can_unpack_vcu_torquecmd(d, &b);
    UT_ASSERT(b.Counter == 7U);
    UT_NEAR(b.TrqFL, -123.5f, 0.26f);
    UT_NEAR(b.TrqFR, 250.0f, 0.26f);
    UT_NEAR(b.TrqRL, 0.5f, 0.26f);
    UT_NEAR(b.TrqRR, -1024.0f, 0.26f);
    UT_ASSERT((b.InvEnable == 1U) && (b.SafeState == 2U));
}

/* @verifies REQ-COM-004 */
static void test_can_saturates_out_of_range(void)
{
    zc_can_bms_limits_t a;
    zc_can_bms_limits_t b;
    memset(&a, 0, sizeof(a));
    a.TCellMax = 500.0f;   /* above 215 degC raw ceiling */
    a.TCellMin = -100.0f;  /* below -40 */
    uint8_t d[8];
    zc_can_pack_bms_limits(&a, d);
    zc_can_unpack_bms_limits(d, &b);
    UT_NEAR(b.TCellMax, 215.0f, 0.01f);
    UT_NEAR(b.TCellMin, -40.0f, 0.01f);
}

void suite_can(void)
{
    UT_RUN(test_can_roundtrip_signed_and_scaled);
    UT_RUN(test_can_saturates_out_of_range);
}

/* ============================ Scheduler ============================ */
static int g_runs_a;
static int g_runs_b;
static uint32_t g_fake_us;
static void task_a(void *c) { (void)c; g_runs_a++; g_fake_us += 50U; }
static void task_b(void *c) { (void)c; g_runs_b++; g_fake_us += 900U; }
static uint32_t fake_clock(void) { return g_fake_us; }

/* @verifies REQ-PLT-001 REQ-PLT-002 */
static void test_sched_rates_offsets_overrun(void)
{
    zc_sched_t s;
    g_runs_a = 0;
    g_runs_b = 0;
    g_fake_us = 0U;
    zc_sched_init(&s, fake_clock);
    UT_ASSERT(zc_sched_add(&s, "a", 10U, 0U, 100U, task_a, NULL));
    UT_ASSERT(zc_sched_add(&s, "b", 100U, 5U, 500U, task_b, NULL));
    UT_ASSERT(!zc_sched_add(&s, "bad", 10U, 10U, 1U, task_a, NULL)); /* offset >= period */
    for (int i = 0; i < 1000; i++) {
        zc_sched_tick(&s);
    }
    UT_ASSERT(g_runs_a == 100);
    UT_ASSERT(g_runs_b == 10);
    UT_ASSERT(s.tasks[0].overruns == 0U);
    UT_ASSERT(s.tasks[1].overruns == 10U);
    UT_ASSERT(s.tasks[1].max_exec_us == 900U);
    UT_NEAR(zc_sched_utilisation_pct(&s), 1.5f, 1e-3f);
}

void suite_sched(void) { UT_RUN(test_sched_rates_offsets_overrun); }

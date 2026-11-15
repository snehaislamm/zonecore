#include <string.h>

#include "utest.h"
#include "zc/zc_body.h"

static zc_body_in_t body_idle(void)
{
    zc_body_in_t in;
    memset(&in, 0, sizeof(in));
    in.ambient_lux = 20000.0f;
    return in;
}

/* count rising edges and on-time of an output over a window */
typedef struct { int edges; int on_ms; } sig_t;

static sig_t run_body(zc_body_t *b, const zc_body_in_t *in, int ms, int which)
{
    sig_t s = {0, 0};
    bool prev = false;
    for (int t = 0; t < ms; t += 10) {
        zc_body_step_10ms(b, in);
        const bool v = (which == 0) ? b->out_turn_l : ((which == 1) ? b->out_turn_r : b->out_stop);
        if (v && !prev) { s.edges++; }
        if (v) { s.on_ms += 10; }
        prev = v;
    }
    return s;
}

/* @verifies REQ-BODY-001 */
static void test_indicator_frequency_duty(void)
{
    zc_body_t b;
    zc_body_init(&b);
    zc_body_in_t in = body_idle();
    in.stalk_left = true;
    zc_body_step_10ms(&b, &in);
    UT_ASSERT(b.out_turn_l); /* first flash immediately */
    const sig_t s = run_body(&b, &in, 60000, 0);
    UT_ASSERT((s.edges >= 60) && (s.edges <= 120));
    const float duty = (float)s.on_ms / 60000.0f;
    UT_ASSERT((duty > 0.4f) && (duty < 0.6f));
    UT_ASSERT(!b.out_turn_r);
}

/* @verifies REQ-BODY-002 */
static void test_hazard_priority(void)
{
    zc_body_t b;
    zc_body_init(&b);
    zc_body_in_t in = body_idle();
    in.stalk_right = true;
    in.hazard_btn = true;
    zc_body_step_10ms(&b, &in);
    in.hazard_btn = false;
    const sig_t l = run_body(&b, &in, 2000, 0);
    zc_body_init(&b);
    in.hazard_btn = true;
    zc_body_step_10ms(&b, &in);
    in.hazard_btn = false;
    const sig_t r = run_body(&b, &in, 2000, 1);
    UT_ASSERT(l.edges >= 2 && r.edges >= 2);
}

/* @verifies REQ-BODY-003 */
static void test_comfort_three_flashes(void)
{
    zc_body_t b;
    zc_body_init(&b);
    zc_body_in_t in = body_idle();
    int edges = 0;
    bool prev = false;
    for (int t = 0; t < 5000; t += 10) {
        in.stalk_left = (t < 200); /* 200 ms tap */
        zc_body_step_10ms(&b, &in);
        if (b.out_turn_l && !prev) { edges++; }
        prev = b.out_turn_l;
    }
    UT_ASSERT(edges == 3);
}

/* @verifies REQ-BODY-004 */
static void test_regen_stop_lamp_thresholds(void)
{
    zc_body_t b;
    zc_body_init(&b);
    zc_body_in_t in = body_idle();
    in.speed_mps = 15.0f;
    in.decel_mps2 = 0.6f;
    zc_body_step_10ms(&b, &in);
    UT_ASSERT(!b.out_stop);
    in.decel_mps2 = 1.0f;
    zc_body_step_10ms(&b, &in);
    UT_ASSERT(!b.out_stop); /* may-zone, default off */
    in.decel_mps2 = 1.4f;
    zc_body_step_10ms(&b, &in);
    UT_ASSERT(b.out_stop);
    in.decel_mps2 = 1.0f;
    zc_body_step_10ms(&b, &in);
    UT_ASSERT(b.out_stop); /* hysteresis */
    in.decel_mps2 = 0.5f;
    zc_body_step_10ms(&b, &in);
    UT_ASSERT(!b.out_stop);
}

/* @verifies REQ-BODY-005 */
static void test_ess_and_auto_hazard(void)
{
    zc_body_t b;
    zc_body_init(&b);
    zc_body_in_t in = body_idle();
    in.speed_mps = 25.0f;
    in.brake_sw = true;
    in.decel_mps2 = 7.0f;
    const sig_t s = run_body(&b, &in, 1000, 2);
    UT_ASSERT(b.ess_active);
    UT_ASSERT((s.edges >= 3) && (s.edges <= 5)); /* 4 Hz +/- 1 */
    in.speed_mps = 0.0f;
    in.decel_mps2 = 0.0f;
    zc_body_step_10ms(&b, &in);
    UT_ASSERT(!b.ess_active && b.hazard_auto);
}

/* @verifies REQ-BODY-006 */
static void test_auto_headlamp(void)
{
    zc_body_t b;
    zc_body_init(&b);
    zc_body_in_t in = body_idle();
    in.ambient_lux = 500.0f;
    int ms = 0;
    while (!b.headlamp && (ms < 5000)) {
        zc_body_step_10ms(&b, &in);
        ms += 10;
    }
    UT_ASSERT(ms <= 2000);
    in.ambient_lux = 3000.0f; /* dead band: stays on */
    for (int i = 0; i < 2000; i++) { zc_body_step_10ms(&b, &in); }
    UT_ASSERT(b.headlamp);
}

/* @verifies REQ-BODY-007 REQ-BODY-008 */
static void test_locks_and_crash(void)
{
    zc_body_t b;
    zc_body_init(&b);
    zc_body_in_t in = body_idle();
    in.speed_mps = 3.0f;
    zc_body_step_10ms(&b, &in);
    UT_ASSERT(b.locked);
    in.crash = true;
    zc_body_step_10ms(&b, &in);
    UT_ASSERT(!b.locked && b.hazard_auto && b.out_turn_l && b.out_turn_r);
}

/* @verifies REQ-MECH-002 */
static void test_window_auto_up(void)
{
    zc_win_t w;
    zc_win_init(&w);
    zc_win_in_t in;
    memset(&in, 0, sizeof(in));
    in.pos_mm = 300.0f;
    in.motor_i = 3.0f;
    in.sw_auto_up = true;
    zc_win_step_10ms(&w, &in);
    in.sw_auto_up = false;
    while ((w.motor < 0) && (in.pos_mm > 0.0f)) {
        in.pos_mm -= 1.0f;
        zc_win_step_10ms(&w, &in);
    }
    UT_ASSERT(in.pos_mm <= 0.5f && w.motor == 0 && w.pinch_count == 0U);
}

void suite_body(void)
{
    UT_RUN(test_indicator_frequency_duty);
    UT_RUN(test_hazard_priority);
    UT_RUN(test_comfort_three_flashes);
    UT_RUN(test_regen_stop_lamp_thresholds);
    UT_RUN(test_ess_and_auto_hazard);
    UT_RUN(test_auto_headlamp);
    UT_RUN(test_locks_and_crash);
    UT_RUN(test_window_auto_up);
}

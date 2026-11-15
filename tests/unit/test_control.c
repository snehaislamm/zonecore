#include <string.h>

#include "utest.h"
#include "zc/zc_charge.h"
#include "zc/zc_dynamics.h"
#include "zc/zc_propulsion.h"
#include "zc/zc_safety.h"
#include "zc/zc_thermal.h"

/* ============================ Charging ============================ */
static zc_chg_in_t chg_nominal(void)
{
    zc_chg_in_t in;
    memset(&in, 0, sizeof(in));
    in.plug_present = true;
    in.evse_valid = true;
    in.evse_state = ZC_EVSE_READY;
    in.evse_i_max = 300.0f;
    in.pack_v = 350.0f;
    in.soc = 0.2f;
    in.vmax_cell = 3.7f;
    in.bms_chg_i_lim = 250.0f;
    in.target_soc = 0.8f;
    return in;
}

static void run_chg(zc_chg_t *c, const zc_chg_in_t *in, int ms)
{
    for (int i = 0; i < ms; i += 10) { zc_chg_step_10ms(c, in); }
}

/* @verifies REQ-CHG-001 */
static void test_chg_sequence_and_contactor_gating(void)
{
    zc_chg_t c;
    zc_chg_init(&c);
    zc_chg_in_t in = chg_nominal();
    run_chg(&c, &in, 30);
    UT_ASSERT(c.state == ZC_CHG_CABLE_CHECK);
    UT_ASSERT(!c.hv_request && !c.dc_contactors);
    in.evse_state = ZC_EVSE_CABLE_CHECK_OK;
    in.evse_isolation_ok = true;
    run_chg(&c, &in, 20);
    UT_ASSERT(c.state == ZC_CHG_PRECHARGE && c.hv_request);
    in.hv_closed = true;
    in.evse_v = 300.0f; /* 50 V mismatch */
    run_chg(&c, &in, 500);
    UT_ASSERT(c.state == ZC_CHG_PRECHARGE && !c.dc_contactors);
    in.evse_v = 345.0f;
    run_chg(&c, &in, 10);
    UT_ASSERT(c.state == ZC_CHG_CHARGING && c.dc_contactors);
}

static void to_charging(zc_chg_t *c, zc_chg_in_t *in)
{
    zc_chg_init(c);
    *in = chg_nominal();
    run_chg(c, in, 30);
    in->evse_state = ZC_EVSE_CABLE_CHECK_OK;
    in->evse_isolation_ok = true;
    in->hv_closed = true;
    in->evse_v = in->pack_v;
    run_chg(c, in, 30);
    in->evse_state = ZC_EVSE_CHARGING;
}

/* @verifies REQ-CHG-002 */
static void test_chg_current_limited(void)
{
    zc_chg_t c;
    zc_chg_in_t in;
    to_charging(&c, &in);
    UT_ASSERT(c.state == ZC_CHG_CHARGING);
    in.evse_i_max = 120.0f;
    run_chg(&c, &in, 10000);
    UT_NEAR(c.i_req, 120.0f, 0.01f);
    in.bms_chg_i_lim = 80.0f;
    run_chg(&c, &in, 1000);
    UT_NEAR(c.i_req, 80.0f, 0.01f);
}

/* @verifies REQ-CHG-004 */
static void test_chg_evse_timeout_abort(void)
{
    zc_chg_t c;
    zc_chg_in_t in;
    to_charging(&c, &in);
    run_chg(&c, &in, 5000);
    in.evse_valid = false;
    run_chg(&c, &in, 10);
    UT_ASSERT(c.state == ZC_CHG_FAULT);
    UT_ASSERT(c.end_reason == ZC_CHG_END_EVSE_TIMEOUT);
    UT_ASSERT(!c.dc_contactors && (c.i_req <= 0.0f));
}

/* @verifies REQ-CHG-005 */
static void test_chg_unplug_and_estop(void)
{
    zc_chg_t c;
    zc_chg_in_t in;
    to_charging(&c, &in);
    in.evse_estop = true;
    run_chg(&c, &in, 10);
    UT_ASSERT(c.state == ZC_CHG_FAULT && c.end_reason == ZC_CHG_END_ESTOP);
    to_charging(&c, &in);
    in.plug_present = false;
    run_chg(&c, &in, 10);
    UT_ASSERT(c.state == ZC_CHG_FAULT && c.end_reason == ZC_CHG_END_UNPLUG);
}

/* @verifies REQ-CHG-006 */
static void test_chg_target_soc_rampdown(void)
{
    zc_chg_t c;
    zc_chg_in_t in;
    to_charging(&c, &in);
    run_chg(&c, &in, 8000);
    in.evse_i = c.i_req;
    in.soc = 0.81f;
    run_chg(&c, &in, 10);
    UT_ASSERT(c.state == ZC_CHG_STOPPING && c.dc_contactors);
    run_chg(&c, &in, 1000);
    UT_ASSERT(c.i_req <= 0.0f && c.state == ZC_CHG_STOPPING); /* EVSE still reports current */
    in.evse_i = 1.0f;
    run_chg(&c, &in, 10);
    UT_ASSERT(c.state == ZC_CHG_COMPLETE && !c.dc_contactors);
    UT_ASSERT(c.end_reason == ZC_CHG_END_TARGET_SOC);
}

void suite_charge(void)
{
    UT_RUN(test_chg_sequence_and_contactor_gating);
    UT_RUN(test_chg_current_limited);
    UT_RUN(test_chg_evse_timeout_abort);
    UT_RUN(test_chg_unplug_and_estop);
    UT_RUN(test_chg_target_soc_rampdown);
}

/* ============================ Thermal ============================ */
/* @verifies REQ-THM-001 REQ-THM-003 */
static void test_thermal_modes(void)
{
    zc_thermal_t t;
    zc_thermal_init(&t);
    zc_th_in_t in = {.tmax = 33.0f, .tmin = 30.0f, .coolant_t = 25.0f, .fast_charging = false, .hv_available = true};
    zc_thermal_step_100ms(&t, &in);
    UT_ASSERT(t.mode == ZC_TH_OFF);       /* 33 < 35 drive threshold */
    in.fast_charging = true;
    zc_thermal_step_100ms(&t, &in);
    UT_ASSERT(t.mode == ZC_TH_COOL);      /* 33 > 32 DCFC threshold */
    UT_ASSERT(t.chiller_w > 0.0f && t.pump > 0.4f);

    zc_thermal_init(&t);
    in.tmax = 16.0f;
    in.tmin = 14.0f;
    zc_thermal_step_100ms(&t, &in);
    UT_ASSERT(t.mode == ZC_TH_HEAT && t.heater_w > 0.0f);
    in.fast_charging = false;             /* 14 degC is fine for driving */
    zc_thermal_init(&t);
    zc_thermal_step_100ms(&t, &in);
    UT_ASSERT(t.mode == ZC_TH_OFF);
}

void suite_thermal(void) { UT_RUN(test_thermal_modes); }

/* ============================ Propulsion ============================ */
static zc_prop_in_t prop_nominal(float pedal)
{
    zc_prop_in_t in;
    memset(&in, 0, sizeof(in));
    in.apps1_pct = pedal;
    in.apps2_pct = pedal;
    in.veh_speed_mps = 10.0f;
    in.motor_speed_avg_rads = 240.0f;
    in.pack_v = 360.0f;
    in.dch_i_lim = 600.0f;
    in.chg_i_lim = 300.0f;
    in.hv_ready = true;
    return in;
}

/* @verifies REQ-PROP-001 */
static void test_apps_disagreement(void)
{
    zc_prop_t p;
    zc_prop_init(&p);
    zc_prop_in_t in = prop_nominal(50.0f);
    for (int i = 0; i < 100; i++) { zc_prop_step_10ms(&p, &in); }
    UT_ASSERT(p.t_demand > 100.0f);
    in.apps2_pct = 30.0f;
    for (int i = 0; i < 9; i++) { zc_prop_step_10ms(&p, &in); }
    UT_ASSERT(!p.apps_fault);
    zc_prop_step_10ms(&p, &in);
    UT_ASSERT(p.apps_fault);
    UT_ASSERT(p.t_demand <= 0.0f);
    in.apps2_pct = 50.0f; /* latched even after recovery */
    zc_prop_step_10ms(&p, &in);
    UT_ASSERT(p.apps_fault && p.t_demand <= 0.0f);
}

/* @verifies REQ-PROP-002 */
static void test_brake_throttle_override(void)
{
    zc_prop_t p;
    zc_prop_init(&p);
    zc_prop_in_t in = prop_nominal(60.0f);
    for (int i = 0; i < 100; i++) { zc_prop_step_10ms(&p, &in); }
    in.brake = true;
    zc_prop_step_10ms(&p, &in);
    UT_ASSERT(p.bto_active && p.t_demand <= 0.0f);
    in.brake = false; /* releasing brake is not enough */
    zc_prop_step_10ms(&p, &in);
    UT_ASSERT(p.bto_active);
    in.apps1_pct = in.apps2_pct = 2.0f;
    zc_prop_step_10ms(&p, &in);
    UT_ASSERT(!p.bto_active);
}

/* @verifies REQ-PROP-003 */
static void test_power_limit(void)
{
    zc_prop_t p;
    zc_prop_init(&p);
    zc_prop_in_t in = prop_nominal(100.0f);
    in.dch_i_lim = 100.0f; /* 36 kW */
    for (int i = 0; i < 200; i++) { zc_prop_step_10ms(&p, &in); }
    const float p_mech = p.t_demand * in.motor_speed_avg_rads;
    UT_ASSERT(p_mech <= (in.dch_i_lim * in.pack_v) + 1.0f);
}

/* @verifies REQ-PROP-004 */
static void test_one_pedal_regen(void)
{
    const float t_hi = zc_prop_pedal_map(0.0f, 20.0f);
    const float decel = -(t_hi * ZC_GEAR_RATIO / ZC_WHEEL_R_M) / ZC_VEH_MASS_KG;
    UT_NEAR(decel, 1.5f, 0.01f);
    UT_NEAR(zc_prop_pedal_map(0.0f, 0.4f), 0.0f, 1e-3f);
    UT_ASSERT(zc_prop_pedal_map(100.0f, 5.0f) > 900.0f);
}

void suite_propulsion(void)
{
    UT_RUN(test_apps_disagreement);
    UT_RUN(test_brake_throttle_override);
    UT_RUN(test_power_limit);
    UT_RUN(test_one_pedal_regen);
}

/* ============================ Dynamics ============================ */
/* @verifies REQ-DYN-003 */
static void test_tv_torque_neutral(void)
{
    zc_dyn_t d;
    zc_dyn_init(&d);
    zc_dyn_in_t in;
    memset(&in, 0, sizeof(in));
    in.t_total = 400.0f;
    for (unsigned i = 0U; i < ZC_N_WHEELS; i++) { in.wheel_w[i] = 20.0f / ZC_WHEEL_R_M; }
    d.v_est = 20.0f;
    in.steer_rad = 0.05f;
    in.yaw_rate = 0.0f; /* under-rotating: TV must push yaw */
    in.tv_enable = true;
    for (int k = 0; k < 20; k++) { zc_dyn_step_10ms(&d, &in); }
    const float sum = d.t_cmd[0] + d.t_cmd[1] + d.t_cmd[2] + d.t_cmd[3];
    UT_NEAR(sum, 400.0f, 0.5f);
    UT_ASSERT(d.t_cmd[1] > d.t_cmd[0]); /* right > left for a left turn */
    UT_ASSERT(d.mz_cmd > 0.0f);
}

static void test_yaw_reference_friction_limit(void)
{
    const float r = zc_dyn_yaw_reference(30.0f, 0.3f);
    UT_ASSERT(r <= (0.9f * 9.81f / 30.0f) + 1e-4f);
    UT_NEAR(zc_dyn_yaw_reference(0.0f, 0.3f), 0.0f, 1e-6f);
}

/* @verifies REQ-DYN-001 */
static void test_tcs_cuts_spinning_wheel(void)
{
    zc_dyn_t d;
    zc_dyn_init(&d);
    zc_dyn_in_t in;
    memset(&in, 0, sizeof(in));
    in.t_total = 800.0f;
    in.tcs_enable = true;
    for (unsigned i = 0U; i < ZC_N_WHEELS; i++) { in.wheel_w[i] = 5.0f / ZC_WHEEL_R_M; }
    in.wheel_w[2] = 8.0f / ZC_WHEEL_R_M; /* RL spinning */
    d.v_est = 5.0f;
    for (int k = 0; k < 10; k++) { zc_dyn_step_10ms(&d, &in); }
    UT_ASSERT(d.tcs_active[2] && !d.tcs_active[3]);
    UT_ASSERT(d.t_cmd[2] < 0.5f * d.t_cmd[3]);
}

void suite_dynamics(void)
{
    UT_RUN(test_tv_torque_neutral);
    UT_RUN(test_yaw_reference_friction_limit);
    UT_RUN(test_tcs_cuts_spinning_wheel);
}

/* ============================ Safety ============================ */
/* @verifies REQ-SAF-002 */
static void test_torque_monitor(void)
{
    zc_tqmon_t m;
    zc_tqmon_init(&m);
    for (int i = 0; i < 100; i++) { UT_ASSERT(!zc_tqmon_step_10ms(&m, 0.0f, 10.0f, 0.0f)); }
    for (int i = 0; i < 4; i++) { UT_ASSERT(!zc_tqmon_step_10ms(&m, 0.0f, 10.0f, 300.0f)); }
    UT_ASSERT(zc_tqmon_step_10ms(&m, 0.0f, 10.0f, 300.0f)); /* 50 ms */
}

/* @verifies REQ-SAF-001 */
static void test_alive_supervision(void)
{
    zc_wdgm_t w;
    zc_wdgm_init(&w);
    const uint8_t se = zc_wdgm_add(&w, 9U, 11U);
    for (int c = 0; c < 5; c++) {
        for (int i = 0; i < 10; i++) { zc_wdgm_checkpoint(&w, se); }
        zc_wdgm_main(&w);
    }
    UT_ASSERT(!w.fault);
    zc_wdgm_main(&w); /* missed one cycle: tolerated */
    UT_ASSERT(!w.fault);
    zc_wdgm_main(&w);
    UT_ASSERT(w.fault);
}

/* @verifies REQ-SAF-003 */
static void test_ssm_latching(void)
{
    zc_ssm_in_t in;
    memset(&in, 0, sizeof(in));
    zc_safe_state_t s = ZC_SAFE_NORMAL;
    in.apps_fault = true;
    s = zc_ssm_arbitrate(s, &in);
    UT_ASSERT(s == ZC_SAFE_STOP);
    in.apps_fault = false;
    s = zc_ssm_arbitrate(s, &in);
    UT_ASSERT(s == ZC_SAFE_STOP);
    in.bms_level = 3U;
    s = zc_ssm_arbitrate(s, &in);
    UT_ASSERT(s == ZC_SAFE_HV_OFF);
}

/* @verifies REQ-SAF-004 */
static void test_dtc_status_bits(void)
{
    zc_dtc_t d;
    zc_dtc_init(&d);
    zc_dtc_report(&d, ZC_DTC_APPS_PLAUS, true, 1234U, 0.5f, 12.0f);
    zc_dtc_report(&d, ZC_DTC_APPS_PLAUS, true, 1334U, 0.5f, 12.0f);
    const zc_dtc_entry_t *e = &d.e[ZC_DTC_APPS_PLAUS];
    UT_ASSERT((e->status & ZC_DTC_ST_CONFIRMED) != 0U);
    UT_ASSERT((e->status & ZC_DTC_ST_TEST_FAILED) != 0U);
    UT_ASSERT(e->occurrences == 1U && e->first_fail_ms == 1234U);
    zc_dtc_report(&d, ZC_DTC_APPS_PLAUS, false, 1434U, 0.5f, 12.0f);
    UT_ASSERT((e->status & ZC_DTC_ST_TEST_FAILED) == 0U);
    UT_ASSERT((e->status & ZC_DTC_ST_CONFIRMED) != 0U);
    UT_ASSERT(zc_dtc_confirmed_count(&d) == 1U);
}

void suite_safety(void)
{
    UT_RUN(test_torque_monitor);
    UT_RUN(test_alive_supervision);
    UT_RUN(test_ssm_latching);
    UT_RUN(test_dtc_status_bits);
}

/**
 * @file zc_vcu.c
 * @brief Integration: owns every module instance, wires signals between them,
 *        defines the task set and handles CAN I/O.
 *
 * Task set (1 ms tick, rate monotonic, offsets spread load):
 *   T0  bms     10 ms  off 0  SOC EKF, cell monitor, contactor SM
 *   T1  drive   10 ms  off 1  propulsion, dynamics, torque monitor, inverter tx
 *   T2  body    10 ms  off 2  lighting, locks, window, charge session, E2E rx timers
 *   T3  slow   100 ms  off 5  thermal, SOP limits, balancing, WdgM, DTC, BMS tx
 */
#if defined(ZC_HOST)
#define _POSIX_C_SOURCE 199309L /* NOLINT(bugprone-reserved-identifier,cert-dcl37-c,cert-dcl51-cpp) */
#include <time.h>
#endif

#include "zc/zc_vcu.h"

#include <string.h>

#include "zc/zc_bms.h"
#include "zc/zc_body.h"
#include "zc/zc_can_db.h"
#include "zc/zc_charge.h"
#include "zc/zc_dynamics.h"
#include "zc/zc_e2e.h"
#include "zc/zc_propulsion.h"
#include "zc/zc_safety.h"
#include "zc/zc_sched.h"
#include "zc/zc_thermal.h"

#define TXQ_LEN 64U

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} can_frame_t;

typedef struct {
    zc_vcu_in_t   in;
    zc_vcu_out_t  out;
    zc_sched_t    sched;
    zc_bms_t      bms;
    zc_bms_meas_t meas;
    zc_chg_t      chg;
    zc_thermal_t  th;
    zc_prop_t     prop;
    zc_dyn_t      dyn;
    zc_body_t     body;
    zc_win_t      win;
    zc_tqmon_t    tqmon;
    zc_wdgm_t     wdgm;
    zc_dtc_t      dtc;
    zc_safe_state_t safe;
    uint8_t       se_bms, se_drive, se_body, se_slow;
    /* COM */
    zc_e2e_rx_t   evse_rx;
    zc_can_evse_status_t evse;
    zc_e2e_tx_t   tx_trq, tx_bms, tx_lim, tx_chg;
    can_frame_t   txq[TXQ_LEN];
    uint8_t       txq_head, txq_tail;
    uint8_t       chg_tx_div;
    /* fault injection */
    uint8_t       inj;
    float         inj_value;
} vcu_t;

static vcu_t g;

#if defined(ZC_HOST)
static uint32_t host_clock_us(void)
{
    struct timespec ts;
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(((uint64_t)ts.tv_sec * 1000000ULL) + ((uint64_t)ts.tv_nsec / 1000ULL));
}
#define ZC_CLOCK host_clock_us
#else
#define ZC_CLOCK NULL
#endif

static void tx_push(uint32_t id, const uint8_t data[8], uint8_t dlc)
{
    const uint8_t next = (uint8_t)((g.txq_head + 1U) % TXQ_LEN);
    if (next != g.txq_tail) { /* drop newest on overflow */
        g.txq[g.txq_head].id = id;
        g.txq[g.txq_head].dlc = dlc;
        (void)memcpy(g.txq[g.txq_head].data, data, 8U);
        g.txq_head = next;
    }
}

static bool hv_ready(void) { return g.bms.hv == ZC_HV_CLOSED; }

/* --------------------------------------------------------------- T0 */
static void task_bms(void *ctx)
{
    (void)ctx;
    (void)memcpy(g.meas.cell_v, g.in.cell_v, sizeof(g.meas.cell_v));
    (void)memcpy(g.meas.cell_t, g.in.cell_t, sizeof(g.meas.cell_t));
    g.meas.pack_i = g.in.pack_i;
    g.meas.pack_v = g.in.pack_v;
    g.meas.bus_v = g.in.bus_v;

    const bool drive = (g.in.drive_request != 0U) && (g.in.plug_present == 0U);
    bool req = drive || g.chg.hv_request;
    if (g.safe == ZC_SAFE_HV_OFF) {
        req = false;
    }
    zc_bms_step_10ms(&g.bms, &g.meas, req, zc_chg_active(&g.chg));
    zc_wdgm_checkpoint(&g.wdgm, g.se_bms);
}

/* --------------------------------------------------------------- T1 */
static void task_drive(void *ctx)
{
    (void)ctx;
#if defined(ZC_TEST_HOOKS)
    if (g.inj == (uint8_t)ZC_INJ_STALL_DRIVE_TASK) {
        return; /* simulated task hang: no checkpoint, no output refresh */
    }
#endif
    float wsum = 0.0f;
    for (uint8_t i = 0U; i < ZC_N_WHEELS; i++) {
        wsum += g.in.wheel_w[i];
    }
    const float wavg = wsum / (float)ZC_N_WHEELS;

    const zc_prop_in_t pin = {
        .apps1_pct = g.in.apps1,
        .apps2_pct = g.in.apps2,
        .brake = g.in.brake_sw != 0U,
        .veh_speed_mps = g.dyn.v_est,
        .motor_speed_avg_rads = wavg * ZC_GEAR_RATIO,
        .pack_v = g.in.pack_v,
        .dch_i_lim = g.bms.dch_i_lim,
        .chg_i_lim = g.bms.chg_i_lim,
        .safe_state = (uint8_t)g.safe,
        .hv_ready = hv_ready() && (g.in.plug_present == 0U),
    };
    zc_prop_step_10ms(&g.prop, &pin);

    zc_dyn_in_t din;
    din.t_total = g.prop.t_demand;
    (void)memcpy(din.wheel_w, g.in.wheel_w, sizeof(din.wheel_w));
    din.steer_rad = g.in.steer_rad;
    din.yaw_rate = g.in.yaw_rate;
    din.ax = g.in.ax;
    din.tcs_enable = g.in.tcs_enable != 0U;
    din.tv_enable = g.in.tv_enable != 0U;
    zc_dyn_step_10ms(&g.dyn, &din);

    float cmd[ZC_N_WHEELS];
    float sum = 0.0f;
    for (uint8_t i = 0U; i < ZC_N_WHEELS; i++) {
        cmd[i] = g.dyn.t_cmd[i];
#if defined(ZC_TEST_HOOKS)
        if (g.inj == (uint8_t)ZC_INJ_TORQUE_OFFSET) {
            cmd[i] += g.inj_value;
        }
#endif
        sum += cmd[i];
    }

    /* level-2 monitor reads the pedal independently of the propulsion path */
    const float pedal_raw = zc_clampf(zc_minf(g.in.apps1, g.in.apps2), 0.0f, 100.0f);
    if (zc_tqmon_step_10ms(&g.tqmon, pedal_raw, g.dyn.v_est, sum)) {
        for (uint8_t i = 0U; i < ZC_N_WHEELS; i++) {
            cmd[i] = 0.0f; /* immediate torque cut, independent of SSM latency */
        }
    }

    const bool inv_en = hv_ready() && (g.safe < ZC_SAFE_HV_OFF) && !g.tqmon.fault;
    for (uint8_t i = 0U; i < ZC_N_WHEELS; i++) {
        g.out.trq_cmd[i] = inv_en ? cmd[i] : 0.0f;
    }
    g.out.inv_enable = inv_en ? 1U : 0U;

    zc_can_vcu_torquecmd_t m;
    (void)memset(&m, 0, sizeof(m));
    m.TrqFL = g.out.trq_cmd[0];
    m.TrqFR = g.out.trq_cmd[1];
    m.TrqRL = g.out.trq_cmd[2];
    m.TrqRR = g.out.trq_cmd[3];
    m.InvEnable = g.out.inv_enable;
    m.SafeState = (uint8_t)g.safe;
    uint8_t f[8];
    zc_can_pack_vcu_torquecmd(&m, f);
    zc_e2e_protect(&g.tx_trq, f);
    tx_push(ZC_CAN_ID_VCU_TORQUECMD, f, 8U);

    zc_wdgm_checkpoint(&g.wdgm, g.se_drive);
}

/* --------------------------------------------------------------- T2 */
static void task_body(void *ctx)
{
    (void)ctx;
    zc_e2e_rx_tick(&g.evse_rx, 10U);

    const zc_chg_in_t cin = {
        .plug_present = g.in.plug_present != 0U,
        .evse_valid = zc_e2e_usable(&g.evse_rx),
        .evse_state = g.evse.EvseState,
        .evse_v = g.evse.PresentVoltage,
        .evse_i = g.evse.PresentCurrent,
        .evse_i_max = (float)g.evse.MaxCurrent,
        .evse_isolation_ok = g.evse.IsolationOk != 0U,
        .evse_estop = g.evse.EStop != 0U,
        .pack_v = g.in.pack_v,
        .soc = g.bms.soc,
        .vmax_cell = g.bms.vmax,
        .bms_chg_i_lim = g.bms.chg_i_lim,
        .bms_level = (uint8_t)g.bms.level,
        .hv_closed = hv_ready(),
        .target_soc = g.in.target_soc,
    };
    zc_chg_step_10ms(&g.chg, &cin);

    g.chg_tx_div++;
    if ((g.chg_tx_div >= 5U) && (g.in.plug_present != 0U)) {
        g.chg_tx_div = 0U;
        zc_can_vcu_chargereq_t r;
        (void)memset(&r, 0, sizeof(r));
        r.ChargeState = (uint8_t)g.chg.state;
        r.TargetVoltage = g.chg.v_req;
        r.TargetCurrent = g.chg.i_req;
        r.ChargeEnable = g.chg.enable ? 1U : 0U;
        r.ChargeComplete = g.chg.complete ? 1U : 0U;
        r.SocDisplay = g.bms.soc * 100.0f;
        uint8_t f[8];
        zc_can_pack_vcu_chargereq(&r, f);
        zc_e2e_protect(&g.tx_chg, f);
        tx_push(ZC_CAN_ID_VCU_CHARGEREQ, f, 8U);
    }

    const zc_body_in_t bin = {
        .stalk_left = g.in.stalk_left != 0U,
        .stalk_right = g.in.stalk_right != 0U,
        .hazard_btn = g.in.hazard_btn != 0U,
        .brake_sw = g.in.brake_sw != 0U,
        .decel_mps2 = -g.in.ax,
        .speed_mps = g.dyn.v_est,
        .ambient_lux = g.in.ambient_lux,
        .crash = g.in.crash != 0U,
    };
    zc_body_step_10ms(&g.body, &bin);

    const zc_win_in_t win = {
        .sw_up = g.in.win_up != 0U,
        .sw_down = g.in.win_down != 0U,
        .sw_auto_up = g.in.win_auto_up != 0U,
        .motor_i = g.in.win_current,
        .pos_mm = g.in.win_pos_mm,
    };
    zc_win_step_10ms(&g.win, &win);
    zc_wdgm_checkpoint(&g.wdgm, g.se_body);
}

/* --------------------------------------------------------------- T3 */
static void task_slow(void *ctx)
{
    (void)ctx;
    zc_bms_step_100ms(&g.bms, &g.meas, zc_chg_active(&g.chg));

    const zc_th_in_t tin = {
        .tmax = g.bms.tmax,
        .tmin = g.bms.tmin,
        .coolant_t = g.in.coolant_t,
        .fast_charging = (g.chg.state >= ZC_CHG_CABLE_CHECK) && (g.chg.state <= ZC_CHG_STOPPING),
        .hv_available = hv_ready() || (g.in.plug_present != 0U),
    };
    zc_thermal_step_100ms(&g.th, &tin);

    zc_wdgm_checkpoint(&g.wdgm, g.se_slow);
    zc_wdgm_main(&g.wdgm);

    const zc_ssm_in_t sin = {
        .bms_level = (uint8_t)g.bms.level,
        .apps_fault = g.prop.apps_fault,
        .tqmon_fault = g.tqmon.fault,
        .wdgm_fault = g.wdgm.fault,
        .crash = g.in.crash != 0U,
        .thermal_derate = g.bms.tmax > 55.0f,
    };
    g.safe = zc_ssm_arbitrate(g.safe, &sin);

    /* diagnostics */
    const uint32_t t = g.sched.now_ms;
    const float soc = g.bms.soc;
    const float v = g.dyn.v_est;
    const uint32_t f = g.bms.faults;
    zc_dtc_report(&g.dtc, ZC_DTC_CELL_OV, (f & (ZC_BMS_F_OV | ZC_BMS_F_OV_CRIT)) != 0UL, t, soc, v);
    zc_dtc_report(&g.dtc, ZC_DTC_CELL_UV, (f & (ZC_BMS_F_UV | ZC_BMS_F_UV_CRIT)) != 0UL, t, soc, v);
    zc_dtc_report(&g.dtc, ZC_DTC_CELL_OT, (f & (ZC_BMS_F_OT | ZC_BMS_F_OT_CRIT)) != 0UL, t, soc, v);
    zc_dtc_report(&g.dtc, ZC_DTC_CELL_VSENSE, (f & ZC_BMS_F_VSENSE) != 0UL, t, soc, v);
    zc_dtc_report(&g.dtc, ZC_DTC_CELL_TSENSE, (f & ZC_BMS_F_TSENSE) != 0UL, t, soc, v);
    zc_dtc_report(&g.dtc, ZC_DTC_PRECHARGE, (f & ZC_BMS_F_PRECHARGE) != 0UL, t, soc, v);
    zc_dtc_report(&g.dtc, ZC_DTC_CONTACTOR_WELD, (f & ZC_BMS_F_WELD) != 0UL, t, soc, v);
    zc_dtc_report(&g.dtc, ZC_DTC_APPS_PLAUS, g.prop.apps_fault, t, soc, v);
    zc_dtc_report(&g.dtc, ZC_DTC_TORQUE_MON, g.tqmon.fault, t, soc, v);
    zc_dtc_report(&g.dtc, ZC_DTC_ALIVE_SUP, g.wdgm.fault, t, soc, v);
    zc_dtc_report(&g.dtc, ZC_DTC_EVSE_COMM, g.chg.end_reason == ZC_CHG_END_EVSE_TIMEOUT, t, soc, v);
    zc_dtc_report(&g.dtc, ZC_DTC_CHG_ABORT, g.chg.state == ZC_CHG_FAULT, t, soc, v);

    /* BMS broadcast */
    uint8_t fr[8];
    zc_can_bms_status_t s;
    (void)memset(&s, 0, sizeof(s));
    s.HvState = (uint8_t)g.bms.hv;
    s.PackVoltage = g.in.pack_v;
    s.PackCurrent = g.in.pack_i;
    s.SOC = g.bms.soc * 100.0f;
    s.FaultLevel = (uint8_t)g.bms.level;
    s.Balancing = (g.bms.bal_count > 0U) ? 1U : 0U;
    zc_can_pack_bms_status(&s, fr);
    zc_e2e_protect(&g.tx_bms, fr);
    tx_push(ZC_CAN_ID_BMS_STATUS, fr, 8U);

    zc_can_bms_limits_t l;
    (void)memset(&l, 0, sizeof(l));
    l.ChgCurLim = g.bms.chg_i_lim;
    l.DchCurLim = g.bms.dch_i_lim;
    l.TCellMax = g.bms.tmax;
    l.TCellMin = g.bms.tmin;
    zc_can_pack_bms_limits(&l, fr);
    zc_e2e_protect(&g.tx_lim, fr);
    tx_push(ZC_CAN_ID_BMS_LIMITS, fr, 8U);
}

/* ================================================================ API */
void zc_vcu_init(float soc_init, float soc_init_var)
{
    (void)memset(&g, 0, sizeof(g));
    zc_bms_init(&g.bms, soc_init, soc_init_var);
    zc_chg_init(&g.chg);
    zc_thermal_init(&g.th);
    zc_prop_init(&g.prop);
    zc_dyn_init(&g.dyn);
    zc_body_init(&g.body);
    zc_win_init(&g.win);
    zc_tqmon_init(&g.tqmon);
    zc_dtc_init(&g.dtc);
    zc_wdgm_init(&g.wdgm);
    g.se_bms = zc_wdgm_add(&g.wdgm, 9U, 11U);
    g.se_drive = zc_wdgm_add(&g.wdgm, 9U, 11U);
    g.se_body = zc_wdgm_add(&g.wdgm, 9U, 11U);
    g.se_slow = zc_wdgm_add(&g.wdgm, 1U, 1U);
    g.safe = ZC_SAFE_NORMAL;

    zc_e2e_rx_init(&g.evse_rx, (uint16_t)ZC_CAN_ID_EVSE_STATUS, 3U, 250U);
    zc_e2e_tx_init(&g.tx_trq, (uint16_t)ZC_CAN_ID_VCU_TORQUECMD);
    zc_e2e_tx_init(&g.tx_bms, (uint16_t)ZC_CAN_ID_BMS_STATUS);
    zc_e2e_tx_init(&g.tx_lim, (uint16_t)ZC_CAN_ID_BMS_LIMITS);
    zc_e2e_tx_init(&g.tx_chg, (uint16_t)ZC_CAN_ID_VCU_CHARGEREQ);

    zc_sched_init(&g.sched, ZC_CLOCK);
    (void)zc_sched_add(&g.sched, "bms", 10U, 0U, 150U, task_bms, NULL);
    (void)zc_sched_add(&g.sched, "drive", 10U, 1U, 120U, task_drive, NULL);
    (void)zc_sched_add(&g.sched, "body", 10U, 2U, 80U, task_body, NULL);
    (void)zc_sched_add(&g.sched, "slow", 100U, 5U, 400U, task_slow, NULL);
}

static void publish(zc_vcu_out_t *o)
{
    o->trq_demand = g.prop.t_demand;
    o->safe_state = (uint8_t)g.safe;
    o->apps_fault = g.prop.apps_fault ? 1U : 0U;
    o->bto_active = g.prop.bto_active ? 1U : 0U;
    o->tqmon_fault = g.tqmon.fault ? 1U : 0U;
    o->wdgm_fault = g.wdgm.fault ? 1U : 0U;
    o->v_est = g.dyn.v_est;
    o->yaw_ref = g.dyn.yaw_ref;
    o->mz_cmd = g.dyn.mz_cmd;
    (void)memcpy(o->slip, g.dyn.slip, sizeof(o->slip));
    o->hv_state = (uint8_t)g.bms.hv;
    o->k_neg = g.bms.k.neg ? 1U : 0U;
    o->k_pos = g.bms.k.pos ? 1U : 0U;
    o->k_pre = g.bms.k.pre ? 1U : 0U;
    o->k_dcc = g.chg.dc_contactors ? 1U : 0U;
    o->bms_level = (uint8_t)g.bms.level;
    o->bal_count = g.bms.bal_count;
    o->bms_faults = g.bms.faults;
    o->soc = g.bms.soc;
    o->soc_cc = g.bms.soc_cc;
    o->chg_i_lim = g.bms.chg_i_lim;
    o->dch_i_lim = g.bms.dch_i_lim;
    o->vmax = g.bms.vmax;
    o->vmin = g.bms.vmin;
    o->tmax = g.bms.tmax;
    o->chg_state = (uint8_t)g.chg.state;
    o->chg_end = (uint8_t)g.chg.end_reason;
    o->chg_i_req = g.chg.i_req;
    o->chg_v_req = g.chg.v_req;
    o->th_mode = (uint8_t)g.th.mode;
    o->chiller_w = g.th.chiller_w;
    o->heater_w = g.th.heater_w;
    o->pump = g.th.pump;
    o->turn_l = g.body.out_turn_l ? 1U : 0U;
    o->turn_r = g.body.out_turn_r ? 1U : 0U;
    o->stop_lamp = g.body.out_stop ? 1U : 0U;
    o->headlamp = g.body.headlamp ? 1U : 0U;
    o->locked = g.body.locked ? 1U : 0U;
    o->hazard = (g.body.hazard_latched || g.body.hazard_auto) ? 1U : 0U;
    o->ess = g.body.ess_active ? 1U : 0U;
    o->win_motor = g.win.motor;
    o->win_pinch = (g.win.pinch_count > 0U) ? 1U : 0U;
    o->win_force_est = g.win.f_est;
    o->dtc_confirmed = zc_dtc_confirmed_count(&g.dtc);
    uint32_t ovr = 0U;
    for (uint8_t i = 0U; i < g.sched.n_tasks; i++) {
        ovr += g.sched.tasks[i].overruns;
    }
    o->sched_overruns = ovr;
    o->time_ms = g.sched.now_ms;
}

void zc_vcu_step(const zc_vcu_in_t *in, zc_vcu_out_t *out, uint32_t n_ms)
{
    g.in = *in;
    for (uint32_t i = 0U; i < n_ms; i++) {
        zc_sched_tick(&g.sched);
    }
    /* outputs written by tasks (torques, enable) live in g.out */
    publish(&g.out);
    *out = g.out;
}

void zc_vcu_can_rx(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    if ((id == ZC_CAN_ID_EVSE_STATUS) && (dlc == 8U)) {
        uint8_t f[8];
        (void)memcpy(f, data, 8U);
#if defined(ZC_TEST_HOOKS)
        if (g.inj == (uint8_t)ZC_INJ_EVSE_CRC) {
            f[0] ^= 0x5AU;
        }
#endif
        const zc_e2e_status_t st = zc_e2e_check(&g.evse_rx, f);
        if ((st == ZC_E2E_OK) || (st == ZC_E2E_OK_SOME_LOST)) {
            zc_can_unpack_evse_status(f, &g.evse);
        }
    }
}

uint8_t zc_vcu_can_tx_pop(uint32_t *id, uint8_t *data, uint8_t *dlc)
{
    uint8_t ok = 0U;
    if (g.txq_tail != g.txq_head) {
        *id = g.txq[g.txq_tail].id;
        *dlc = g.txq[g.txq_tail].dlc;
        (void)memcpy(data, g.txq[g.txq_tail].data, 8U);
        g.txq_tail = (uint8_t)((g.txq_tail + 1U) % TXQ_LEN);
        ok = 1U;
    }
    return ok;
}

void zc_vcu_inject(uint8_t inj, float value)
{
    g.inj = inj;
    g.inj_value = value;
}

uint8_t zc_vcu_dtc(uint8_t idx, uint32_t *code, uint8_t *status, uint16_t *occ)
{
    uint8_t ok = 0U;
    if (idx < (uint8_t)ZC_DTC_COUNT) {
        *code = g.dtc.e[idx].code;
        *status = g.dtc.e[idx].status;
        *occ = g.dtc.e[idx].occurrences;
        ok = 1U;
    }
    return ok;
}

uint8_t zc_vcu_cell_balancing(uint8_t cell) { return zc_bms_cell_balancing(&g.bms, cell) ? 1U : 0U; }

void zc_vcu_abi_sizes(uint32_t *in_sz, uint32_t *out_sz)
{
    *in_sz = (uint32_t)sizeof(zc_vcu_in_t);
    *out_sz = (uint32_t)sizeof(zc_vcu_out_t);
}

float zc_vcu_sched_util_pct(void) { return zc_sched_utilisation_pct(&g.sched); }

uint32_t zc_vcu_task_max_exec_us(uint8_t task)
{
    return (task < g.sched.n_tasks) ? g.sched.tasks[task].max_exec_us : 0U;
}

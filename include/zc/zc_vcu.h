/**
 * @file zc_vcu.h
 * @brief Vehicle Control Unit integration layer and the flat C ABI used by
 *        the Python software-in-the-loop (SIL) bench via ctypes.
 *
 * All booleans in the ABI are uint8_t so the struct layout maps 1:1 onto
 * ctypes without padding surprises. Keep sim/zcsil.py in sync (a layout
 * check runs at load time: zc_vcu_abi_sizes()).
 */
#ifndef ZC_VCU_H
#define ZC_VCU_H

#include <stdint.h>

#include "zc/zc_params.h"

typedef struct {
    /* HV battery sensing (AFE + current sensor + HV measurement) */
    float   cell_v[ZC_BMS_N_CELLS];
    float   cell_t[ZC_BMS_N_TEMPS];
    float   pack_i;
    float   pack_v;
    float   bus_v;
    float   coolant_t;
    /* driver controls */
    float   apps1;
    float   apps2;
    uint8_t brake_sw;
    uint8_t drive_request;
    /* chassis sensors */
    float   wheel_w[ZC_N_WHEELS];
    float   steer_rad;
    float   yaw_rate;
    float   ax;
    /* charging */
    uint8_t plug_present;
    float   target_soc;
    /* body */
    uint8_t stalk_left;
    uint8_t stalk_right;
    uint8_t hazard_btn;
    uint8_t crash;
    float   ambient_lux;
    uint8_t win_up;
    uint8_t win_down;
    uint8_t win_auto_up;
    float   win_current;
    float   win_pos_mm;
    /* feature switches (calibration / test) */
    uint8_t tcs_enable;
    uint8_t tv_enable;
} zc_vcu_in_t;

typedef struct {
    /* propulsion */
    float   trq_cmd[ZC_N_WHEELS];
    float   trq_demand;
    uint8_t inv_enable;
    uint8_t safe_state;
    uint8_t apps_fault;
    uint8_t bto_active;
    uint8_t tqmon_fault;
    uint8_t wdgm_fault;
    /* dynamics */
    float   v_est;
    float   yaw_ref;
    float   mz_cmd;
    float   slip[ZC_N_WHEELS];
    /* BMS */
    uint8_t hv_state;
    uint8_t k_neg;
    uint8_t k_pos;
    uint8_t k_pre;
    uint8_t k_dcc;
    uint8_t bms_level;
    uint8_t bal_count;
    uint32_t bms_faults;
    float   soc;
    float   soc_cc;
    float   chg_i_lim;
    float   dch_i_lim;
    float   vmax;
    float   vmin;
    float   tmax;
    /* charging */
    uint8_t chg_state;
    uint8_t chg_end;
    float   chg_i_req;
    float   chg_v_req;
    /* thermal */
    uint8_t th_mode;
    float   chiller_w;
    float   heater_w;
    float   pump;
    /* body */
    uint8_t turn_l;
    uint8_t turn_r;
    uint8_t stop_lamp;
    uint8_t headlamp;
    uint8_t locked;
    uint8_t hazard;
    uint8_t ess;
    int8_t  win_motor;
    uint8_t win_pinch;
    float   win_force_est;
    /* diagnostics */
    uint16_t dtc_confirmed;
    uint32_t sched_overruns;
    uint32_t time_ms;
} zc_vcu_out_t;

/* fault injection hooks (test builds only; compiled out without ZC_TEST_HOOKS) */
typedef enum {
    ZC_INJ_NONE = 0,
    ZC_INJ_STALL_DRIVE_TASK = 1,   /**< drive task stops executing */
    ZC_INJ_TORQUE_OFFSET = 2,      /**< corrupt each wheel command by +value Nm */
    ZC_INJ_EVSE_CRC = 3            /**< corrupt received EVSE CRC */
} zc_inj_t;

void zc_vcu_init(float soc_init, float soc_init_var);
/** Run n_ms one-millisecond scheduler ticks with the given inputs. */
void zc_vcu_step(const zc_vcu_in_t *in, zc_vcu_out_t *out, uint32_t n_ms);
/** Deliver a received CAN frame. */
void zc_vcu_can_rx(uint32_t id, const uint8_t *data, uint8_t dlc);
/** Pop the next transmitted CAN frame; returns 0 when queue empty. */
uint8_t zc_vcu_can_tx_pop(uint32_t *id, uint8_t *data, uint8_t *dlc);
void zc_vcu_inject(uint8_t inj, float value);
/** DTC readout (UDS 0x19-style): returns 0 when idx out of range. */
uint8_t zc_vcu_dtc(uint8_t idx, uint32_t *code, uint8_t *status, uint16_t *occ);
/** Per-cell balancing state readback. */
uint8_t zc_vcu_cell_balancing(uint8_t cell);
/** ABI guard: sizeof(in), sizeof(out). */
void zc_vcu_abi_sizes(uint32_t *in_sz, uint32_t *out_sz);
/** Scheduler stats: utilisation (budget-based) and per-task max exec [us]. */
float zc_vcu_sched_util_pct(void);
uint32_t zc_vcu_task_max_exec_us(uint8_t task);

#endif

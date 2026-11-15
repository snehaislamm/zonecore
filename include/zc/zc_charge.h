/**
 * @file zc_charge.h
 * @brief DC fast-charge session controller (EV side). The session flow is an
 *        abstraction of DIN SPEC 70121 / ISO 15118-2 DC: parameter discovery,
 *        cable (insulation) check, precharge voltage match, current demand
 *        with CC/CV, and a controlled ramp-down. Pilot/PLC physical layers
 *        are out of scope; EVSE data arrives as an E2E-protected CAN frame.
 */
#ifndef ZC_CHARGE_H
#define ZC_CHARGE_H

#include <stdbool.h>
#include <stdint.h>

#include "zc/zc_util.h"

typedef enum {
    ZC_CHG_IDLE = 0,
    ZC_CHG_CONNECTED,
    ZC_CHG_CABLE_CHECK,
    ZC_CHG_PRECHARGE,
    ZC_CHG_CHARGING,
    ZC_CHG_STOPPING,
    ZC_CHG_COMPLETE,
    ZC_CHG_FAULT
} zc_chg_state_t;

typedef enum {
    ZC_EVSE_NOT_READY = 0,
    ZC_EVSE_READY = 1,
    ZC_EVSE_CABLE_CHECK_OK = 2,
    ZC_EVSE_PRECHARGE = 3,
    ZC_EVSE_CHARGING = 4,
    ZC_EVSE_STOPPED = 5,
    ZC_EVSE_FAULT = 6
} zc_evse_state_t;

typedef enum {
    ZC_CHG_END_NONE = 0,
    ZC_CHG_END_TARGET_SOC,
    ZC_CHG_END_FULL,
    ZC_CHG_END_UNPLUG,
    ZC_CHG_END_EVSE_TIMEOUT,
    ZC_CHG_END_EVSE_FAULT,
    ZC_CHG_END_ESTOP,
    ZC_CHG_END_BMS_FAULT,
    ZC_CHG_END_STEP_TIMEOUT
} zc_chg_end_t;

typedef struct {
    bool  plug_present;      /**< proximity detection */
    bool  evse_valid;        /**< E2E: fresh, CRC-correct EVSE data */
    uint8_t evse_state;
    float evse_v;
    float evse_i;
    float evse_i_max;
    bool  evse_isolation_ok;
    bool  evse_estop;
    float pack_v;
    float soc;
    float vmax_cell;
    float bms_chg_i_lim;
    uint8_t bms_level;
    bool  hv_closed;
    float target_soc;
} zc_chg_in_t;

typedef struct {
    zc_chg_state_t state;
    zc_chg_end_t   end_reason;
    uint32_t       timer_ms;
    zc_pid_t       cv_pid;
    zc_ratelim_t   i_ramp;
    /* outputs */
    float          i_req;
    float          v_req;
    bool           hv_request;
    bool           dc_contactors;
    bool           enable;
    bool           complete;
} zc_chg_t;

void zc_chg_init(zc_chg_t *c);
void zc_chg_step_10ms(zc_chg_t *c, const zc_chg_in_t *in);
bool zc_chg_active(const zc_chg_t *c);

#endif

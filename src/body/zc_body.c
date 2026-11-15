#include "zc/zc_body.h"

#include <string.h>

#define TICK_MS            10U
#define COMFORT_TAP_MS     400U
#define COMFORT_FLASHES    3U
#define LUX_ON_DWELL_MS    1000U   /* switch ON within 2 s of darkness */
#define LUX_OFF_DWELL_MS   10000U  /* switch OFF after sustained brightness */
#define AUTOLOCK_SPEED     (8.0f / 3.6f)
#define AUTO_HAZARD_OFF_V  (10.0f / 3.6f)

void zc_body_init(zc_body_t *b) { (void)memset(b, 0, sizeof(*b)); }

static void flasher_restart(zc_body_t *b)
{
    b->flash_ms = 0U;
    b->flash_phase_on = true; /* first flash immediately on activation */
}

static void indicators(zc_body_t *b, const zc_body_in_t *in)
{
    /* hazard push-button toggles on rising edge */
    if (in->hazard_btn && !b->prev_hazard_btn) {
        b->hazard_latched = !b->hazard_latched;
        if (!b->hazard_latched) {
            b->hazard_auto = false;
        }
        flasher_restart(b);
    }
    b->prev_hazard_btn = in->hazard_btn;

    /* stalk: hold = continuous, short tap = 3-flash comfort (lane change) */
    int8_t stalk = 0;
    if (in->stalk_left) {
        stalk = -1;
    } else if (in->stalk_right) {
        stalk = 1;
    } else {
        /* neutral */
    }
    if (stalk != 0) {
        if ((b->stalk_hold_ms == 0U) || (stalk != b->active_dir)) {
            b->active_dir = stalk;
            b->comfort_flashes = 0U;
            b->stalk_hold_ms = 0U;
            flasher_restart(b);
        }
        b->stalk_hold_ms += TICK_MS;
    } else {
        if ((b->stalk_hold_ms > 0U) && (b->stalk_hold_ms < COMFORT_TAP_MS)) {
            b->comfort_flashes = COMFORT_FLASHES; /* count includes the one in progress */
        } else if (b->stalk_hold_ms >= COMFORT_TAP_MS) {
            b->active_dir = 0;
        } else {
            /* idle */
        }
        b->stalk_hold_ms = 0U;
    }

    const bool hazard = b->hazard_latched || b->hazard_auto;
    const bool dir_on = (b->stalk_hold_ms > 0U) || (b->comfort_flashes > 0U);
    if (!dir_on && !hazard) {
        b->active_dir = 0;
    }

    /* flasher timebase */
    b->flash_ms += TICK_MS;
    if (b->flash_ms >= ZC_FLASH_PERIOD_MS) {
        b->flash_ms = 0U;
        if ((b->comfort_flashes > 0U) && (b->stalk_hold_ms == 0U)) {
            b->comfort_flashes--;
            if (b->comfort_flashes == 0U) {
                b->active_dir = 0;
            }
        }
    }
    b->flash_phase_on = b->flash_ms < ZC_FLASH_ON_MS;

    /* hazard has priority over direction indication */
    if (hazard) {
        b->out_turn_l = b->flash_phase_on;
        b->out_turn_r = b->flash_phase_on;
    } else {
        b->out_turn_l = (b->active_dir < 0) && b->flash_phase_on;
        b->out_turn_r = (b->active_dir > 0) && b->flash_phase_on;
    }
}

static void stop_lamps(zc_body_t *b, const zc_body_in_t *in)
{
    /* regenerative braking: lamp ON above 1.3 m/s^2, OFF below 0.7 m/s^2 */
    if (in->decel_mps2 > ZC_REGEN_LAMP_ON) {
        b->regen_lamp = true;
    } else if (in->decel_mps2 < ZC_REGEN_LAMP_OFF) {
        b->regen_lamp = false;
    } else {
        /* hysteresis band: hold */
    }

    /* emergency stop signal */
    if (!b->ess_active && (in->speed_mps > ZC_ESS_MIN_SPEED) && (in->decel_mps2 >= ZC_ESS_ON_DECEL)) {
        b->ess_active = true;
        b->ess_was_active = true;
        b->ess_ms = 0U;
    } else if (b->ess_active && (in->decel_mps2 < ZC_ESS_OFF_DECEL)) {
        b->ess_active = false;
    } else {
        /* hold */
    }
    /* after an emergency stop to standstill, hazards come on automatically */
    if (b->ess_was_active && (in->speed_mps < 0.3f) && !b->hazard_auto) {
        b->hazard_auto = true;
        flasher_restart(b);
    }
    if (b->hazard_auto && (in->speed_mps > AUTO_HAZARD_OFF_V) && !in->crash) {
        b->hazard_auto = false;
        b->ess_was_active = false;
    }

    const bool steady = in->brake_sw || b->regen_lamp;
    if (b->ess_active) {
        b->ess_ms += TICK_MS;
        b->out_stop = (b->ess_ms % ZC_ESS_PERIOD_MS) < (ZC_ESS_PERIOD_MS / 2U);
    } else {
        b->out_stop = steady;
    }
}

static void headlamps(zc_body_t *b, const zc_body_in_t *in)
{
    if (in->ambient_lux < ZC_LUX_ON) {
        b->lux_dark_ms += TICK_MS;
        b->lux_bright_ms = 0U;
    } else if (in->ambient_lux > ZC_LUX_OFF) {
        b->lux_bright_ms += TICK_MS;
        b->lux_dark_ms = 0U;
    } else {
        b->lux_dark_ms = 0U;
        b->lux_bright_ms = 0U;
    }
    if (!b->headlamp && (b->lux_dark_ms >= LUX_ON_DWELL_MS)) {
        b->headlamp = true;
    } else if (b->headlamp && (b->lux_bright_ms >= LUX_OFF_DWELL_MS)) {
        b->headlamp = false;
    } else {
        /* hold */
    }
}

void zc_body_step_10ms(zc_body_t *b, const zc_body_in_t *in)
{
    if (in->crash) {
        b->locked = false;        /* unlock for rescue access */
        if (!b->hazard_auto) {
            b->hazard_auto = true;
            flasher_restart(b);
        }
    } else if (in->speed_mps > AUTOLOCK_SPEED) {
        b->locked = true;
    } else {
        /* hold */
    }
    stop_lamps(b, in);
    indicators(b, in);
    headlamps(b, in);
}

/* ======================= Anti-pinch window ======================= */
void zc_win_init(zc_win_t *w) { (void)memset(w, 0, sizeof(*w)); }

void zc_win_step_10ms(zc_win_t *w, const zc_win_in_t *in)
{
    w->pinch_event = false;

    if (w->reversing) {
        w->motor = 1;
        if ((in->pos_mm >= w->reverse_target) || (in->pos_mm >= ZC_WIN_FULL_OPEN_MM)) {
            w->reversing = false;
            w->motor = 0;
        }
    } else {
        if (in->sw_auto_up) {
            w->auto_up = true;
        }
        if (in->sw_down) {
            w->auto_up = false;
            w->motor = 1;
        } else if (in->sw_up || w->auto_up) {
            w->motor = -1;
        } else {
            w->motor = 0;
        }
        if ((w->motor < 0) && (in->pos_mm <= 0.5f)) {
            w->motor = 0;
            w->auto_up = false;
        }
        if ((w->motor > 0) && (in->pos_mm >= ZC_WIN_FULL_OPEN_MM)) {
            w->motor = 0;
        }
    }

    if (w->motor < 0) {
        w->run_ms += TICK_MS;
        /* learn free-running current during the first 150 ms and outside the
         * pinch zone; then estimate obstruction force from the excess */
        const bool learn = (w->run_ms <= 150U) || (in->pos_mm > ZC_WIN_PINCH_ZONE_MAX);
        if (learn) {
            w->i_baseline = (w->run_ms <= 10U) ? in->motor_i : (0.8f * w->i_baseline) + (0.2f * in->motor_i);
        }
        w->f_est = zc_maxf((in->motor_i - w->i_baseline) * ZC_WIN_N_PER_A, 0.0f);
        const bool in_zone = (in->pos_mm >= ZC_WIN_PINCH_ZONE_MIN) && (in->pos_mm <= ZC_WIN_PINCH_ZONE_MAX);
        if (in_zone && !learn && (w->f_est > ZC_WIN_PINCH_N)) {
            w->pinch_event = true;
            w->pinch_count++;
            w->auto_up = false;
            w->reversing = true;
            w->reverse_target = in->pos_mm + ZC_WIN_REVERSE_MM;
            w->motor = 1;
        }
    } else {
        w->run_ms = 0U;
        w->f_est = 0.0f;
    }
}

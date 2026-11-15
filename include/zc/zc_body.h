/**
 * @file zc_body.h
 * @brief Body controls: exterior lighting (direction indicators, hazard,
 *        stop lamps incl. regenerative-braking and emergency-stop-signal
 *        logic, automatic headlamps), door locks, and an anti-pinch power
 *        window (mechanism control).
 *
 * Lighting behaviour is *modelled on* UN R48 / UN R13-H / FMVSS 108 and the
 * window on FMVSS 118 / UN R21; see docs/homologation.md for the mapping and
 * the caveat that the current regulation text is the authority.
 */
#ifndef ZC_BODY_H
#define ZC_BODY_H

#include <stdbool.h>
#include <stdint.h>

#include "zc/zc_util.h"

#define ZC_FLASH_PERIOD_MS   670U   /* ~90 flashes/min */
#define ZC_FLASH_ON_MS       340U
#define ZC_ESS_PERIOD_MS     250U   /* 4 Hz */
#define ZC_ESS_ON_DECEL      6.0f   /* m/s^2 */
#define ZC_ESS_OFF_DECEL     2.5f
#define ZC_ESS_MIN_SPEED     (50.0f / 3.6f)
#define ZC_REGEN_LAMP_ON     1.3f
#define ZC_REGEN_LAMP_OFF    0.7f
#define ZC_LUX_ON            1000.0f
#define ZC_LUX_OFF           7000.0f

typedef struct {
    bool  stalk_left;
    bool  stalk_right;
    bool  hazard_btn;      /**< momentary push button */
    bool  brake_sw;
    float decel_mps2;      /**< positive = decelerating */
    float speed_mps;
    float ambient_lux;
    bool  crash;
} zc_body_in_t;

typedef struct {
    /* indicator */
    bool     hazard_latched;
    bool     hazard_auto;      /**< activated by ESS / crash */
    bool     prev_hazard_btn;
    int8_t   active_dir;       /**< -1 left, +1 right, 0 none */
    uint8_t  comfort_flashes;
    uint32_t stalk_hold_ms;
    uint32_t flash_ms;
    bool     flash_phase_on;
    /* stop lamps */
    bool     regen_lamp;
    bool     ess_active;
    bool     ess_was_active;
    uint32_t ess_ms;
    /* headlamps */
    bool     headlamp;
    uint32_t lux_dark_ms;
    uint32_t lux_bright_ms;
    /* locks */
    bool     locked;
    /* outputs */
    bool     out_turn_l;
    bool     out_turn_r;
    bool     out_stop;
} zc_body_t;

void zc_body_init(zc_body_t *b);
void zc_body_step_10ms(zc_body_t *b, const zc_body_in_t *in);

/* ---------------- Anti-pinch window ---------------- */
#define ZC_WIN_FULL_OPEN_MM   450.0f
#define ZC_WIN_PINCH_ZONE_MIN 4.0f
#define ZC_WIN_PINCH_ZONE_MAX 200.0f
#define ZC_WIN_REVERSE_MM     200.0f
#define ZC_WIN_N_PER_A        40.0f    /* motor force constant (calibrated) */
#define ZC_WIN_PINCH_N        65.0f    /* detection threshold, < 100 N limit */

typedef struct {
    bool  sw_up;
    bool  sw_down;
    bool  sw_auto_up;   /**< one-touch close */
    float motor_i;      /**< [A] */
    float pos_mm;       /**< 0 = fully closed */
} zc_win_in_t;

typedef struct {
    int8_t   motor;         /**< -1 close, 0 stop, +1 open */
    bool     auto_up;
    bool     reversing;
    float    reverse_target;
    bool     pinch_event;
    uint32_t pinch_count;
    float    i_baseline;
    float    f_est;
    uint32_t run_ms;
} zc_win_t;

void zc_win_init(zc_win_t *w);
void zc_win_step_10ms(zc_win_t *w, const zc_win_in_t *in);

#endif

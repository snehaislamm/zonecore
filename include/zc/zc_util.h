/**
 * @file zc_util.h
 * @brief Shared embedded control primitives: saturation, rate limiting,
 *        debouncing, low-pass filtering, 1-D / 2-D lookup tables, PID.
 *
 * Coding rules (MISRA-C:2012 inspired): no dynamic memory, no recursion,
 * fixed-width types only, every function total over its input domain.
 */
#ifndef ZC_UTIL_H
#define ZC_UTIL_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define ZC_ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

float zc_clampf(float x, float lo, float hi);
float zc_minf(float a, float b);
float zc_maxf(float a, float b);
float zc_absf(float x);

/* ---------- Rate limiter (slew limiter) ---------- */
typedef struct {
    float rise_per_s;  /**< max positive slope [unit/s] */
    float fall_per_s;  /**< max negative slope magnitude [unit/s] */
    float y;           /**< current output */
} zc_ratelim_t;

void  zc_ratelim_init(zc_ratelim_t *r, float rise_per_s, float fall_per_s, float y0);
float zc_ratelim_step(zc_ratelim_t *r, float target, float dt_s);

/* ---------- First-order low-pass ---------- */
typedef struct {
    float alpha;
    float y;
    bool  primed;
} zc_lpf_t;

void  zc_lpf_init(zc_lpf_t *f, float cutoff_hz, float dt_s);
float zc_lpf_step(zc_lpf_t *f, float x);

/* ---------- Up/down counter debounce (fault qualification) ---------- */
typedef struct {
    uint16_t set_count;    /**< consecutive-ish samples to qualify a fault */
    uint16_t clear_count;  /**< samples to de-qualify (healing) */
    uint16_t counter;
    bool     state;
} zc_debounce_t;

void zc_debounce_init(zc_debounce_t *d, uint16_t set_count, uint16_t clear_count);
bool zc_debounce_step(zc_debounce_t *d, bool raw_fault);

/* ---------- Lookup tables (linear interpolation, clamped extrapolation) ---------- */
typedef struct {
    const float *x;  /**< strictly increasing breakpoints */
    const float *y;
    uint8_t      n;
} zc_lut1d_t;

typedef struct {
    const float *x;  /**< row axis, length nx */
    const float *y;  /**< column axis, length ny */
    const float *z;  /**< row-major nx*ny */
    uint8_t      nx;
    uint8_t      ny;
} zc_lut2d_t;

float zc_lut1d(const zc_lut1d_t *t, float x);
float zc_lut2d(const zc_lut2d_t *t, float x, float y);
/** d/dx of a 1-D table at x (piecewise slope). */
float zc_lut1d_slope(const zc_lut1d_t *t, float x);

/* ---------- PID with back-calculation anti-windup ---------- */
typedef struct {
    float kp, ki, kd;
    float kaw;          /**< back-calculation gain */
    float out_min, out_max;
    float integ;
    float prev_err;
    bool  primed;
} zc_pid_t;

void  zc_pid_init(zc_pid_t *p, float kp, float ki, float kd, float out_min, float out_max);
void  zc_pid_reset(zc_pid_t *p);
float zc_pid_step(zc_pid_t *p, float setpoint, float meas, float dt_s);

#endif /* ZC_UTIL_H */

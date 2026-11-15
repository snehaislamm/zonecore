#include "zc/zc_util.h"

#include <math.h>

float zc_clampf(float x, float lo, float hi)
{
    float r = x;
    if (r < lo) { r = lo; }
    if (r > hi) { r = hi; }
    return r;
}

float zc_minf(float a, float b) { return (a < b) ? a : b; }
float zc_maxf(float a, float b) { return (a > b) ? a : b; }
float zc_absf(float x) { return (x < 0.0f) ? -x : x; }

void zc_ratelim_init(zc_ratelim_t *r, float rise_per_s, float fall_per_s, float y0)
{
    r->rise_per_s = rise_per_s;
    r->fall_per_s = fall_per_s;
    r->y = y0;
}

float zc_ratelim_step(zc_ratelim_t *r, float target, float dt_s)
{
    const float up = r->rise_per_s * dt_s;
    const float dn = r->fall_per_s * dt_s;
    const float d = zc_clampf(target - r->y, -dn, up);
    r->y += d;
    return r->y;
}

void zc_lpf_init(zc_lpf_t *f, float cutoff_hz, float dt_s)
{
    const float tau = 1.0f / (2.0f * 3.14159265f * cutoff_hz);
    f->alpha = dt_s / (tau + dt_s);
    f->y = 0.0f;
    f->primed = false;
}

float zc_lpf_step(zc_lpf_t *f, float x)
{
    if (!f->primed) {
        f->y = x;
        f->primed = true;
    } else {
        f->y += f->alpha * (x - f->y);
    }
    return f->y;
}

void zc_debounce_init(zc_debounce_t *d, uint16_t set_count, uint16_t clear_count)
{
    d->set_count = (set_count == 0U) ? 1U : set_count;
    d->clear_count = (clear_count == 0U) ? 1U : clear_count;
    d->counter = 0U;
    d->state = false;
}

bool zc_debounce_step(zc_debounce_t *d, bool raw_fault)
{
    if (!d->state) {
        if (raw_fault) {
            d->counter++;
            if (d->counter >= d->set_count) {
                d->state = true;
                d->counter = 0U;
            }
        } else if (d->counter > 0U) {
            d->counter--; /* leaky: intermittent noise does not accumulate forever */
        } else {
            /* hold */
        }
    } else {
        if (!raw_fault) {
            d->counter++;
            if (d->counter >= d->clear_count) {
                d->state = false;
                d->counter = 0U;
            }
        } else {
            d->counter = 0U;
        }
    }
    return d->state;
}

static uint8_t lut_seg(const float *x, uint8_t n, float v)
{
    uint8_t i = 0U;
    while ((i < (uint8_t)(n - 2U)) && (v > x[i + 1U])) {
        i++;
    }
    return i;
}

float zc_lut1d(const zc_lut1d_t *t, float x)
{
    float r;
    if (t->n == 0U) {
        r = 0.0f;
    } else if ((t->n == 1U) || (x <= t->x[0])) {
        r = t->y[0];
    } else if (x >= t->x[t->n - 1U]) {
        r = t->y[t->n - 1U];
    } else {
        const uint8_t i = lut_seg(t->x, t->n, x);
        const float f = (x - t->x[i]) / (t->x[i + 1U] - t->x[i]);
        r = t->y[i] + f * (t->y[i + 1U] - t->y[i]);
    }
    return r;
}

float zc_lut1d_slope(const zc_lut1d_t *t, float x)
{
    float r = 0.0f;
    if (t->n >= 2U) {
        const float xc = zc_clampf(x, t->x[0], t->x[t->n - 1U]);
        const uint8_t i = lut_seg(t->x, t->n, xc);
        r = (t->y[i + 1U] - t->y[i]) / (t->x[i + 1U] - t->x[i]);
    }
    return r;
}

float zc_lut2d(const zc_lut2d_t *t, float x, float y)
{
    const float xc = zc_clampf(x, t->x[0], t->x[t->nx - 1U]);
    const float yc = zc_clampf(y, t->y[0], t->y[t->ny - 1U]);
    const uint8_t i = lut_seg(t->x, t->nx, xc);
    const uint8_t j = lut_seg(t->y, t->ny, yc);
    const float fx = (xc - t->x[i]) / (t->x[i + 1U] - t->x[i]);
    const float fy = (yc - t->y[j]) / (t->y[j + 1U] - t->y[j]);
    const float z00 = t->z[(i * t->ny) + j];
    const float z01 = t->z[(i * t->ny) + j + 1U];
    const float z10 = t->z[((i + 1U) * t->ny) + j];
    const float z11 = t->z[((i + 1U) * t->ny) + j + 1U];
    const float z0 = z00 + fy * (z01 - z00);
    const float z1 = z10 + fy * (z11 - z10);
    return z0 + fx * (z1 - z0);
}

void zc_pid_init(zc_pid_t *p, float kp, float ki, float kd, float out_min, float out_max)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->kaw = 10.0f; /* [1/s] back-calculation tracking gain */
    p->out_min = out_min;
    p->out_max = out_max;
    zc_pid_reset(p);
}

void zc_pid_reset(zc_pid_t *p)
{
    p->integ = 0.0f;
    p->prev_err = 0.0f;
    p->primed = false;
}

float zc_pid_step(zc_pid_t *p, float setpoint, float meas, float dt_s)
{
    const float err = setpoint - meas;
    float deriv = 0.0f;
    if (p->primed && (dt_s > 0.0f)) {
        deriv = (err - p->prev_err) / dt_s;
    }
    p->prev_err = err;
    p->primed = true;

    const float u_unsat = (p->kp * err) + p->integ + (p->kd * deriv);
    const float u = zc_clampf(u_unsat, p->out_min, p->out_max);
    /* back-calculation: bleed the integrator by the saturation excess */
    p->integ += dt_s * ((p->ki * err) + (p->kaw * (u - u_unsat)));
    p->integ = zc_clampf(p->integ, p->out_min, p->out_max);
    return u;
}

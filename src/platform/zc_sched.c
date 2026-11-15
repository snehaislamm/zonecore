#include "zc/zc_sched.h"

#include <string.h>

void zc_sched_init(zc_sched_t *s, zc_clock_us_fn_t clock_us)
{
    (void)memset(s, 0, sizeof(*s));
    s->clock_us = clock_us;
}

bool zc_sched_add(zc_sched_t *s, const char *name, uint32_t period_ms, uint32_t offset_ms,
                  uint32_t budget_us, zc_task_fn_t fn, void *ctx)
{
    bool ok = false;
    if ((s->n_tasks < ZC_SCHED_MAX_TASKS) && (fn != NULL) && (period_ms > 0U) &&
        (offset_ms < period_ms)) {
        zc_task_t *t = &s->tasks[s->n_tasks];
        (void)memset(t, 0, sizeof(*t));
        t->name = name;
        t->period_ms = period_ms;
        t->offset_ms = offset_ms;
        t->budget_us = budget_us;
        t->fn = fn;
        t->ctx = ctx;
        s->n_tasks++;
        ok = true;
    }
    return ok;
}

void zc_sched_tick(zc_sched_t *s)
{
    for (uint8_t i = 0U; i < s->n_tasks; i++) {
        zc_task_t *t = &s->tasks[i];
        if ((s->now_ms % t->period_ms) == t->offset_ms) {
            const uint32_t t0 = (s->clock_us != NULL) ? s->clock_us() : 0U;
            t->fn(t->ctx);
            const uint32_t t1 = (s->clock_us != NULL) ? s->clock_us() : 0U;
            const uint32_t exec = t1 - t0; /* modulo arithmetic handles wrap */
            t->runs++;
            if (exec > t->max_exec_us) {
                t->max_exec_us = exec;
            }
            if ((t->budget_us > 0U) && (exec > t->budget_us)) {
                t->overruns++;
                s->overrun_flag = true;
            }
        }
    }
    s->now_ms++;
}

float zc_sched_utilisation_pct(const zc_sched_t *s)
{
    float u = 0.0f;
    for (uint8_t i = 0U; i < s->n_tasks; i++) {
        u += (float)s->tasks[i].budget_us / ((float)s->tasks[i].period_ms * 1000.0f);
    }
    return 100.0f * u;
}

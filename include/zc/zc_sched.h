/**
 * @file zc_sched.h
 * @brief Static, rate-monotonic, run-to-completion cyclic executive.
 *
 * Mirrors the structure of an AUTOSAR OS / FreeRTOS periodic task set on an
 * automotive MCU: tasks are declared in a const-sized table ordered by
 * priority (shortest period first). A 1 ms tick releases due tasks; each
 * task's execution time is measured against its budget and its release
 * jitter is tracked, and overruns are reported to the safety layer.
 */
#ifndef ZC_SCHED_H
#define ZC_SCHED_H

#include <stdbool.h>
#include <stdint.h>

#define ZC_SCHED_MAX_TASKS 8U

typedef void (*zc_task_fn_t)(void *ctx);
typedef uint32_t (*zc_clock_us_fn_t)(void);

typedef struct {
    const char  *name;
    uint32_t     period_ms;
    uint32_t     offset_ms;   /**< phase offset to spread CPU load */
    uint32_t     budget_us;   /**< WCET budget */
    zc_task_fn_t fn;
    void        *ctx;
    /* runtime statistics */
    uint32_t     runs;
    uint32_t     overruns;
    uint32_t     max_exec_us;
} zc_task_t;

typedef struct {
    zc_task_t        tasks[ZC_SCHED_MAX_TASKS];
    uint8_t          n_tasks;
    uint32_t         now_ms;
    zc_clock_us_fn_t clock_us;   /**< optional high-res clock for WCET measurement */
    bool             overrun_flag;
} zc_sched_t;

void zc_sched_init(zc_sched_t *s, zc_clock_us_fn_t clock_us);
/** @return false if table is full or the task is malformed. */
bool zc_sched_add(zc_sched_t *s, const char *name, uint32_t period_ms, uint32_t offset_ms,
                  uint32_t budget_us, zc_task_fn_t fn, void *ctx);
/** Advance one 1 ms tick and run all released tasks in priority order. */
void zc_sched_tick(zc_sched_t *s);
/** CPU utilisation bound U = sum(C_i/T_i) from budgets, in percent. */
float zc_sched_utilisation_pct(const zc_sched_t *s);

#endif

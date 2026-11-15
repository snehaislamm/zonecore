/* Minimal xUnit-style harness for host unit tests. No allocation, no deps.
 * Tag each test with  @verifies REQ-xxx  so tools/trace.py can build the
 * requirements traceability matrix. */
#ifndef UTEST_H
#define UTEST_H

#include <stdio.h>

extern int ut_checks;
extern int ut_failures;
extern int ut_current_failed;

void ut_run(const char *name, void (*fn)(void));
void ut_fail(const char *file, int line, const char *expr);

#define UT_ASSERT(c)                                    \
    do {                                                \
        ut_checks++;                                    \
        if (!(c)) {                                     \
            ut_fail(__FILE__, __LINE__, #c);            \
            return;                                     \
        }                                               \
    } while (0)

static inline int ut_near(double a, double b, double tol) { return ((a - b) <= tol) && ((b - a) <= tol); }
#define UT_NEAR(a, b, tol) UT_ASSERT(ut_near((double)(a), (double)(b), (double)(tol)))
#define UT_RUN(fn) ut_run(#fn, fn)

void suite_util(void);
void suite_e2e(void);
void suite_can(void);
void suite_sched(void);
void suite_bms(void);
void suite_charge(void);
void suite_thermal(void);
void suite_propulsion(void);
void suite_dynamics(void);
void suite_body(void);
void suite_safety(void);

#endif

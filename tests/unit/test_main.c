#include "utest.h"

int ut_checks = 0;
int ut_failures = 0;
int ut_current_failed = 0;
static int ut_tests = 0;

void ut_fail(const char *file, int line, const char *expr)
{
    printf("    FAIL %s:%d: %s\n", file, line, expr);
    ut_current_failed = 1;
}

void ut_run(const char *name, void (*fn)(void))
{
    ut_current_failed = 0;
    ut_tests++;
    fn();
    if (ut_current_failed != 0) {
        ut_failures++;
        printf("[FAIL] %s\n", name);
    } else {
        printf("[ ok ] %s\n", name);
    }
}

int main(void)
{
    suite_util();
    suite_e2e();
    suite_can();
    suite_sched();
    suite_bms();
    suite_charge();
    suite_thermal();
    suite_propulsion();
    suite_dynamics();
    suite_body();
    suite_safety();
    printf("\n%d tests, %d checks, %d failed\n", ut_tests, ut_checks, ut_failures);
    return (ut_failures == 0) ? 0 : 1;
}

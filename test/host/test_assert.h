/* Non-fatal host-test checks: report file:line and values, count failures, keep going. */
#pragma once

#include <math.h>
#include <stdio.h>

static int test_failures;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n",                     \
                    __FILE__, __LINE__, #cond);                              \
            test_failures++;                                                 \
        }                                                                    \
    } while (0)

/* |actual - expected| <= tol; prints both values on failure. */
#define CHECK_NEAR(actual, expected, tol)                                    \
    do {                                                                     \
        const double a_ = (double)(actual), e_ = (double)(expected);         \
        const double t_ = (double)(tol);                                     \
        if (!(fabs(a_ - e_) <= t_)) {                                        \
            fprintf(stderr, "%s:%d: CHECK_NEAR failed: %s = %.9g, "          \
                    "expected %s = %.9g, tol %.3g\n", __FILE__, __LINE__,    \
                    #actual, a_, #expected, e_, t_);                         \
            test_failures++;                                                 \
        }                                                                    \
    } while (0)

/* Exit status for main(): 0 when every check passed. */
static inline int test_report(const char *name)
{
    if (test_failures)
        fprintf(stderr, "%s: %d check(s) FAILED\n", name, test_failures);
    else
        printf("%s: OK\n", name);
    return test_failures ? 1 : 0;
}

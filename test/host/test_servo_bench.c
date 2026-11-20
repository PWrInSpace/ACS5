// Golden tests for the servo bench sequence: segment boundaries, one fin at a
// time, lock timing, wrap-around and the J4 socket names.
#include "test_assert.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sensor_drivers/servos/servo_bench.h"

static int nonzero_fins(const servo_bench_cmd_t *c) {
    int n = 0;
    for (int i = 0; i < SERVO_BENCH_FIN_COUNT; i++)
        if (c->fin_deg[i] != 0.0f) n++;
    return n;
}

static void test_period(void) {
    CHECK(servo_bench_period_ms() == 18000u);
}

static void test_fin_segments(void) {
    servo_bench_cmd_t c = servo_bench_cmd_at(0);
    CHECK(c.active_fin == 0);
    CHECK(c.fin_deg[0] == 10.0f);
    CHECK(c.fin_deg[1] == 0.0f && c.fin_deg[2] == 0.0f && c.fin_deg[3] == 0.0f);
    CHECK(c.lock_us == CANARD_LOCK_US_LOCKED);

    c = servo_bench_cmd_at(1500);
    CHECK(c.active_fin == 0 && c.fin_deg[0] == -10.0f);

    c = servo_bench_cmd_at(2500);
    CHECK(c.active_fin == 0 && c.fin_deg[0] == 0.0f);

    c = servo_bench_cmd_at(3000);
    CHECK(c.active_fin == 1 && c.fin_deg[1] == 10.0f && c.fin_deg[0] == 0.0f);

    c = servo_bench_cmd_at(11999);
    CHECK(c.active_fin == 3 && c.fin_deg[3] == 0.0f);
    CHECK(c.lock_us == CANARD_LOCK_US_LOCKED);
}

static void test_lock_segment(void) {
    servo_bench_cmd_t c = servo_bench_cmd_at(12000);
    CHECK(c.active_fin == -1);
    CHECK(nonzero_fins(&c) == 0);
    CHECK(c.lock_us == CANARD_LOCK_US_UNLOCKED);

    c = servo_bench_cmd_at(15000);
    CHECK(c.active_fin == -1 && c.lock_us == CANARD_LOCK_US_LOCKED);
}

static void test_wraps(void) {
    servo_bench_cmd_t a = servo_bench_cmd_at(0);
    servo_bench_cmd_t b = servo_bench_cmd_at(18000);
    CHECK(memcmp(&a, &b, sizeof a) == 0);

    servo_bench_cmd_t m = servo_bench_cmd_at(UINT32_MAX);
    CHECK(nonzero_fins(&m) <= 1);
}

static void test_one_fin_at_a_time(void) {
    for (uint32_t t = 0; t < 18000u; t++) {
        servo_bench_cmd_t c = servo_bench_cmd_at(t);
        CHECK(nonzero_fins(&c) <= 1);
    }
}

static void test_socket_names(void) {
    CHECK(strcmp(servo_bench_socket_name(0), "J4 CH2") == 0);
    CHECK(strcmp(servo_bench_socket_name(3), "J4 CH5") == 0);
    CHECK(strcmp(servo_bench_socket_name(4), "?") == 0);
}

int main(void) {
    test_period();
    test_fin_segments();
    test_lock_segment();
    test_wraps();
    test_one_fin_at_a_time();
    test_socket_names();
    return test_report("servo_bench");
}

// Pad datum tracking: the first pad sample is withheld, later ones retarget QNH.
#include "test_assert.h"

#include "flight/baro_datum.h"

static void test_first_pad_sample_is_withheld(void)
{
    baro_datum_t d = {0};
    baro_datum_action_t a = baro_datum_on_sample(&d, true);
    CHECK(a.retarget_qnh);
    CHECK(!a.publish);
    CHECK(d.set);
}

static void test_later_pad_samples_publish_and_track(void)
{
    baro_datum_t d = {0};
    (void)baro_datum_on_sample(&d, true);
    baro_datum_action_t a = baro_datum_on_sample(&d, true);
    CHECK(a.retarget_qnh);
    CHECK(a.publish);
}

static void test_flight_freezes_datum(void)
{
    baro_datum_t d = {0};
    (void)baro_datum_on_sample(&d, true);
    baro_datum_action_t a = baro_datum_on_sample(&d, false);
    CHECK(!a.retarget_qnh);
    CHECK(a.publish);
}

static void test_no_pad_sample_keeps_datum_unset(void)
{
    baro_datum_t d = {0};
    baro_datum_action_t a = baro_datum_on_sample(&d, false);
    CHECK(a.publish);
    CHECK(!d.set);
}

int main(void)
{
    test_first_pad_sample_is_withheld();
    test_later_pad_samples_publish_and_track();
    test_flight_freezes_datum();
    test_no_pad_sample_keeps_datum_unset();
    return test_report("baro_datum");
}

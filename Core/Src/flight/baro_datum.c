#include "flight/baro_datum.h"

baro_datum_action_t baro_datum_on_sample(baro_datum_t *d, bool on_pad)
{
    baro_datum_action_t a = { .retarget_qnh = on_pad, .publish = true };

    /* The driver computes altitude at publish time against the QNH then in force,
     * so the first pad sample still carries the standard-atmosphere datum. */
    if (on_pad && !d->set) {
        d->set    = true;
        a.publish = false;
    }
    return a;
}

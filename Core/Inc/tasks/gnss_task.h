/*
 * ACS5 - GNSS task public interface.
 *
 * The fix snapshot, its mutex and the idle semaphore are private to
 * gnss_task.c. Other tasks read the fix only through gnss_get_fix().
 */
#pragma once

#include <stdbool.h>

#include "flight/flight_data.h"

/* FreeRTOS task entry point. */
void gnss_task(void *arg);

/*
 * gnss_get_fix - copy the latest published fix into *out.
 *
 * Never blocks. Returns false, leaving *out untouched, if the GNSS task has
 * not started yet or the snapshot is being written at this instant; the caller
 * should keep its previous value and try again on its next cycle.
 *
 * A true return only means *out is a consistent snapshot. Check out->fix_valid
 * and out->ned_valid before using the position, both are normal to be false.
 */
bool gnss_get_fix(gnss_fix_t *out);

/*
 * gnss_capture_datum - ask the task to latch the NED datum from the next
 * valid 3D fix. Call once from the flight state machine while on the pad.
 * Before the datum is latched, ned_m is all zeros and ned_valid is false.
 */
void gnss_capture_datum(void);

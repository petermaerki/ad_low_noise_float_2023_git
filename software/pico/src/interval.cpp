#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"

#include "interval.h"

void interval_reset(interval_t *status_counter)
{
    status_counter->counter = 0;
    status_counter->last_counter = 0;
    status_counter->last_time_us = time_us_64();
}

bool interval_over_SPS(interval_t *status_counter, const uint32_t interval_ms)
{
    uint64_t time_us = time_us_64();
    uint32_t duration_us = time_us - status_counter->last_time_us;
    if (duration_us < interval_ms)
    {
        return false;
    }
    printf("%s=%0.0f\n", status_counter->label, (1000000.0 * status_counter->counter) / duration_us);
    status_counter->last_time_us = time_us;
    status_counter->counter = 0;
    return true;
}

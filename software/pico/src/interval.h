#include <stdlib.h>

/*
'interval_over()' will return True whenever interval_us is over.

During an interval, 'counter' is incremented whenever 'fifo_full' or 'usb_error'.
*/
typedef struct
{
    // Actual counter
    uint32_t counter;
    // Counter when last signalled.
    uint32_t last_counter;
    // Time when last signalled.
    uint64_t last_time_us;
    uint32_t interval_us;
    const char *label;
} interval_t;

void interval_reset(interval_t *interval);

bool interval_over_SPS(interval_t *interval);

static inline void interval_init(interval_t *interval, const uint32_t interval_us, const char *label)
{
    interval->last_counter = 0;
    interval->last_time_us = 0;
    interval->interval_us = interval_us;
    interval->label = label;
}

static inline uint32_t interval_over(interval_t *interval)
{
    uint32_t duration_us = time_us_64() - interval->last_time_us;
    if (duration_us > interval->interval_us)
    {
        return duration_us;
    }
    return 0;
}

/*
Print to stdout if the counter incremented
*/
// static inline void interval_print_incremented(interval_t *interval)
// {
//     if (interval->counter > interval->last_counter)
//     {
//         if (interval_over(interval))
//         {
//             printf("%s=%d->%d\n", interval->label, interval->last_counter, interval->counter);
//             // interval->last_counter = interval->counter;
//             // interval->last_time_us = time_us_64();
//             interval_reset(interval);
//         }
//     }
// }


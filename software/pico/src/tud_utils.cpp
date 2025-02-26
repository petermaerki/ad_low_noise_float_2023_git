#include <stdlib.h>
// #include <stdint.h>
// #include <stdbool.h>
#include <tusb.h>

#include "tud_utils.h"

const uint32_t DURATION_S_US = 1000000;
const uint32_t DURATION_100MS_US = 100000;

bool tud_write_buf(uint8_t itf, const uint8_t *buf, uint32_t bytes)
{
    const uint8_t *p = buf;
    size_t remaining = bytes;
    uint64_t end_us = time_us_64() + DURATION_100MS_US;
    while (true)
    {
        if (remaining == 0)
        {
            return true;
        }
        uint32_t written = tud_cdc_n_write(itf, p, remaining);
        if (written > 0)
        {
            remaining -= written;
            p += written;
            continue;
        }
        if (time_us_64() > end_us)
        {
            return false;
        }
        tud_task();
    }
    return false;
}

bool tud_write_line(uint8_t itf, const char *p)
{
    bool success = tud_write_buf(itf, (uint8_t *)p, strlen(p));
    if (success)
    {
        tud_cdc_n_write_flush(itf);
    }
    return success;
}

bool tud_send_separator(uint8_t itf, errors_t *errors)
{
    uint8_t errors_lsb = errors->errors & 0xFF;
    uint8_t errors_msb = (errors->errors >> 8) & 0xFF;
    uint8_t separator[6] = {0x7F, 0xFF, 0xFF, errors_msb, errors_lsb, 0x42};
    separator[5] = errors->crc ^ (0x7F ^ 0xFF ^ 0xFF ^ errors_msb ^ errors_lsb);
    return tud_write_buf(itf, separator, sizeof(separator));
}

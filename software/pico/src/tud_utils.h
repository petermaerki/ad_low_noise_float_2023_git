#ifndef _TUD_UTILS_H_
#define _TUD_UTILS_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "protocol.h"

/*
Write one line to 'itf'
*/
bool tud_write_line(uint8_t itf, const char *line);

/*
Write one line to 'itf'.
Return true on success.
Times out after 100ms and returns false;
*/
bool tud_write_buf(uint8_t itf, const uint8_t *buf, uint32_t bytes);

bool tud_send_separator(uint8_t itf, errors_t *errors);

#endif /* _TUD_UTILS_H_ */
#include <stdlib.h>
#include <stdarg.h>

#include "ads127l21/ads127l21.h"

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

enum
{
    MODE_MEASURE = 0,
    MODE_STATUS,
    MODE_IDLE
};

// A sequence: the measurements between two separators
// 1000 measurements is about 33s
const uint16_t SEQUENCE_LEN_MIN = 1000;
// 30_000 measurements is about 1s
const uint32_t SEQUENCE_LEN_MAX = 32 * 1000;

const uint16_t ERROR_MOCKED = 0x01;                      // 1<<0
const uint16_t ERROR_ADS127_MOD = STATUS1_MOD_FLAG_MASK; // 1<<1
const uint16_t ERROR_ADS127_ADC = STATUS1_ADC_ERR_MASK;  // 1<<2
const uint16_t ERROR_FIFO = 0x01 << 3;
const uint16_t ERROR_ADS127_SPI = STATUS1_SPI_ERR_MASK;  // 1<<4
const uint16_t ERROR_ADS127_POR = STATUS1_POR_FLAG_MASK; // 1<<5
const uint16_t ERROR_ADS127_ALV = STATUS1_ALV_FLAG_MASK; // 1<<6
const uint16_t ERROR_OVLD = 0x01 << 7;
const uint16_t ERROR_STATUS_BITNUMBER_J42 = 8;
const uint16_t ERROR_STATUS_J42 = 0x01 << (ERROR_STATUS_BITNUMBER_J42);     // 1<<8
const uint16_t ERROR_STATUS_J43 = 0x01 << (ERROR_STATUS_BITNUMBER_J42 + 1); // 1<<9
const uint16_t ERROR_STATUS_J44 = 0x01 << (ERROR_STATUS_BITNUMBER_J42 + 2); // 1<<10
const uint16_t ERROR_STATUS_J45 = 0x01 << (ERROR_STATUS_BITNUMBER_J42 + 3); // 1<<11
const uint16_t ERROR_STATUS_J46 = 0x01 << (ERROR_STATUS_BITNUMBER_J42 + 4); // 1<<12

const uint8_t ERROR_BYTE_MASK_ADS127 = ERROR_ADS127_MOD | ERROR_ADS127_ADC | ERROR_ADS127_SPI | ERROR_ADS127_POR | ERROR_ADS127_ALV;

const uint16_t ERROR_STATUS_JXX_MASK = ERROR_STATUS_J42 | ERROR_STATUS_J43 | ERROR_STATUS_J44 | ERROR_STATUS_J45 | ERROR_STATUS_J46;

const uint8_t COMMAND_START = 's';
const uint8_t COMMAND_STOP = 'p';
const uint8_t COMMAND_RESET = 'r';
const uint8_t COMMAND_MOCKED_ERROR = 'e';
const uint8_t COMMAND_MOCKED_CRC = 'c';
const uint8_t COMMAND_QUEUE_LEN = 'q';

const size_t PROTOCOL_BUF_SIZE = 1024;

typedef struct
{
    uint8_t mode;
    char buf[PROTOCOL_BUF_SIZE];
} protocol_t;

inline void protocol_buf_reset(protocol_t *protocol)
{
    protocol->buf[0] = '\0';
}

inline void protocol_printf(protocol_t *protocol, const char *fmt, ...)
{
    int written;
    va_list args;

    size_t len = strlen(protocol->buf);
    char *start = &protocol->buf[len];
    size_t remaining = PROTOCOL_BUF_SIZE - len;
    va_start(args, fmt);
    written = vsnprintf(start, remaining, fmt, args);
    va_end(args);

    assert(len + written < PROTOCOL_BUF_SIZE);
}

typedef struct
{
    // Actual errors
    uint16_t errors;
    uint16_t errors_last;
    uint8_t crc;
    uint32_t current_sequence_len;
} errors_t;

static inline void errors_init(errors_t *errors)
{
    errors->errors = 0x00;
    errors->errors_last = 0x00;
    errors->crc = 0x00;
    errors->current_sequence_len = 0;
}

static inline void errors_reset(errors_t *errors)
{
    errors_init(errors);
}

static inline void update_crc_buf(errors_t *errors, uint8_t *buf, uint32_t size)
{
    while (size--)
    {
        errors->crc ^= *buf++;
    }
}

static inline void tweak_separator_special_case(uint8_t *buf, uint32_t size)
{
    if ((buf[0] == 0x7F) && (buf[1] == 0xFF) && (buf[2] == 0xFF))
    {
        // Avoid a measurement looking like a separator
        buf[2] = 0xFE;
    }
}

#endif /* PROTOCOL_H_ */

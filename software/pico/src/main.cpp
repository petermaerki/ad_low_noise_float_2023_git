#include <stdio.h>
#include <stdlib.h>
#include <bsp/board_api.h>
#include <tusb.h>
#include <pico/stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "ads127l21/ads127l21.h"
#include "interval.h"
#include "protocol.h"
#include "tud_utils.h"
#include "relais.h"

/*
DONE GPIO28 GPIO27 CLK
DONE - GPIO28 TX
DONE - GPIO29 RX
TODO - GPIO26 AD
TODO RELAIS
DODO Konfiguration
*/

/*
https://github.com/WeActStudio/LogicAnalyzerV1

SIGROK LogicAnalyzer
DAQ0-DRDY
DAQ1-SCLK
DAQ2-START
DAQ3-CLK_MODULATOR
DAQ4-READ_SPI
DAQ5-USB_CONNECTED
DAQ6-USB_WRITING
DAQ7-
*/
const uint16_t SPI_CLK_kHZ = 6250;
// const uint16_t SPI_CLK_kHZ = 12500;
static queue_t adc_measurement_fifo;
const int FIFO_LENGTH = 32768; // 8096, 16384, 32768
const int ADC_MEASUREMENT_BYTES = 3;
const int ADC_MEASUREMENT_RAW_BYTES = ADC_MEASUREMENT_BYTES + 1;
const int ADC_MEASUREMENT_RAW_BYTE_STATUS = 0;
const int ADC_MEASUREMENT_RAW_BYTE_MEAS0 = 1;
const int ADC_MEASUREMENT_RAW_BYTE_MEAS1 = 2;
const int ADC_MEASUREMENT_RAW_BYTE_MEAS2 = 3;

static errors_t errors;
static protocol_t protocol = {
    .mode = MODE_IDLE,
};
static bool usb_connected = false;
static bool usb_connected_before = false;

const uint32_t MS_US = 1000;                      // 1ms
const uint32_t S_US = 1000 * MS_US;               // 1s
const uint32_t SEPARATOR_STATUS_US = 100 * MS_US; // Maximum one message every 100 ms

static interval_t interval_fifo_full;
static interval_t interval_usb_write_failed;
static interval_t interval_sps;
static interval_t interval_separator;

static char command_line[64] = {0};
typedef struct
{
    uint8_t register_FILTER1;
    uint8_t register_MUX;
    uint8_t resolution22;
    uint32_t additional_SPI_reads;
    uint32_t sequence_len_max;
    bool update;
} adc_reset_t;
static adc_reset_t adc_reset = {
    .register_FILTER1 = FILTER1_FLTR_OSR_WIDEBAND_128,
    .register_MUX = MUX_DEFAULT,
    .resolution22 = 0,
    .additional_SPI_reads = 0,
    .sequence_len_max = SEQUENCE_LEN_MAX,
    .update = true,
};

/*
See ads127l21.pdf, 7.3.5.1.6 FIR3 Default Coefficients

The group delay (expressed in units of time) is constant
versus frequency, equal to 34 / fDATA.
empirically measured offset o 3: We use 37!
*/
static const uint16_t FILTER_DELAY = 37;
static uint8_t delay_IN_disable_t[FILTER_DELAY] = {};
static uint16_t delay_idx0 = FILTER_DELAY;

void custom_cdc_task(void);

/*
Return the state of the jumpers
J42: bit 0. Value is 1 if jumper is set.
J43: bit 1. Value is 1 if jumper is set.
*/
static uint32_t get_status_J42_J46()
{
    uint32_t gpio_all_32 = gpio_get_all();
    uint32_t value_J42 = gpio_all_32 >> PIN_J42; // Shift all bits to the right
    return ~value_J42 & 0b11111;
}

static void pwm_set_clkdiv2(uint slice_num, float divider)
{
    check_slice_num_param(slice_num);
    valid_params_if(HARDWARE_PWM, divider >= 1.f && divider < 256.f);
    volatile uint8_t i = (uint8_t)divider;
    volatile uint8_t f = (uint8_t)((divider - i) * (0x01 << 4));
    pwm_set_clkdiv_int_frac4(slice_num, i, f);
}

static void clk_modulator_init(void)
{
    gpio_init(PIN_CLK_MODULATOR);
    gpio_set_function(PIN_CLK_MODULATOR, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PIN_CLK_MODULATOR);
    pwm_set_clkdiv(slice_num, 3.0f);
    pwm_set_wrap(slice_num, 1);
    pwm_set_gpio_level(PIN_CLK_MODULATOR, 1);
    pwm_set_enabled(slice_num, true);
}

static inline void queue_clear(queue_t *q)
{
    // Clear queue
    q->wptr = 0;
    q->rptr = 0;
}

static void inline handle_relais()
{
    if (adc_hw->cs & ADC_CS_READY_BITS)
    {
        // See also (2.5us): uint16_t result = adc_read();
        // if (gpio_get(nDRDY_PIN))
        // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
        // const float conversion_factor = 3.3f / (1 << 12);
        // printf("Raw value: 0x%03x, voltage: %f V\n", result, result * conversion_factor);

        gpio_put(PIN_DAQ7, 1);
        uint16_t adc_12 = (uint16_t)adc_hw->result;

        volatile bool x = relais_capacitor_connected();
        if ((adc_12 > AD_grenze_12) && (!relais_capacitor_connected()))
        {
            // Supply high enough but relais still off
            relais_pulse(true);
            printf("Capacitor connected\n");
        };
        if ((adc_12 < AD_grenze_12) && (relais_capacitor_connected()))
        {
            // Supply to low but relais still on
            relais_pulse(false);
            printf("Capacitor disconnected\n");
        };
    }
    hw_set_bits(&adc_hw->cs, ADC_CS_START_ONCE_BITS);
    gpio_put(PIN_DAQ7, 0);
}

void core1_entry()
{
    while (true)
    {
        handle_relais();

        if (adc_reset.update)
        {
            adc_reset.update = false;
            writeSingleRegister(FILTER1_ADDRESS, adc_reset.register_FILTER1);
            writeSingleRegister(MUX_ADDRESS, adc_reset.register_MUX);
            // Reset all error flags
            writeSingleRegister(STATUS1_ADDRESS, 0xFF);
        }

        if (gpio_get(nDRDY_PIN))
        {
            // We are waiting for the next falling edge of nDRDY_PIN
            tight_loop_contents();
            continue;
        }
        // Falling edge of nDRDY_PIN

        //
        // Read ADC value
        //
        gpio_put(PIN_DAQ4, 1);
        // adc_channel_t adc;
        // readData(&adc);
        // adc.data &= ERROR_BYTE_MASK_ADS127;

        assert(STATUS_ENABLED);
        assert(!SPI_CRC_ENABLED);

        uint8_t adc_measurement_raw[ADC_MEASUREMENT_RAW_BYTES];
        int read = spi_read_blocking(SPI_PORT, 0x00, adc_measurement_raw, ADC_MEASUREMENT_RAW_BYTES);
        assert(read == ADC_MEASUREMENT_RAW_BYTES);

        if (adc_reset.resolution22)
        {
            // In labber mode, we only use 22bits of the 24bits of the adc.
            if ((adc_measurement_raw[ADC_MEASUREMENT_RAW_BYTE_MEAS0] == 0x7F) && (adc_measurement_raw[ADC_MEASUREMENT_RAW_BYTE_MEAS1] == 0xFF) && (adc_measurement_raw[ADC_MEASUREMENT_RAW_BYTE_MEAS2] > 0xFB))
            {
                // Measurements higher than `0x7F 0xFF 0xFB` will be clipped to `0x7F 0xFF 0xFB`.
                adc_measurement_raw[ADC_MEASUREMENT_RAW_BYTE_MEAS2] = 0xFB;
            }
            // The two last bits are overwritten by IN_diasable and IN_t.
            uint8_t actual_IN_disable_t = ((!gpio_get(PIN_J46_IN_t)) << 1) | (!gpio_get(PIN_J45_IN_disable));

            // Write into a circular buffer to delay IN_disable and IN_t for FILTER_DELAY samples.
            delay_idx0--;
            uint8_t delayed_IN_disable_t = delay_IN_disable_t[delay_idx0];
            delay_IN_disable_t[delay_idx0] = actual_IN_disable_t;
            if (delay_idx0 == 0)
            {
                delay_idx0 = FILTER_DELAY;
            }

            // Reset
            adc_measurement_raw[ADC_MEASUREMENT_RAW_BYTE_MEAS2] &= ~0b00000011;
            // Set
            adc_measurement_raw[ADC_MEASUREMENT_RAW_BYTE_MEAS2] |= delayed_IN_disable_t;
        }

        // nDRDY_PIN must have confirmed by now
        while (!gpio_get(nDRDY_PIN))
        {
            tight_loop_contents();
        }
        // assert(gpio_get(nDRDY_PIN));

        //
        // Send ADC value to core0 via queue
        //
        if (usb_connected)
        {
            bool success = queue_try_add(&adc_measurement_fifo, &adc_measurement_raw);
            if (success)
            {
                interval_sps.counter++;
            }
            else
            {
                interval_fifo_full.counter++;
                errors.errors |= ERROR_FIFO;

                queue_clear(&adc_measurement_fifo);
            }
        }
        else
        {
            queue_clear(&adc_measurement_fifo);
        }

        // The following loop provokes noise on the SPI traces
        for (uint32_t i = 0; i < adc_reset.additional_SPI_reads; i++)
        {
            uint8_t adc_measurement_raw[ADC_MEASUREMENT_RAW_BYTES];
            int read = spi_read_blocking(SPI_PORT, 0x00, adc_measurement_raw, ADC_MEASUREMENT_RAW_BYTES);
            assert(read == ADC_MEASUREMENT_RAW_BYTES);
        }

        gpio_put(PIN_DAQ4, 0);
    }
}

void check_endianness()
{
    volatile uint32_t val = 0x12345678;
    uint8_t *ptr = (uint8_t *)&val;

    val <<= 8;

    printf("First byte in memory: 0x%X\n", ptr[0]);

    if (ptr[0] == 0x78)
    {
        printf("System is Little-endian (LSB first).\n");
    }
    else if (ptr[0] == 0x12)
    {
        printf("System is Big-endian (MSB first).\n");
    }
    else
    {
        printf("Unknown endianness!\n");
    }
}

static inline void send_separator(errors_t *errors)
{
    if (errors->current_sequence_len < SEQUENCE_LEN_MIN)
    {
        // Avoid burst
        return;
    }
    if (errors->current_sequence_len < adc_reset.sequence_len_max)
    {
        // Need more data for the sequence
        if (errors->errors == errors->errors_last)
        {
            // No changes: No separator!
            return;
        }
    }
    printf("SEPARATOR(0x%02X)\n", errors->errors);
    tud_send_separator(ITF_MEASUREMENTS, errors);
    errors->crc = 0x00;
    // The error bits will be set by the error and cleared when sending the separator - here!
    // The status bits should remain!
    errors->errors_last = errors->errors; // Remember remaining ERROR_STATUS_JXX_MASK bits
    errors->current_sequence_len = 0;
    return;
}

/*
On maximal speed (SPS 97656) the sequence length SEQUENCE_LEN_MAX will
send a sequence at least every 300ms.

For all other speeds the sequence length should be adjusted
to a seqence every 300ms to be send.

*/
uint32_t calculate_sequence_len_max(uint32_t register_FILTER1)
{
    /* Register 0x09 (FILTER1) definition
     * |-----------------------------------------------------------------------------------------------|
     * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
     * |-----------------------------------------------------------------------------------------------|
     * |           FLTR_SEL[2:0]           |                       FLTR_OSR[4:0]                       |
     * |-----------------------------------------------------------------------------------------------|
     */

    uint8_t osr = register_FILTER1 & 0b11111;
    // Range of osr: [2..7] which correspondes to SPS [97656..3052] => sequence_len_max [SEQUENCE_LEN_MAX..1000]
    uint32_t sequence_len_max = (SEQUENCE_LEN_MAX << 2) >> osr;
    if (sequence_len_max > SEQUENCE_LEN_MAX)
    {
        return SEQUENCE_LEN_MAX;
    }
    if (sequence_len_max < SEQUENCE_LEN_MIN)
    {
        return SEQUENCE_LEN_MIN;
    }
    return sequence_len_max;
}

void execute_command_line()
{
    printf("COMMAND=%s\n", command_line);
    if (command_line[0] == COMMAND_START)
    {
        printf("CMD_START=1\n");
        interval_reset(&interval_sps);
        interval_reset(&interval_fifo_full);
        errors_reset(&errors);
        protocol.mode = MODE_MEASURE;
        return;
    }
    if (command_line[0] == COMMAND_RESET)
    {
        // See: components/ad/ADS127L21.pdf
        // first parameter: FLTR_OSR (Speed)
        // second paramete: MUX
        // Example 97_656SPS: r-02-00
        // Example 48_828SPS: r-03-00
        unsigned int register_FILTER1 = FILTER1_FLTR_OSR_WIDEBAND_128;
        unsigned int register_MUX = MUX_DEFAULT;
        unsigned int resolution22 = 0;
        unsigned int additional_SPI_reads = 0;
        int elements_converted = sscanf(command_line + 1, "-%x-%x-%d-%d", &register_FILTER1, &register_MUX, &resolution22, &additional_SPI_reads);
        adc_reset.register_FILTER1 = register_FILTER1;
        adc_reset.register_MUX = register_MUX;
        adc_reset.resolution22 = resolution22;
        adc_reset.additional_SPI_reads = additional_SPI_reads;
        adc_reset.sequence_len_max = calculate_sequence_len_max(adc_reset.register_FILTER1);
        adc_reset.update = true;

        protocol_buf_reset(&protocol);
        protocol_printf(&protocol, "BEGIN=1\n");
        protocol_printf(&protocol, "PROGRAM=" PICO_PROGRAM_NAME "(" PICO_PROGRAM_VERSION_STRING ")\n");
        protocol_printf(&protocol, "REGISTER_FILTER1=0x%02X\n", register_FILTER1);
        protocol_printf(&protocol, "REGISTER_MUX=0x%02X\n", register_MUX);
        protocol_printf(&protocol, "RESOLUTION22=%d\n", resolution22);
        protocol_printf(&protocol, "ADDITIONAL_SPI_READS=%d\n", additional_SPI_reads);
        protocol_printf(&protocol, "SEQUENCE_LEN_MIN=%d\n", SEQUENCE_LEN_MIN);
        protocol_printf(&protocol, "SEQUENCE_LEN_MAX=%d\n", adc_reset.sequence_len_max);
        protocol_printf(&protocol, "ERROR_MOCKED=0x%04x\n", ERROR_MOCKED);
        protocol_printf(&protocol, "ERROR_ADS127_MOD=0x%04x\n", ERROR_ADS127_MOD);
        protocol_printf(&protocol, "ERROR_ADS127_ADC=0x%04x\n", ERROR_ADS127_ADC);
        protocol_printf(&protocol, "ERROR_FIFO=0x%04x\n", ERROR_FIFO);
        protocol_printf(&protocol, "ERROR_ADS127_SPI=0x%04x\n", ERROR_ADS127_SPI);
        protocol_printf(&protocol, "ERROR_ADS127_POR=0x%04x\n", ERROR_ADS127_POR);
        protocol_printf(&protocol, "ERROR_ADS127_ALV=0x%04x\n", ERROR_ADS127_ALV);
        protocol_printf(&protocol, "ERROR_OVLD=0x%04x\n", ERROR_OVLD);
        protocol_printf(&protocol, "ERROR_STATUS_J42=0x%04x\n", ERROR_STATUS_J42);
        protocol_printf(&protocol, "ERROR_STATUS_J43=0x%04x\n", ERROR_STATUS_J43);
        protocol_printf(&protocol, "ERROR_STATUS_J44=0x%04x\n", ERROR_STATUS_J44);
        protocol_printf(&protocol, "ERROR_STATUS_J45=0x%04x\n", ERROR_STATUS_J45);
        protocol_printf(&protocol, "ERROR_STATUS_J46=0x%04x\n", ERROR_STATUS_J46);

        uint32_t status_J42_J46 = get_status_J42_J46();
        protocol_printf(&protocol, "STATUS_J42_J46=0b%05b\n", status_J42_J46);

        protocol_printf(&protocol, "END=1\n");
        printf(protocol.buf);
        protocol.mode = MODE_STATUS;
        return;
    }
    if (command_line[0] == COMMAND_STOP)
    {
        printf("CMD_STOP=1\n");
        protocol.mode = MODE_STATUS;
        return;
    }

    if (command_line[0] == COMMAND_MOCKED_ERROR)
    {
        printf("MOCKED_ERROR=1\n");
        errors.errors |= ERROR_MOCKED;
        return;
    }
    if (command_line[0] == COMMAND_MOCKED_CRC)
    {
        printf("MOCKED_CRC=1\n");
        errors.crc ^= 0x02;
        return;
    }
    if (command_line[0] == COMMAND_QUEUE_LEN)
    {
        printf("QUEUE_LEN=%d\n", queue_get_level(&adc_measurement_fifo));
        return;
    }
    printf("WARNING:unknown command '%s'\n", command_line);
}

void handle_input(uint8_t input)
{
    printf("input: 0x%02X\n", input);
    if ((input == '\r') || (input == '\n'))
    {
        if (command_line[0] != 0)
        {
            execute_command_line();
        }
        command_line[0] = '\0';
        return;
    }
    size_t len = strlen(command_line);
    if (len < sizeof(command_line) - 1)
    {
        command_line[len] = input;
        command_line[len + 1] = '\0';
    }
}

void uart_rx_task()
{
    if (uart_is_readable(uart_default))
    {
        volatile uint8_t command = uart_getc(uart_default);
        // printf("uart: %c(0x%02X)", command, command);
        handle_input(command);
    }
}

int main()
{
    interval_init(&interval_fifo_full, S_US, "FIFO_FULL");
    interval_init(&interval_usb_write_failed, S_US, "USB_WRITE_FAILED");
    interval_init(&interval_sps, 2 * S_US, "SPS");
    errors_init(&errors);

    // Initialize TinyUSB stack
    board_init();
    relais_init();

    //  init device stack on configured roothub port
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL,
    };
    bool success = tusb_init(BOARD_TUD_RHPORT, &dev_init);
    assert(success);

    // TinyUSB board init callback after init
    if (board_init_after_tusb)
    {
        board_init_after_tusb();
    }

    // let pico sdk use the first cdc interface for std io
    stdio_init_all();

    // assert(1 == 0);

    // check_endianness();

    // gpio_init(nRESET_PIN);
    // gpio_set_dir(nRESET_PIN, GPIO_OUT);
    // gpio_put(nRESET_PIN, 1);

    // sleep_ms(100);
    for (int i = 0; i < PIN_J_COUNT; i++)
    {
        gpio_init(i + PIN_J42);
        gpio_set_pulls(i + PIN_J42, true, false);
        gpio_set_dir(i + PIN_J42, GPIO_IN);
    }

    clk_modulator_init();
    gpio_init(PIN_DAQ4);
    gpio_set_dir(PIN_DAQ4, GPIO_OUT);
    gpio_init(PIN_DAQ5);
    gpio_set_dir(PIN_DAQ5, GPIO_OUT);
    gpio_init(PIN_DAQ6);
    gpio_set_dir(PIN_DAQ6, GPIO_OUT);
    gpio_init(PIN_DAQ7);
    gpio_set_dir(PIN_DAQ7, GPIO_OUT);

    gpio_init(PIN_OVLD_N);
    gpio_set_pulls(PIN_OVLD_N, false, false);
    gpio_set_dir(PIN_OVLD_N, GPIO_IN);

    spi_init(SPI_PORT, SPI_CLK_kHZ * 1000);
    // Set SPI format: 8 bits per transfer, polarity = 0, phase = 1
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(nCS_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(nCS_PIN, GPIO_OUT);
    gpio_put(nCS_PIN, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    InitADC();

    queue_init(&adc_measurement_fifo, ADC_MEASUREMENT_RAW_BYTES, FIFO_LENGTH);

    multicore_launch_core1(core1_entry);

    printf("STARTED\n");

    while (1)
    {
        // TinyUSB device task | must be called regurlarly
        tud_task();

        custom_cdc_task();

        uart_rx_task();
    }
}

void status_interval()
{
    if (uint32_t duration_us = interval_over(&interval_sps))
    {
        printf("SPS=%0.0f\n", (1000000.0 * interval_sps.counter) / duration_us);
        interval_reset(&interval_sps);
    }
}

void custom_cdc_task(void)
{
    static uint32_t counter = 0;
    const uint32_t big = 5000;
    const uint32_t seconds5 = 5000000;

    uint8_t adc_measurement_raw[ADC_MEASUREMENT_RAW_BYTES];
    bool adc_measurement_available = queue_try_remove(&adc_measurement_fifo, adc_measurement_raw);

    if (protocol.mode == MODE_IDLE)
    {
        interval_reset(&interval_fifo_full);
        return;
    }
    if (protocol.mode == MODE_STATUS)
    {
        tud_write_line(ITF_MEASUREMENTS, protocol.buf);
        protocol.mode = MODE_IDLE;
        return;
    }
    assert(protocol.mode == MODE_MEASURE);

    usb_connected = tud_cdc_n_connected(ITF_MEASUREMENTS);
    bool usb_connected_changed = usb_connected != usb_connected_before;
    usb_connected_before = usb_connected;
    gpio_put(PIN_DAQ5, usb_connected);
    if (usb_connected_changed)
    {
        printf("CONNECTED=%d\n", usb_connected);
    }
    if (!usb_connected)
    {
        return;
    }

    // Connected
    if (usb_connected_changed)
    {
        // First time after connection: Make sure to reset
        interval_reset(&interval_sps);
        interval_reset(&interval_fifo_full);
        errors_reset(&errors);
        // protocol.mode == MODE_IDLE;
    }

    if (!adc_measurement_available)
    {
        return;
    }

    status_interval();

    gpio_put(PIN_DAQ6, 1);

    //
    // Prepare measurement and send it
    //
    // Reset
    errors.errors &= !ERROR_BYTE_MASK_ADS127;
    // Set
    errors.errors |= adc_measurement_raw[0] & ERROR_BYTE_MASK_ADS127;

    // Reset
    errors.errors &= !ERROR_OVLD;
    // Set
    errors.errors |= gpio_get(PIN_OVLD_N) ? 0 : ERROR_OVLD;

    // Reset
    errors.errors &= !ERROR_STATUS_JXX_MASK;
    // Set
    errors.errors |= get_status_J42_J46() << ERROR_STATUS_BITNUMBER_J42;

    tweak_separator_special_case(&adc_measurement_raw[1], ADC_MEASUREMENT_BYTES);
    update_crc_buf(&errors, &adc_measurement_raw[1], ADC_MEASUREMENT_BYTES);

    //
    // Send measurement
    //
    tud_write_buf(ITF_MEASUREMENTS, &adc_measurement_raw[1], ADC_MEASUREMENT_BYTES);
    errors.current_sequence_len++;

    //
    // If required: Send a separator
    //
    send_separator(&errors);

    gpio_put(PIN_DAQ6, 0);
}

// callback when data is received on a CDC interface
void tud_cdc_rx_cb(uint8_t itf)
{
    assert(itf == ITF_MEASUREMENTS);

    uint8_t command[1024];
    uint32_t count = tud_cdc_n_read(itf, &command, sizeof(command));
    if (count == 0)
    {
        printf("WARNING: command of size 0\n");
        return;
    }
    // printf("tud_cdc_rx_cb: %d\n", count);
    for (uint32_t i = 0; i < count; i++)
    {
        handle_input(command[i]);
    }
}

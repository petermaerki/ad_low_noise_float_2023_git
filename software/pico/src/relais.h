/*

*/
#ifndef _RELAIS_H_
#define _RELAIS_H_

#include "pico/stdlib.h"

#define PIN_RELAIS_COUNT 4

#define PIN_RELAIS_RESET 22 // 22-25
#define PIN_RELAIS_SET 18   // 18-21
#define PIN_RELAIS_IS_CAPACITOR_CONNECTED 17
#define PIN_MASK_RELAIS_SET (0b1111 << PIN_RELAIS_SET)
#define PIN_MASK_RELAIS_RESET (0b1111 << PIN_RELAIS_RESET)

static const uint64_t RELAIS_PULSE_US = 3000;

static const float R211_ohm = 1000.0;
static const float R232_ohm = 1000.0;
static const float R233_ohm = 1000.0;
// info  spannung_9V_grenze_V = 9
static const float spannung_AD_grenze_V = 3.0;
static const float supply_Pi_V = 3.3;
static const float AD_grenze_V = R233_ohm / (R211_ohm + R232_ohm + R233_ohm) * spannung_AD_grenze_V;
static const uint16_t AD_grenze_12 = (2 << 12) * AD_grenze_V / supply_Pi_V;

static void inline relais_init()
{
    adc_init();
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    adc_select_input(0);
    // Read first value
    adc_read();

    gpio_init(PIN_RELAIS_IS_CAPACITOR_CONNECTED);
    gpio_set_pulls(PIN_RELAIS_IS_CAPACITOR_CONNECTED, true, false);
    gpio_set_dir(PIN_RELAIS_IS_CAPACITOR_CONNECTED, GPIO_IN);

    for (uint i = 0; i < PIN_RELAIS_COUNT; i++)
    {
        gpio_init(i + PIN_RELAIS_SET);
        gpio_init(i + PIN_RELAIS_RESET);
        gpio_put(i + PIN_RELAIS_SET, 0);
        gpio_put(i + PIN_RELAIS_RESET, 0);
        gpio_set_dir(i + PIN_RELAIS_SET, GPIO_OUT);
        gpio_set_dir(i + PIN_RELAIS_RESET, GPIO_OUT);
    }
}
static bool inline relais_capacitor_connected()
{
    return gpio_get(PIN_RELAIS_IS_CAPACITOR_CONNECTED);
}

static void inline relais_LOW(uint32_t mask)
{
    gpio_put_masked_n(0, mask, 0x0);
}
static void inline relais_HIGH(uint32_t mask)
{
    gpio_put_masked_n(0, mask, mask);
}

static void inline relais_pulse(bool connect_capacitor)
{
    relais_HIGH(connect_capacitor ? PIN_MASK_RELAIS_SET : PIN_MASK_RELAIS_RESET);

    sleep_us(RELAIS_PULSE_US);

    relais_LOW(PIN_MASK_RELAIS_SET);
    relais_LOW(PIN_MASK_RELAIS_RESET);
}

#endif /* _RELAIS_H_ */
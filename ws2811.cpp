#include <stdio.h>
#include <string.h> // memcpy

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

#include "ws2811.hpp"

/*
void __isr ws2811_isr() {
    if (pio_sm_is_rx_fifo_full(pio0, 0)) {
        printf("ERROR\n");
    }
    const uint value = pio_sm_get(pio0, 0);

    static uint ledIdx = 0;

    static uint throttle = 0;
    if (ledIdx == 0 && throttle++ != 100) {
        return;
    }

    printf("RX [%u] LED %u: ", time_us_32() / 1000, ledIdx);
    print_led_state(value);
    ledIdx = (ledIdx + 1) % ws2811_NUM_LEDS_TO_EMULATE;

    if (ledIdx == 0) {
        throttle = 0;
    }
}
*/

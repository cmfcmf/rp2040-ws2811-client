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

// const uint SIDESET_PIN = 12;

void core1_entry() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    while (true) {
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(200);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        sleep_ms(200);
    }
}

static void print_led_state(const LED led) {
    printf("%3u\t%3u\t%3u\n", led.colors.r, led.colors.g, led.colors.b);
}

int main()
{
    stdio_init_all();
    puts("Hello, world!");

    multicore_launch_core1(core1_entry);

    auto ws2811 = WS2811Client<NUM_LEDS_TO_EMULATE>();

    while (true) {
        const auto leds = ws2811.getLEDsAtomic();
        for (auto it = leds.begin(); it != leds.end(); it++) {
            printf("[%7u] LED %u: ", time_us_32() / 1000, std::distance(leds.begin(), it));
            print_led_state(*it);
        }
        sleep_ms(500);
        tight_loop_contents();
    }
}

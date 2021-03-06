#pragma once

#include <stdio.h>
#include "hardware/pio.h"

#include "pico-my-project.hpp"
#include "ws2811.pio.h"

union LED
{
  uint32_t value;
  struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
  } colors;
};

class WS2811Client {
private:
  PIO pio;
  uint offset;
  uint sm;
  pio_sm_config sm_conf;

  uint ws2811_dma_gather_chan;
  uint ws2811_dma_ctrl_chan;

  volatile uint32_t led_state[NUM_LEDS_TO_EMULATE];
  const volatile uint32_t *led_state_address;

  inline void initGPIO() {
    pio_sm_set_consecutive_pindirs(pio, sm, DATA_IN_PIN, 1, GPIO_IN);
    pio_gpio_init(pio, DATA_IN_PIN);

    pio_sm_set_consecutive_pindirs(pio, sm, SIDESET_PIN, 1, GPIO_OUT);
    pio_gpio_init(pio, SIDESET_PIN);

    pio_sm_set_consecutive_pindirs(pio, sm, DATA_OUT_PIN, 1, GPIO_OUT);
    pio_gpio_init(pio, DATA_OUT_PIN);
  }

  inline void initSMConfig() {
    sm_conf = ws2811_program_get_default_config(offset);

    sm_config_set_in_pins(&sm_conf, DATA_IN_PIN);
    sm_config_set_jmp_pin(&sm_conf, DATA_IN_PIN);
    sm_config_set_out_pins(&sm_conf, DATA_OUT_PIN, 1);
    sm_config_set_set_pins(&sm_conf, DATA_OUT_PIN, 1);

    sm_config_set_sideset_pins(&sm_conf, SIDESET_PIN);

    sm_config_set_in_shift(&sm_conf, false, true, 24); // shift left, auto push after 24 bit
    sm_config_set_out_shift(&sm_conf, false, false, 0); // shift left, no auto pull
    sm_config_set_fifo_join(&sm_conf, PIO_FIFO_JOIN_RX);

    sm_config_set_clkdiv(&sm_conf, 1);
  }

  inline void initDMA() {
    dma_channel_config dma_ctrl_conf = dma_channel_get_default_config(ws2811_dma_ctrl_chan);
    dma_channel_config dma_gather_conf = dma_channel_get_default_config(ws2811_dma_gather_chan);

    // CTRL
    {
      channel_config_set_transfer_data_size(&dma_ctrl_conf, DMA_SIZE_32);
      channel_config_set_read_increment(&dma_ctrl_conf, false);
      channel_config_set_write_increment(&dma_ctrl_conf, false);
      dma_channel_configure(
        ws2811_dma_ctrl_chan,
        &dma_ctrl_conf,
        &dma_hw->ch[ws2811_dma_gather_chan].al2_write_addr_trig, // write
        &led_state_address,                                      // read
        1,
        false
      );
    }

    // GATHER
    {
      channel_config_set_transfer_data_size(&dma_gather_conf, DMA_SIZE_32);
      channel_config_set_read_increment(&dma_gather_conf, false);
      channel_config_set_write_increment(&dma_gather_conf, true);
      channel_config_set_dreq(&dma_gather_conf, pio_get_dreq(pio, sm, false));
      channel_config_set_chain_to(&dma_gather_conf, ws2811_dma_ctrl_chan);
      dma_channel_configure(
        ws2811_dma_gather_chan,
        &dma_gather_conf,
        &led_state[0], // write
        &pio->rxf[sm], // read
        NUM_LEDS_TO_EMULATE,
        false
      );
    }

    dma_channel_start(ws2811_dma_ctrl_chan);
  }

  inline void runSM() {
    pio_sm_clear_fifos(pio, sm);
    pio_sm_init(pio, sm, offset, &sm_conf);

    // init
    pio_sm_exec_wait_blocking(pio, sm, pio_encode_set(pio_y, 20));
    pio_sm_exec_wait_blocking(pio, sm, pio_encode_mov(pio_osr, pio_y));
    pio_sm_exec_wait_blocking(pio, sm, pio_encode_out(pio_null, 5));

    // wait for first reset pulse
    bool reset_finished = false;
    while (!reset_finished) {
      while (gpio_get(DATA_IN_PIN))
        ;

      const auto us = time_us_32();
      reset_finished = true;
      while (time_us_32() - us < 10) {
        if (gpio_get(DATA_IN_PIN)) {
          reset_finished = false;
          break;
        }
        tight_loop_contents();
      }
    }
    pio_sm_set_enabled(pio, sm, true);
  }

public:
  WS2811Client() : led_state_address(&led_state[0]) {
    pio = pio0;
    offset = pio_add_program(pio, &ws2811_program);
    sm = pio_claim_unused_sm(pio, true);
    ws2811_dma_gather_chan = dma_claim_unused_channel(true);
    ws2811_dma_ctrl_chan = dma_claim_unused_channel(true);

    initGPIO();
    initSMConfig();
    initDMA();
    runSM();
  }

  const LED getLED(uint idx) const {
    const auto val = led_state[idx];

    return {
      .colors = {
        .r = (uint8_t)((val >>  8) & 0xFF),
        .g = (uint8_t)((val >> 16) & 0xFF),
        .b = (uint8_t)((val >>  0) & 0xFF)
      }
    };
  }

  const uint getNumLEDs() const {
    return NUM_LEDS_TO_EMULATE;
  }
};
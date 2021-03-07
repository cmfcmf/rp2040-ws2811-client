# RP2040 WS2811/WS2812/WS2812B Client

A proof of concept of a WS2811/WS2812/WS2812B client implementation for the [RP2040](https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf), the microcontroller powering the new(ish) [Raspberry Pi Pico](https://www.raspberrypi.org/products/raspberry-pi-pico/).
The [WS2811](https://cdn-shop.adafruit.com/datasheets/WS2811.pdf) is a controller IC that understands the WS2811 protocol and drives an RGB led.
WS2812/WS2812B are RGB leds with integrated WS2811 controllers.

The timing of the protocol is somewhat loosely defined.
More information on the protocol can be found [here](https://cpldcpu.wordpress.com/2014/01/14/light_ws2812-library-v2-0-part-i-understanding-the-ws2812/).

This implementation uses the _Programmable IO_ (PIO) interface in combination with DMA to run the implementation in parallel to the other microcontroller operations.
It currently requires 31 PIO instructions and 2 DMA channels.

The code supports the emulation of 1 to 10 WS2811 chips.

## Hardware

Keep in mind that the RP2040 is a 3,3V microcontroller, whereas the WS2811 protocol uses at least 5V.
This means that the data input and output lines cannot be directly connected to the RP2040.

In my testing, I used two resistors as a voltage divider to convert the input signal from 5V to 3,3V:

```
DATA_IN o---| 680 |---o---o RP2040
                      |
                      #
                      # 1k
                      #
                      |
                     GND
```

For the output signal, I used one channel of a 74HCT244 buffer IC to convert the 3,3V signal from the RP2040 back to the 5V signal required for the next WS2811.

## Other Options

1. **Using an of-the-shelf WS2811 controller and measuring its PWM outputs.**
   This approach works somewhat, but is not straight forward to implement, since the WS2811 controllers do not generate a linear PWM waveform.
   There appears to not be any documentation on the exact PWM waveform, which makes the measurment a guessing game.
2. **Using AVR microcontrollers.**
   AVR microcontrollers are too slow to measure and pass through the WS2811 signal.
3. **Using RP2040 bit-banging.**
   If you do not want to use the PIO, you could also implement this using bit-banging.
   This would mean higher CPU overhead, but improve flexibility.

## License

MIT
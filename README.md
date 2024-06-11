# rpi5-ws2812x-fu

Fork of https://github.com/praktronics/rpi5-rp1-spi

Using the SPI on the Raspberry Pi 5 through direct register control on the RP1 to drive a WS2812x LED strip.

To build and run:
```bash
    /rpi5-ws2812x-fu $ mkdir build
    /rpi5-ws2812x-fu $ cd build
    /rpi5-ws2812x-fu/build $ cmake ..
    /rpi5-ws2812x-fu/build $ cmake --build .
```
run with sudo or as root
```bash
    /rpi5-ws2812x-fu/build/bin $ sudo ./rpi5-rp1-spi
```
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

To compile a JNI `.so` library:

```bash
g++ -c -fPIC -I/usr/lib/jvm/default-java/include -I/usr/lib/jvm/default-java/include/linux rpi5-ws2812x-fu-jni.c -o jezor_jni_RPi5RP1SPI.o
```

```bash
g++ -shared -fPIC -o librpi5ws2812xfu.so jezor_jni_RPi5RP1SPI.o -lc
```

This will match the following Java class:

```java
package jezor.jni;

public class RPi5RP1SPI {

    public native void initializeStrip(int ledsCount);
    public native int getLedsCount();
    public native int getGreenComponent(int ledIndex);
    public native int getRedComponent(int ledIndex);
    public native int getBlueComponent(int ledIndex);
    public native void setLed(int ledIndex, int red, int green, int blue);
    public native void renderStrip();
    public native void closeStrip();

}
```
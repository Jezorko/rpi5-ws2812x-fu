# rpi5-ws2812x-fu

Fork of https://github.com/praktronics/rpi5-rp1-spi

Using the SPI on the Raspberry Pi 5 through direct register control on the RP1 to drive a WS2812x LED strip.

To compile a JNI `.so` library use the `build.bash` script.

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
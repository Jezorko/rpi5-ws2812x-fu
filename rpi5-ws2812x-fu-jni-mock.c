#include <jni.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

extern "C"
{

int data_length = 0;
uint8_t* data;

JNIEXPORT void JNICALL Java_jezor_jni_RPi5RP1SPI_initializeStrip(JNIEnv* env, jobject thisObject, jint leds_count) {
    data_length = leds_count * 3;
    data = (uint8_t*) malloc(leds_count * 3);
}

JNIEXPORT jint JNICALL Java_jezor_jni_RPi5RP1SPI_getLedsCount(JNIEnv* env, jobject thisObject) {
    return data_length / 3;
}

JNIEXPORT jint JNICALL Java_jezor_jni_RPi5RP1SPI_getGreenComponent(JNIEnv* env, jobject thisObject, jint led_index) {
    return data[led_index * 3];
}

JNIEXPORT jint JNICALL Java_jezor_jni_RPi5RP1SPI_getRedComponent(JNIEnv* env, jobject thisObject, jint led_index) {
    return data[led_index * 3 + 1];
}

JNIEXPORT jint JNICALL Java_jezor_jni_RPi5RP1SPI_getBlueComponent(JNIEnv* env, jobject thisObject, jint led_index) {
    return data[led_index * 3 + 2];
}

JNIEXPORT void JNICALL Java_jezor_jni_RPi5RP1SPI_setLed(JNIEnv* env, jobject thisObject, jint led_index, jint red, jint green, jint blue) {
    int data_index = led_index * 3;
    data[data_index] = green;
    data[data_index + 1] = red;
    data[data_index + 2] = blue;
}

JNIEXPORT void JNICALL Java_jezor_jni_RPi5RP1SPI_renderStrip(JNIEnv* env, jobject thisObject) {
    printf("Rendering:");
    for (int i = 0; i < data_length; ++i) {
        printf(" 0x%02x", data[i]);
    }
    printf("\n");
}

JNIEXPORT void JNICALL Java_jezor_jni_RPi5RP1SPI_closeStrip(JNIEnv* env, jobject thisObject) {
    free(data);
}

}
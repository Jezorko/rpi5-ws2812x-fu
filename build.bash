#!/usr/bin/env bash

outputName='librpi5ws2812xfu'
objectFileName="${outputName}.o"
sharedObjectFileName="${outputName}.so"

defaultJavaDirectory='/usr/lib/jvm/default-java'
if [ ! -d "${defaultJavaDirectory}" ]; then
  defaultJavaDirectory="/usr/lib/jvm/default"
fi

g++ -c -fPIC \
  "-I${defaultJavaDirectory}/include" \
  "-I${defaultJavaDirectory}/include/linux" \
  rpi5-ws2812x-fu-jni.c -o "${objectFileName}" \
  || exit "${?}"

g++ -shared -fPIC \
  -o "${sharedObjectFileName}" \
  "${objectFileName}" -lc \
  || exit "${?}"

rm "${objectFileName}"
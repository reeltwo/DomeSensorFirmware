TARGET=ProMini
PORT=/dev/ttyUSB0
GITHUB_REPOS=\
reeltwo/Reeltwo \
adafruit/Adafruit_MCP4725
include ../Arduino.mk

ifeq ("$(FULLSIZE)","1")
ARDUINO_OPTS+='-prefs="compiler.cpp.extra_flags=-DDOME_SENSOR_RING_FULL_SIZE=1"'
endif

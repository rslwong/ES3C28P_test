# ===========================================================================
# ES3C28P_cam_idf - ESP-IDF build of the Tapo camera viewer
#
# Self-contained like the Arduino sketches: ESP-IDF and its toolchain install
# UNDER this directory (.esp-idf/, .espressif/), and the app's libraries are
# vendored in components/. Copy the folder anywhere and run:
#
#   make deps      # one-time: fetch ESP-IDF + the esp32s3 toolchain (~4 GB)
#   make flash     # build and upload
#   make monitor   # serial monitor (Ctrl-] to quit)
#   make test      # host check of the torn-image detector
#
# Override the serial port if auto-detect misses it:
#   make flash PORT=/dev/cu.usbmodem1101
# ===========================================================================

IDF_VER := v5.5.6          # same v5.5 line the ESP32 Arduino core 3.3.x is built on
SHELL   := /bin/bash
export IDF_PATH       := $(CURDIR)/.esp-idf
export IDF_TOOLS_PATH := $(CURDIR)/.espressif
IDF  := source $(IDF_PATH)/export.sh >/dev/null && idf.py
PORT ?= $(firstword $(wildcard /dev/cu.usbmodem* /dev/ttyACM*))

.PHONY: all deps build flash monitor clean test

all: build

## deps: install ESP-IDF + toolchain locally (run once)
deps:
	test -d $(IDF_PATH) || git clone --depth 1 --branch $(strip $(IDF_VER)) \
	    --recursive --shallow-submodules https://github.com/espressif/esp-idf.git $(IDF_PATH)
	$(IDF_PATH)/install.sh esp32s3
	rm -rf $(IDF_TOOLS_PATH)/dist      # downloaded archives, ~0.5 GB

build:
	$(IDF) build

flash:
	$(IDF) -p $(PORT) flash

monitor:
	$(IDF) -p $(PORT) monitor

clean:
	$(IDF) fullclean

## test: host-side check of the torn-JPEG detector (plain C compiler, no ESP-IDF)
test:
	mkdir -p build
	cc -Wall -Wextra -Imain -o build/jpegcheck_test test/jpegcheck_test.c main/jpegcheck.c
	build/jpegcheck_test

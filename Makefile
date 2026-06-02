# ===========================================================================
# ES3C28P graphics + touch demo - build & flash with arduino-cli
#
# Everything is self-contained: the ESP32 core, toolchain and all libraries
# are installed UNDER this directory (see arduino-cli.yaml). Nothing outside
# this folder is referenced.
#
# Typical use:
#   make deps      # one-time: download core + libraries into ./.arduino
#   make flash     # compile and upload to the board
#   make monitor   # open the serial monitor
#
# Override the serial port if auto-detect misses it:
#   make flash PORT=/dev/cu.usbmodem1101
# ===========================================================================

CONFIG    := arduino-cli.yaml
ACLI      := arduino-cli --config-file $(CONFIG)
SKETCH    := ES3C28P_demo
BUILD_DIR := build

# ESP32-S3 with USB-CDC serial on boot, 16MB flash, OPI PSRAM (N16R8).
FQBN := esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB

# Auto-detect the board's serial port (override on the command line: PORT=...)
PORT ?= $(shell $(ACLI) board list 2>/dev/null | awk '/USB/ {print $$1; exit}')

CORE_DIR := .arduino/data/packages/esp32

.PHONY: all deps build upload flash monitor ports clean distclean

all: build

## deps: install ESP32 core + libraries locally (run once)
deps:
	$(ACLI) core update-index
	$(ACLI) core install esp32:esp32
	$(ACLI) lib install "Adafruit ILI9341" "Adafruit GFX Library" "Adafruit BusIO"

# Auto-run deps the first time if the core isn't there yet.
$(CORE_DIR):
	$(MAKE) deps

## build: compile the sketch
build: | $(CORE_DIR)
	$(ACLI) compile --fqbn $(FQBN) --build-path $(BUILD_DIR) $(SKETCH)

## upload: flash the already-compiled binary to the board
upload:
	@test -n "$(PORT)" || { echo "ERROR: no serial port found. Plug in the board or run 'make upload PORT=/dev/cu.xxx' (see 'make ports')."; exit 1; }
	$(ACLI) upload --fqbn $(FQBN) --port $(PORT) --input-dir $(BUILD_DIR) $(SKETCH)

## flash: compile + upload
flash: build upload

## monitor: open the serial monitor at 115200 baud
monitor:
	@test -n "$(PORT)" || { echo "ERROR: no serial port found. Run 'make monitor PORT=/dev/cu.xxx' (see 'make ports')."; exit 1; }
	$(ACLI) monitor --port $(PORT) --config baudrate=115200

## ports: list attached boards / serial ports
ports:
	$(ACLI) board list

## clean: remove build artifacts
clean:
	rm -rf $(BUILD_DIR)

## distclean: also remove the locally-installed core + libraries
distclean: clean
	rm -rf .arduino

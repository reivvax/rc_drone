# Run from the repo root. Override variables like: make flash PORT=/dev/ttyACM1
SHELL    := /bin/bash
IDF_PATH ?= $(HOME)/esp/esp-idf
PORT     ?= /dev/ttyACM0
FW_DIR   ?= firmware/camera_hello
PYTHON   ?= uv run

.PHONY: flash capture

## Rebuild the firmware and flash it to the board
flash:
	. $(IDF_PATH)/export.sh > /dev/null && idf.py -C $(FW_DIR) -p $(PORT) build flash

## Receive and display camera frames (no saving). Extra flags: make capture ARGS="--video"
capture:
	$(PYTHON) tools/serial_capture.py --port $(PORT) $(ARGS)

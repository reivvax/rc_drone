# Run from the repo root. Override variables like: make flash PORT=/dev/ttyACM1
SHELL    := /bin/bash
IDF_PATH ?= $(HOME)/esp/esp-idf
PORT     ?= /dev/ttyACM0
FW_DIR   ?= firmware/camera_hello
PYTHON   ?= uv run
CAM_HOST ?= 10.172.151.168

.PHONY: flash capture capture-wifi menuconfig monitor view

## Rebuild the firmware and flash it to the board
flash:
	. $(IDF_PATH)/export.sh > /dev/null && idf.py -C $(FW_DIR) -p $(PORT) build flash

## Receive and display camera frames (no saving). Extra flags: make capture ARGS="--video"
capture:
	$(PYTHON) tools/serial_capture.py --port $(PORT) $(ARGS)

## Same viewer for the Wi-Fi stream:  make capture-wifi [CAM_HOST=...] ARGS="--aruco --video"
capture-wifi:
	$(PYTHON) tools/serial_capture.py --wifi $(CAM_HOST) $(ARGS)

## Open the firmware's configuration menu (Wi-Fi credentials, frame size, ...)
##   make menuconfig FW_DIR=firmware/camera_wifi
menuconfig:
	. $(IDF_PATH)/export.sh > /dev/null && idf.py -C $(FW_DIR) menuconfig

## Show the board's log over USB (Ctrl+] to exit)
monitor:
	. $(IDF_PATH)/export.sh > /dev/null && idf.py -C $(FW_DIR) -p $(PORT) monitor

## Low-latency view of the Wi-Fi stream:  make view [CAM_HOST=192.168.x.y]
view:
	ffplay -hide_banner -fflags nobuffer -flags low_delay -framedrop -window_title xiao-cam http://$(CAM_HOST):81/stream

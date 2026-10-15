# Kagafei ES2832AA2 (nRF52832) insect camera — build / flash / receive.
#
# west must be run from inside the NCS workspace, so every west target cd's
# into $(NCS_DIR) and points back at this directory with -d/--build-dir.
# Override any variable on the command line, e.g.:
#   make build NCS_VERSION=v3.4.0 TOOLCHAIN_ID=ccc010f809

NCS_ROOT      ?= /opt/nordic/ncs
NCS_VERSION   ?= v3.4.0
TOOLCHAIN_ID  ?= ccc010f809
NCS_DIR       := $(NCS_ROOT)/$(NCS_VERSION)
TOOLCHAIN_DIR := $(NCS_ROOT)/toolchains/$(TOOLCHAIN_ID)

BOARD     ?= Kagafei_ES2832AA2/nrf52832
APP_DIR   := $(CURDIR)
BUILD_DIR ?= $(APP_DIR)/build
# runners.yaml defaults to nrfutil; this board is flashed with a J-Link.
RUNNER    ?= jlink

VENV   ?= $(APP_DIR)/.venv
PYTHON ?= python3
PY     := $(VENV)/bin/python

export PATH                   := $(TOOLCHAIN_DIR)/bin:/usr/local/bin:$(PATH)
export ZEPHYR_BASE            := $(NCS_DIR)/zephyr
export ZEPHYR_SDK_INSTALL_DIR := $(TOOLCHAIN_DIR)/opt/zephyr-sdk

WEST := cd $(NCS_DIR) && west

.DEFAULT_GOAL := help
.PHONY: help check build pristine flash rtt menuconfig ram rom clean \
        venv receive video distclean

help: ## Show this help
	@grep -E '^[a-zA-Z_-]+:.*## ' $(MAKEFILE_LIST) | \
	  awk 'BEGIN {FS = ":.*## "}; {printf "  \033[36m%-11s\033[0m %s\n", $$1, $$2}'

check: ## Verify the NCS install, toolchain and J-Link tools are present
	@test -d $(NCS_DIR)       || { echo "missing NCS: $(NCS_DIR)"; exit 1; }
	@test -d $(TOOLCHAIN_DIR) || { echo "missing toolchain: $(TOOLCHAIN_DIR)"; exit 1; }
	@command -v west     >/dev/null || { echo "west not on PATH"; exit 1; }
	@command -v JLinkExe >/dev/null || { echo "JLinkExe not found (install SEGGER J-Link)"; exit 1; }
	@echo "OK: NCS $(NCS_VERSION), toolchain $(TOOLCHAIN_ID), $$(west --version)"

build: ## Incremental firmware build
	$(WEST) build -p auto -b $(BOARD) -d $(BUILD_DIR) $(APP_DIR)

pristine: ## Clean rebuild of the firmware
	$(WEST) build -p always -b $(BOARD) -d $(BUILD_DIR) $(APP_DIR)

flash: build ## Build, then flash over J-Link
	$(WEST) flash -r $(RUNNER) -d $(BUILD_DIR)

rtt: ## Open the RTT console (printk/log output) over J-Link
	$(WEST) rtt -r $(RUNNER) -d $(BUILD_DIR)

menuconfig: ## Interactive Kconfig editor for the app image
	$(WEST) build -d $(BUILD_DIR) -t menuconfig

ram: ## RAM usage report
	$(WEST) build -d $(BUILD_DIR) -t ram_report

rom: ## Flash usage report
	$(WEST) build -d $(BUILD_DIR) -t rom_report

clean: ## Delete the build directory
	rm -rf $(BUILD_DIR)

venv: $(PY) ## Create .venv and install receiver dependencies

$(PY): requirements.txt
	$(PYTHON) -m venv $(VENV)
	$(PY) -m pip install --upgrade pip
	$(PY) -m pip install -r requirements.txt
	@touch $(PY)

receive: $(PY) ## Run the BLE receiver with the live image view
	$(PY) save_to_file_bt.py

video: $(PY) ## Run the BLE receiver that records to a video file
	$(PY) save_to_file_bt_video.py

distclean: clean ## Delete the build directory and .venv
	rm -rf $(VENV) __pycache__

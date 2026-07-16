# OpenEMS: STM32H562 Firmware Build System
# BOARD=rgt6 (default LQFP64) | BOARD=vgt6 (LQFP100 GPIOE pinout)
# Quality: WERROR=1, LINT_ERROR=0|1, make ci-local / secrets-check / format

.PHONY: all clean host-test firmware firmware-rgt6 firmware-vgt6 help \
        secrets-check lint-includes format format-all format-check ci-local

COMPILER_ARM = arm-none-eabi-g++
OBJCOPY_ARM = arm-none-eabi-objcopy
CXX_HOST ?= g++
CLANG_FORMAT ?= clang-format
PYTHON ?= python3

# Opt-in -Werror (ci-local Stage1 sets WERROR=1)
WERROR ?= 0
ifeq ($(WERROR),1)
  WERROR_FLAG = -Werror
else
  WERROR_FLAG =
endif

# Lint: default warn-only until layering PRs; LINT_ERROR=1 to fail
LINT_ERROR ?= 0
LINT_PHASE ?= A

BOARD ?= rgt6
ifeq ($(BOARD),vgt6)
  BOARD_CFLAGS = -DEMS_BOARD_VGT6
  BOARD_LABEL  = VGT6
  BIN_SUFFIX   = -vgt6
else
  BOARD_CFLAGS = -DEMS_BOARD_RGT6
  BOARD_LABEL  = RGT6
  BIN_SUFFIX   = -rgt6
endif

CFLAGS_COMMON = -std=c++17 -Wall -Wextra $(WERROR_FLAG)
CFLAGS_ARM = $(CFLAGS_COMMON) -DTARGET_STM32H562 -DNDEBUG -mcpu=cortex-m33 -mthumb \
             -fno-exceptions -fno-rtti -ffunction-sections -fdata-sections \
             -g0 -O2 -I./src $(BOARD_CFLAGS)
# -I. so test/*.cpp can #include "test/harness.h"
CFLAGS_HOST = $(CFLAGS_COMMON) -DEMS_HOST_TEST -DEMS_BOARD_RGT6 -O2 -g -I. -I./src

SRC_DIR = src
TEST_DIR = test
BUILD_DIR = /tmp/openems-build
BIN_DIR = $(BUILD_DIR)/bin
OBJ_DIR = $(BUILD_DIR)/obj/$(BOARD)
ELF_DIR = $(BUILD_DIR)/elf
HOST_DIR = $(BUILD_DIR)/host
LINKER_SCRIPT = linker/stm32h562.ld
FIRMWARE_ELF = $(ELF_DIR)/openems$(BIN_SUFFIX).elf
FIRMWARE_HEX = $(BIN_DIR)/openems$(BIN_SUFFIX).hex
FIRMWARE_BIN = $(BIN_DIR)/openems$(BIN_SUFFIX).bin
# Convenience aliases without suffix
FIRMWARE_BIN_ALIAS = $(BIN_DIR)/openems.bin
LDFLAGS_ARM = -mcpu=cortex-m33 -mthumb -nostartfiles \
              -Wl,--gc-sections -Wl,-Map=$(ELF_DIR)/openems$(BIN_SUFFIX).map \
              -T$(LINKER_SCRIPT)

ENGINE_SRC = $(SRC_DIR)/engine/calibration.cpp \
             $(SRC_DIR)/engine/engine_config.cpp \
             $(SRC_DIR)/engine/fuel_calc.cpp $(SRC_DIR)/engine/fuel_trim.cpp $(SRC_DIR)/engine/ign_calc.cpp \
             $(SRC_DIR)/engine/knock.cpp $(SRC_DIR)/engine/auxiliaries.cpp \
             $(SRC_DIR)/engine/table3d.cpp $(SRC_DIR)/engine/quick_crank.cpp \
             $(SRC_DIR)/engine/transient_fuel.cpp \
             $(SRC_DIR)/engine/ecu_sched.cpp \
             $(SRC_DIR)/engine/ecu_sched_angle.cpp \
             $(SRC_DIR)/engine/diagnostic_manager.cpp \
             $(SRC_DIR)/engine/map_estimator.cpp \
             $(SRC_DIR)/engine/output_test.cpp \
             $(SRC_DIR)/engine/xtau_autocalib.cpp \
             $(SRC_DIR)/engine/etb_control.cpp \
             $(SRC_DIR)/engine/torque_manager.cpp \
             $(SRC_DIR)/engine/misfire_detect.cpp \
             $(SRC_DIR)/engine/ewg_control.cpp

DRV_SRC = $(SRC_DIR)/drv/ckp.cpp $(SRC_DIR)/drv/sensors.cpp
APP_SRC = $(SRC_DIR)/app/ui_protocol.cpp \
          $(SRC_DIR)/app/ui_protocol_state.cpp \
          $(SRC_DIR)/app/ui_protocol_pages.cpp \
          $(SRC_DIR)/app/ui_protocol_envelope.cpp \
          $(SRC_DIR)/app/can_stack.cpp \
          $(SRC_DIR)/app/can_rx_map.cpp \
          $(SRC_DIR)/app/datalog.cpp \
          $(SRC_DIR)/app/nvm_boot.cpp \
          $(SRC_DIR)/app/vehicle_inputs_bridge.cpp
HAL_COMMON_SRC = $(SRC_DIR)/hal/adc.cpp $(SRC_DIR)/hal/can.cpp \
                  $(SRC_DIR)/hal/uart.cpp $(SRC_DIR)/hal/flash.cpp \
                  $(SRC_DIR)/hal/etb_driver.cpp $(SRC_DIR)/hal/tle8888.cpp \
                  $(SRC_DIR)/hal/ewg_driver.cpp \
                  $(SRC_DIR)/hal/flex_fuel.cpp \
                  $(SRC_DIR)/hal/sdmmc.cpp
HAL_STM32H562_SRC = $(SRC_DIR)/hal/stm32h562/system.cpp \
                    $(SRC_DIR)/hal/stm32h562/timer.cpp \
                    $(SRC_DIR)/hal/stm32h562/usb_cdc.cpp \
                    $(SRC_DIR)/hal/stm32h562/gpio.cpp

FIRMWARE_SRC = $(ENGINE_SRC) $(DRV_SRC) $(APP_SRC) $(HAL_COMMON_SRC) \
               $(HAL_STM32H562_SRC) $(SRC_DIR)/main_stm32.cpp \
               $(SRC_DIR)/startup_stm32h562.cpp
FIRMWARE_OBJ = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(FIRMWARE_SRC))
# Host tests split into suites (hygiene PR-06+); order of sources is free.
# Entry: test/run_all.cpp. Exact PASS count enforced by suite_registry + main order.
HOST_TEST_HARNESS = $(TEST_DIR)/harness.cpp \
                    $(TEST_DIR)/fixtures.cpp \
                    $(TEST_DIR)/ui_helpers.cpp \
                    $(TEST_DIR)/run_all.cpp
HOST_TEST_SUITES = $(TEST_DIR)/test_etb.cpp \
                   $(TEST_DIR)/test_torque.cpp \
                   $(TEST_DIR)/test_ckp.cpp \
                   $(TEST_DIR)/test_sensors.cpp \
                   $(TEST_DIR)/test_fuel.cpp \
                   $(TEST_DIR)/test_ign.cpp \
                   $(TEST_DIR)/test_aux_knock.cpp \
                   $(TEST_DIR)/test_timer.cpp \
                   $(TEST_DIR)/test_sched.cpp \
                   $(TEST_DIR)/test_engine_misc.cpp \
                   $(TEST_DIR)/test_math.cpp \
                   $(TEST_DIR)/test_protocol.cpp \
                   $(TEST_DIR)/test_output.cpp
HOST_TEST_SRC = $(ENGINE_SRC) $(DRV_SRC) $(APP_SRC) $(HAL_COMMON_SRC) \
                $(SRC_DIR)/hal/stm32h562/timer.cpp \
                $(SRC_DIR)/hal/stm32h562/system.cpp \
                $(HOST_TEST_HARNESS) $(HOST_TEST_SUITES)
HOST_TEST_BIN = $(HOST_DIR)/mvp_bench_tests

all: help

help:
	@echo "OpenEMS Build System"
	@echo "======================================"
	@echo "Usage: make [target] [BOARD=rgt6|vgt6] [WERROR=0|1]"
	@echo ""
	@echo "  host-test       Host regression (always RGT6 pin map stubs)"
	@echo "  firmware        Build for BOARD (default rgt6)"
	@echo "  firmware-rgt6   Build RGT6 bin"
	@echo "  firmware-vgt6   Build VGT6 bin (GPIOE INJ/IGN/ETB)"
	@echo "  clean           Remove /tmp/openems-build"
	@echo ""
	@echo "Quality gates:"
	@echo "  secrets-check   Fail if wifi_credentials.h (etc.) is tracked"
	@echo "  lint-includes   Phase A/B include policy (LINT_PHASE=A|B LINT_ERROR=0|1)"
	@echo "  format          clang-format dirty git files only (on-touch)"
	@echo "  format-all      clang-format entire src/test (explicit; large diff)"
	@echo "  format-check    Dry-run format on dirty files"
	@echo "  ci-local        Stage1: secrets + host/fw WERROR (see tools/ci_local.sh)"
	@echo ""
	@echo "Outputs: /tmp/openems-build/bin/openems-rgt6.bin | openems-vgt6.bin"

host-test:
	@mkdir -p $(HOST_DIR)
	@echo "  HOST $(HOST_TEST_BIN)"
	@$(CXX_HOST) $(CFLAGS_HOST) $(HOST_TEST_SRC) -o $(HOST_TEST_BIN) -lm
	@$(HOST_TEST_BIN)

firmware-rgt6:
	@$(MAKE) firmware BOARD=rgt6

firmware-vgt6:
	@$(MAKE) firmware BOARD=vgt6

firmware: $(OBJ_DIR) $(ELF_DIR) $(BIN_DIR) $(FIRMWARE_ELF) $(FIRMWARE_HEX) $(FIRMWARE_BIN)
	@cp -f $(FIRMWARE_BIN) $(FIRMWARE_BIN_ALIAS)
	@echo "Building STM32H562 $(BOARD_LABEL) firmware..."
	@echo "   Package: $(BOARD_LABEL)  (-DEMS_BOARD_$(shell echo $(BOARD) | tr a-z A-Z))"
	@command -v $(COMPILER_ARM) >/dev/null 2>&1 || { echo "ERROR: $(COMPILER_ARM) not found"; exit 1; }
	@command -v $(OBJCOPY_ARM) >/dev/null 2>&1 || { echo "ERROR: $(OBJCOPY_ARM) not found"; exit 1; }
	@echo "Firmware build successful"
	@echo "   ELF: $(FIRMWARE_ELF)"
	@echo "   HEX: $(FIRMWARE_HEX)"
	@echo "   BIN: $(FIRMWARE_BIN)"
	@echo "   alias: $(FIRMWARE_BIN_ALIAS)"

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	@mkdir -p $(dir $@)
	@echo "  CXX [$(BOARD)] $<"
	@$(COMPILER_ARM) $(CFLAGS_ARM) -c $< -o $@

$(FIRMWARE_ELF): $(FIRMWARE_OBJ) $(LINKER_SCRIPT) | $(ELF_DIR)
	@echo "  LD  $@"
	@$(COMPILER_ARM) $(CFLAGS_ARM) $(LDFLAGS_ARM) $(FIRMWARE_OBJ) -o $@

$(FIRMWARE_HEX): $(FIRMWARE_ELF) | $(BIN_DIR)
	@echo "  HEX $@"
	@$(OBJCOPY_ARM) -O ihex $< $@

$(FIRMWARE_BIN): $(FIRMWARE_ELF) | $(BIN_DIR)
	@echo "  BIN $@"
	@$(OBJCOPY_ARM) -O binary $< $@

clean:
	@rm -rf $(BUILD_DIR)
	@echo "Build artifacts cleaned"

# ── Quality / hygiene ─────────────────────────────────────────────────────────
secrets-check:
	@bash tools/secrets_check.sh

lint-includes:
	@LINT_ERROR=$(LINT_ERROR) $(PYTHON) tools/lint_includes.py --phase $(LINT_PHASE) \
		$(if $(filter 1,$(LINT_ERROR)),--error,)

# On-touch only: format files changed vs HEAD (or staged). Requires clang-format.
FORMAT_SRC = $(shell git diff --name-only --diff-filter=ACMR HEAD -- 'src/**/*.cpp' 'src/**/*.h' 'test/**/*.cpp' 'test/**/*.h' 2>/dev/null; \
	git diff --cached --name-only --diff-filter=ACMR -- 'src/**/*.cpp' 'src/**/*.h' 'test/**/*.cpp' 'test/**/*.h' 2>/dev/null)
format:
	@command -v $(CLANG_FORMAT) >/dev/null 2>&1 || { \
		echo "ERROR: $(CLANG_FORMAT) not found — install clang-format or set CLANG_FORMAT="; exit 1; }
	@files=$$(printf '%s\n' $(FORMAT_SRC) | sort -u | sed '/^$$/d'); \
	if [ -z "$$files" ]; then echo "format: no dirty C++ files"; exit 0; fi; \
	echo "format (on-touch):"; echo "$$files" | sed 's/^/  /'; \
	echo "$$files" | xargs -r $(CLANG_FORMAT) -i

format-all:
	@command -v $(CLANG_FORMAT) >/dev/null 2>&1 || { \
		echo "ERROR: $(CLANG_FORMAT) not found"; exit 1; }
	@echo "WARNING: format-all rewrites entire src/ and test/ — large review noise"
	@find src test -type f \( -name '*.cpp' -o -name '*.h' \) -print0 | xargs -0 $(CLANG_FORMAT) -i
	@echo "format-all: done"

format-check:
	@command -v $(CLANG_FORMAT) >/dev/null 2>&1 || { \
		echo "ERROR: $(CLANG_FORMAT) not found"; exit 1; }
	@files=$$(printf '%s\n' $(FORMAT_SRC) | sort -u | sed '/^$$/d'); \
	if [ -z "$$files" ]; then echo "format-check: no dirty C++ files"; exit 0; fi; \
	echo "$$files" | xargs -r $(CLANG_FORMAT) --dry-run -Werror

ci-local:
	@bash tools/ci_local.sh $(or $(STAGE),1)

$(BIN_DIR):
	@mkdir -p $@

$(OBJ_DIR):
	@mkdir -p $@

$(ELF_DIR):
	@mkdir -p $@

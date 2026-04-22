# OpenEMS-v3: STM32H562RGT6 Firmware Build System
# Target: ARM Cortex-M33 @ 250 MHz
# Build for host-based testing or STM32H562 embedded

.PHONY: all clean host-test firmware help test-fuel test-ign test-knock test-aux test-ckp test-sensors test-all

# Configuration
COMPILER_HOST = g++
COMPILER_ARM = arm-none-eabi-g++
CFLAGS_COMMON = -std=c++17 -Wall -Wextra
CFLAGS_HOST = $(CFLAGS_COMMON) -DEMS_HOST_TEST -O2 -I./src
CFLAGS_ARM = $(CFLAGS_COMMON) -DTARGET_STM32H562 -mcpu=cortex-m33 -mthumb \
             -fno-exceptions -fno-rtti -O2 -I./src

# Directories
SRC_DIR = src
TEST_DIR = test
BUILD_DIR = /tmp/openems-v3-build
BIN_DIR = $(BUILD_DIR)/bin
OBJ_DIR = $(BUILD_DIR)/obj
ELF_DIR = $(BUILD_DIR)/elf

# Source files (common across all builds)
ENGINE_SRC = $(SRC_DIR)/engine/fuel_calc.cpp $(SRC_DIR)/engine/ign_calc.cpp \
             $(SRC_DIR)/engine/knock.cpp $(SRC_DIR)/engine/auxiliaries.cpp \
             $(SRC_DIR)/engine/table3d.cpp $(SRC_DIR)/engine/quick_crank.cpp \
             $(SRC_DIR)/engine/cycle_sched.cpp $(SRC_DIR)/engine/ecu_sched.cpp

DRV_SRC = $(SRC_DIR)/drv/ckp.cpp $(SRC_DIR)/drv/sensors.cpp

APP_SRC = $(SRC_DIR)/app/tuner_studio.cpp $(SRC_DIR)/app/can_stack.cpp

HAL_COMMON_SRC = $(SRC_DIR)/hal/adc.cpp $(SRC_DIR)/hal/can.cpp $(SRC_DIR)/hal/uart.cpp \
                 $(SRC_DIR)/hal/flexnvm.cpp

# Note: These have implementations (adc.cpp, etc.) but they reference HAL registers
# For host tests we use only headers and stubs

# Host test stubs (weak stub implementations for missing HAL functions)
HAL_TEST_STUBS = $(SRC_DIR)/hal_test_missing_stubs.cpp

# STM32H562 HAL implementations
HAL_STM32H562_SRC = $(SRC_DIR)/hal/stm32h562/system.cpp \
                    $(SRC_DIR)/hal/stm32h562/timer.cpp

all: help

help:
	@echo "OpenEMS-v3 Build System (Greenfield)"
	@echo "======================================"
	@echo "Usage: make [target]"
	@echo ""
	@echo "Host Testing (Recommended for Development):"
	@echo "  host-test      Build & run all 17 unit tests"
	@echo "  test-fuel      Test fuel calculation"
	@echo "  test-ign       Test ignition timing"
	@echo "  test-all       Run all individual tests"
	@echo ""
	@echo "STM32H562 Firmware:"
	@echo "  firmware       Build firmware (.elf/.hex)"
	@echo ""
	@echo "Maintenance:"
	@echo "  clean          Remove build artifacts"
	@echo ""
	@echo "Examples:"
	@echo "  make host-test"
	@echo "  make test-fuel"
	@echo "  make firmware"

# ============================================================================
# HOST-BASED UNIT TESTS (Primary for Development)
# ============================================================================

host-test: $(BIN_DIR) test-fuel test-ign test-knock test-aux test-ckp test-sensors test-all
	@echo ""
	@echo "✅ Phase 2: All host-based tests completed!"

test-fuel: $(BIN_DIR)
	@echo "[1/7] Testing fuel calculation..."
	@$(COMPILER_HOST) $(CFLAGS_HOST) $(ENGINE_SRC) $(DRV_SRC) $(HAL_TEST_STUBS) \
		$(TEST_DIR)/engine/test_fuel.cpp -o $(BIN_DIR)/test_fuel
	@$(BIN_DIR)/test_fuel

test-ign: $(BIN_DIR)
	@echo "[2/7] Testing ignition timing..."
	@$(COMPILER_HOST) $(CFLAGS_HOST) $(ENGINE_SRC) $(DRV_SRC) $(HAL_TEST_STUBS) \
		$(TEST_DIR)/engine/test_ign.cpp -o $(BIN_DIR)/test_ign
	@$(BIN_DIR)/test_ign

test-knock: $(BIN_DIR)
	@echo "[3/7] Testing knock detection..."
	@$(COMPILER_HOST) $(CFLAGS_HOST) $(ENGINE_SRC) $(DRV_SRC) $(HAL_TEST_STUBS) \
		$(TEST_DIR)/engine/test_knock.cpp -o $(BIN_DIR)/test_knock
	@$(BIN_DIR)/test_knock

test-aux: $(BIN_DIR)
	@echo "[4/7] Testing auxiliaries..."
	@$(COMPILER_HOST) $(CFLAGS_HOST) $(ENGINE_SRC) $(DRV_SRC) $(APP_SRC) $(HAL_TEST_STUBS) \
		$(TEST_DIR)/engine/test_auxiliaries.cpp -o $(BIN_DIR)/test_auxiliaries
	@$(BIN_DIR)/test_auxiliaries

test-ckp: $(BIN_DIR)
	@echo "[5/7] Testing CKP synchronization..."
	@$(COMPILER_HOST) $(CFLAGS_HOST) $(ENGINE_SRC) $(DRV_SRC) $(HAL_TEST_STUBS) \
		$(TEST_DIR)/drv/test_ckp.cpp -o $(BIN_DIR)/test_ckp
	@$(BIN_DIR)/test_ckp

test-sensors: $(BIN_DIR)
	@echo "[6/7] Testing sensors..."
	@$(COMPILER_HOST) $(CFLAGS_HOST) $(ENGINE_SRC) $(DRV_SRC) $(HAL_TEST_STUBS) \
		$(TEST_DIR)/drv/test_sensors.cpp -o $(BIN_DIR)/test_sensors
	@$(BIN_DIR)/test_sensors

test-all: $(BIN_DIR)
	@echo "[7/7] Running remaining tests..."
	@$(COMPILER_HOST) $(CFLAGS_HOST) $(ENGINE_SRC) $(DRV_SRC) $(APP_SRC) $(HAL_TEST_STUBS) \
		$(TEST_DIR)/engine/test_quick_crank.cpp -o $(BIN_DIR)/test_quick_crank
	@$(BIN_DIR)/test_quick_crank
	@$(COMPILER_HOST) $(CFLAGS_HOST) $(ENGINE_SRC) $(DRV_SRC) $(APP_SRC) $(HAL_TEST_STUBS) \
		$(TEST_DIR)/app/test_ts_protocol.cpp -o $(BIN_DIR)/test_ts_protocol
	@$(BIN_DIR)/test_ts_protocol
	@$(COMPILER_HOST) $(CFLAGS_HOST) $(ENGINE_SRC) $(DRV_SRC) $(APP_SRC) \
		$(HAL_COMMON_SRC) $(HAL_TEST_STUBS) $(TEST_DIR)/app/test_can.cpp \
		-o $(BIN_DIR)/test_can
	@$(BIN_DIR)/test_can

# ============================================================================
# STM32H562 FIRMWARE BUILD
# ============================================================================

firmware: $(OBJ_DIR) $(ELF_DIR)
	@echo "🔨 Building STM32H562 firmware..."
	@echo "   Target: STM32H562RGT6 (250 MHz Cortex-M33)"
	@echo ""
	@command -v $(COMPILER_ARM) >/dev/null 2>&1 || { \
		echo "❌ $(COMPILER_ARM) not found"; \
		echo "Install: sudo apt-get install gcc-arm-none-eabi"; \
		exit 1; }
	@echo "✅ ARM toolchain found"
	@echo "Compiling STM32H562 firmware..."
	@$(COMPILER_ARM) $(CFLAGS_ARM) -c $(ENGINE_SRC) -o $(OBJ_DIR)/engine.o
	@$(COMPILER_ARM) $(CFLAGS_ARM) -c $(DRV_SRC) -o $(OBJ_DIR)/drv.o
	@$(COMPILER_ARM) $(CFLAGS_ARM) -c $(APP_SRC) -o $(OBJ_DIR)/app.o
	@$(COMPILER_ARM) $(CFLAGS_ARM) -c $(HAL_COMMON_SRC) -o $(OBJ_DIR)/hal.o
	@$(COMPILER_ARM) $(CFLAGS_ARM) -c $(HAL_STM32H562_SRC) -o $(OBJ_DIR)/hal_stm32.o
	@$(COMPILER_ARM) $(CFLAGS_ARM) -c $(SRC_DIR)/main_stm32.cpp -o $(OBJ_DIR)/main.o
	@echo "✅ Firmware compilation successful"
	@echo "   Objects: $(OBJ_DIR)/"
	@echo "   Note: Link requires linker script & startup code (Phase 3+)"

# ============================================================================
# CLEANUP
# ============================================================================

clean:
	@rm -rf $(BUILD_DIR)
	@echo "✅ Build artifacts cleaned"

$(BIN_DIR):
	@mkdir -p $@

$(OBJ_DIR):
	@mkdir -p $@

$(ELF_DIR):
	@mkdir -p $@

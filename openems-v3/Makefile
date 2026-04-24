# OpenEMS-v3: STM32H562RGT6 Firmware Build System
# Target: ARM Cortex-M33 @ 250 MHz
# Build for STM32H562 embedded

.PHONY: all clean host-test firmware help

# Configuration
COMPILER_ARM = arm-none-eabi-g++
CFLAGS_COMMON = -std=c++17 -Wall -Wextra
CFLAGS_ARM = $(CFLAGS_COMMON) -DTARGET_STM32H562 -mcpu=cortex-m33 -mthumb \
             -fno-exceptions -fno-rtti -O2 -I./src

# Directories
SRC_DIR = src
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


# STM32H562 HAL implementations
HAL_STM32H562_SRC = $(SRC_DIR)/hal/stm32h562/system.cpp \
                    $(SRC_DIR)/hal/stm32h562/timer.cpp \
                    $(SRC_DIR)/hal/stm32h562/usb_cdc.cpp

all: help

help:
	@echo "OpenEMS-v3 Build System (Greenfield)"
	@echo "======================================"
	@echo "Usage: make [target]"
	@echo ""
	@echo "Host Testing:"
	@echo "  host-test      Temporarily disabled (test suite removed)"
	@echo ""
	@echo "STM32H562 Firmware:"
	@echo "  firmware       Build firmware (.elf/.hex)"
	@echo ""
	@echo "Maintenance:"
	@echo "  clean          Remove build artifacts"
	@echo ""
	@echo "Examples:"
	@echo "  make host-test"
	@echo "  make firmware"

# ============================================================================
# HOST-BASED UNIT TESTS
# ============================================================================

host-test:
	@echo "⚠️ Host test suite temporarily removed and will be reintroduced later."
	@echo "   Use: make firmware"

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

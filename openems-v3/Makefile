# OpenEMS-v3: STM32H562RGT6 Firmware Build System
# Target: ARM Cortex-M33 @ 250 MHz
# Buildfor host-based testing or STM32H562 embedded

.PHONY: all clean test host-test firmware help

# Configuration
COMPILER_HOST = g++
COMPILER_ARM = arm-none-eabi-g++
CFLAGS_COMMON = -std=c++17 -Wall -Wextra -Werror
CFLAGS_HOST = $(CFLAGS_COMMON) -DEMS_HOST_TEST -O2 -I./src
CFLAGS_ARM = $(CFLAGS_COMMON) -mcpu=cortex-m33 -mthumb -fno-exceptions -fno-rtti -O2 -I./src

# Directories
SRC_DIR = src
TEST_DIR = test
BUILD_DIR = /tmp/openems-v3-build
BIN_DIR = $(BUILD_DIR)/bin

# Source files
ENGINE_SRC = $(SRC_DIR)/engine/fuel_calc.cpp $(SRC_DIR)/engine/ign_calc.cpp \
             $(SRC_DIR)/engine/knock.cpp $(SRC_DIR)/engine/auxiliaries.cpp \
             $(SRC_DIR)/engine/table3d.cpp $(SRC_DIR)/engine/quick_crank.cpp \
             $(SRC_DIR)/engine/cycle_sched.cpp $(SRC_DIR)/engine/ecu_sched.cpp

DRV_SRC = $(SRC_DIR)/drv/ckp.cpp $(SRC_DIR)/drv/sensors.cpp

APP_SRC = $(SRC_DIR)/app/tuner_studio.cpp $(SRC_DIR)/app/can_stack.cpp

HAL_SRC = $(SRC_DIR)/hal/adc.cpp $(SRC_DIR)/hal/can.cpp $(SRC_DIR)/hal/uart.cpp

HAL_STM32_SRC = $(SRC_DIR)/hal/stm32h562/system.cpp $(SRC_DIR)/hal/stm32h562/adc.cpp \
                $(SRC_DIR)/hal/stm32h562/can.cpp $(SRC_DIR)/hal/stm32h562/uart.cpp \
                $(SRC_DIR)/hal/stm32h562/timer.cpp $(SRC_DIR)/hal/stm32h562/flash.cpp

# Test files
TEST_FILES = $(wildcard $(TEST_DIR)/*/*.cpp)

all: help

help:
	@echo "OpenEMS-v3 Build System"
	@echo "================================"
	@echo "Usage: make [target]"
	@echo ""
	@echo "Targets:"
	@echo "  host-test     Build & run host-based unit tests"
	@echo "  firmware      Build STM32H562 firmware (experimental)"
	@echo "  clean         Remove build artifacts"
	@echo ""
	@echo "Example: make host-test"

# Host-based testing (primary for v3 development)
host-test: $(BIN_DIR)
	@echo "Building OpenEMS-v3 host tests..."
	@mkdir -p $(BIN_DIR)
	$(COMPILER_HOST) $(CFLAGS_HOST) \
		$(ENGINE_SRC) $(DRV_SRC) $(APP_SRC) \
		$(TEST_DIR)/engine/test_fuel.cpp \
		-o $(BIN_DIR)/test_fuel && $(BIN_DIR)/test_fuel
	$(COMPILER_HOST) $(CFLAGS_HOST) \
		$(ENGINE_SRC) $(DRV_SRC) $(APP_SRC) \
		$(TEST_DIR)/engine/test_ign.cpp \
		-o $(BIN_DIR)/test_ign && $(BIN_DIR)/test_ign
	$(COMPILER_HOST) $(CFLAGS_HOST) \
		$(ENGINE_SRC) $(DRV_SRC) $(APP_SRC) \
		$(TEST_DIR)/engine/test_knock.cpp \
		-o $(BIN_DIR)/test_knock && $(BIN_DIR)/test_knock
	$(COMPILER_HOST) $(CFLAGS_HOST) \
		$(ENGINE_SRC) $(DRV_SRC) \
		$(TEST_DIR)/drv/test_ckp.cpp \
		-o $(BIN_DIR)/test_ckp && $(BIN_DIR)/test_ckp
	$(COMPILER_HOST) $(CFLAGS_HOST) \
		$(ENGINE_SRC) $(DRV_SRC) \
		$(TEST_DIR)/drv/test_sensors.cpp \
		-o $(BIN_DIR)/test_sensors && $(BIN_DIR)/test_sensors
	@echo "✅ All host tests completed"

firmware:
	@echo "STM32H562 firmware build (TODO: linker script, startup code)"
	@echo "arm-none-eabi-gcc flags to be configured"

clean:
	@rm -rf $(BUILD_DIR)
	@echo "✅ Build artifacts cleaned"

$(BIN_DIR):
	@mkdir -p $@

# OpenEMS: STM32H562 Firmware Build System
# BOARD=rgt6 (default LQFP64) | BOARD=vgt6 (LQFP100 GPIOE pinout)

.PHONY: all clean host-test firmware firmware-rgt6 firmware-vgt6 help

COMPILER_ARM = arm-none-eabi-g++
OBJCOPY_ARM = arm-none-eabi-objcopy
CXX_HOST ?= g++

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

CFLAGS_COMMON = -std=c++17 -Wall -Wextra
CFLAGS_ARM = $(CFLAGS_COMMON) -DTARGET_STM32H562 -DNDEBUG -mcpu=cortex-m33 -mthumb \
             -fno-exceptions -fno-rtti -ffunction-sections -fdata-sections \
             -g0 -O2 -I./src $(BOARD_CFLAGS)
CFLAGS_HOST = $(CFLAGS_COMMON) -DEMS_HOST_TEST -DEMS_BOARD_RGT6 -O2 -g -I./src

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
APP_SRC = $(SRC_DIR)/app/ui_protocol.cpp $(SRC_DIR)/app/can_stack.cpp \
          $(SRC_DIR)/app/can_rx_map.cpp \
          $(SRC_DIR)/app/datalog.cpp
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
HOST_TEST_SRC = $(ENGINE_SRC) $(DRV_SRC) $(APP_SRC) $(HAL_COMMON_SRC) \
                $(SRC_DIR)/hal/stm32h562/timer.cpp \
                $(SRC_DIR)/hal/stm32h562/system.cpp \
                $(TEST_DIR)/mvp_bench_tests.cpp
HOST_TEST_BIN = $(HOST_DIR)/mvp_bench_tests

all: help

help:
	@echo "OpenEMS Build System"
	@echo "======================================"
	@echo "Usage: make [target] [BOARD=rgt6|vgt6]"
	@echo ""
	@echo "  host-test       Host regression (always RGT6 pin map stubs)"
	@echo "  firmware        Build for BOARD (default rgt6)"
	@echo "  firmware-rgt6   Build RGT6 bin"
	@echo "  firmware-vgt6   Build VGT6 bin (GPIOE INJ/IGN/ETB)"
	@echo "  clean           Remove /tmp/openems-build"
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

$(BIN_DIR):
	@mkdir -p $@

$(OBJ_DIR):
	@mkdir -p $@

$(ELF_DIR):
	@mkdir -p $@

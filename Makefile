# OpenEMS: STM32H562RGT6 Firmware Build System
# Target: ARM Cortex-M33 @ 250 MHz

.PHONY: all clean host-test firmware help

COMPILER_ARM = arm-none-eabi-g++
OBJCOPY_ARM = arm-none-eabi-objcopy

CFLAGS_COMMON = -std=c++17 -Wall -Wextra
CFLAGS_ARM = $(CFLAGS_COMMON) -DTARGET_STM32H562 -DNDEBUG -mcpu=cortex-m33 -mthumb \
             -fno-exceptions -fno-rtti -ffunction-sections -fdata-sections \
             -g0 -O2 -I./src

SRC_DIR = src
BUILD_DIR = /tmp/openems-build
BIN_DIR = $(BUILD_DIR)/bin
OBJ_DIR = $(BUILD_DIR)/obj
ELF_DIR = $(BUILD_DIR)/elf
LINKER_SCRIPT = linker/stm32h562.ld
FIRMWARE_ELF = $(ELF_DIR)/openems.elf
FIRMWARE_HEX = $(BIN_DIR)/openems.hex
FIRMWARE_BIN = $(BIN_DIR)/openems.bin
LDFLAGS_ARM = -mcpu=cortex-m33 -mthumb -nostartfiles \
              -Wl,--gc-sections -Wl,-Map=$(ELF_DIR)/openems.map \
              -T$(LINKER_SCRIPT)

ENGINE_SRC = $(SRC_DIR)/engine/fuel_calc.cpp $(SRC_DIR)/engine/ign_calc.cpp \
             $(SRC_DIR)/engine/knock.cpp $(SRC_DIR)/engine/auxiliaries.cpp \
             $(SRC_DIR)/engine/table3d.cpp $(SRC_DIR)/engine/quick_crank.cpp \
             $(SRC_DIR)/engine/ecu_sched.cpp

DRV_SRC = $(SRC_DIR)/drv/ckp.cpp $(SRC_DIR)/drv/sensors.cpp
APP_SRC = $(SRC_DIR)/app/tuner_studio.cpp $(SRC_DIR)/app/can_stack.cpp
HAL_COMMON_SRC = $(SRC_DIR)/hal/adc.cpp $(SRC_DIR)/hal/can.cpp \
                 $(SRC_DIR)/hal/uart.cpp $(SRC_DIR)/hal/flash.cpp
HAL_STM32H562_SRC = $(SRC_DIR)/hal/stm32h562/system.cpp \
                    $(SRC_DIR)/hal/stm32h562/timer.cpp \
                    $(SRC_DIR)/hal/stm32h562/usb_cdc.cpp

FIRMWARE_SRC = $(ENGINE_SRC) $(DRV_SRC) $(APP_SRC) $(HAL_COMMON_SRC) \
               $(HAL_STM32H562_SRC) $(SRC_DIR)/main_stm32.cpp \
               $(SRC_DIR)/startup_stm32h562.cpp
FIRMWARE_OBJ = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(FIRMWARE_SRC))

all: help

help:
	@echo "OpenEMS Build System"
	@echo "======================================"
	@echo "Usage: make [target]"
	@echo ""
	@echo "Host Testing:"
	@echo "  host-test      Temporarily disabled"
	@echo ""
	@echo "STM32H562 Firmware:"
	@echo "  firmware       Build firmware (.elf/.hex/.bin)"
	@echo ""
	@echo "Maintenance:"
	@echo "  clean          Remove build artifacts"

host-test:
	@echo "Host test suite temporarily disabled."
	@echo "Use: make firmware"

firmware: $(OBJ_DIR) $(ELF_DIR) $(BIN_DIR) $(FIRMWARE_ELF) $(FIRMWARE_HEX) $(FIRMWARE_BIN)
	@echo "Building STM32H562 firmware..."
	@echo "   Target: STM32H562RGT6 (250 MHz Cortex-M33)"
	@command -v $(COMPILER_ARM) >/dev/null 2>&1 || { echo "ERROR: $(COMPILER_ARM) not found"; exit 1; }
	@command -v $(OBJCOPY_ARM) >/dev/null 2>&1 || { echo "ERROR: $(OBJCOPY_ARM) not found"; exit 1; }
	@echo "Firmware build successful"
	@echo "   ELF: $(FIRMWARE_ELF)"
	@echo "   HEX: $(FIRMWARE_HEX)"
	@echo "   BIN: $(FIRMWARE_BIN)"

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	@mkdir -p $(dir $@)
	@echo "  CXX $<"
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

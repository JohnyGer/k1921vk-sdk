# Makefile for Climate Control System - K1921VK028
# Simple and reliable build system!

# Project name
PROJECT = climate-control

# SDK paths
SDK_ROOT = ../..
PLATFORM_DIR = $(SDK_ROOT)/platform
PLIB028_DIR = $(PLATFORM_DIR)/plib028
DEVICE_DIR = $(PLATFORM_DIR)/Device/NIIET/K1921VK028
CMSIS_DIR = $(PLATFORM_DIR)/CMSIS/Core/Include

# Toolchain
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

# Build directory
BUILD_DIR = build

# C sources
C_SOURCES = \
src/app/main.c \
src/hal/can_hal.c \
src/hal/uart_hal.c \
src/hal/timer_hal.c \
src/hal/gpio_hal.c \
src/protocol/can_protocol.c \
src/protocol/uart_protocol.c \
src/managers/sensor_manager.c \
$(DEVICE_DIR)/Source/system_K1921VK028.c \
$(PLIB028_DIR)/src/plib028_can.c \
$(PLIB028_DIR)/src/plib028_uart.c \
$(PLIB028_DIR)/src/plib028_dma.c \
$(PLIB028_DIR)/src/plib028_tmr.c \
$(PLIB028_DIR)/src/plib028_gpio.c \
$(PLIB028_DIR)/src/plib028_rcu.c

# ASM sources
ASM_SOURCES = \
$(DEVICE_DIR)/Source/GCC/startup_K1921VK028.S

# CPU options
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# C defines
C_DEFS = \
-DUSE_PLIB028 \
-DK1921VK028

# Include paths
C_INCLUDES = \
-Iinc \
-I$(PLIB028_DIR)/inc \
-I$(DEVICE_DIR)/Include \
-I$(CMSIS_DIR)

# Compile flags
CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) -std=c11 -Wall -Wextra -fdata-sections -ffunction-sections

# Debug vs Release
ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2 -O0
else
CFLAGS += -Os -flto
endif

# Linker script
LDSCRIPT = $(DEVICE_DIR)/Source/GCC/K1921VK028.ld

# Libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(PROJECT).map,--cref -Wl,--gc-sections

# Default target
all: $(BUILD_DIR)/$(PROJECT).elf $(BUILD_DIR)/$(PROJECT).hex $(BUILD_DIR)/$(PROJECT).bin

# Build objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.S=.o)))
vpath %.S $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@echo "CC $<"
	@$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.S Makefile | $(BUILD_DIR)
	@echo "AS $<"
	@$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(PROJECT).elf: $(OBJECTS) Makefile
	@echo "LD $@"
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	@$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo "HEX $@"
	@$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo "BIN $@"
	@$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

# Clean
clean:
	rm -fR $(BUILD_DIR)

# Flash using OpenOCD
flash: all
	openocd -f ../../tools/openocd/openocd-snippets/k1921vk028/connect_jlink_swd.cfg \
		-c "program $(BUILD_DIR)/$(PROJECT).elf verify reset exit"

# Dependencies
-include $(wildcard $(BUILD_DIR)/*.d)

.PHONY: all clean flash

CC = arm-none-eabi-gcc
CP = arm-none-eabi-objcopy
SZ = arm-none-eabi-size

HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

TARGET = drone
BUILD_DIR = build
SRC_DIR = Src

C_SOURCES = \
Src/main.c

C_INCLUDES = \
-IInc \
-ICMSIS_Inc \
-IStm32_Inc

LIBS = -lc -lm -lnosys

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
CPU = -mcpu=cortex-m7
FPU = -mfpu=fpv5-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

#  -mcpu=cortex-m7 selects the cortex-m7 processor
#  -mthumb selects the thumb instruction set
#  -mfpu=fpv5-d16 specifies what floating-point hardware (or hardware emulation) is available on the target
#  -mfloat-abi=hard enables hard boards which use an on-chip floating point unit
#  https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html

CFLAGS = $(MCU) $(C_INCLUDES) -fdata-sections -ffunction-sections -g

LDFLAGS = $(MCU) $(LIBS)

RELEASE_BIN_FILE = $(BUILD_DIR)/$(TARGET).bin
RELEASE_ELF_FILE = $(BUILD_DIR)/$(TARGET).bin

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wall $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

load: $(BUILD_DIR)/$(TARGET).bin 
	openocd -f interface/stlink-v2-1.cfg -f target/stm32f7x.cfg -c init -c "reset halt" -c halt -c "flash write_image erase $(RELEASE_BIN_FILE) 0x8000000" -c "verify_image $(RELEASE_BIN_FILE) 0x8000000" -c "reset run" -c shutdown

#	OPENOCD Flag
#	-f is used to define the interface for programming the board
# 	-c runs a command

clean:
	-rm -fR $(BUILD_DIR)

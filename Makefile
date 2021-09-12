PROJ_NAME = drone
VL53L0X_NAME = vl53l0x

# Directory containing all relevent source code
CMSIS_DIR = CMSIS
ST_DIR = STM32
VL53L0X_DIR = VL53L0X
VL53L0X_CORE_SRC_DIR = VL53L0X/core/src
VL53L0X_PLATFORM_SRC_DIR = VL53L0X/platform/src
UNIT_TESTS_DIR = Unit_Tests
BUILD_DIR = build
LINKER_SCRIPT = $(ST_DIR)/STM32F767ZITx_FLASH.ld
ST_SRC_DIR = STM32/Src
SRC_DIR = Src
FREERTOS_DIR = FreeRTOS
FREERTOS_SRC_DIR = FreeRTOS/Source
FREERTOS_PORTABLE_COMPILER_SRC_DIR = FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1
FREERTOS_PORTABLE_MEM_SRC_DIR = FreeRTOS/Source/portable/MemMang
##### Toolchain #####
TRIPLE  = 	arm-none-eabi
CC 		=	${TRIPLE}-gcc
LD 		= 	${TRIPLE}-ld
AS 		= 	${TRIPLE}-as
OBJCOPY =  	${TRIPLE}-objcopy
GDB 	= 	${TRIPLE}-gdb

#### Hardware Info ####
CPU = -mcpu=cortex-m7
FPU = -mfpu=fpv5-d16
FLOAT-ABI = -mfloat-abi=hard

##### Compiler options #####
CFLAGS = -g -Wall -T$(LINKER_SCRIPT)
CFLAGS += $(CPU) -mthumb $(FPU) $(FLOAT-ABI) -fdata-sections -ffunction-sections --specs=nosys.specs

##### Project specific libraries #####
SRC_FILES += $(wildcard Src/*.c)
CFLAGS += -IInc

##### CMSIS libraries and source code #####
CFLAGS += -I$(CMSIS_DIR)/Inc

##### ST libraries and source code #####
ST_SRC_FILES += $(wildcard $(ST_DIR)/Src/*.c)
CFLAGS += -I$(ST_DIR)/Inc
ST_ASM_FILES += $(wildcard $(ST_DIR)/Src/*.s)

##### FreeRTOS libraries and source code #####
FREERTOS_SRC_FILES += $(wildcard $(FREERTOS_SRC_DIR)/*.c)
CFLAGS += -I$(FREERTOS_DIR)/include \
-I$(FREERTOS_DIR)/Source/include \
-I$(FREERTOS_DIR)/Source/CMSIS_RTOS \
-I$(FREERTOS_DIR)/Source/portable/GCC/ARM_CM7/r0p1

##### VL53L0X libraries and source code #####
VL53L0X_PLATFORM_SRC_FILES += $(wildcard $(VL53L0X_DIR)/platform/src/*.c)
VL53L0X_CORE_SRC_FILES += $(wildcard $(VL53L0X_DIR)/core/src/*.c)
CFLAGS += -I$(VL53L0X_DIR)/core/inc -I$(VL53L0X_DIR)/platform/inc 
# To enable logs for VL53L0X
#CFLAGS += -D VL53L0X_LOG_ENABLE

DRONE_OBJECTS = \
$(SRC_FILES:Src/%.c=$(BUILD_DIR)/%.o) \
$(ST_SRC_FILES:$(ST_DIR)/Src/%.c=$(BUILD_DIR)/%.o) \
$(ST_ASM_FILES:$(ST_DIR)/Src/%.s=$(BUILD_DIR)/%.o) \
$(FREERTOS_SRC_FILES:$(FREERTOS_SRC_DIR)/%.c=$(BUILD_DIR)/%.o) \
$(VL53L0X_CORE_SRC_FILES:$(VL53L0X_DIR)/core/src/%.c=$(BUILD_DIR)/%.o) \
$(VL53L0X_PLATFORM_SRC_FILES:$(VL53L0X_DIR)/platform/src/%.c=$(BUILD_DIR)/%.o) \
$(BUILD_DIR)/port.o \
$(BUILD_DIR)/heap_4.o

RELEASE_BIN_FILE = $(BUILD_DIR)/$(PROJ_NAME).bin
DEBUG_ELF_FILE = $(BUILD_DIR)/$(PROJ_NAME).elf

$(BUILD_DIR):
	mkdir $@

all:  $(BUILD_DIR)/$(PROJ_NAME).bin | $(BUILD_DIR)

##### Flash code to board using OpenOCD #####
load: $(BUILD_DIR)/$(PROJ_NAME).bin
	openocd -f interface/stlink-v2-1.cfg -f target/stm32f7x.cfg -c init -c "reset halt" -c "flash write_image erase $^ 0x8000000" -c "verify_image $^ 0x8000000" -c "reset run" -c shutdown

##### Debug code to board using OpenOCD #####
# Call make connect first and then make gdb in another window
connect:
	openocd -f interface/stlink-v2-1.cfg -f target/stm32f7x.cfg -c init -c "reset halt" -c halt

gdb:
	$(GDB) --eval-command="target remote localhost:3333" --eval-command="monitor reset halt" $(DEBUG_ELF_FILE)

print:
	echo $(DRONE_OBJECTS)
clean:
	rm -f $(BUILD_DIR)/*

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: $(ST_SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

#-c tells compiler not to link the file
$(BUILD_DIR)/%.o: $(ST_SRC_DIR)/%.s | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $^ -o $@

$(BUILD_DIR)/%.o: $(FREERTOS_SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: $(FREERTOS_PORTABLE_COMPILER_SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: $(FREERTOS_PORTABLE_MEM_SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: $(VL53L0X_CORE_SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: $(VL53L0X_PLATFORM_SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(PROJ_NAME).elf: $(DRONE_OBJECTS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $^

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(OBJCOPY) -O binary $^ $@

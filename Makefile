PROJ_NAME = drone

# Directory containing all relevent source code
CMSIS_DIR = CMSIS
ST_DIR = STM32
BUILD_DIR = build
LINKER_SCRIPT = $(ST_DIR)/STM32F767ZITx_FLASH.ld
ST_SRC_DIR = STM32/Src
SRC_DIR = Src

##### Toolchain #####
TRIPLE  = 	arm-none-eabi
CC 		=	${TRIPLE}-gcc
LD 		= 	${TRIPLE}-ld
AS 		= 	${TRIPLE}-as
GDB 	= 	${TRIPLE}-gdb
OBJCOPY =  	${TRIPLE}-objcopy

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

OBJECTS = $(SRC_FILES:Src/%.c=$(BUILD_DIR)/%.o) $(ST_SRC_FILES:$(ST_DIR)/Src/%.c=$(BUILD_DIR)/%.o) $(ST_ASM_FILES:$(ST_DIR)/Src/%.s=$(BUILD_DIR)/%.o)

$(BUILD_DIR):
	mkdir $@

all:  $(BUILD_DIR)/$(PROJ_NAME).bin | $(BUILD_DIR)

test:
	@echo $(OBJECTS)

##### Flash code to board using OpenOCD #####
load: $(BUILD_DIR)/$(PROJ_NAME).bin
	openocd -f interface/stlink-v2-1.cfg -f target/stm32f7x.cfg -c init -c "reset halt" -c "flash write_image erase $^ 0x8000000" -c "verify_image $^ 0x8000000" -c "reset run" -c shutdown

##### Print out disassembly of specified function using GDB #####
##### USAGE EXAMPLE: 	make disass FUNC=main 				#####
disass: $(BUILD_DIR)/$(PROJ_NAME).elf
	$(GDB) $^ -batch -ex 'disass /r $(FUNC)'

clean:
	rm -f $(BUILD_DIR)/$(PROJ_NAME).bin $(BUILD_DIR)/$(PROJ_NAME).elf $(BUILD_DIR)/*.o

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: $(ST_SRC_DIR)/%.c
	$(CC) -c $(CFLAGS) $< -o $@

#-c tells compiler not to link the file
$(BUILD_DIR)/%.o: $(ST_SRC_DIR)/%.s
	$(CC) $(CFLAGS) -c $^ -o $@

$(BUILD_DIR)/$(PROJ_NAME).elf: $(OBJECTS)
	$(CC) $(CFLAGS) -o $@ $^

$(BUILD_DIR)/$(PROJ_NAME).bin: $(BUILD_DIR)/$(PROJ_NAME).elf
	$(OBJCOPY) -O binary $^ $@
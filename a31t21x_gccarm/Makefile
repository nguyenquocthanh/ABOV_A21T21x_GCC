SRC_DIR = sources
INC_DIR = sources/core/
OUTPUT_DIR = output
CC=arm-none-eabi-gcc
OBJCOPY=objcopy
MACH=cortex-m0plus
CFLAGS= -c -mcpu=$(MACH) -std=gnu11 -O0 -I$(INC_DIR)
LDFLAGS= -mcpu=$(MACH) -mfloat-abi=soft --specs=nano.specs -T a31t21x_ls.ld -Wl,-Map=final.map
CPYFLAGS= $(OBJCOPY) -O ihex

SRCS = $(SRC_DIR)/main.c $(SRC_DIR)/a31t21x_startup.c $(SRC_DIR)/syscalls.c
OBJS = $(OUTPUT_DIR)/main.o $(OUTPUT_DIR)/a31t21x_startup.o $(OUTPUT_DIR)/syscalls.o $(OUTPUT_DIR)/output.elf $(OUTPUT_DIR)/output.hex

all:$(OBJS)
$(OUTPUT_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC)	$(CFLAGS) -o $@	$^

clean:
	rm -rf output/*

$(OUTPUT_DIR)/output.elf:$(OUTPUT_DIR)/main.o $(OUTPUT_DIR)/a31t21x_startup.o $(OUTPUT_DIR)/syscalls.o
	$(CC)	$(LDFLAGS)	-o	$@	$^

$(OUTPUT_DIR)/output.hex:$(OUTPUT_DIR)/output.elf
	$(CPYFLAGS)	$^	$@


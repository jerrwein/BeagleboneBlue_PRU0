
# The top-most name
BASE_NAME := dsm2_decoder

# The all-inclusive binary to be run on host
HOST_BIN := $(BASE_NAME)

# The binaries to be run on the PRUs
PRU0_BINS := pru0_text.bin pru0_data.bin

# Tool chain
CC := gcc
LD := gcc
STRIP := strip
PASM := pasm
DTC := dtc

# Identify all source files
HOST_LOADER_C_FILE := host_main.c
HOST_C_FILES := mio.c AM335X_GPIO.c
PRU0_C_FILES := pru-0.c pru_hal.c

# Identify all dependent header files
HOST_H_FILES := mio.h AM335X_GPIO.h
PRU_H_FILES := pru_hal.h

# Compiler/Linker flags, directives etc.
C_FLAGS := -Wall -O2
PRU_C_FLAGS := --c_src_interlist --verbose --section_sizes
L_FLAGS += -L/lib
L_LIBS += -lprussdrv

# Set paths for TI CGT tools and needed resource files
PRU_CGT_BIN_PATH := /usr/bin
PRU_CGT_BASE_PATH := /usr/share/ti/cgt-pru

# Correlate sources with binaries
HOST_LOADER_O_FILE := $(HOST_LOADER_C_FILE:.c=.o)
HOST_O_FILES := $(HOST_C_FILES:.c=.o)
PRU0_OBJ_FILES := $(PRU0_C_FILES:.c=.obj)

# Entry point for PRU binary
PRU0_START_ADDR := 0x`/usr/bin/dispru ./pru0.elf | grep _c_int00 | cut -f1 -d\ `

all: $(HOST_BIN) $(PRU0_BINS) $(DTBO_FILES)

$(HOST_LOADER_O_FILE) : $(HOST_LOADER_C_FILE) $(HOST_H_FILES)

$(HOST_BIN): $(HOST_LOADER_O_FILE) $(HOST_O_FILES)
	$(LD) -static -o $@ $(HOST_LOADER_O_FILE) $(HOST_O_FILES) $(L_FLAGS) $(L_LIBS)
	$(STRIP) $@

$(PRU0_BINS): pru0.elf
	@echo '-- Converting elf to binary --'
	$(PRU_CGT_BIN_PATH)/hexpru ./pru0_bin.cmd ./pru0.elf
	@echo
	@echo 'INFO: PRU0_START_ADDR = '0x`/usr/bin/dispru ./pru0.elf | grep _c_int00 | cut -f1 -d\ `
	@echo 'INFO: '`grep '^.data' pru0.map | awk '{print ".data usage: " $$4;}'`
	@echo 'INFO: '`grep '^.text' pru0.map | awk '{print ".text usage: " $$4;}' `
	@echo 'INFO: '`grep '^.cinit' pru0.map | awk '{print ".cinit usage: " $$4;}' `
	@echo 'INFO: '`grep '^.stack' pru0.map | awk '{print ".stack usage: " $$4;}' `
	@echo

pru0.elf: $(PRU0_OBJ_FILES)
#	echo '------ Compile/Link PRU-0 elf binary ------'
	$(PRU_CGT_BIN_PATH)/clpru --silicon_version=2 --hardware_mac=on -i$(PRU_CGT_BASE_PATH)/include -i$(PRU_CGT_BASE_PATH)/lib -z $(PRU0_OBJ_FILES) -llibc.a -m pru0.map -o pru0.elf AM33xx-lnk.cmd
#	echo '------ Completed Compile/Link PRU elf binary ------'

%.obj : %.c $(PRU_H_FILES)
#	echo '------ Compile PRU c source file ------'
	$(PRU_CGT_BIN_PATH)/clpru --silicon_version=2 --hardware_mac=on $(PRU_C_FLAGS) -i$(PRU_CGT_BASE_PATH)/include -i$(PRU_CGT_BASE_PATH)/lib -c $<
#	echo '------ Completed compile of PRU c source file ------'

$(HOST_LOADER_O_FILE) : pru0.elf pru0.elf
	$(CC) -DPRU0_START_ADDR=$(PRU0_START_ADDR) $(C_FLAGS) -c -o $@ $(HOST_LOADER_C_FILE)

#$(HOST_O_FILES) : $(HOST_C_FILES) $(HOST_H_FILES)
%.o : %.c
	$(CC) $(C_FLAGS) -c -o $@ $<

%.bin : %.p %.s
	$(PASM) -V2 -b $<

.PHONY	: clean all
clean	:
	-rm -f $(HOST_LOADER_O_FILE) $(HOST_O_FILES)
	-rm -f $(PRU0_OBJ_FILES)
	-rm -f $(HOST_BIN)
	-rm -f pru0.elf pru0.map
	-rm -f pru0_main.asm pru_hal.asm
	-rm -f $(PRU0_BINS)

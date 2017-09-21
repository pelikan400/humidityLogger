###############################################################################
# Makefile for the project twi_demo
###############################################################################

## General Flags
PROJECT = twi_demo
MCU = atmega324p
TARGET = twi_demo.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99 -I../include       -DF_TWI=100000    -DF_CPU=8000000UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=twi_demo.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Objects that must be built in order to link
OBJECTS = i2c.o i2c_dev_search.o i2c_read.o i2c_write.o twi_demo.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) twi_demo.hex twi_demo.eep twi_demo.lss size

## Compile
i2c.o: ../lib/i2c.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

i2c_dev_search.o: ../lib/i2c_dev_search.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

i2c_read.o: ../lib/i2c_read.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

i2c_write.o: ../lib/i2c_write.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

twi_demo.o: ../twi_demo.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) twi_demo.elf dep/* twi_demo.hex twi_demo.eep twi_demo.lss twi_demo.map


## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)


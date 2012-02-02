###############################################################################
# Makefile for the project ElisaII-avr-studio
###############################################################################

## General Flags
PROJECT = ElisaII-avr-studio
MCU = atmega2560
TARGET = Elisa3-avr-studio.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99       -DF_CPU=8000000UL -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wsign-compare
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=Elisa3-avr-studio.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Objects that must be built in order to link
OBJECTS = main.o spi.o mirf.o e_remote_control.o speed_control.o variables.o ports_io.o twimaster.o usart.o utility.o accelerometer.o adc.o motors.o behaviors.o leds.o sensors.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) Elisa3-avr-studio.hex Elisa3-avr-studio.eep Elisa3-avr-studio.lss size

## Compile
main.o: ../main.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

spi.o: ../spi.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

mirf.o: ../mirf.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

e_remote_control.o: ../e_remote_control.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

speed_control.o: ../speed_control.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

variables.o: ../variables.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ports_io.o: ../ports_io.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

twimaster.o: ../twimaster.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

usart.o: ../usart.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

utility.o: ../utility.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

accelerometer.o: ../accelerometer.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

adc.o: ../adc.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

motors.o: ../motors.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

behaviors.o: ../behaviors.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

leds.o: ../leds.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

sensors.o: ../sensors.c
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
	-rm -rf $(OBJECTS) Elisa3-avr-studio.elf dep/* Elisa3-avr-studio.hex Elisa3-avr-studio.eep Elisa3-avr-studio.lss Elisa3-avr-studio.map


## Other dependencies
-include $(shell mkdir dep 2>NUL) $(wildcard dep/*)


DEVICE     = atmega328p
CLOCK      = 16000000
PROGRAMMER = -c arduino -b 115200 -P COM3
OBJECTS    = main.o time.o ultrasonic.o motors.o servo.o uart.o
TEST_OBJECTS = ultrasonic.o servo.o motors.o main.o time.o
FUSES      = -U hfuse:w:0xde:m -U lfuse:w:0xff:m -U efuse:w:0x05:m

AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE)
COMPILE = avr-gcc -Wall -Os -std=gnu99 -DDEBUG_SAMPLES -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)

# Add at the top, after initial variable definitions
CFLAGS += -O2
LDFLAGS += -Wl,-u,vfprintf -lprintf_flt -lm

# Modify the linking rule to include LDFLAGS
%.elf: %.o $(OBJECTS)
	$(COMPILE) $(LDFLAGS) -o $@ $^

# symbolic targets:
all:	main.hex

.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:main.hex:i

fuse:
	$(AVRDUDE) $(FUSES)

# Xcode uses the Makefile targets "", "clean" and "install"
install: flash fuse

# if you use a bootloader, change the command below appropriately:
load: all
	bootloadHID main.hex

clean:
	rm -f main.hex main.elf $(OBJECTS) test.hex test.elf $(TEST_OBJECTS)

# file targets:
main.elf: $(OBJECTS)
	$(COMPILE) -o main.elf $(OBJECTS) $(LDFLAGS)

main.hex: main.elf
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
	avr-size --format=avr --mcu=$(DEVICE) main.elf
# If you have an EEPROM section, you must also create a hex file for the
# EEPROM and add it to the "flash" target.

# Targets for code debugging and analysis:
disasm:	main.elf
	avr-objdump -d main.elf

cpp:
	$(COMPILE) -E main.c

test: test_readings.hex
	$(AVRDUDE) -U flash:w:test_readings.hex:i

test_readings.hex: test_readings.elf
	avr-objcopy -O ihex -R .eeprom test_readings.elf test_readings.hex
	
# Modify test_readings.elf rule
test_readings.elf: test_readings.o uart.o ultrasonic.o servo.o motor.o
	$(COMPILE) $(LDFLAGS) -o test_readings.elf test_readings.o uart.o ultrasonic.o servo.o motor.o

DEVICE      = atmega328p
CLOCK       = 16000000
PROGRAMMER  = -c buspirate -P /dev/tty.usbserial*
FUSES       = -U hfuse:w:0xdf:m -U lfuse:w:0xe7:m
AVRDUDEOPTS = -V

# Tune the lines below only if you know what you are doing:

OBJECTS    = remotx.o pwm_isr.o

AVRDUDE = avrdude $(AVRDUDEOPTS) $(PROGRAMMER) -p $(DEVICE)
COMPILE = avr-gcc -std=gnu99 -Wall -Os -g -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)

TARGET=remotx.elf
HEXFILE=$(TARGET:.elf=.hex)

# symbolic targets:
all:	$(TARGET)

.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -c $< -o $@

.c.s:
	$(COMPILE) -S $< -o $@

program: $(HEXFILE)
	$(AVRDUDE) -U flash:w:remotx.hex:i

fuse:
	$(AVRDUDE) $(FUSES)

clean:
	rm -f $(TARGET) $(HEXFILE) $(OBJECTS)

$(OBJECTS): globals.h

$(TARGET): $(OBJECTS)
	$(COMPILE) -o $(TARGET) $(OBJECTS)

$(HEXFILE): $(TARGET)
	rm -f $(HEXFILE)
	avr-objcopy -j .text -j .data -O ihex remotx.elf remotx.hex

disasm:	remotx.elf
	avr-objdump -d remotx.elf

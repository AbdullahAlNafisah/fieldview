MCU=atmega2560
F_CPU=16000000UL

# Auto-detect first likely serial port
PORT ?= $(firstword $(wildcard /dev/ttyACM* /dev/ttyUSB*))
BAUD ?= 115200
BAUD_SERIAL ?= 115200
PROGRAMMER ?= wiring

CXX=avr-g++
OBJCOPY=avr-objcopy
CXXFLAGS=-mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -std=gnu++17 -Wall -Wextra -ffunction-sections -fdata-sections -Iinclude
# CXXFLAGS= -mmcu=$(MCU) -DF_CPU=$(F_CPU) -O2 -std=gnu++17 -Wall -Wextra -ffunction-sections -fdata-sections -Iinclude
LDFLAGS=-Wl,--gc-sections

SRC := $(wildcard src/*.cpp)
OBJ := $(SRC:.cpp=.o)

all: main.hex

main.elf: $(OBJ)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

main.hex: main.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

flash: main.hex
	@if [ -z "$(PORT)" ]; then \
	  echo "No serial port found. Plug the board and/or set PORT=/dev/ttyACM0"; exit 2; \
	fi
	@echo "Flashing on $(PORT) with -c $(PROGRAMMER) @ $(BAUD) ..."
	avrdude -v -p m2560 -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -D -U flash:w:main.hex:i

# Open serial monitor
screen:
	@if [ -z "$(PORT)" ]; then \
	  echo "No serial port found. Plug the board and/or set PORT=/dev/ttyACM0"; exit 2; \
	fi
	@echo "Opening serial monitor on $(PORT) @ $(BAUD_SERIAL) ..."
	screen $(PORT) $(BAUD_SERIAL)

# List ports for convenience
ports:
	@echo "Available serial ports:"; ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true

clean:
	rm -f $(OBJ) main.elf main.hex

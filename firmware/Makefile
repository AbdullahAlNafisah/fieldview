# ===== Serial/Programmer settings =====
# Try to auto-detect a likely serial port (first match of these globs).
# Users can override by running: make PORT=/dev/ttyACM0
PORT ?= $(firstword $(wildcard /dev/ttyACM* /dev/ttyUSB*))

# ===== User settings =====
BAUD ?= 115200                  # Arduino Mega2560 bootloader baud
PROGRAMMER ?= wiring            # Bootloader protocol for Mega2560

# ===== Toolchain =====
MCU     := atmega2560
F_CPU   := 16000000UL
CXX     := avr-g++
CC      := avr-gcc
OBJCOPY := avr-objcopy
AVRDUDE := avrdude

# ===== Flags =====
CXXFLAGS := -std=gnu++17 -Os -DF_CPU=$(F_CPU) -mmcu=$(MCU) -Wall -Wextra \
            -fno-exceptions -fno-rtti
CFLAGS  := -std=gnu11   -Os -DF_CPU=$(F_CPU) -mmcu=$(MCU) -Wall -Wextra
LDFLAGS := -mmcu=$(MCU)

# ===== Sources / outputs =====
SRC_DIR    := src
BUILD_DIR  := build
TARGET     := $(BUILD_DIR)/mag3110_quad
SOURCES    := $(SRC_DIR)/mag3110_quad.cpp
OBJECTS    := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(SOURCES))

# ===== Default target =====
all: $(TARGET).hex

# ===== Build rules =====
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJECTS)
	$(CXX) $(LDFLAGS) $^ -o $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# ===== Flash to the board =====
flash: $(TARGET).hex
	# Make sure the port exists and is accessible; you may need: sudo usermod -a -G dialout $$USER
	stty -F $(PORT) hupcl
	$(AVRDUDE) -v -p $(MCU) -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -D -U flash:w:$<:i

# ===== Utilities =====
size: $(TARGET).elf
	avr-size --mcu=$(MCU) --format=avr $<

clean:
	@rm -rf $(BUILD_DIR)

.PHONY: all flash clean size

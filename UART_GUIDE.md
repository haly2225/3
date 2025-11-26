# STM32 Oscilloscope - UART Communication Guide

## Overview

This version adds **UART (Serial)** communication support alongside SPI. UART is **much more reliable** than SPI slave mode and easier to debug!

## Why UART Instead of SPI?

### Problems with SPI Slave:
- ❌ Timing synchronization issues - slave must have data ready BEFORE master clocks
- ❌ Master controls clock - slave can't control when data is sent
- ❌ Difficult to debug - no visibility into why data isn't transmitted
- ❌ Requires precise wiring and signal integrity

### Benefits of UART:
- ✅ Self-clocked - each side generates its own clock
- ✅ Reliable asynchronous communication
- ✅ Easy to use with USB-to-Serial adapters
- ✅ Can connect directly to Pi4 hardware UART
- ✅ Simple 3-wire connection (TX, RX, GND)
- ✅ Better error detection

## Hardware Setup

### Option 1: USB-to-Serial Adapter (Recommended for testing)

**Components needed:**
- USB-to-Serial adapter (CH340, CP2102, FT232, etc.)
- 3 jumper wires

**Wiring:**
```
STM32 Blue Pill          USB-to-Serial Adapter
────────────────────────────────────────────────
PA9  (USART1_TX)    →    RX
PA10 (USART1_RX)    →    TX  (optional, not used)
GND                 →    GND
```

**Notes:**
- STM32 TX goes to adapter RX (and vice versa)
- Only TX is needed for one-way data transmission
- Adapter provides USB power, or use separate 3.3V supply

### Option 2: Direct Connection to Raspberry Pi 4

**Wiring:**
```
STM32 Blue Pill          Raspberry Pi 4
──────────────────────────────────────────
PA9  (USART1_TX)    →    GPIO15 (RXD)
PA10 (USART1_RX)    →    GPIO14 (TXD)  (optional)
GND                 →    GND
3.3V (optional)     ←    3.3V Pin 1
```

**Enable Pi4 UART:**
```bash
sudo raspi-config
# Interface Options → Serial Port
# Login shell: No
# Serial hardware: Yes
sudo reboot
```

**Device name:** `/dev/ttyAMA0` or `/dev/serial0`

## STM32 Firmware

The firmware has been updated to support **dual communication**:
- **UART** on PA9/PA10 @ 2 Mbps (primary, recommended)
- **SPI** on PA4/PA5/PA6/PA7 (backup, for compatibility)

### Key Features:
- 512 samples per frame
- Test pattern mode (sawtooth wave) for debugging
- Timer-triggered ADC @ ~600 kHz
- DMA for efficient data transfer
- Packet format: `0xAA 0x55 [frame_counter_16bit] [samples...]`

### Configuration:
- **Clock:** 64 MHz (HSI-based PLL)
- **ADC:** PA0 (ADC1_IN0)
- **UART:** PA9=TX, PA10=RX, 2 Mbps, 8N1
- **LED:** PC13 (toggles on each frame)

## Python Scripts

### 1. UART Test Script

**File:** `uart_test.py`

**Purpose:** Test UART communication and verify data reception

**Usage:**
```bash
python3 uart_test.py [port] [baudrate]
```

**Examples:**
```bash
# USB-to-Serial adapter (usually ttyUSB0)
python3 uart_test.py /dev/ttyUSB0 2000000

# Pi4 hardware UART
python3 uart_test.py /dev/ttyAMA0 2000000

# Auto-detect (uses /dev/ttyUSB0)
python3 uart_test.py
```

**Output:**
```
✓ Packet #   1 | Frame: 0001 | Samples: 512 | Vpp: 3.300V | Avg: 1.650V
  First 10 ADC values:    0,    8,   16,   24,   32,   40,   48,   56,   64,   72...
  First 10 voltages:   0.00V, 0.01V, 0.01V, 0.02V, 0.03V, 0.03V, 0.04V, 0.05V, 0.05V, 0.06V...
```

### 2. UART Oscilloscope Display

**File:** `oscilloscope_uart.py`

**Purpose:** Real-time oscilloscope GUI with waveform display

**Usage:**
```bash
python3 oscilloscope_uart.py [port] [baudrate]
```

**Examples:**
```bash
python3 oscilloscope_uart.py /dev/ttyUSB0 2000000
python3 oscilloscope_uart.py /dev/ttyAMA0 2000000
```

**Features:**
- Real-time waveform display
- Dark theme UI (green trace on black background)
- Statistics: Vpp, Max, Min, Avg, FPS
- Auto-scaling Y-axis
- ~20 FPS refresh rate

## Installation

### On Raspberry Pi 4:

```bash
# Install dependencies
sudo apt-get update
sudo apt-get install python3-pip python3-serial python3-numpy python3-matplotlib

# Or using pip
pip3 install pyserial numpy matplotlib

# Add user to dialout group (for serial port access)
sudo usermod -a -G dialout $USER
# Then logout and login again

# Make scripts executable
chmod +x uart_test.py oscilloscope_uart.py

# Test connection
python3 uart_test.py
```

## Troubleshooting

### No Serial Port Found

**Check available ports:**
```bash
ls -l /dev/ttyUSB* /dev/ttyAMA* /dev/serial*
```

**Check if device is recognized:**
```bash
dmesg | grep tty    # Look for USB serial adapter
lsusb               # Should show CH340, CP2102, etc.
```

### Permission Denied

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Or temporarily fix permissions
sudo chmod 666 /dev/ttyUSB0

# Check current permissions
ls -l /dev/ttyUSB0
```

### No Data Received

1. **Check STM32 is powered:** LED should blink 3 times at startup, then toggle
2. **Check wiring:** TX → RX, GND → GND
3. **Verify firmware is flashed:** Build and upload latest code
4. **Try different baudrate:** 115200, 921600, 1000000, 2000000
5. **Check voltage levels:** Both devices should use 3.3V logic

### Data Corruption / Frame Errors

1. **Lower baudrate:** Try 1000000 or 921600 instead of 2000000
2. **Shorter wires:** Use wires < 30cm for high-speed UART
3. **Add ground:** Ensure good ground connection
4. **Check voltage:** Both devices at 3.3V (not 5V!)

### Pi4 UART Issues

**Disable Bluetooth to free up hardware UART:**
```bash
sudo nano /boot/config.txt
# Add: dtoverlay=disable-bt
sudo systemctl disable hbt-uart
sudo reboot
```

**Check which UART is available:**
```bash
ls -l /dev/serial*
# serial0 should point to ttyAMA0 (hardware UART)
```

## Test Pattern vs Real ADC Data

The firmware includes **TEST MODE** with a sawtooth wave pattern for debugging.

### Test Mode (Current):
- Generates sawtooth wave 0 → 4095
- Good for verifying UART communication works
- You should see: **Vpp ≈ 3.3V, sawtooth wave pattern**

### Switch to Real ADC Data:

**Edit:** `ok/Core/Src/main.c`, line 62

**Change from:**
```c
uint16_t val = (i * 8) & 0x0FFF;  // Test pattern
// uint16_t val = adc_buffer[i];  // Commented out
```

**To:**
```c
// uint16_t val = (i * 8) & 0x0FFF;  // Test pattern (commented)
uint16_t val = adc_buffer[i];       // Use real ADC data
```

Then rebuild and reflash firmware.

## Performance

### Current Settings:
- **ADC Sample Rate:** ~600 kHz (TIM3 trigger)
- **Samples per Frame:** 512
- **Frame Rate:** ~1170 frames/second (theoretical)
- **UART Baudrate:** 2 Mbps
- **Data Rate:** 1028 bytes/frame × 1170 fps ≈ 1.2 MB/s
- **Display Update:** 20 FPS (Python GUI limitation)

### Bottlenecks:
- **Actual limiting factor:** Python matplotlib (~20 FPS)
- **UART can handle:** 2000000 / (10 bits/byte) = 200 KB/s = 194 frames/s
- **To improve:** Use C++ or lower-level graphics library

## Migration from SPI

If you were using the SPI version, UART is a drop-in replacement:

### Advantages over SPI:
- ✅ No master-slave synchronization issues
- ✅ No clock timing requirements
- ✅ Works with simple USB adapter
- ✅ Better for debugging (can use serial terminal)
- ✅ More reliable data transmission

### To switch:
1. **Hardware:** Replace 4-wire SPI with 3-wire UART
2. **Software:** Use `uart_test.py` or `oscilloscope_uart.py`
3. **Done!** Everything else is the same (packet format unchanged)

## Protocol Specification

Both SPI and UART use the **same packet format**:

```
Byte 0:       0xAA (marker start)
Byte 1:       0x55 (marker header)
Byte 2-3:     Frame counter (16-bit, big-endian)
Byte 4-1027:  ADC samples (512 × 16-bit, big-endian)
Total:        1028 bytes per packet
```

**Sample Encoding:**
- 12-bit ADC value (0-4095)
- Stored as 16-bit big-endian (MSB first)
- Voltage = (ADC_value / 4095) × 3.3V

## Next Steps

1. **Test UART communication:**
   ```bash
   python3 uart_test.py
   ```
   You should see: ✓ packets with sawtooth pattern

2. **Run oscilloscope display:**
   ```bash
   python3 oscilloscope_uart.py
   ```
   You should see: Green sawtooth waveform

3. **Connect real signal:**
   - Connect signal to PA0 (0-3.3V only!)
   - Edit main.c to use real ADC data (see above)
   - Rebuild and flash
   - Run oscilloscope again

4. **Optimize:**
   - Adjust ADC sample rate (TIM3 period)
   - Change buffer size if needed
   - Tune UART baudrate for your cables

## Support

**If you have issues:**
1. Run `uart_test.py` first to verify basic communication
2. Check LED on PC13 - should toggle (indicates firmware running)
3. Use a serial terminal (minicom, screen) to see raw data
4. Try lower baudrate (921600 or 1000000)
5. Check wiring with multimeter (continuity test)

**Serial Terminal Test:**
```bash
# View raw data (Ctrl+C to exit)
screen /dev/ttyUSB0 2000000

# Or with minicom
minicom -D /dev/ttyUSB0 -b 2000000
```

You should see binary data streaming (0xAA 0x55 markers visible).

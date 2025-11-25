# Python Oscilloscope for Raspberry Pi 4

Python scripts để hiển thị dữ liệu từ STM32 oscilloscope.

## Installation

### 1. Enable SPI trên Raspberry Pi

```bash
sudo raspi-config
# Interface Options → SPI → Enable
```

### 2. Install Python dependencies

```bash
# Update system
sudo apt-get update

# Install Python packages
pip3 install spidev numpy matplotlib

# Or using system package manager
sudo apt-get install python3-spidev python3-numpy python3-matplotlib
```

## Usage

### Test SPI Communication

Chạy script test để kiểm tra kết nối SPI:

```bash
python3 spi_test.py
```

Output mẫu:
```
✓ Packet #1 | Frame: 12345 | Marker @0
  First 10 samples:
    [0] ADC: 2048  →  1.65V
    [1] ADC: 2050  →  1.65V
    ...
```

### Run Oscilloscope Display

Chạy giao diện oscilloscope:

```bash
python3 oscilloscope_display.py
```

## Files

| File | Description |
|------|-------------|
| `oscilloscope_display.py` | Real-time oscilloscope với matplotlib GUI |
| `spi_test.py` | Test script để debug SPI communication |

## Features

### oscilloscope_display.py

- ✅ Real-time waveform display
- ✅ Auto-scaling
- ✅ Statistics display (Vpp, Max, Min, FPS)
- ✅ Dark theme UI
- ✅ 20 FPS update rate

### spi_test.py

- ✅ Raw packet inspection
- ✅ Marker detection
- ✅ Sample voltage calculation
- ✅ Success rate tracking

## Troubleshooting

### "No such file or directory: '/dev/spidev0.0'"

```bash
# Enable SPI
sudo raspi-config
# Interface Options → SPI → Enable
sudo reboot
```

### "Permission denied: '/dev/spidev0.0'"

```bash
# Add user to spi group
sudo usermod -a -G spi $USER
sudo reboot
```

### No data received

1. Check wiring (GND must be connected!)
2. Check STM32 is running (LED PC13 blinking?)
3. Reduce SPI speed in script:
   ```python
   SPI_SPEED = 1000000  # Try 1 MHz instead of 8 MHz
   ```

### Data corruption

1. Check ground connection
2. Reduce SPI speed
3. Add 1kΩ series resistors on SPI lines
4. Shorten wires

## Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `Ctrl+C` | Stop and exit |
| `Q` | Quit (when matplotlib window active) |

## Performance

| Parameter | Value |
|-----------|-------|
| SPI Speed | 8 MHz (default) |
| GUI Update | 20 FPS (50ms) |
| Buffer Size | 512 samples |
| Sample Rate | 600 kHz (STM32) |

## Advanced Usage

### Adjust SPI Speed

Edit trong script:

```python
SPI_SPEED = 16000000  # 16 MHz (max)
# or
SPI_SPEED = 1000000   # 1 MHz (safe)
```

### Change GUI Update Rate

```python
# In oscilloscope_display.py
ani = animation.FuncAnimation(..., interval=50, ...)  # 50ms = 20 FPS
# Change to:
ani = animation.FuncAnimation(..., interval=100, ...)  # 100ms = 10 FPS
```

## Example Output

```
✓ SPI initialized: 8.0 MHz
✓ Packet size: 1028 bytes
✓ Sample rate: 600 kHz
✓ Press Ctrl+C to exit

[Matplotlib window opens showing real-time waveform]

Statistics:
Frame: 12345
FPS:   15.2
Vpp:   3.15V
Max:   3.25V
Min:   0.10V
Pkts:  912
```

## Credits

Based on STM32 oscilloscope firmware using timer-triggered ADC + DMA + SPI slave.

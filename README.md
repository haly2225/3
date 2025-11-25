# Hybrid Oscilloscope - STM32 + Raspberry Pi 4

Based on [VanAn's DIY Oscilloscope](https://github.com/VanAn-nd/DIY-Oscilloscope-base-STM32F103C8T6) with modifications for Raspberry Pi 4 display.

## Architecture

- **STM32F103C8T6**: ADC sampling + SPI slave transmission
- **Raspberry Pi 4**: SPI master + Qt display (using `ol.cpp`)

## Hardware Configuration

### STM32F103C8T6 (Blue Pill)
- **System Clock**: 64MHz (HSI-based PLL)
- **ADC**: ADC1 Channel 0 (PA0)
- **Sample Rate**: ~600 kHz
- **Buffer Size**: 512 samples
- **SPI Mode**: Slave

### Connections (STM32 → Raspberry Pi 4)

| STM32 Pin | Function | RPi Pin | RPi Function |
|-----------|----------|---------|--------------|
| PA5 | SPI1_SCK | GPIO11 | SCLK |
| PA6 | SPI1_MISO | GPIO9 | MISO |
| PA7 | SPI1_MOSI | GPIO10 | MOSI |
| PA4 | SPI1_NSS | GPIO8 | CE0 |
| GND | Ground | GND | Ground |
| PA0 | ADC1_IN0 | - | Signal input |

## SPI Packet Format

### Structure

\`\`\`
┌──────┬──────┬────────────┬─────────────────────┐
│ 0xAA │ 0x55 │ Frame (2B) │ ADC Data (512×2 B)  │
└──────┴──────┴────────────┴─────────────────────┘
Total: 1028 bytes
\`\`\`

- **Byte 0-1**: Header markers (\`0xAA 0x55\`)
- **Byte 2-3**: Frame counter (big-endian)
- **Byte 4-1027**: ADC samples (512 samples × 2 bytes each, big-endian)

### ADC Data

- **Resolution**: 12-bit (0-4095)
- **Voltage Range**: 0 - 3.3V
- **Formula**: \`Voltage = (ADC_Value / 4095) × 3.3V\`

## Performance

| Parameter | Value |
|-----------|-------|
| System Clock | 64 MHz |
| ADC Clock | ~10.67 MHz (64MHz ÷ 6) |
| Sample Rate | ~600 kHz |
| Samples/Buffer | 512 |
| Buffer Time | ~0.85 ms |
| Packet Size | 1028 bytes |

## Build & Flash

1. Open project in STM32CubeIDE
2. Build configuration: Debug
3. Flash to STM32F103C8T6

## Usage with Raspberry Pi

The Raspberry Pi code (\`ol.cpp\`) reads packets via SPI and displays on Qt GUI. Current packet format is **compatible with existing \`ol.cpp\`**.

### Raspberry Pi SPI Configuration

\`\`\`bash
# Enable SPI on Raspberry Pi
sudo raspi-config
# Interface Options → SPI → Enable
\`\`\`

### Compile Qt Display

\`\`\`bash
g++ ol.cpp -o oscilloscope -std=c++17 $(pkg-config --cflags --libs Qt5Widgets) -fPIC
./oscilloscope
\`\`\`

## Features from VanAn Design

✅ **Adopted:**
- Timer-triggered ADC (precise sampling)
- DMA circular buffer
- Optimized ADC timing
- 64MHz clock

❌ **Removed:**
- ST7789 LCD display
- Dual ADC mode (simplified to single channel)
- Button controls
- On-board processing

## Modifications

| Original VanAn | This Project |
|----------------|--------------|
| ST7789 240×240 display | Raspberry Pi Qt display |
| Dual ADC (2 channels) | Single ADC (1 channel) |
| SPI Master (to LCD) | SPI Slave (to RPi) |
| 80MHz overclock | 64MHz (HSI PLL) |
| 1 MSPS/channel | 600 kHz |
| 2048 samples | 512 samples |

## Troubleshooting

### No data on Raspberry Pi
1. Check SPI wiring
2. Verify SPI is enabled on RPi
3. Check LED on PC13 is toggling (indicates ADC activity)

### Data corruption
1. Verify packet header (0xAA 0x55)
2. Reduce SPI clock speed on RPi (max 16 MHz)
3. Check ground connection

## Credits

- **Original Design**: Bui Van An (vanan92) - [DIY Oscilloscope v1.2.2](https://github.com/VanAn-nd/DIY-Oscilloscope-base-STM32F103C8T6)
- **Raspberry Pi Integration**: Modified for SPI slave transmission

## License

Feel free to use and modify this code!

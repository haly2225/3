# Dual Channel Oscilloscope - SPI Protocol Documentation

## Overview

This STM32 firmware implements a **dual-channel oscilloscope** that captures data from two ADC channels simultaneously and transmits the data to a Raspberry Pi 4 via SPI.

## Hardware Configuration

### STM32F103C8T6 (Blue Pill)
- **System Clock**: 80MHz (overclocked from 72MHz)
- **ADC Clock**: 80MHz / 6 = 13.33 MHz
- **Sampling Rate**: ~200 kHz per channel (configurable via TIM3)
- **Sample Buffer**: 2048 samples per channel

### ADC Channels
- **Channel 1**: PA2 → ADC1_IN2
- **Channel 2**: PA4 → ADC2_IN4

### SPI Interface (STM32 Slave Mode)
- **MOSI**: PA7 (SPI1_MOSI)
- **MISO**: PA6 (SPI1_MISO)
- **SCK**: PA5 (SPI1_SCK)
- **NSS**: PA4 (SPI1_NSS) - Hardware controlled
- **Mode**: Slave, 8-bit data, MSB first
- **Clock Polarity**: CPOL=0 (idle low)
- **Clock Phase**: CPHA=0 (1st edge)

## SPI Packet Format

### Packet Structure

```
┌──────────────┬──────────────┬───────────────┬─────────────────────────────────────┐
│ Byte 0       │ Byte 1       │ Byte 2-3      │ Byte 4 - 16,387                     │
├──────────────┼──────────────┼───────────────┼─────────────────────────────────────┤
│ 0xAA         │ 0x55         │ Frame Counter │ Interleaved CH1/CH2 Data            │
│ (Marker)     │ (Header)     │ (Big-endian)  │ (2048 samples × 2 channels × 2 bytes)│
└──────────────┴──────────────┴───────────────┴─────────────────────────────────────┘
```

**Total packet size**: 4 + (2048 × 2 × 2) = **16,388 bytes**

### Data Interleaving

The ADC data is interleaved as follows:

```
Offset  | Content
--------+------------------
4-5     | CH1[0] (16-bit, big-endian)
6-7     | CH2[0] (16-bit, big-endian)
8-9     | CH1[1]
10-11   | CH2[1]
...     | ...
16384-5 | CH1[2047]
16386-7 | CH2[2047]
```

### ADC Data Format

- **Resolution**: 12-bit (0-4095)
- **Voltage Range**: 0 - 3.3V
- **Conversion Formula**: `Voltage = (ADC_Value / 4095) × 3.3V`
- **Data Alignment**: Right-aligned, stored as 16-bit big-endian

## Timing Characteristics

### Dual ADC Simultaneous Mode

The STM32 uses **dual regular simultaneous mode** where both ADC1 and ADC2 are triggered simultaneously by TIM3 and convert in parallel.

- **ADC1**: Master ADC, triggers DMA
- **ADC2**: Slave ADC, synchronized with ADC1
- **DMA Transfer**: 32-bit words (ADC1 data in lower 16 bits, ADC2 data in upper 16 bits)

### Sampling Rate Calculation

```
Trigger Rate = 80 MHz / (Prescaler + 1) / (AutoReload + 1)

Current settings (TIM3):
- Prescaler = 0
- AutoReload = 399
- Rate = 80M / 1 / 400 = 200 kHz per channel
```

To achieve **1 MSPS** (like original VanAn design):
- Set `TIM_InitStruct.Autoreload = 79;`
- Rate = 80M / 1 / 80 = 1 MHz

### Data Acquisition Flow

```
TIM3 Trigger → ADC1 + ADC2 Convert (simultaneous)
            ↓
        DMA Transfer (32-bit)
            ↓
    DMA Complete Interrupt
            ↓
    Stop TIM3, Set conversion_ready flag
            ↓
        Main Loop Processing:
        - Extract CH1/CH2 from 32-bit buffer
        - Pack into SPI transmission buffer (interleaved)
        - Restart DMA
        - Restart TIM3
```

## Raspberry Pi 4 Integration

### SPI Master Configuration (Raspberry Pi)

- **SPI Device**: `/dev/spidev0.0`
- **Speed**: 16 MHz (recommended)
- **Mode**: SPI_MODE_0 (CPOL=0, CPHA=0)
- **Bits per word**: 8

### Packet Parsing Example (C++)

```cpp
constexpr uint16_t BUFFER_SIZE = 2048;
constexpr uint16_t PACKET_SIZE = 4 + BUFFER_SIZE * 2 * 2;  // 16,388 bytes

std::vector<uint8_t> rx_buf(PACKET_SIZE);
std::vector<float> ch1_voltage(BUFFER_SIZE);
std::vector<float> ch2_voltage(BUFFER_SIZE);

// Read from SPI
read(spi_fd, rx_buf.data(), PACKET_SIZE);

// Find packet header
if (rx_buf[0] == 0xAA && rx_buf[1] == 0x55) {
    uint16_t frame = (rx_buf[2] << 8) | rx_buf[3];

    // Extract interleaved data
    for (size_t i = 0; i < BUFFER_SIZE; i++) {
        size_t offset = 4 + i * 4;

        // CH1 data (big-endian)
        uint16_t ch1_raw = (rx_buf[offset + 0] << 8) | rx_buf[offset + 1];
        ch1_voltage[i] = ch1_raw * (3.3f / 4095.0f);

        // CH2 data (big-endian)
        uint16_t ch2_raw = (rx_buf[offset + 2] << 8) | rx_buf[offset + 3];
        ch2_voltage[i] = ch2_raw * (3.3f / 4095.0f);
    }
}
```

## Performance Characteristics

### Theoretical Maximum Sample Rate

- **ADC Conversion Time**: 1.5 cycles (sampling) + 12.5 cycles (conversion) = 14 cycles
- **ADC Clock**: 13.33 MHz
- **Max Sample Rate**: 13.33 MHz / 14 = ~950 kHz per channel

### Actual Sample Rate (Current Configuration)

- **Timer Trigger**: 200 kHz
- **Samples per Buffer**: 2048
- **Buffer Fill Time**: 2048 / 200k = 10.24 ms
- **Data Throughput**: 16,388 bytes / 10.24ms ≈ 1.6 MB/s

## Modifications from Original VanAn Design

| Feature | Original VanAn | This Implementation |
|---------|---------------|---------------------|
| Display | ST7789 LCD (240×240) | Raspberry Pi 4 via SPI |
| SPI Mode | Master (to LCD) | Slave (to RPi) |
| Data Format | Internal processing | Packet-based transmission |
| Trigger | Software with button input | External (RPi controlled) |
| Sample Rate | 1 MSPS | 200 kHz (adjustable) |
| Buffer Size | 2048 samples | 2048 samples |

## Troubleshooting

### No Data Received on Raspberry Pi

1. Check SPI wiring (MOSI, MISO, SCK, NSS)
2. Verify SPI clock speed is not too high (max 16 MHz recommended)
3. Ensure proper ground connection between STM32 and RPi
4. Check LED on PC13 is toggling (indicates ADC activity)

### Data Corruption

1. Verify packet header (0xAA 0x55) is present
2. Check for proper SPI mode configuration on both sides
3. Reduce SPI clock speed if errors persist
4. Add pull-up resistor (4.7kΩ) on NSS line if needed

### Lower Than Expected Sample Rate

1. Check TIM3 AutoReload value in `MX_TIM3_Init()`
2. Verify system clock is running at 80MHz (check with debugger)
3. Ensure DMA and ADC interrupts are not being blocked

## Future Enhancements

- [ ] Adjustable sample rate control from Raspberry Pi
- [ ] Hardware trigger input (external signal)
- [ ] Continuous streaming mode (no packet framing)
- [ ] Differential input mode support
- [ ] On-board signal conditioning (amplifier/attenuator control)

---

**Based on**: Bui Van An (vanan92) - DIY Oscilloscope v1.2.2
**Modified for**: Raspberry Pi 4 display integration
**Date**: November 2025

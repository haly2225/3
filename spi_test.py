#!/usr/bin/env python3
"""
Simple SPI Test - Read and display raw packets from STM32

Usage:
    python3 spi_test.py
"""

import spidev
import time
import sys

# SPI Configuration
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 8000000  # 8 MHz

# Packet Configuration
MARKER_START = 0xAA
MARKER_HEADER = 0x55
BUFFER_SIZE = 512
PACKET_SIZE = 4 + BUFFER_SIZE * 2

def test_spi():
    """Test SPI communication and display packets"""
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = SPI_SPEED
    spi.mode = 0

    print("═" * 60)
    print("  STM32 SPI Communication Test")
    print("═" * 60)
    print(f"SPI Bus:    {SPI_BUS}")
    print(f"SPI Device: {SPI_DEVICE}")
    print(f"Speed:      {SPI_SPEED/1e6:.1f} MHz")
    print(f"Packet:     {PACKET_SIZE} bytes")
    print("─" * 60)
    print("Press Ctrl+C to stop\n")

    packet_count = 0
    good_packets = 0

    try:
        while True:
            # Read packet
            rx_data = spi.readbytes(PACKET_SIZE)
            packet_count += 1

            # Find marker
            marker_pos = -1
            for i in range(len(rx_data) - 1):
                if rx_data[i] == MARKER_START and rx_data[i+1] == MARKER_HEADER:
                    marker_pos = i
                    break

            if marker_pos >= 0:
                # Parse frame number
                frame_num = (rx_data[marker_pos + 2] << 8) | rx_data[marker_pos + 3]

                # Parse first few samples
                samples = []
                for i in range(min(10, BUFFER_SIZE)):
                    offset = marker_pos + 4 + i * 2
                    if offset + 1 < len(rx_data):
                        adc_val = (rx_data[offset] << 8) | rx_data[offset + 1]
                        voltage = (adc_val / 4095) * 3.3
                        samples.append((adc_val, voltage))

                good_packets += 1

                # Display
                print(f"✓ Packet #{packet_count:4d} | Frame: {frame_num:5d} | Marker @{marker_pos:3d}")
                print(f"  First 10 samples:")
                for i, (adc, volt) in enumerate(samples):
                    print(f"    [{i:2d}] ADC: {adc:4d}  →  {volt:5.2f}V")
                print(f"  Success rate: {good_packets}/{packet_count} ({100*good_packets/packet_count:.1f}%)")
                print("─" * 60)
            else:
                print(f"✗ Packet #{packet_count:4d} | No marker found")
                # Show first 20 bytes for debugging
                print(f"  First 20 bytes: {' '.join(f'{b:02X}' for b in rx_data[:20])}")
                print("─" * 60)

            time.sleep(0.1)  # 100ms between reads

    except KeyboardInterrupt:
        print("\n\n⏹ Test stopped")
        print(f"Total packets: {packet_count}")
        print(f"Good packets:  {good_packets}")
        print(f"Success rate:  {100*good_packets/packet_count:.1f}%")
    finally:
        spi.close()
        print("✓ SPI closed")

if __name__ == "__main__":
    test_spi()

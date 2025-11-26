#!/usr/bin/env python3
"""
STM32 Dual Channel SPI Communication Test
Receives 2-channel oscilloscope data via SPI from STM32F103
"""

import spidev
import time
import sys

# SPI Configuration
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED_HZ = 8000000  # 8 MHz

# Packet Configuration
NUM_SAMPLES = 512  # Samples per channel
HEADER_BYTES = 4   # AA 55 + 2-byte frame counter
CH1_BYTES = NUM_SAMPLES * 2  # Channel 1 data (1024 bytes)
CH2_BYTES = NUM_SAMPLES * 2  # Channel 2 data (1024 bytes)
PACKET_SIZE = HEADER_BYTES + CH1_BYTES + CH2_BYTES  # 2052 bytes total

# ADC to Voltage conversion (12-bit ADC, 3.3V reference)
ADC_MAX = 4095
VREF = 3.3

def adc_to_voltage(adc_value):
    """Convert 12-bit ADC value to voltage"""
    return (adc_value / ADC_MAX) * VREF

def parse_packet(data):
    """Parse SPI packet into channels"""
    # Check for marker
    if data[0] != 0xAA or data[1] != 0x55:
        return None, None, None

    # Frame counter
    frame = (data[2] << 8) | data[3]

    # Channel 1 data (bytes 4 to 1027)
    ch1_samples = []
    for i in range(NUM_SAMPLES):
        idx = 4 + i * 2
        val = (data[idx] << 8) | data[idx + 1]
        ch1_samples.append(val)

    # Channel 2 data (bytes 1028 to 2051)
    ch2_samples = []
    for i in range(NUM_SAMPLES):
        idx = 4 + CH1_BYTES + i * 2
        val = (data[idx] << 8) | data[idx + 1]
        ch2_samples.append(val)

    return frame, ch1_samples, ch2_samples

def calculate_stats(samples, channel_name):
    """Calculate statistics for a channel"""
    if not samples:
        return {}

    voltages = [adc_to_voltage(s) for s in samples]
    vmin = min(voltages)
    vmax = max(voltages)
    vavg = sum(voltages) / len(voltages)
    vpp = vmax - vmin

    return {
        'vmin': vmin,
        'vmax': vmax,
        'vavg': vavg,
        'vpp': vpp,
        'channel': channel_name
    }

def main():
    print("=" * 60)
    print("  STM32 Dual Channel SPI Communication Test")
    print("=" * 60)

    # Initialize SPI
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = SPI_SPEED_HZ
    spi.mode = 0  # Mode 0: CPOL=0, CPHA=0

    print(f"SPI Bus:    {SPI_BUS}")
    print(f"SPI Device: {SPI_DEVICE}")
    print(f"Speed:      {SPI_SPEED_HZ/1e6:.1f} MHz")
    print(f"Packet:     {PACKET_SIZE} bytes ({NUM_SAMPLES} samples/channel)")
    print("-" * 60)
    print("Press Ctrl+C to stop\n")

    packet_count = 0
    success_count = 0
    last_frame = None

    try:
        while True:
            # Read packet from STM32
            data = spi.readbytes(PACKET_SIZE)
            packet_count += 1

            # Parse packet
            frame, ch1, ch2 = parse_packet(data)

            if frame is None:
                print(f"✗ Packet # {packet_count:3d} | No marker found")
                print(f"  First 20 bytes: {' '.join(f'{b:02X}' for b in data[:20])}")
            else:
                success_count += 1

                # Calculate statistics
                ch1_stats = calculate_stats(ch1, "CH1")
                ch2_stats = calculate_stats(ch2, "CH2")

                # Detect frame drops
                frame_status = ""
                if last_frame is not None:
                    expected = (last_frame + 1) & 0xFFFF
                    if frame != expected:
                        dropped = (frame - expected) & 0xFFFF
                        frame_status = f" [DROPPED {dropped} frames!]"
                last_frame = frame

                print(f"✓ Packet # {packet_count:3d} | Frame: {frame:5d}{frame_status}")

                # Show CH1 stats
                print(f"  CH1 (PA2): Vavg={ch1_stats['vavg']:.2f}V  "
                      f"Vmin={ch1_stats['vmin']:.2f}V  Vmax={ch1_stats['vmax']:.2f}V  "
                      f"Vpp={ch1_stats['vpp']:.2f}V")

                # Show CH2 stats
                print(f"  CH2 (PA4): Vavg={ch2_stats['vavg']:.2f}V  "
                      f"Vmin={ch2_stats['vmin']:.2f}V  Vmax={ch2_stats['vmax']:.2f}V  "
                      f"Vpp={ch2_stats['vpp']:.2f}V")

                # Show first few samples from each channel
                print(f"  CH1 first 5: {' '.join(f'{s:4d}' for s in ch1[:5])}")
                print(f"  CH2 first 5: {' '.join(f'{s:4d}' for s in ch2[:5])}")

                print(f"  Success rate: {success_count}/{packet_count} ({100.0*success_count/packet_count:.1f}%)")

            print("-" * 60)
            time.sleep(0.1)  # 100ms between reads

    except KeyboardInterrupt:
        print("\n\nStopped by user")
    finally:
        spi.close()
        print(f"\nFinal statistics:")
        print(f"  Total packets: {packet_count}")
        print(f"  Successful:    {success_count} ({100.0*success_count/packet_count:.1f}%)")
        print(f"  Failed:        {packet_count - success_count}")

if __name__ == "__main__":
    main()

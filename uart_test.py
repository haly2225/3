#!/usr/bin/env python3
"""
STM32 UART Communication Test
Reads oscilloscope data from STM32 via UART (Serial port)
Much more reliable than SPI slave mode!

Hardware connection:
  STM32 Blue Pill  <->  USB-to-Serial Adapter (or Pi4 UART)
  PA9  (UART TX)   ->   RX
  PA10 (UART RX)   ->   TX  (optional, not used for one-way data)
  GND              ->   GND

Or connect directly to Raspberry Pi 4:
  STM32 PA9 -> Pi4 GPIO15 (RXD)
  STM32 PA10 -> Pi4 GPIO14 (TXD)
  GND -> GND

Usage:
  python3 uart_test.py [port] [baudrate]

Examples:
  python3 uart_test.py /dev/ttyUSB0 2000000
  python3 uart_test.py /dev/ttyAMA0 2000000  # Pi4 hardware UART
"""

import serial
import time
import sys

# Packet format
MARKER_START = 0xAA
MARKER_HEADER = 0x55
BUFFER_SIZE = 512
PACKET_SIZE = 4 + BUFFER_SIZE * 2  # 1028 bytes total

# ADC constants
ADC_MAX = 4095  # 12-bit ADC
VCC = 3.3       # Reference voltage

def find_packet_marker(data):
    """Find AA 55 marker in data stream"""
    for i in range(len(data) - 1):
        if data[i] == MARKER_START and data[i+1] == MARKER_HEADER:
            return i
    return -1

def parse_packet(data, marker_pos):
    """Parse packet from data starting at marker position"""
    if marker_pos + PACKET_SIZE > len(data):
        return None

    # Extract frame counter
    frame = (data[marker_pos + 2] << 8) | data[marker_pos + 3]

    # Extract ADC samples
    samples = []
    for i in range(BUFFER_SIZE):
        offset = marker_pos + 4 + i * 2
        if offset + 1 < len(data):
            adc_val = (data[offset] << 8) | data[offset + 1]
            samples.append(adc_val)

    return {'frame': frame, 'samples': samples}

def main():
    # Parse command line arguments
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 2000000

    print("╔══════════════════════════════════════════════════════════════╗")
    print("  STM32 UART Communication Test")
    print("╚══════════════════════════════════════════════════════════════╝")
    print(f"Port:       {port}")
    print(f"Baudrate:   {baudrate:,} bps")
    print(f"Packet:     {PACKET_SIZE} bytes")
    print("─" * 60)
    print("Press Ctrl+C to stop\n")

    try:
        # Open serial port
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0
        )

        print(f"✓ Serial port {port} opened successfully")

        # Statistics
        packet_count = 0
        success_count = 0
        last_frame = -1

        # Sync: read until we find a marker
        print("⌛ Syncing to packet stream...")
        sync_data = bytearray()
        while True:
            chunk = ser.read(100)
            if not chunk:
                print("⚠ No data received. Is STM32 powered and programmed?")
                time.sleep(1)
                continue
            sync_data.extend(chunk)
            marker_pos = find_packet_marker(sync_data)
            if marker_pos >= 0:
                # Discard everything before marker
                sync_data = sync_data[marker_pos:]
                print(f"✓ Found marker at position {marker_pos}, starting read...\n")
                break
            # Keep last 10 bytes in case marker is split
            sync_data = sync_data[-10:]

        # Main read loop
        buffer = bytearray(sync_data)
        while True:
            # Read enough data for at least one packet
            chunk = ser.read(PACKET_SIZE * 2)
            if chunk:
                buffer.extend(chunk)

            # Try to parse packet
            marker_pos = find_packet_marker(buffer)
            if marker_pos >= 0 and len(buffer) >= marker_pos + PACKET_SIZE:
                packet = parse_packet(buffer, marker_pos)
                packet_count += 1

                if packet and len(packet['samples']) == BUFFER_SIZE:
                    success_count += 1
                    frame = packet['frame']
                    samples = packet['samples']

                    # Check frame continuity
                    if last_frame >= 0:
                        expected = (last_frame + 1) & 0xFFFF
                        if frame != expected:
                            print(f"⚠ Frame skip: {last_frame} -> {frame} (expected {expected})")
                    last_frame = frame

                    # Calculate statistics from samples
                    min_val = min(samples)
                    max_val = max(samples)
                    avg_val = sum(samples) / len(samples)
                    min_v = (min_val / ADC_MAX) * VCC
                    max_v = (max_val / ADC_MAX) * VCC
                    avg_v = (avg_val / ADC_MAX) * VCC
                    vpp = max_v - min_v

                    # Print packet info
                    print(f"✓ Packet #{packet_count:4d} | Frame: {frame:04d} | "
                          f"Samples: {len(samples):3d} | "
                          f"Vpp: {vpp:.3f}V | Avg: {avg_v:.3f}V")

                    # Show first 10 samples every 10 packets
                    if packet_count % 10 == 1:
                        sample_str = ", ".join([f"{s:4d}" for s in samples[:10]])
                        print(f"  First 10 ADC values: {sample_str}...")
                        voltage_str = ", ".join([f"{(s/ADC_MAX)*VCC:.2f}V" for s in samples[:10]])
                        print(f"  First 10 voltages:   {voltage_str}...")

                else:
                    print(f"✗ Packet #{packet_count:4d} | Parse error")

                # Remove processed packet from buffer
                buffer = buffer[marker_pos + PACKET_SIZE:]
            else:
                # Not enough data yet, keep reading
                time.sleep(0.01)

    except serial.SerialException as e:
        print(f"\n✗ Serial port error: {e}")
        print("\nTroubleshooting:")
        print("  1. Check if device is connected: ls -l /dev/ttyUSB* /dev/ttyAMA*")
        print("  2. Add user to dialout group: sudo usermod -a -G dialout $USER")
        print("  3. Check permissions: sudo chmod 666 /dev/ttyUSB0")
        print("  4. Try different port: /dev/ttyUSB0, /dev/ttyUSB1, /dev/ttyAMA0")
        return 1

    except KeyboardInterrupt:
        print("\n\n─" * 60)
        print(f"📊 Statistics:")
        print(f"  Total packets:     {packet_count}")
        print(f"  Successful:        {success_count}")
        if packet_count > 0:
            success_rate = (success_count / packet_count) * 100
            print(f"  Success rate:      {success_rate:.1f}%")
        print("─" * 60)

    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("✓ Serial port closed")

    return 0

if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
STM32 Oscilloscope - UART Version
Real-time oscilloscope display using UART communication

Hardware connection:
  STM32 PA9 (UART TX) -> USB-to-Serial RX (or Pi4 GPIO15)
  STM32 GND -> GND

Usage:
  python3 oscilloscope_uart.py [port] [baudrate]

Examples:
  python3 oscilloscope_uart.py /dev/ttyUSB0 2000000
  python3 oscilloscope_uart.py /dev/ttyAMA0 2000000
"""

import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time
import sys

# Packet format
MARKER_START = 0xAA
MARKER_HEADER = 0x55
BUFFER_SIZE = 512
PACKET_SIZE = 4 + BUFFER_SIZE * 2

# ADC constants
ADC_MAX = 4095
VCC = 3.3

# Display settings
DISPLAY_SAMPLES = BUFFER_SIZE
UPDATE_INTERVAL = 50  # ms, ~20 FPS

class OscilloscopeUART:
    def __init__(self, port='/dev/ttyUSB0', baudrate=2000000):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.buffer = bytearray()

        # Data storage
        self.voltage_data = np.zeros(DISPLAY_SAMPLES)
        self.time_axis = np.arange(DISPLAY_SAMPLES) / 600000  # 600kHz sample rate

        # Statistics
        self.frame_count = 0
        self.last_update_time = time.time()
        self.fps_buffer = deque(maxlen=30)

        # Setup plot
        self.setup_plot()

        # Open serial port
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            print(f"✓ Connected to {port} at {baudrate:,} bps")

            # Sync to data stream
            print("⌛ Syncing to packet stream...")
            self.sync_to_stream()
            print("✓ Synced successfully!\n")

        except serial.SerialException as e:
            print(f"✗ Error opening {port}: {e}")
            sys.exit(1)

    def sync_to_stream(self):
        """Sync to the packet stream by finding the first marker"""
        sync_data = bytearray()
        for _ in range(100):  # Try 100 times
            chunk = self.ser.read(100)
            if chunk:
                sync_data.extend(chunk)
                marker_pos = self.find_marker(sync_data)
                if marker_pos >= 0:
                    self.buffer = sync_data[marker_pos:]
                    return
                sync_data = sync_data[-10:]  # Keep last 10 bytes
            time.sleep(0.01)
        raise Exception("Could not sync to data stream")

    def find_marker(self, data):
        """Find AA 55 marker in data"""
        for i in range(len(data) - 1):
            if data[i] == MARKER_START and data[i+1] == MARKER_HEADER:
                return i
        return -1

    def read_packet(self):
        """Read one complete packet from serial"""
        # Read more data
        chunk = self.ser.read(PACKET_SIZE * 2)
        if chunk:
            self.buffer.extend(chunk)

        # Try to parse packet
        marker_pos = self.find_marker(self.buffer)
        if marker_pos >= 0 and len(self.buffer) >= marker_pos + PACKET_SIZE:
            # Extract frame counter
            frame = (self.buffer[marker_pos + 2] << 8) | self.buffer[marker_pos + 3]

            # Extract samples
            samples = []
            for i in range(BUFFER_SIZE):
                offset = marker_pos + 4 + i * 2
                adc_val = (self.buffer[offset] << 8) | self.buffer[offset + 1]
                voltage = (adc_val / ADC_MAX) * VCC
                samples.append(voltage)

            # Remove processed data
            self.buffer = self.buffer[marker_pos + PACKET_SIZE:]

            return np.array(samples)

        return None

    def setup_plot(self):
        """Setup matplotlib figure"""
        plt.style.use('dark_background')
        self.fig, self.ax = plt.subplots(figsize=(12, 6))
        self.fig.canvas.manager.set_window_title('STM32 Oscilloscope - UART')

        # Main waveform plot
        self.line, = self.ax.plot(self.time_axis * 1e6, self.voltage_data,
                                   color='#00ff00', linewidth=1.5)

        # Axis labels
        self.ax.set_xlabel('Time (µs)', fontsize=12, color='white')
        self.ax.set_ylabel('Voltage (V)', fontsize=12, color='white')
        self.ax.set_title('STM32 Oscilloscope - UART Communication',
                         fontsize=14, color='white', pad=20)

        # Grid
        self.ax.grid(True, alpha=0.3, linestyle='--')
        self.ax.set_facecolor('#000000')

        # Set initial limits
        self.ax.set_xlim(0, (DISPLAY_SAMPLES / 600000) * 1e6)
        self.ax.set_ylim(0, VCC)

        # Stats text
        self.stats_text = self.ax.text(0.02, 0.98, '', transform=self.ax.transAxes,
                                       verticalalignment='top', fontsize=10,
                                       color='yellow', family='monospace',
                                       bbox=dict(boxstyle='round', facecolor='black',
                                               alpha=0.7))

    def update_plot(self, frame_num):
        """Animation update function"""
        # Read new data
        samples = self.read_packet()

        if samples is not None:
            self.voltage_data = samples
            self.frame_count += 1

            # Update plot
            self.line.set_ydata(self.voltage_data)

            # Calculate statistics
            v_max = np.max(self.voltage_data)
            v_min = np.min(self.voltage_data)
            v_pp = v_max - v_min
            v_avg = np.mean(self.voltage_data)

            # Calculate FPS
            current_time = time.time()
            dt = current_time - self.last_update_time
            if dt > 0:
                fps = 1.0 / dt
                self.fps_buffer.append(fps)
            self.last_update_time = current_time
            avg_fps = np.mean(self.fps_buffer) if len(self.fps_buffer) > 0 else 0

            # Update stats text
            stats = (f"Frame: {self.frame_count:6d}\n"
                    f"Vpp:   {v_pp:5.3f} V\n"
                    f"Max:   {v_max:5.3f} V\n"
                    f"Min:   {v_min:5.3f} V\n"
                    f"Avg:   {v_avg:5.3f} V\n"
                    f"FPS:   {avg_fps:5.1f}")
            self.stats_text.set_text(stats)

            # Auto-scale Y axis if needed
            if v_max > VCC * 0.95 or v_min < 0.05:
                self.ax.set_ylim(v_min - 0.1, v_max + 0.1)
            else:
                self.ax.set_ylim(0, VCC)

        return self.line, self.stats_text

    def run(self):
        """Start the oscilloscope display"""
        ani = animation.FuncAnimation(self.fig, self.update_plot,
                                     interval=UPDATE_INTERVAL,
                                     blit=True, cache_frame_data=False)
        plt.tight_layout()
        plt.show()

    def close(self):
        """Close serial port"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("✓ Serial port closed")

def main():
    # Parse command line arguments
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 2000000

    print("╔══════════════════════════════════════════════════════════════╗")
    print("  STM32 Oscilloscope - UART Communication")
    print("╚══════════════════════════════════════════════════════════════╝")
    print()

    scope = None
    try:
        scope = OscilloscopeUART(port, baudrate)
        scope.run()
    except KeyboardInterrupt:
        print("\n✓ Stopped by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if scope:
            scope.close()

if __name__ == "__main__":
    main()

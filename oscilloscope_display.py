#!/usr/bin/env python3
"""
STM32 Oscilloscope Display for Raspberry Pi 4
Real-time oscilloscope display reading data from STM32 via SPI

Requirements:
    pip3 install spidev numpy matplotlib

Usage:
    python3 oscilloscope_display.py
"""

import spidev
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time
import struct

# SPI Configuration
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 8000000  # 8 MHz (safe speed)

# Packet Configuration
MARKER_START = 0xAA
MARKER_HEADER = 0x55
BUFFER_SIZE = 512
PACKET_SIZE = 4 + BUFFER_SIZE * 2  # Header (4) + Data (512*2) = 1028 bytes

# ADC Configuration
ADC_MAX = 4095
VCC = 3.3
SAMPLE_RATE = 600000.0  # 600 kHz

class OscilloscopeDisplay:
    def __init__(self):
        # Initialize SPI
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.max_speed_hz = SPI_SPEED
        self.spi.mode = 0  # SPI Mode 0 (CPOL=0, CPHA=0)

        # Data buffers
        self.voltage_buffer = deque(maxlen=BUFFER_SIZE)
        self.time_buffer = deque(maxlen=BUFFER_SIZE)

        # Statistics
        self.frame_count = 0
        self.good_packets = 0
        self.last_frame_num = None
        self.fps = 0
        self.last_fps_time = time.time()

        # Setup plot
        self.setup_plot()

        print(f"✓ SPI initialized: {SPI_SPEED/1e6:.1f} MHz")
        print(f"✓ Packet size: {PACKET_SIZE} bytes")
        print(f"✓ Sample rate: {SAMPLE_RATE/1e3:.0f} kHz")
        print(f"✓ Press Ctrl+C to exit\n")

    def setup_plot(self):
        """Setup matplotlib figure and axes"""
        plt.style.use('dark_background')
        self.fig, self.ax = plt.subplots(figsize=(12, 6))

        # Grid
        self.ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
        self.ax.set_facecolor('#0a0a0a')

        # Labels
        self.ax.set_xlabel('Time (ms)', fontsize=12, color='#00ff00')
        self.ax.set_ylabel('Voltage (V)', fontsize=12, color='#00ff00')
        self.ax.set_title('STM32 Oscilloscope - 600kHz',
                         fontsize=14, color='#ffff00', fontweight='bold')

        # Y-axis limits
        self.ax.set_ylim(-0.2, 3.5)

        # Line object for waveform
        self.line, = self.ax.plot([], [], color='#ffff00', linewidth=1.5, label='ADC Signal')

        # Text for statistics
        self.stats_text = self.ax.text(0.02, 0.98, '', transform=self.ax.transAxes,
                                       fontsize=10, verticalalignment='top',
                                       color='#00ffff', family='monospace',
                                       bbox=dict(boxstyle='round', facecolor='black', alpha=0.7))

        self.ax.legend(loc='upper right', fontsize=10)
        plt.tight_layout()

    def read_spi_packet(self):
        """Read one packet from SPI"""
        try:
            # Read packet
            rx_data = self.spi.readbytes(PACKET_SIZE)

            # Find marker
            marker_pos = -1
            for i in range(len(rx_data) - 1):
                if rx_data[i] == MARKER_START and rx_data[i+1] == MARKER_HEADER:
                    marker_pos = i
                    break

            if marker_pos < 0:
                return None

            # Parse header
            if marker_pos + 3 >= len(rx_data):
                return None

            frame_num = (rx_data[marker_pos + 2] << 8) | rx_data[marker_pos + 3]

            # Parse ADC data
            samples = []
            for i in range(BUFFER_SIZE):
                offset = marker_pos + 4 + i * 2
                if offset + 1 >= len(rx_data):
                    break

                # Big-endian 16-bit
                adc_val = (rx_data[offset] << 8) | rx_data[offset + 1]

                if adc_val > ADC_MAX:
                    continue

                voltage = (adc_val / ADC_MAX) * VCC
                samples.append(voltage)

            if len(samples) != BUFFER_SIZE:
                return None

            self.good_packets += 1
            return {'frame': frame_num, 'samples': samples}

        except Exception as e:
            print(f"SPI read error: {e}")
            return None

    def update_plot(self, frame):
        """Animation update function"""
        # Read new packet
        packet = self.read_spi_packet()

        if packet is None:
            return self.line, self.stats_text

        self.frame_count += 1
        samples = packet['samples']
        frame_num = packet['frame']

        # Update buffers
        self.voltage_buffer.clear()
        self.time_buffer.clear()

        for i, voltage in enumerate(samples):
            self.voltage_buffer.append(voltage)
            self.time_buffer.append(i / SAMPLE_RATE * 1000)  # Convert to ms

        # Update line data
        self.line.set_data(list(self.time_buffer), list(self.voltage_buffer))

        # Auto-scale X axis
        if len(self.time_buffer) > 0:
            self.ax.set_xlim(0, max(self.time_buffer) * 1.1)

        # Calculate statistics
        vmin = min(self.voltage_buffer) if len(self.voltage_buffer) > 0 else 0
        vmax = max(self.voltage_buffer) if len(self.voltage_buffer) > 0 else 0
        vpp = vmax - vmin

        # Calculate FPS
        now = time.time()
        if now - self.last_fps_time >= 1.0:
            self.fps = self.frame_count / (now - self.last_fps_time)
            self.frame_count = 0
            self.last_fps_time = now

        # Update statistics text
        stats_str = f"Frame: {frame_num:5d}\n"
        stats_str += f"FPS:   {self.fps:5.1f}\n"
        stats_str += f"Vpp:   {vpp:5.2f}V\n"
        stats_str += f"Max:   {vmax:5.2f}V\n"
        stats_str += f"Min:   {vmin:5.2f}V\n"
        stats_str += f"Pkts:  {self.good_packets:5d}"

        self.stats_text.set_text(stats_str)

        return self.line, self.stats_text

    def run(self):
        """Start animation"""
        ani = animation.FuncAnimation(self.fig, self.update_plot,
                                     interval=50,  # 50ms update (20 FPS max)
                                     blit=True,
                                     cache_frame_data=False)
        plt.show()

    def close(self):
        """Cleanup"""
        self.spi.close()
        print("\n✓ SPI closed")

def main():
    try:
        scope = OscilloscopeDisplay()
        scope.run()
    except KeyboardInterrupt:
        print("\n\n⏹ Stopped by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            scope.close()
        except:
            pass

if __name__ == "__main__":
    main()

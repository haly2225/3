#!/usr/bin/env python3
"""
STM32 Oscilloscope - 3 CHANNEL VERSION
Supports packet validation with markers and checksum
For Raspberry Pi 4 + STM32F103C8T6
"""

import sys
import time
import threading
import struct
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

# Protocol constants (must match STM32)
CMD_START = 0xF1
CMD_STOP = 0xF2
CMD_DATA = 0xF3

MARKER_START = 0x1234
MARKER_END = 0x5678

BLOCK_SIZE = 512
NUM_CHANNELS = 3
SAMPLES_PER_CHANNEL = BLOCK_SIZE

# Packet structure
PACKET_HEADER_SIZE = 2  # start_marker (uint16)
PACKET_DATA_SIZE = BLOCK_SIZE * NUM_CHANNELS * 2  # data array (uint16 * samples * channels)
PACKET_FOOTER_SIZE = 4  # end_marker (uint16) + checksum (uint16)
PACKET_TOTAL_SIZE = PACKET_HEADER_SIZE + PACKET_DATA_SIZE + PACKET_FOOTER_SIZE

SAMPLE_RATE = 200000  # Per-channel sample rate (600kHz total / 3 channels)


class DataPacket:
    """Represents a data packet from STM32"""
    def __init__(self):
        self.start_marker = 0
        self.data = np.zeros(BLOCK_SIZE * NUM_CHANNELS, dtype=np.uint16)
        self.end_marker = 0
        self.checksum = 0
        self.valid = False


class STM32Scope(QtCore.QObject):
    data_ready = QtCore.pyqtSignal(np.ndarray, np.ndarray, np.ndarray)  # CH0, CH1, CH2
    status_update = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._run = False
        self.spi = None

    def start(self):
        try:
            import spidev
        except ImportError:
            print("ERROR: pip3 install spidev")
            sys.exit(1)

        self._run = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._run = False
        if self.spi:
            try:
                self.spi.xfer2([CMD_STOP])
                self.spi.close()
            except:
                pass

    def _init_spi(self):
        if self.spi:
            return

        import spidev
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 2000000  # 2MHz for stable transfer
        self.spi.mode = 0  # CPOL=0, CPHA=0
        self.spi.lsbfirst = False  # MSB first

    def _parse_packet(self, raw_bytes):
        """Parse and validate packet from STM32"""
        packet = DataPacket()

        if len(raw_bytes) < PACKET_TOTAL_SIZE:
            return packet

        try:
            # Parse header
            packet.start_marker = struct.unpack('<H', raw_bytes[0:2])[0]

            # Parse data array (interleaved channels)
            data_bytes = raw_bytes[2:2 + PACKET_DATA_SIZE]
            packet.data = np.frombuffer(data_bytes, dtype='<u2')  # Little-endian uint16

            # Parse footer
            footer_offset = 2 + PACKET_DATA_SIZE
            packet.end_marker = struct.unpack('<H', raw_bytes[footer_offset:footer_offset+2])[0]
            packet.checksum = struct.unpack('<H', raw_bytes[footer_offset+2:footer_offset+4])[0]

            # Validate markers
            if packet.start_marker != MARKER_START:
                return packet

            if packet.end_marker != MARKER_END:
                return packet

            # Validate checksum
            calculated_sum = MARKER_START + np.sum(packet.data, dtype=np.uint32) + MARKER_END
            calculated_checksum = calculated_sum & 0xFFFF

            if packet.checksum != calculated_checksum:
                return packet

            # Validate data range (12-bit ADC)
            if np.max(packet.data) > 4095:
                return packet

            packet.valid = True

        except Exception as e:
            pass

        return packet

    def _deinterleave_channels(self, interleaved_data):
        """Split interleaved data into separate channels"""
        # Data format: CH0, CH1, CH2, CH0, CH1, CH2, ...
        ch0 = interleaved_data[0::3]
        ch1 = interleaved_data[1::3]
        ch2 = interleaved_data[2::3]
        return ch0, ch1, ch2

    def _loop(self):
        try:
            self._init_spi()

            # Start acquisition
            self.spi.xfer2([CMD_START])
            time.sleep(0.5)

            frame_count = 0
            good_frames = 0
            bad_frames = 0
            last_fps_time = time.time()

            while self._run:
                try:
                    # Request data
                    self.spi.xfer2([CMD_DATA])
                    time.sleep(0.015)  # Wait for STM32 to prepare data

                    # Read packet
                    raw_data = bytearray()
                    chunk_size = 1024
                    remaining = PACKET_TOTAL_SIZE

                    while remaining > 0:
                        to_read = min(chunk_size, remaining)
                        chunk = self.spi.readbytes(to_read)
                        raw_data.extend(chunk)
                        remaining -= len(chunk)
                        if remaining > 0:
                            time.sleep(0.002)

                    # Parse and validate packet
                    packet = self._parse_packet(raw_data)

                    if packet.valid:
                        # De-interleave channels
                        ch0, ch1, ch2 = self._deinterleave_channels(packet.data)

                        # Convert to voltage
                        ch0_voltage = ch0.astype(np.float32) * (3.3 / 4095.0)
                        ch1_voltage = ch1.astype(np.float32) * (3.3 / 4095.0)
                        ch2_voltage = ch2.astype(np.float32) * (3.3 / 4095.0)

                        self.data_ready.emit(ch0_voltage, ch1_voltage, ch2_voltage)
                        good_frames += 1
                    else:
                        bad_frames += 1

                    # Update FPS
                    frame_count += 1
                    if time.time() - last_fps_time >= 2.0:
                        fps = frame_count / (time.time() - last_fps_time)
                        success_rate = good_frames / (good_frames + bad_frames) * 100 if (good_frames + bad_frames) > 0 else 0
                        self.status_update.emit(
                            f"FPS: {fps:.1f} | Success: {success_rate:.0f}% | Good: {good_frames} | Bad: {bad_frames}"
                        )
                        frame_count = 0
                        good_frames = 0
                        bad_frames = 0
                        last_fps_time = time.time()

                    time.sleep(0.02)

                except Exception as e:
                    print(f"Read error: {e}")
                    bad_frames += 1
                    time.sleep(0.1)

        except Exception as e:
            print(f"Fatal error: {e}")
            import traceback
            traceback.print_exc()


class ScopeGUI(QtWidgets.QWidget):

    def __init__(self, scope):
        super().__init__()
        self.scope = scope

        self.setWindowTitle("STM32 Oscilloscope - 3 Channels")
        self.resize(1400, 900)
        self.setStyleSheet("background:#0a0a0a;")

        # Data buffers
        self.ch0_data = np.zeros(SAMPLES_PER_CHANNEL, dtype=np.float32)
        self.ch1_data = np.zeros(SAMPLES_PER_CHANNEL, dtype=np.float32)
        self.ch2_data = np.zeros(SAMPLES_PER_CHANNEL, dtype=np.float32)

        self.hold = False
        self.trigger_mode = 1  # 0=Spectrum, 1=Rising, 2=Falling, 3=None
        self.trigger_level = 1.65
        self.trigger_channel = 0

        self._build_ui()

        self.scope.data_ready.connect(self.on_data)
        self.scope.status_update.connect(self.on_status)
        self.scope.start()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_display)
        self.timer.start(30)

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setSpacing(0)
        layout.setContentsMargins(0, 0, 0, 0)

        # Header
        header = self._create_header()
        layout.addWidget(header)

        # Plots
        plots_widget = QtWidgets.QWidget()
        plots_layout = QtWidgets.QVBoxLayout(plots_widget)
        plots_layout.setSpacing(5)

        self.plot_ch0 = self._create_plot("Channel 1 (PA0)", (0, 255, 0))
        self.plot_ch1 = self._create_plot("Channel 2 (PA1)", (255, 255, 0))
        self.plot_ch2 = self._create_plot("Channel 3 (PB0)", (0, 255, 255))

        plots_layout.addWidget(self.plot_ch0)
        plots_layout.addWidget(self.plot_ch1)
        plots_layout.addWidget(self.plot_ch2)

        layout.addWidget(plots_widget)

        # Controls
        controls = self._create_controls()
        layout.addWidget(controls)

    def _create_header(self):
        header = QtWidgets.QWidget()
        header.setFixedHeight(70)
        header.setStyleSheet("background:#0f0f0f; border-bottom:3px solid #0a0;")
        h_layout = QtWidgets.QVBoxLayout(header)

        title = QtWidgets.QLabel("⚡ STM32 OSCILLOSCOPE - 3 CHANNELS")
        title.setFont(QtGui.QFont("Monospace", 18, QtGui.QFont.Bold))
        title.setStyleSheet("color:#0f0;")
        h_layout.addWidget(title)

        self.lbl_status = QtWidgets.QLabel("Initializing...")
        self.lbl_status.setStyleSheet("color:#ff0; font-size:12px; font-weight:bold;")
        h_layout.addWidget(self.lbl_status)

        return header

    def _create_plot(self, title, color):
        pg.setConfigOptions(antialias=False)

        plot = pg.PlotWidget()
        plot.setBackground((5, 5, 5))
        plot.showGrid(x=True, y=True, alpha=0.2)
        plot.setLabel('left', 'Voltage (V)', **{'color': color})
        plot.setLabel('bottom', 'Sample', **{'color': '#888'})
        plot.setTitle(title, color=color, size='14pt')

        curve = plot.plot(pen=pg.mkPen(color, width=2))
        plot.curve = curve

        trig_line = pg.InfiniteLine(
            angle=0, pos=self.trigger_level,
            pen=pg.mkPen((255, 128, 0), width=2, style=QtCore.Qt.DashLine)
        )
        plot.addItem(trig_line)
        plot.trig_line = trig_line

        plot.setYRange(0, 3.3, padding=0.02)
        plot.setXRange(0, SAMPLES_PER_CHANNEL, padding=0)

        return plot

    def _create_controls(self):
        controls = QtWidgets.QWidget()
        controls.setFixedHeight(120)
        controls.setStyleSheet("background:#0f0f0f; border-top:3px solid #333;")
        c_layout = QtWidgets.QHBoxLayout(controls)
        c_layout.setContentsMargins(20, 10, 20, 10)

        # Hold button
        self.btn_hold = QtWidgets.QPushButton("▶ RUNNING")
        self.btn_hold.setCheckable(True)
        self.btn_hold.setFixedSize(140, 50)
        self.btn_hold.setStyleSheet("""
            QPushButton {
                background:#0a0; color:#fff; font-size:16px;
                font-weight:bold; border:3px solid #0f0; border-radius:6px;
            }
            QPushButton:checked {
                background:#a00; border-color:#f00;
            }
        """)
        self.btn_hold.clicked.connect(self.toggle_hold)
        c_layout.addWidget(self.btn_hold)

        c_layout.addSpacing(30)

        # Measurements
        meas_widget = QtWidgets.QWidget()
        meas_widget.setStyleSheet("QLabel { color:#0ff; font-size:13px; font-weight:bold; }")
        m_layout = QtWidgets.QGridLayout(meas_widget)
        m_layout.setSpacing(15)

        self.lbl_ch0_vpp = QtWidgets.QLabel("CH1: ---")
        self.lbl_ch1_vpp = QtWidgets.QLabel("CH2: ---")
        self.lbl_ch2_vpp = QtWidgets.QLabel("CH3: ---")
        self.lbl_freq = QtWidgets.QLabel("Freq: ---")
        self.lbl_trigger = QtWidgets.QLabel("Trigger: CH0 @ 1.65V")
        self.lbl_sample_rate = QtWidgets.QLabel(f"Sample Rate: {SAMPLE_RATE//1000}kHz/ch")

        self.lbl_ch0_vpp.setStyleSheet("color:#0f0; font-size:14px; font-weight:bold;")
        self.lbl_ch1_vpp.setStyleSheet("color:#ff0; font-size:14px; font-weight:bold;")
        self.lbl_ch2_vpp.setStyleSheet("color:#0ff; font-size:14px; font-weight:bold;")
        self.lbl_freq.setStyleSheet("color:#f80; font-size:14px; font-weight:bold;")

        m_layout.addWidget(self.lbl_ch0_vpp, 0, 0)
        m_layout.addWidget(self.lbl_ch1_vpp, 0, 1)
        m_layout.addWidget(self.lbl_ch2_vpp, 0, 2)
        m_layout.addWidget(self.lbl_freq, 1, 0)
        m_layout.addWidget(self.lbl_trigger, 1, 1)
        m_layout.addWidget(self.lbl_sample_rate, 1, 2)

        c_layout.addWidget(meas_widget)
        c_layout.addStretch()

        # Trigger controls
        trig_widget = QtWidgets.QWidget()
        trig_layout = QtWidgets.QVBoxLayout(trig_widget)

        lbl_trig = QtWidgets.QLabel("TRIGGER")
        lbl_trig.setStyleSheet("color:#f80; font-size:12px; font-weight:bold;")
        trig_layout.addWidget(lbl_trig)

        # Trigger mode selector
        self.combo_trig_mode = QtWidgets.QComboBox()
        self.combo_trig_mode.addItems(["Spectrum", "Rising", "Falling", "None"])
        self.combo_trig_mode.setCurrentIndex(1)  # Default: Rising
        self.combo_trig_mode.currentIndexChanged.connect(self.change_trigger_mode)
        self.combo_trig_mode.setStyleSheet("background:#222; color:#fff; padding:5px;")
        trig_layout.addWidget(self.combo_trig_mode)

        # Trigger channel selector
        self.combo_trig_ch = QtWidgets.QComboBox()
        self.combo_trig_ch.addItems(["CH1", "CH2", "CH3"])
        self.combo_trig_ch.currentIndexChanged.connect(self.change_trigger_channel)
        self.combo_trig_ch.setStyleSheet("background:#222; color:#fff; padding:5px;")
        trig_layout.addWidget(self.combo_trig_ch)

        c_layout.addWidget(trig_widget)

        return controls

    def toggle_hold(self):
        self.hold = not self.hold
        self.btn_hold.setText("⏸ HOLD" if self.hold else "▶ RUNNING")

    def change_trigger_mode(self, index):
        self.trigger_mode = index
        mode_names = ["Spectrum", "Rising", "Falling", "None"]
        self.lbl_trigger.setText(f"Trigger: {mode_names[index]} | CH{self.trigger_channel+1} @ {self.trigger_level:.2f}V")

    def change_trigger_channel(self, index):
        self.trigger_channel = index
        mode_names = ["Spectrum", "Rising", "Falling", "None"]
        self.lbl_trigger.setText(f"Trigger: {mode_names[self.trigger_mode]} | CH{index+1} @ {self.trigger_level:.2f}V")

    @QtCore.pyqtSlot(np.ndarray, np.ndarray, np.ndarray)
    def on_data(self, ch0, ch1, ch2):
        if not self.hold:
            self.ch0_data = ch0
            self.ch1_data = ch1
            self.ch2_data = ch2

    @QtCore.pyqtSlot(str)
    def on_status(self, status):
        self.lbl_status.setText(status)

    def find_trigger(self, data):
        """
        Find trigger point based on trigger mode
        trigger_mode: 0=Spectrum, 1=Rising, 2=Falling, 3=None
        """
        # Mode 3: None (Free run) - no trigger
        if self.trigger_mode == 3:
            return 0

        # Mode 0: Spectrum - not implemented for time domain, return 0
        if self.trigger_mode == 0:
            return 0

        threshold = self.trigger_level

        # Search in first portion of buffer for trigger point (allow pre-trigger samples)
        search_end = min(len(data) - 10, int(len(data) * 0.4))

        # Mode 1: Rising edge trigger
        if self.trigger_mode == 1:
            for i in range(1, search_end):
                if data[i-1] < threshold and data[i] >= threshold:
                    return i

        # Mode 2: Falling edge trigger
        elif self.trigger_mode == 2:
            for i in range(1, search_end):
                if data[i-1] > threshold and data[i] <= threshold:
                    return i

        return 0

    def calculate_frequency(self, data):
        """Calculate frequency from zero crossings"""
        try:
            # Use mean instead of median for better threshold
            vmin = np.min(data)
            vmax = np.max(data)
            vpp = vmax - vmin

            # Need at least 0.5V swing to measure
            if vpp < 0.5:
                return 0

            threshold = (vmax + vmin) / 2.0
            crossings = []

            # Find rising edges
            for i in range(1, len(data)):
                if data[i-1] < threshold <= data[i]:
                    crossings.append(i)

            if len(crossings) >= 3:
                # Calculate periods between crossings
                periods = np.diff(crossings)

                # Remove outliers (periods that differ too much)
                median_period = np.median(periods)
                valid_periods = [p for p in periods if abs(p - median_period) < median_period * 0.5]

                if len(valid_periods) >= 2:
                    avg_period = np.mean(valid_periods)
                    # Calculate frequency from sample rate
                    freq = SAMPLE_RATE / avg_period

                    # Sanity check: 1Hz to 100kHz
                    if 1 < freq < 100000:
                        return freq
        except Exception as e:
            pass
        return 0

    def update_display(self):
        if self.hold or len(self.ch0_data) == 0:
            return

        # Select trigger data
        if self.trigger_channel == 0:
            trig_data = self.ch0_data
        elif self.trigger_channel == 1:
            trig_data = self.ch1_data
        else:
            trig_data = self.ch2_data

        # Find trigger point
        trig_idx = self.find_trigger(trig_data)

        # Roll data to align trigger
        ch0_display = np.roll(self.ch0_data, -trig_idx) if trig_idx > 0 else self.ch0_data
        ch1_display = np.roll(self.ch1_data, -trig_idx) if trig_idx > 0 else self.ch1_data
        ch2_display = np.roll(self.ch2_data, -trig_idx) if trig_idx > 0 else self.ch2_data

        # Update plots
        self.plot_ch0.curve.setData(ch0_display)
        self.plot_ch1.curve.setData(ch1_display)
        self.plot_ch2.curve.setData(ch2_display)

        # Update measurements every 5 frames
        if not hasattr(self, '_counter'):
            self._counter = 0
        self._counter += 1

        if self._counter % 5 == 0:
            # Calculate Vpp for each channel
            ch0_vpp = np.max(self.ch0_data) - np.min(self.ch0_data)
            ch1_vpp = np.max(self.ch1_data) - np.min(self.ch1_data)
            ch2_vpp = np.max(self.ch2_data) - np.min(self.ch2_data)

            self.lbl_ch0_vpp.setText(f"CH1: {ch0_vpp:.3f}Vpp")
            self.lbl_ch1_vpp.setText(f"CH2: {ch1_vpp:.3f}Vpp")
            self.lbl_ch2_vpp.setText(f"CH3: {ch2_vpp:.3f}Vpp")

            # Calculate frequency from trigger channel
            freq = self.calculate_frequency(trig_data)
            if freq > 0:
                freq_str = f"{freq/1000:.2f}kHz" if freq >= 1000 else f"{freq:.0f}Hz"
            else:
                freq_str = "---"
            self.lbl_freq.setText(f"Freq: {freq_str}")

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Space:
            self.toggle_hold()
        elif event.key() == QtCore.Qt.Key_1:
            self.combo_trig_ch.setCurrentIndex(0)
        elif event.key() == QtCore.Qt.Key_2:
            self.combo_trig_ch.setCurrentIndex(1)
        elif event.key() == QtCore.Qt.Key_3:
            self.combo_trig_ch.setCurrentIndex(2)


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")

    print("="*80)
    print("⚡ STM32 OSCILLOSCOPE - 3 CHANNEL VERSION")
    print("="*80)
    print("\n🔧 CONFIGURATION:")
    print(f"  • Channels: {NUM_CHANNELS}")
    print(f"  • Samples per channel: {SAMPLES_PER_CHANNEL}")
    print(f"  • Per-channel sample rate: {SAMPLE_RATE}Hz ({SAMPLE_RATE//1000}kHz)")
    print(f"  • Total ADC trigger rate: {SAMPLE_RATE * NUM_CHANNELS}Hz ({SAMPLE_RATE * NUM_CHANNELS//1000}kHz)")
    print(f"  • Packet size: {PACKET_TOTAL_SIZE} bytes")
    print(f"  • Validation: Markers + Checksum")
    print("\n📡 PROTOCOL:")
    print(f"  • START marker: 0x{MARKER_START:04X}")
    print(f"  • END marker: 0x{MARKER_END:04X}")
    print(f"  • Commands: START(0x{CMD_START:02X}) STOP(0x{CMD_STOP:02X}) DATA(0x{CMD_DATA:02X})")
    print("\n📌 PIN MAPPING:")
    print("  • CH1: PA0 (ADC_IN0)")
    print("  • CH2: PA1 (ADC_IN1)")
    print("  • CH3: PB0 (ADC_IN8)")
    print("\n⌨️  CONTROLS:")
    print("  • SPACE: Hold/Run")
    print("  • 1/2/3: Select trigger channel")
    print("="*80)

    scope = STM32Scope()
    gui = ScopeGUI(scope)
    gui.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

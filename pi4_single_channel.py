#!/usr/bin/env python3
"""
STM32 Oscilloscope - Single Channel via SPI (STM32F103C8T6 → Raspberry Pi 4)
- Based on GameInstance.com oscilloscope logic
- Backend SPI with packet validation (markers + checksum + metadata)
- 1 channel: PB0 (CH1)
- 11 time bases (71.4 kHz - 2571 kHz) controlled by STM32 buttons
- 4 trigger modes (Spectrum/Rising/Falling/None) controlled by STM32 buttons
- PWM test signal: PA2 @ 1kHz
"""

import sys, threading, time
import numpy as np
import struct
from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

# ======================== SPI Protocol Constants ========================

CMD_START = 0xF1
CMD_STOP = 0xF2
CMD_DATA = 0xF3
CMD_SET_TIMEBASE = 0xF4
CMD_SET_TRIGGER = 0xF5

MARKER_START = 0x1234
MARKER_END = 0x5678

BUFFER_SIZE = 1024  # STM32 sends 1024 samples (single channel)

# Packet structure from main_spi_v2.cpp:
PACKET_HEADER_SIZE = 8  # start_marker + time_base_id + trigger_mode_id + sample_rate
PACKET_DATA_SIZE = 2048  # 1024 samples × 2 bytes
PACKET_FOOTER_SIZE = 4  # end_marker + checksum
PACKET_TOTAL_SIZE = PACKET_HEADER_SIZE + PACKET_DATA_SIZE + PACKET_FOOTER_SIZE

# Time base configuration from GameInstance oscilloscope (11 time bases)
DT_FS = [2571, 2571, 2571, 1800, 1384, 878, 667, 529, 429, 143, 71.4]  # kHz
TRIGGER_NAMES = ["Spectrum", "Rising", "Falling", "None"]

# Default sample rate (will be updated from packet metadata)
SAMPLE_RATE = 529000.0  # Default: time_base=7 (529 kHz)

print(f"📊 STM32 Oscilloscope - GameInstance Logic")
print(f"   • 11 time bases: {DT_FS[-1]:.1f} kHz - {DT_FS[0]:.1f} kHz")
print(f"   • Single channel: PB0")
print(f"   • Buffer size: {BUFFER_SIZE} samples")
print(f"   • 4 trigger modes: Spectrum/Rising/Falling/None")
print()

# ======================== SPI Scope Provider ========================

class DataPacket:
    """Represents a data packet from STM32"""
    def __init__(self):
        self.start_marker = 0
        self.time_base_id = 0
        self.trigger_mode_id = 0
        self.sample_rate = 0
        self.data = np.zeros(BUFFER_SIZE, dtype=np.uint16)
        self.end_marker = 0
        self.checksum = 0
        self.valid = False


class SPIScopeProvider(QtCore.QObject):
    """SPI provider - nhận data từ STM32 qua SPI"""
    # emit block: {'sr': float, 't': np.ndarray, 'ch1': nd, 'time_base_id': int, 'trigger_mode_id': int}
    block = QtCore.pyqtSignal(dict)
    status_update = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._run = False
        self.spi = None
        self._idx = 0
        self.sr = SAMPLE_RATE

    def start(self):
        try:
            import spidev  # noqa: F401
        except ImportError:
            QtWidgets.QMessageBox.critical(
                None, "Thiếu spidev",
                "Hãy cài: sudo apt install python3-spidev"
            )
            sys.exit(2)

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
        self.spi.max_speed_hz = 2000000  # 2MHz
        self.spi.mode = 0
        self.spi.lsbfirst = False
        print("✓ SPI initialized: 2MHz, Mode 0")

    def _find_start_marker(self, raw_bytes):
        """Find START marker in byte stream"""
        for i in range(len(raw_bytes) - 1):
            marker = struct.unpack('<H', raw_bytes[i:i+2])[0]
            if marker == MARKER_START:
                return i
        return -1

    def _parse_packet(self, raw_bytes):
        """Parse and validate packet"""
        packet = DataPacket()

        # Find START marker first
        marker_offset = self._find_start_marker(raw_bytes)
        if marker_offset < 0:
            return packet

        # Adjust raw_bytes to start from marker
        if marker_offset > 0:
            raw_bytes = raw_bytes[marker_offset:]

        if len(raw_bytes) < PACKET_TOTAL_SIZE:
            return packet

        try:
            # Parse header (8 bytes)
            packet.start_marker = struct.unpack('<H', raw_bytes[0:2])[0]
            packet.time_base_id = struct.unpack('<H', raw_bytes[2:4])[0]
            packet.trigger_mode_id = struct.unpack('<H', raw_bytes[4:6])[0]
            packet.sample_rate = struct.unpack('<H', raw_bytes[6:8])[0]

            # Parse data (1024 samples)
            data_bytes = raw_bytes[8:8 + PACKET_DATA_SIZE]
            packet.data = np.frombuffer(data_bytes, dtype='<u2')

            # Parse footer
            footer_offset = 8 + PACKET_DATA_SIZE
            packet.end_marker = struct.unpack('<H', raw_bytes[footer_offset:footer_offset+2])[0]
            packet.checksum = struct.unpack('<H', raw_bytes[footer_offset+2:footer_offset+4])[0]

            # Validate markers
            if packet.start_marker != MARKER_START:
                return packet
            if packet.end_marker != MARKER_END:
                return packet

            # Validate checksum (matching main_spi_v2.cpp)
            calculated_sum = MARKER_START + MARKER_END
            calculated_sum += packet.time_base_id
            calculated_sum += packet.trigger_mode_id
            calculated_sum += packet.sample_rate
            calculated_sum += np.sum(packet.data, dtype=np.uint32)
            calculated_checksum = calculated_sum & 0xFFFF
            if packet.checksum != calculated_checksum:
                return packet

            # Validate data range (12-bit ADC)
            if np.max(packet.data) > 4095:
                return packet

            packet.valid = True

        except Exception:
            pass

        return packet

    def _loop(self):
        try:
            self._init_spi()

            # Start acquisition
            self.spi.xfer2([CMD_START])
            time.sleep(0.5)
            print("✓ Sent START command")

            frame_count = 0
            good_frames = 0
            bad_frames = 0
            last_fps_time = time.time()

            while self._run:
                try:
                    # Request data
                    self.spi.xfer2([CMD_DATA])
                    time.sleep(0.050)  # Wait for STM32 to prepare data

                    # Read packet
                    raw_data = bytearray()
                    chunk_size = 1024
                    read_size = PACKET_TOTAL_SIZE + 2048  # Extra buffer for marker search
                    remaining = read_size

                    while remaining > 0:
                        to_read = min(chunk_size, remaining)
                        chunk = self.spi.readbytes(to_read)
                        raw_data.extend(chunk)
                        remaining -= len(chunk)
                        if remaining > 0:
                            time.sleep(0.001)

                    # Parse and validate
                    packet = self._parse_packet(raw_data)

                    if packet.valid:
                        # Update sample rate from packet metadata
                        self.sr = float(packet.sample_rate) * 1000.0  # kHz → Hz

                        # Debug: Show packet metadata for first 3 frames
                        if good_frames < 3:
                            print(f"   📦 Packet metadata:")
                            print(f"      Time base ID: {packet.time_base_id} ({DT_FS[packet.time_base_id]:.1f} kHz)")
                            print(f"      Trigger mode: {packet.trigger_mode_id} ({TRIGGER_NAMES[packet.trigger_mode_id]})")
                            print(f"      Sample rate: {packet.sample_rate} kHz")

                        # Convert to voltage (0-3.3V)
                        ch1_v = packet.data.astype(np.float32) * (3.3 / 4095.0)

                        # Show signal stats for first 5 frames
                        if good_frames < 5:
                            vmin = np.min(ch1_v)
                            vmax = np.max(ch1_v)
                            vpp = vmax - vmin
                            print(f"✅ Frame #{good_frames+1}: PB0 {vmin:.2f}-{vmax:.2f}V (Vpp={vpp:.2f}V) | SR={self.sr/1000:.1f}kHz")

                        # Create time array
                        n = ch1_v.size
                        t = (np.arange(self._idx, self._idx + n, dtype=np.float64) / self.sr).astype(np.float32)
                        self._idx += n

                        # Emit block with metadata
                        self.block.emit({
                            'sr': float(self.sr),
                            't': t,
                            'ch1': ch1_v,
                            'time_base_id': int(packet.time_base_id),
                            'trigger_mode_id': int(packet.trigger_mode_id)
                        })

                        good_frames += 1
                    else:
                        bad_frames += 1

                    # Update status
                    frame_count += 1
                    if time.time() - last_fps_time >= 2.0:
                        fps = frame_count / (time.time() - last_fps_time)
                        success_rate = good_frames / (good_frames + bad_frames) * 100 if (good_frames + bad_frames) > 0 else 0
                        status = f"FPS: {fps:.1f} | Success: {success_rate:.0f}% | Good: {good_frames} | Bad: {bad_frames}"
                        self.status_update.emit(status)
                        print(status)
                        frame_count = 0
                        good_frames = 0
                        bad_frames = 0
                        last_fps_time = time.time()

                    time.sleep(0.005)

                except Exception as e:
                    print(f"Read error: {e}")
                    bad_frames += 1
                    time.sleep(0.1)

        except Exception as e:
            print(f"Fatal error: {e}")
            import traceback
            traceback.print_exc()


# ======================== Signal Stats ========================

def calculate_frequency(x: np.ndarray, fs: float):
    """Calculate frequency using zero-crossing with hysteresis"""
    if x.size < 10:
        return 0.0

    vmax = float(np.max(x))
    vmin = float(np.min(x))
    vpp = vmax - vmin

    if vpp < 0.05:  # Too small
        return 0.0

    # Hysteresis thresholds
    vmid = (vmax + vmin) / 2.0
    hyst = vpp * 0.05
    thresh_high = vmid + hyst
    thresh_low = vmid - hyst

    # State machine for rising edge detection
    crosses = []
    state = x[0] > vmid

    for i in range(1, len(x)):
        if state:  # Currently HIGH
            if x[i] < thresh_low:
                state = False
        else:  # Currently LOW
            if x[i] > thresh_high:
                state = True
                crosses.append(i)  # Rising edge

    if len(crosses) >= 3:
        periods = np.diff(crosses)
        median_period = np.median(periods)
        T = median_period / fs
        if T > 0:
            return 1.0 / T

    return 0.0


# ======================== Scope UI ========================

class ScopePanelUI(QtWidgets.QWidget):
    def __init__(self, scope_provider: SPIScopeProvider):
        super().__init__()
        self.scope_provider = scope_provider

        self.setWindowTitle("STM32 Oscilloscope - GameInstance Logic (1 CH)")
        self.resize(1400, 900)
        self.setStyleSheet("background:#1A1A1A;")

        # Data buffers - for single channel
        self.N = int(2571000 * 1.0)  # 1 second buffer at max sample rate
        self.t = np.zeros(self.N, dtype=np.float32)
        self.ch1 = np.zeros(self.N, dtype=np.float32)
        self.iw = 0
        self.sr = SAMPLE_RATE

        # Display settings
        self.time_div = 0.001  # 1ms/div default
        self.volts_div = 0.5
        self.offset = 0.0
        self.trigger_lvl = 1.65
        self.trigger_mode = 'AUTO'
        self.trigger_edge = 1  # 1=Rising
        self.trigger_armed = True
        self.trigger_found = False

        # STM32 metadata
        self.stm32_time_base_id = 7
        self.stm32_trigger_mode_id = 1

        # Frequency averaging
        self.freq_history = []
        self.freq_avg_count = 5

        self._build_ui()

        # Connect signals
        self.scope_provider.block.connect(self.on_block)
        self.scope_provider.status_update.connect(self.on_status)
        self.scope_provider.start()

        # Update timer
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(50)  # 20 FPS

    def _build_ui(self):
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        # Top bar
        top_widget = QtWidgets.QWidget()
        top_widget.setStyleSheet("background:#2D2D2D; color:#DDD; border-radius: 8px;")
        top = QtWidgets.QHBoxLayout(top_widget)
        top.setContentsMargins(15, 10, 15, 10)

        title = QtWidgets.QLabel("⚡ STM32 OSCILLOSCOPE - GAMEINSTANCE LOGIC")
        title.setFont(QtGui.QFont("Consolas", 16, QtGui.QFont.Bold))
        title.setStyleSheet("color:#0f0;")

        self.lbl_sr = QtWidgets.QLabel(f"SR: {self.sr/1000:.1f} kS/s")
        self.lbl_sr.setFont(QtGui.QFont("Consolas", 14, QtGui.QFont.Bold))

        self.lbl_status = QtWidgets.QLabel("Initializing...")
        self.lbl_status.setFont(QtGui.QFont("Consolas", 12))
        self.lbl_status.setStyleSheet("color:#ff0;")

        top.addWidget(title)
        top.addStretch(1)
        top.addWidget(self.lbl_sr)
        top.addSpacing(20)
        top.addWidget(self.lbl_status)

        root.addWidget(top_widget)

        # Center: Plot + Controls
        center_widget = QtWidgets.QWidget()
        center = QtWidgets.QHBoxLayout(center_widget)
        center.setContentsMargins(10, 10, 10, 10)
        center.setSpacing(10)

        # Plot
        pg.setConfigOptions(antialias=False)
        self.plot = pg.PlotWidget()
        self.plot.setBackground((26, 26, 26))

        grid = pg.GridItem()
        grid.setPen(pg.mkPen(color=(100, 100, 100), style=QtCore.Qt.DashLine))
        grid.setOpacity(0.3)
        self.plot.addItem(grid)

        self.plot.setMenuEnabled(False)
        self.plot.setMouseEnabled(x=False, y=False)

        # Curve (PB0=green)
        self.cur1 = self.plot.plot(pen=pg.mkPen((0, 255, 0), width=3), clipToView=True)

        # Text label
        self.txt1 = pg.TextItem(color=(0, 255, 0))
        fnt = QtGui.QFont("Consolas", 12, QtGui.QFont.Bold)
        self.txt1.setFont(fnt)
        self.txt1.setAnchor((1, 0))
        self.plot.addItem(self.txt1)

        center.addWidget(self.plot, 1)

        # Right panel: Controls
        right = QtWidgets.QVBoxLayout()
        right.setContentsMargins(10, 10, 10, 10)
        right.setSpacing(10)

        right.addWidget(self._group_horizontal())
        right.addWidget(self._group_vertical())
        right.addWidget(self._group_stm32_info())
        right.addWidget(self._group_trigger())
        right.addStretch(1)

        center.addLayout(right, 0)
        root.addWidget(center_widget)

        # Bottom: Badges
        badges_widget = QtWidgets.QWidget()
        badges_widget.setStyleSheet("background:#2D2D2D; border-radius: 8px;")
        badges = QtWidgets.QHBoxLayout(badges_widget)
        badges.setContentsMargins(10, 10, 10, 10)
        badges.setSpacing(15)

        self.bad1 = self._badge('⚡ CH1 = PB0 (PIN 21) | 0.5V/div +0.00V', 'rgb(0,255,0)')
        self.badT = self._badge('TRIG CH1 +1.65V', '#ddd')
        self.badSTM32 = self._badge('STM32: TIME=7 (529kHz) | TRIG=Rising', 'rgb(255,165,0)')

        for b in (self.bad1, self.badT, self.badSTM32):
            b.setFont(QtGui.QFont("Consolas", 12, QtGui.QFont.Bold))
            b.setMinimumWidth(200)
            badges.addWidget(b)

        badges.addStretch(1)
        root.addWidget(badges_widget)

        self._update_axes()

    def _badge(self, text, color):
        L = QtWidgets.QLabel(text)
        L.setStyleSheet(f"padding:8px 16px; border-radius:12px; background:#2b2b2b; color:{color};")
        return L

    def _group_horizontal(self):
        box = QtWidgets.QGroupBox("Horizontal (Time)")
        box.setStyleSheet("background:#2D2D2D; color:#DDD; border: 1px solid #444; border-radius: 6px; font-weight: bold;")
        g = QtWidgets.QGridLayout(box)
        g.setContentsMargins(10, 10, 10, 10)
        g.setSpacing(10)

        g.addWidget(QtWidgets.QLabel("TIME/DIV"), 0, 0)
        m = QtWidgets.QPushButton("-")
        p = QtWidgets.QPushButton("+")

        for b in (m, p):
            b.setStyleSheet("background:#3A3A3A; color:#DDD; border: 1px solid #555; padding: 8px; font-size: 14px; font-weight: bold;")

        m.clicked.connect(lambda: self._time_div_scale(1/1.2))
        p.clicked.connect(lambda: self._time_div_scale(1.2))

        g.addWidget(m, 0, 1)
        g.addWidget(p, 0, 2)

        return box

    def _group_vertical(self):
        box = QtWidgets.QGroupBox("Vertical (CH1 - PB0)")
        box.setStyleSheet("background:#2D2D2D; color:#DDD; border: 1px solid #444; border-radius: 6px; font-weight: bold;")
        g = QtWidgets.QGridLayout(box)
        g.setContentsMargins(10, 10, 10, 10)
        g.setSpacing(10)

        g.addWidget(QtWidgets.QLabel("VOLTS/DIV"), 0, 0)
        m = QtWidgets.QPushButton("-")
        p = QtWidgets.QPushButton("+")

        for b in (m, p):
            b.setStyleSheet("background:#3A3A3A; color:#DDD; border: 1px solid #555; padding: 8px; font-size: 14px; font-weight: bold;")

        m.clicked.connect(lambda: self._vdiv(1/1.2))
        p.clicked.connect(lambda: self._vdiv(1.2))

        g.addWidget(m, 0, 1)
        g.addWidget(p, 0, 2)

        g.addWidget(QtWidgets.QLabel("POSITION"), 1, 0)
        d = QtWidgets.QPushButton("↓")
        u = QtWidgets.QPushButton("↑")

        for b in (d, u):
            b.setStyleSheet("background:#3A3A3A; color:#DDD; border: 1px solid #555; padding: 8px; font-size: 14px; font-weight: bold;")

        d.clicked.connect(lambda: self._move_off(-0.2))
        u.clicked.connect(lambda: self._move_off(+0.2))

        g.addWidget(d, 1, 1)
        g.addWidget(u, 1, 2)

        return box

    def _group_stm32_info(self):
        box = QtWidgets.QGroupBox("STM32 Button Controls")
        box.setStyleSheet("background:#2D2D2D; color:#FFA500; border: 2px solid #FFA500; border-radius: 6px; font-weight: bold;")
        g = QtWidgets.QGridLayout(box)
        g.setContentsMargins(10, 10, 10, 10)
        g.setSpacing(10)

        # Info label
        info = QtWidgets.QLabel("Control via STM32 buttons:")
        info.setStyleSheet("color:#DDD; font-size: 11px; font-weight: bold;")
        g.addWidget(info, 0, 0, 1, 2)

        # TIME button info
        time_lbl = QtWidgets.QLabel("TIME (PA15):")
        time_lbl.setStyleSheet("color:#0f0; font-size: 10px;")
        self.time_val = QtWidgets.QLabel("Time base 7 (529 kHz)")
        self.time_val.setStyleSheet("color:#DDD; font-size: 10px;")
        g.addWidget(time_lbl, 1, 0)
        g.addWidget(self.time_val, 1, 1)

        # TRIGGER button info
        trig_lbl = QtWidgets.QLabel("TRIGGER (PB10):")
        trig_lbl.setStyleSheet("color:#0f0; font-size: 10px;")
        self.trig_val = QtWidgets.QLabel("Rising edge")
        self.trig_val.setStyleSheet("color:#DDD; font-size: 10px;")
        g.addWidget(trig_lbl, 2, 0)
        g.addWidget(self.trig_val, 2, 1)

        # FREEZE button info
        freeze_lbl = QtWidgets.QLabel("FREEZE (PB11):")
        freeze_lbl.setStyleSheet("color:#0f0; font-size: 10px;")
        freeze_info = QtWidgets.QLabel("Freeze display")
        freeze_info.setStyleSheet("color:#DDD; font-size: 10px;")
        g.addWidget(freeze_lbl, 3, 0)
        g.addWidget(freeze_info, 3, 1)

        # PWM info
        pwm_lbl = QtWidgets.QLabel("TEST SIGNAL:")
        pwm_lbl.setStyleSheet("color:#ff0; font-size: 10px; font-weight: bold;")
        pwm_info = QtWidgets.QLabel("PA2: 1kHz square wave")
        pwm_info.setStyleSheet("color:#DDD; font-size: 10px;")
        g.addWidget(pwm_lbl, 4, 0)
        g.addWidget(pwm_info, 4, 1)

        return box

    def _group_trigger(self):
        box = QtWidgets.QGroupBox("Trigger")
        box.setStyleSheet("background:#2D2D2D; color:#DDD; border: 1px solid #444; border-radius: 6px; font-weight: bold;")
        g = QtWidgets.QGridLayout(box)
        g.setContentsMargins(10, 10, 10, 10)
        g.setSpacing(10)

        # Trigger Mode
        g.addWidget(QtWidgets.QLabel("MODE"), 0, 0)
        mode_auto = QtWidgets.QPushButton("AUTO")
        mode_norm = QtWidgets.QPushButton("NORM")

        for b in (mode_auto, mode_norm):
            b.setCheckable(True)
            b.setStyleSheet("background:#3A3A3A; color:#DDD; border: 1px solid #555; padding: 8px; font-size: 12px; font-weight: bold;")

        mode_group = QtWidgets.QButtonGroup(self)
        mode_group.addButton(mode_auto, 0)
        mode_group.addButton(mode_norm, 1)
        mode_group.setExclusive(True)
        mode_auto.setChecked(True)
        mode_group.idClicked.connect(self._set_trigger_mode)

        g.addWidget(mode_auto, 0, 1)
        g.addWidget(mode_norm, 0, 2)

        # Trigger Level
        g.addWidget(QtWidgets.QLabel("LEVEL"), 1, 0)
        m = QtWidgets.QPushButton('-')
        p = QtWidgets.QPushButton('+')

        for b in (m, p):
            b.setStyleSheet("background:#3A3A3A; color:#DDD; border: 1px solid #555; padding: 8px; font-size: 14px; font-weight: bold;")

        m.clicked.connect(lambda: self._set_lvl(-0.1))
        p.clicked.connect(lambda: self._set_lvl(+0.1))

        g.addWidget(m, 1, 1)
        g.addWidget(p, 1, 2)

        return box

    @QtCore.pyqtSlot(dict)
    def on_block(self, blk: dict):
        """Receive data block from SPI provider"""
        sr = blk.get('sr', None)
        if sr:
            self.sr = float(sr)

        # Update STM32 metadata
        self.stm32_time_base_id = blk.get('time_base_id', self.stm32_time_base_id)
        self.stm32_trigger_mode_id = blk.get('trigger_mode_id', self.stm32_trigger_mode_id)

        t = blk['t']
        a = blk['ch1']

        n = t.size
        i0 = self.iw
        i1 = (i0 + n) % self.N

        if i0 < i1:
            self.t[i0:i1] = t
            self.ch1[i0:i1] = a
        else:
            k = self.N - i0
            self.t[i0:] = t[:k]
            self.t[:i1] = t[k:]
            self.ch1[i0:] = a[:k]
            self.ch1[:i1] = a[k:]

        self.iw = i1

    @QtCore.pyqtSlot(str)
    def on_status(self, status):
        """Update status label"""
        self.lbl_status.setText(status)

    def update_ui(self):
        """Update display"""
        self.lbl_sr.setText(f"SR: {self.sr/1000:.1f} kS/s")

        win = self.time_div * 10.0
        grab = win * 5.0

        if self.iw == 0 and self.t[0] == 0:
            return

        t_right = self.t[(self.iw - 1) % self.N]
        t_left = t_right - grab
        n_need = int(self.sr * grab) + 4

        i1 = self.iw
        i0 = (i1 - n_need) % self.N

        if i0 < i1:
            tx = self.t[i0:i1]
            a = self.ch1[i0:i1]
        else:
            tx = np.concatenate((self.t[i0:], self.t[:i1]))
            a = np.concatenate((self.ch1[i0:], self.ch1[:i1]))

        m = (tx >= t_left) & (tx <= t_right)
        tx, a = tx[m], a[m]

        if tx.size < 8:
            return

        tx_rel = tx - t_right

        # Trigger logic
        src = a
        src_vpp = np.max(src) - np.min(src)
        hyst = src_vpp * 0.05

        crosses = []
        if self.trigger_edge == 1:  # Rising
            above_high = src > (self.trigger_lvl + hyst)
            below_low = src < (self.trigger_lvl - hyst)
            state = above_high[0]

            for i in range(1, len(src)):
                if state:
                    if below_low[i]:
                        state = False
                else:
                    if above_high[i]:
                        state = True
                        crosses.append(i)

        cross = np.array(crosses)
        self.trigger_found = False

        if cross.size > 0:
            right_idx_start = int(tx.size * 0.5)
            cand = cross[cross >= right_idx_start]
            if cand.size > 0:
                k = int(cand[0])
                trig_pos = -5 * self.time_div
                tx_rel = tx_rel - (tx_rel[k] - trig_pos)
                self.trigger_found = True

        # Handle trigger modes
        if self.trigger_mode == 'NORMAL':
            if not self.trigger_found:
                return

        pre_trigger_time = 2.0 * self.time_div
        mm = (tx_rel >= -win - pre_trigger_time) & (tx_rel <= 0.0)
        tx_rel, a = tx_rel[mm], a[mm]

        if tx_rel.size < 8:
            return

        # Calculate frequency
        freq = calculate_frequency(a, self.sr)

        # Frequency averaging
        if freq > 0:
            self.freq_history.append(freq)
            if len(self.freq_history) > self.freq_avg_count:
                self.freq_history.pop(0)
            if len(self.freq_history) >= 3:
                freq = float(np.median(self.freq_history))
        else:
            self.freq_history.clear()

        vpp = float(np.max(a) - np.min(a))

        # Apply offset
        a = a + self.offset

        # Downsample if needed
        n = tx_rel.size
        mpts = min(5000, n)
        if n > mpts:
            sel = np.linspace(0, n-1, mpts).astype(int)
            txd, ad = tx_rel[sel], a[sel]
        else:
            txd, ad = tx_rel, a

        # Update curve
        self.cur1.setData(txd, ad)

        # Update text label
        view_range = self.plot.getViewBox().viewRange()
        x_max, y_max = view_range[0][1], view_range[1][1]
        self.txt1.setPos(x_max - 20, y_max - 10)
        self.txt1.setText(f"CH1 = PB0 (PIN 21) | Vpp={vpp:.2f}V f={freq:.0f}Hz")

        # Update badges
        self._update_axes()
        self.bad1.setText(f"⚡ CH1 = PB0 (PIN 21) | {self.volts_div:.2f}V/div {self.offset:+.2f}V")

        status = "🔒" if self.trigger_found else "⏩"
        edge_names = ["SPECTRUM", "RISING↑", "FALLING↓", "NONE"]
        edge_name = edge_names[self.trigger_edge]
        self.badT.setText(f"TRIG {self.trigger_mode} {status} {edge_name} CH1 {self.trigger_lvl:+.2f}V")

        # Update STM32 button status
        time_base_str = f"TIME={self.stm32_time_base_id} ({DT_FS[self.stm32_time_base_id]:.1f}kHz)"
        trigger_str = f"TRIG={TRIGGER_NAMES[self.stm32_trigger_mode_id]}"
        self.badSTM32.setText(f"STM32: {time_base_str} | {trigger_str}")

        self.time_val.setText(f"Time base {self.stm32_time_base_id} ({DT_FS[self.stm32_time_base_id]:.1f} kHz)")
        self.trig_val.setText(TRIGGER_NAMES[self.stm32_trigger_mode_id])

    def _update_axes(self):
        ymax = self.volts_div * 4.0
        self.plot.setYRange(-ymax, ymax, padding=0.05)
        win = self.time_div * 10.0
        self.plot.setXRange(-win, 0.0, padding=0)

    def _time_div_scale(self, k):
        self.time_div = float(np.clip(self.time_div * k, 5e-5, 5.0))
        self._update_axes()

    def _vdiv(self, k):
        self.volts_div = float(np.clip(self.volts_div * k, 1e-3, 1000.0))
        self._update_axes()

    def _move_off(self, k):
        self.offset += k * self.volts_div

    def _set_lvl(self, dv):
        self.trigger_lvl += dv

    def _set_trigger_mode(self, mode_id):
        """Set trigger mode: 0=AUTO, 1=NORMAL"""
        modes = ['AUTO', 'NORMAL']
        self.trigger_mode = modes[mode_id]


# ======================== MAIN ========================

def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")

    # Dark palette
    palette = QtGui.QPalette()
    palette.setColor(QtGui.QPalette.Window, QtGui.QColor(26, 26, 26))
    palette.setColor(QtGui.QPalette.WindowText, QtGui.QColor(255, 255, 255))
    palette.setColor(QtGui.QPalette.Base, QtGui.QColor(35, 35, 35))
    palette.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor(53, 53, 53))
    palette.setColor(QtGui.QPalette.ToolTipBase, QtGui.QColor(255, 255, 255))
    palette.setColor(QtGui.QPalette.ToolTipText, QtGui.QColor(255, 255, 255))
    palette.setColor(QtGui.QPalette.Text, QtGui.QColor(255, 255, 255))
    palette.setColor(QtGui.QPalette.Button, QtGui.QColor(53, 53, 53))
    palette.setColor(QtGui.QPalette.ButtonText, QtGui.QColor(255, 255, 255))
    palette.setColor(QtGui.QPalette.BrightText, QtGui.QColor(255, 0, 0))
    palette.setColor(QtGui.QPalette.Link, QtGui.QColor(42, 130, 218))
    palette.setColor(QtGui.QPalette.Highlight, QtGui.QColor(42, 130, 218))
    palette.setColor(QtGui.QPalette.HighlightedText, QtGui.QColor(0, 0, 0))
    app.setPalette(palette)

    print("="*80)
    print("⚡ STM32 OSCILLOSCOPE - GAMEINSTANCE LOGIC")
    print("="*80)
    print()
    print("✅ SINGLE CHANNEL MODE with STM32 Button Controls")
    print("    • 1 channel: PB0 (CH1)")
    print("    • 11 time bases: 71.4 kHz - 2571 kHz")
    print("    • 4 trigger modes: Spectrum/Rising/Falling/None")
    print("    • PWM test signal: PA2 @ 1kHz square wave")
    print()
    print(f"  🎛️  STM32 Button Controls:")
    print(f"     • PA15: TIME button (cycle through 11 time bases)")
    print(f"     • PB10: TRIGGER button (Spectrum/Rising/Falling/None)")
    print(f"     • PB11: FREEZE button (freeze/unfreeze display)")
    print()
    print(f"  📦 Data Protocol:")
    print(f"     • Packet size: {PACKET_TOTAL_SIZE} bytes")
    print(f"     • Header: START + time_base_id + trigger_mode_id + sample_rate")
    print(f"     • Data: {BUFFER_SIZE} samples × 2 bytes")
    print(f"     • Footer: END + Checksum")
    print(f"     • Markers: 0x{MARKER_START:04X} / 0x{MARKER_END:04X}")
    print("="*80)

    scope_provider = SPIScopeProvider()
    ui = ScopePanelUI(scope_provider)
    ui.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

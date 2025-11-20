#!/usr/bin/env python3
"""
Professional STM32 Oscilloscope - Python Version
Features:
- AUTO TRIGGER: Smart switching between triggered/free-run
- NORMAL TRIGGER: Strict trigger mode
- FREE RUN: Continuous update
- Rotary Encoder EC11 support (GPIO 17, 27, 22)
- Phase-locked waveform alignment
- Auto Trigger 50%

Install dependencies:
    sudo apt-get install -y python3-pyqt5 python3-spidev python3-rpi.gpio

Run:
    python3 scope.py
"""

import sys
import time
import struct
import threading
from collections import deque
from enum import Enum
from datetime import datetime
import math

try:
    import spidev
except ImportError:
    print("⚠️  Warning: spidev not available (OK for development)")
    spidev = None

try:
    import RPi.GPIO as GPIO
except ImportError:
    print("⚠️  Warning: RPi.GPIO not available (encoder will not work)")
    GPIO = None

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                              QPushButton, QLabel, QButtonGroup, QRadioButton,
                              QTextEdit)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QPainter, QColor, QPen, QFont

# Constants
MARKER_START = 0xAA
MARKER_HEADER = 0x55
BUFFER_SIZE = 512
PACKET_SIZE = 4 + BUFFER_SIZE * 2
SAMPLE_RATE = 411000.0  # Hz
VCC = 3.3  # Volts
ADC_MAX = 4095
CAPTURE_SIZE = 512

class TriggerMode(Enum):
    AUTO = 0
    NORMAL = 1
    FREE_RUN = 2

class TriggerSlope(Enum):
    RISING = 0
    FALLING = 1

# ============================================================================
# Rotary Encoder EC11 - GPIO Control (RPi.GPIO)
# ============================================================================
class RotaryEncoder:
    def __init__(self):
        self.gpio_clk = 17  # Pin 11
        self.gpio_dt = 27   # Pin 13
        self.gpio_sw = 22   # Pin 15

        self.running = False
        self.thread = None

        self.last_clk = 1
        self.last_sw = 1
        self.last_rotation_time = time.time()
        self.last_button_time = time.time()

        self.on_rotate = None
        self.on_button_press = None

    def init(self):
        """Initialize GPIO using RPi.GPIO"""
        if GPIO is None:
            print("❌ Encoder: RPi.GPIO not available")
            return False

        try:
            print("🔧 Encoder: Initializing EC11 using RPi.GPIO...")

            # Setup GPIO mode
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)

            # Setup pins as inputs with pull-up resistors
            GPIO.setup(self.gpio_clk, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.gpio_dt, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.gpio_sw, GPIO.IN, pull_up_down=GPIO.PUD_UP)

            print("✅ Encoder: GPIO 17, 27, 22 configured as inputs with pull-up")
            print("✅ Encoder: Init OK!")
            return True
        except Exception as e:
            print(f"❌ Encoder: Init failed - {e}")
            return False

    def encoder_loop(self):
        """Main encoder polling loop"""
        print("🔄 Encoder: Loop started")

        # Log initial values
        init_clk = GPIO.input(self.gpio_clk)
        init_dt = GPIO.input(self.gpio_dt)
        init_sw = GPIO.input(self.gpio_sw)
        print(f"🔄 Encoder: Initial values - CLK={init_clk} DT={init_dt} SW={init_sw}")

        loop_count = 0
        while self.running:
            now = time.time()

            # Read current state
            clk = GPIO.input(self.gpio_clk)
            dt = GPIO.input(self.gpio_dt)
            sw = GPIO.input(self.gpio_sw)

            # Debug first 5 reads
            if loop_count < 5:
                print(f"🔄 Encoder: Loop {loop_count} - CLK={clk} DT={dt} SW={sw}")

            # Detect rotation (CLK falling edge)
            if clk == 0 and self.last_clk == 1:
                elapsed = (now - self.last_rotation_time) * 1000

                # Debug
                if loop_count < 10 or elapsed > 5:
                    print(f"🔄 Encoder: CLK falling edge! DT={dt} elapsed={elapsed:.1f}ms")

                # Debounce (> 5ms)
                if elapsed > 5:
                    # FIXED: CW (clockwise) = DT is HIGH (1) at falling edge
                    # CCW (counter-clockwise) = DT is LOW (0) at falling edge
                    direction = 1 if dt == 1 else -1
                    print(f"🎯 Encoder: ROTATION {'CW ⬆️ ' if direction > 0 else 'CCW ⬇️ '} (DT={dt})")
                    if self.on_rotate:
                        self.on_rotate(direction)
                    self.last_rotation_time = now

            # Detect button press (SW falling edge)
            if sw == 0 and self.last_sw == 1:
                elapsed = (now - self.last_button_time) * 1000
                print(f"🔄 Encoder: SW falling edge! elapsed={elapsed:.1f}ms")

                # Debounce (> 200ms)
                if elapsed > 200:
                    print("🎯 Encoder: BUTTON PRESSED")
                    if self.on_button_press:
                        self.on_button_press()
                    self.last_button_time = now

            self.last_clk = clk
            self.last_sw = sw

            loop_count += 1
            time.sleep(0.01)  # Poll every 10ms

        print("🔄 Encoder: Loop stopped")

    def start(self):
        """Start encoder thread"""
        self.running = True
        self.thread = threading.Thread(target=self.encoder_loop, daemon=True)
        self.thread.start()

    def stop(self):
        """Stop encoder thread"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)

        # Cleanup GPIO
        try:
            GPIO.cleanup([self.gpio_clk, self.gpio_dt, self.gpio_sw])
        except:
            pass

    def set_rotation_callback(self, callback):
        """Set rotation callback function"""
        self.on_rotate = callback

    def set_button_callback(self, callback):
        """Set button press callback function"""
        self.on_button_press = callback

# ============================================================================
# SPI Reader
# ============================================================================
class SPIReader:
    def __init__(self):
        self.spi = None
        self.running = False
        self.thread = None

        # Trigger settings
        self.trigger_mode = TriggerMode.AUTO
        self.trigger_slope = TriggerSlope.RISING
        self.trigger_level = 1.65  # Volts

        # Auto Trigger 50%
        self.auto_trigger_50 = False
        self.last_vmax = 3.3
        self.last_vmin = 0.0

        # Data buffers
        self.voltage_buffer = []
        self.time_buffer = []
        self.lock = threading.Lock()

        # Phase-lock state
        self.edge_index = 100  # Target edge position
        self.shift_samples = 0.0

    def init(self, device="/dev/spidev0.0", speed=16000000):
        """Initialize SPI"""
        if spidev is None:
            print("⚠️  SPI: spidev not available")
            return False

        try:
            self.spi = spidev.SpiDev()
            self.spi.open(0, 0)
            self.spi.max_speed_hz = speed
            self.spi.mode = 0b01  # CPOL=0, CPHA=1
            print(f"✅ SPI: Opened {device} at {speed}Hz")
            return True
        except Exception as e:
            print(f"❌ SPI: Init failed - {e}")
            return False

    def read_packet(self):
        """Read one SPI packet"""
        if self.spi is None:
            time.sleep(0.1)
            return None

        try:
            # Read packet
            data = self.spi.readbytes(PACKET_SIZE)

            # Find marker
            for i in range(len(data) - 3):
                if data[i] == MARKER_START and data[i+1] == MARKER_HEADER:
                    # Found marker
                    payload = data[i+4:i+4+BUFFER_SIZE*2]
                    if len(payload) == BUFFER_SIZE * 2:
                        # Decode ADC values
                        samples = []
                        for j in range(0, len(payload), 2):
                            adc_val = (payload[j] << 8) | payload[j+1]
                            samples.append(adc_val)
                        return samples

            return None
        except Exception as e:
            print(f"❌ SPI: Read error - {e}")
            time.sleep(0.01)
            return None

    def parse_packet(self, samples):
        """Parse ADC samples to voltages"""
        if not samples or len(samples) == 0:
            return [], []

        # Convert to volts
        voltages = [(x * VCC / ADC_MAX) for x in samples]

        # Calculate Vpp, Vmax, Vmin
        vmax = max(voltages)
        vmin = min(voltages)
        vpp = vmax - vmin

        # Auto Trigger 50%
        if self.auto_trigger_50 and vpp > 0.5:
            self.last_vmax = vmax
            self.last_vmin = vmin
            mid_level = (vmax + vmin) / 2.0
            if abs(mid_level - self.trigger_level) > 0.02:
                self.trigger_level = mid_level
                print(f"🎯 Auto 50%: {mid_level:.2f}V (Vmax={vmax:.2f}V, Vmin={vmin:.2f}V)")

        # Find trigger edge
        edge_idx = self.find_trigger_edge(voltages)

        if edge_idx >= 0:
            # Phase-locked alignment
            shift = self.edge_index - edge_idx
            self.shift_samples = shift

            # Shift waveform
            shifted_voltages = self.shift_waveform(voltages, shift)

            # Generate time array
            times = [i / SAMPLE_RATE for i in range(len(shifted_voltages))]

            return shifted_voltages, times
        else:
            # No trigger found (free run)
            times = [i / SAMPLE_RATE for i in range(len(voltages))]
            return voltages, times

    def find_trigger_edge(self, voltages):
        """Find trigger edge in waveform"""
        if len(voltages) < 10:
            return -1

        level = self.trigger_level

        # Search for edge
        for i in range(1, len(voltages) - 1):
            if self.trigger_slope == TriggerSlope.RISING:
                # Rising edge
                if voltages[i-1] < level and voltages[i] >= level:
                    return i
            else:
                # Falling edge
                if voltages[i-1] > level and voltages[i] <= level:
                    return i

        return -1

    def shift_waveform(self, voltages, shift):
        """Shift waveform for phase-lock alignment"""
        if abs(shift) < 0.1:
            return voltages

        # Simple integer shift (can use interpolation for sub-sample accuracy)
        shift_int = int(round(shift))

        if shift_int > 0:
            # Shift right
            return [voltages[0]] * shift_int + voltages[:-shift_int]
        elif shift_int < 0:
            # Shift left
            return voltages[-shift_int:] + [voltages[-1]] * (-shift_int)
        else:
            return voltages

    def reader_loop(self):
        """Main SPI reader loop"""
        print("🔄 SPI: Reader loop started")

        while self.running:
            samples = self.read_packet()
            if samples:
                voltages, times = self.parse_packet(samples)

                with self.lock:
                    self.voltage_buffer = voltages
                    self.time_buffer = times
            else:
                time.sleep(0.001)

        print("🔄 SPI: Reader loop stopped")

    def start(self):
        """Start SPI reader thread"""
        self.running = True
        self.thread = threading.Thread(target=self.reader_loop, daemon=True)
        self.thread.start()

    def stop(self):
        """Stop SPI reader thread"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)

        if self.spi:
            self.spi.close()

    def get_data(self):
        """Get latest waveform data"""
        with self.lock:
            return self.voltage_buffer.copy(), self.time_buffer.copy()

    def set_trigger_mode(self, mode):
        """Set trigger mode"""
        self.trigger_mode = mode

    def set_trigger_slope(self, slope):
        """Set trigger slope"""
        self.trigger_slope = slope

    def set_trigger_level(self, level):
        """Set trigger level (volts)"""
        self.trigger_level = level

    def set_auto_trigger_50(self, enabled):
        """Enable/disable Auto Trigger 50%"""
        self.auto_trigger_50 = enabled
        print(f"🎯 Auto Trigger 50%: {'ON' if enabled else 'OFF'}")

    def get_auto_trigger_50(self):
        """Get Auto Trigger 50% state"""
        return self.auto_trigger_50

# ============================================================================
# Scope Display Widget
# ============================================================================
class ScopeDisplay(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(800, 600)

        self.voltages = []
        self.times = []

        self.time_div = 0.001  # 1ms/div
        self.volt_div = 0.5    # 0.5V/div

        self.trigger_mode = TriggerMode.AUTO
        self.trigger_level = 1.65

        self.bg_color = QColor(10, 10, 10)
        self.grid_color = QColor(40, 40, 40)
        self.trace_color = QColor(0, 255, 0)
        self.trigger_color = QColor(255, 255, 0)

    def set_data(self, voltages, times):
        """Update waveform data"""
        self.voltages = voltages
        self.times = times
        self.update()

    def set_time_div(self, div):
        """Set time/division"""
        self.time_div = div
        self.update()

    def set_volt_div(self, div):
        """Set volts/division"""
        self.volt_div = div
        self.update()

    def set_trigger_level(self, level):
        """Set trigger level"""
        self.trigger_level = level
        self.update()

    def set_trigger_mode(self, mode):
        """Set trigger mode"""
        self.trigger_mode = mode
        self.update()

    def paintEvent(self, event):
        """Paint oscilloscope display"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()

        # Background
        painter.fillRect(0, 0, w, h, self.bg_color)

        # Draw grid
        self.draw_grid(painter, w, h)

        # Draw trigger level
        self.draw_trigger_level(painter, w, h)

        # Draw waveform
        self.draw_waveform(painter, w, h)

        # Draw info text
        self.draw_info(painter, w, h)

    def draw_grid(self, painter, w, h):
        """Draw oscilloscope grid"""
        painter.setPen(QPen(self.grid_color, 1))

        # Vertical lines (10 divisions)
        for i in range(11):
            x = i * w / 10
            painter.drawLine(int(x), 0, int(x), h)

        # Horizontal lines (8 divisions)
        for i in range(9):
            y = i * h / 8
            painter.drawLine(0, int(y), w, int(y))

        # Center crosshair
        painter.setPen(QPen(self.grid_color, 2))
        cx = w / 2
        cy = h / 2
        painter.drawLine(int(cx - 10), int(cy), int(cx + 10), int(cy))
        painter.drawLine(int(cx), int(cy - 10), int(cx), int(cy + 10))

    def draw_trigger_level(self, painter, w, h):
        """Draw trigger level line"""
        painter.setPen(QPen(self.trigger_color, 1, Qt.DashLine))

        # Convert trigger level to screen Y
        v_center = VCC / 2.0
        v_range = self.volt_div * 4  # 4 divisions above/below center

        y_offset = (v_center - self.trigger_level) / v_range * (h / 2)
        y = h / 2 + y_offset

        if 0 <= y <= h:
            painter.drawLine(0, int(y), w, int(y))

    def draw_waveform(self, painter, w, h):
        """Draw waveform trace"""
        if not self.voltages or len(self.voltages) < 2:
            return

        painter.setPen(QPen(self.trace_color, 2))

        v_center = VCC / 2.0
        v_range = self.volt_div * 4  # 4 divisions above/below center
        t_range = self.time_div * 10  # 10 divisions

        for i in range(len(self.voltages) - 1):
            if i >= len(self.times):
                break

            # Convert to screen coordinates
            t1 = self.times[i]
            v1 = self.voltages[i]
            t2 = self.times[i+1]
            v2 = self.voltages[i+1]

            x1 = (t1 / t_range) * w
            y1 = h / 2 + (v_center - v1) / v_range * (h / 2)
            x2 = (t2 / t_range) * w
            y2 = h / 2 + (v_center - v2) / v_range * (h / 2)

            # Clip to screen
            if 0 <= x1 <= w and 0 <= x2 <= w:
                painter.drawLine(int(x1), int(y1), int(x2), int(y2))

    def draw_info(self, painter, w, h):
        """Draw info text"""
        painter.setPen(QPen(QColor(200, 200, 200), 1))
        painter.setFont(QFont("Monospace", 10))

        # Time/Div
        time_str = f"Time: {self.format_time(self.time_div)}/div"
        painter.drawText(10, 20, time_str)

        # Volts/Div
        volt_str = f"Volts: {self.volt_div:.2f}V/div"
        painter.drawText(10, 40, volt_str)

        # Trigger mode
        mode_str = f"Trigger: {self.trigger_mode.name}"
        painter.drawText(10, 60, mode_str)

    def format_time(self, seconds):
        """Format time with appropriate unit"""
        if seconds >= 1.0:
            return f"{seconds:.1f}s"
        elif seconds >= 0.001:
            return f"{seconds*1000:.1f}ms"
        else:
            return f"{seconds*1000000:.1f}µs"

# ============================================================================
# Main Window
# ============================================================================
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Professional Oscilloscope - Python")
        self.resize(1400, 900)

        # Initialize components
        self.reader = SPIReader()
        self.encoder = RotaryEncoder()
        self.display = ScopeDisplay()

        # Encoder mode
        self.encoder_mode_volts = False
        self.time_scale_index = 4  # Default: 1ms (index 4 in new array)
        self.volt_scale_index = 3  # Default: 0.5V (index 3 in new array)

        self.time_scales = [
            ("50µs", 0.00005),
            ("100µs", 0.0001),
            ("200µs", 0.0002),
            ("500µs", 0.0005),
            ("1ms", 0.001),
            ("2ms", 0.002),
            ("5ms", 0.005),
            ("10ms", 0.010),
            ("20ms", 0.020),
            ("50ms", 0.050),
            ("100ms", 0.100)
        ]

        self.volt_scales = [
            ("0.05V", 0.05),
            ("0.1V", 0.1),
            ("0.2V", 0.2),
            ("0.5V", 0.5),
            ("1V", 1.0),
            ("2V", 2.0),
            ("5V", 5.0)
        ]

        # Setup UI
        self.setup_ui()

        # Start components
        if self.reader.init():
            self.reader.start()

        if self.encoder.init():
            self.encoder.set_rotation_callback(self.handle_encoder_rotation)
            self.encoder.set_button_callback(self.toggle_encoder_mode)
            self.encoder.start()
            print("✅ Encoder: Ready! Rotate=adjust, Press=toggle mode")
        else:
            print("⚠️  Encoder: Not available (use keyboard instead)")

        # Update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(50)  # 20 FPS

    def setup_ui(self):
        """Setup user interface"""
        main_layout = QHBoxLayout(self)

        # Left: Display
        main_layout.addWidget(self.display, 3)

        # Right: Control panel
        panel = QWidget()
        panel.setStyleSheet("background-color: #2a2a2a;")
        panel.setFixedWidth(220)
        panel_layout = QVBoxLayout(panel)

        # Title
        title = QLabel("CONTROLS")
        title.setStyleSheet("color: white; font-size: 16px; font-weight: bold;")
        panel_layout.addWidget(title)

        # Trigger Mode
        trigger_label = QLabel("Trigger Mode:")
        trigger_label.setStyleSheet("color: white; font-weight: bold;")
        panel_layout.addWidget(trigger_label)

        self.trigger_group = QButtonGroup()
        for i, mode in enumerate(["AUTO", "NORMAL", "FREE RUN"]):
            rb = QRadioButton(mode)
            rb.setStyleSheet("color: white;")
            self.trigger_group.addButton(rb, i)
            panel_layout.addWidget(rb)
            if i == 0:
                rb.setChecked(True)

        self.trigger_group.buttonClicked.connect(
            lambda btn: self.change_trigger_mode(self.trigger_group.id(btn))
        )

        # Auto Trigger 50%
        self.auto_50_btn = QPushButton("Auto Trigger 50% [A]")
        self.auto_50_btn.setCheckable(True)
        self.auto_50_btn.setStyleSheet("""
            QPushButton {
                background-color: #444;
                color: white;
                padding: 8px;
                border-radius: 4px;
            }
            QPushButton:checked {
                background-color: #0a0;
                font-weight: bold;
            }
        """)
        self.auto_50_btn.clicked.connect(self.toggle_auto_50)
        panel_layout.addWidget(self.auto_50_btn)

        # Spacer
        panel_layout.addStretch()

        # Info label
        self.info_label = QLabel("Use Encoder or\nArrow Keys")
        self.info_label.setStyleSheet("color: #888; font-size: 10px;")
        panel_layout.addWidget(self.info_label)

        main_layout.addWidget(panel)

    def update_display(self):
        """Update display with new data"""
        voltages, times = self.reader.get_data()
        self.display.set_data(voltages, times)

    def change_trigger_mode(self, mode_id):
        """Change trigger mode"""
        mode = [TriggerMode.AUTO, TriggerMode.NORMAL, TriggerMode.FREE_RUN][mode_id]
        self.reader.set_trigger_mode(mode)
        self.display.set_trigger_mode(mode)

    def toggle_auto_50(self):
        """Toggle Auto Trigger 50%"""
        enabled = self.auto_50_btn.isChecked()
        self.reader.set_auto_trigger_50(enabled)

    def handle_encoder_rotation(self, direction):
        """Handle encoder rotation"""
        if not self.encoder_mode_volts:
            # Time/Div mode
            if direction > 0 and self.time_scale_index < len(self.time_scales) - 1:
                self.time_scale_index += 1
            elif direction < 0 and self.time_scale_index > 0:
                self.time_scale_index -= 1

            name, value = self.time_scales[self.time_scale_index]
            self.display.set_time_div(value)
            print(f"🎛️  Time/Div: {name}")
        else:
            # Volts/Div mode
            if direction > 0 and self.volt_scale_index < len(self.volt_scales) - 1:
                self.volt_scale_index += 1
            elif direction < 0 and self.volt_scale_index > 0:
                self.volt_scale_index -= 1

            name, value = self.volt_scales[self.volt_scale_index]
            self.display.set_volt_div(value)
            print(f"🎛️  Volts/Div: {name}")

    def toggle_encoder_mode(self):
        """Toggle encoder mode (Time/Div ↔ Volts/Div)"""
        self.encoder_mode_volts = not self.encoder_mode_volts
        mode_str = "VOLTS/DIV" if self.encoder_mode_volts else "TIME/DIV"
        print(f"🎛️  Mode: {mode_str}")
        self.info_label.setText(f"Encoder Mode:\n{mode_str}")

    def keyPressEvent(self, event):
        """Handle keyboard input"""
        # Auto Trigger 50% (Key 'A')
        if event.key() == Qt.Key_A and not event.isAutoRepeat():
            self.auto_50_btn.toggle()
            self.toggle_auto_50()

        # Arrow keys for Time/Div and Volts/Div
        elif event.key() == Qt.Key_Left:
            self.handle_encoder_rotation(-1)
        elif event.key() == Qt.Key_Right:
            self.handle_encoder_rotation(1)
        elif event.key() == Qt.Key_Up:
            # Switch to Volts mode and increase
            self.encoder_mode_volts = True
            self.handle_encoder_rotation(1)
        elif event.key() == Qt.Key_Down:
            # Switch to Volts mode and decrease
            self.encoder_mode_volts = True
            self.handle_encoder_rotation(-1)
        elif event.key() == Qt.Key_Space:
            # Toggle encoder mode
            self.toggle_encoder_mode()

        super().keyPressEvent(event)

    def closeEvent(self, event):
        """Cleanup on close"""
        self.timer.stop()
        self.reader.stop()
        self.encoder.stop()
        event.accept()

# ============================================================================
# Main
# ============================================================================
def main():
    app = QApplication(sys.argv)

    print("=" * 60)
    print("Professional Oscilloscope - Python Version")
    print("=" * 60)

    window = MainWindow()
    window.show()

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()

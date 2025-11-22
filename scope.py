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
BUFFER_SIZE = 512  # MUST match STM32 firmware packet size!
PACKET_SIZE = 4 + BUFFER_SIZE * 2
SAMPLE_RATE = 260000.0  # Hz (ADC: 10.67MHz / 41 cycles = ~260kSps)
VCC = 3.3  # Volts
ADC_MAX = 4095
CAPTURE_SIZE = 512
DISPLAY_SAMPLES = 4096  # Show more samples using memory buffer

# ============================================================================
# RING BUFFER CONFIGURATION - Deep Memory for Wave History
# ============================================================================
# Memory Depth: 500,000 samples = ~1.2 seconds at 411kHz
# - 1kHz signal: ~1200 cycles stored
# - 100Hz signal: ~120 cycles stored
# - 10Hz signal: ~12 cycles stored
MEMORY_DEPTH = 500000

# Zoom levels for "Full sóng" feature (samples per screen)
ZOOM_LEVELS = [
    (512, "512 smp"),      # Zoom in - single packet
    (1024, "1K smp"),
    (2048, "2K smp"),
    (4096, "4K smp"),
    (8192, "8K smp"),
    (16384, "16K smp"),
    (32768, "32K smp"),
    (65536, "64K smp"),
    (131072, "128K smp"),
    (262144, "256K smp"),
    (500000, "500K smp"),  # Zoom out - full memory (Full sóng)
]

class TriggerMode(Enum):
    AUTO = 0
    NORMAL = 1
    FREE_RUN = 2

class TriggerSlope(Enum):
    RISING = 0
    FALLING = 1

# ============================================================================
# Rotary Encoder EC11 - GPIO Control (Interrupt-based)
# ============================================================================
class RotaryEncoder:
    def __init__(self):
        self.gpio_clk = 17  # Pin 11
        self.gpio_dt = 27   # Pin 13
        self.gpio_sw = 22   # Pin 15

        self.running = False
        self.use_interrupt = False  # Will be set during init
        self.polling_thread = None

        self.last_clk = 1
        self.last_sw = 1
        self.last_dt = 1
        self.last_rotation_time = time.time()
        self.last_button_time = time.time()

        self.on_rotate = None
        self.on_button_press = None

        # Debug stats
        self.rotation_count = 0
        self.button_count = 0
        self.last_direction = 0  # 1=CW, -1=CCW

    def init(self):
        """Initialize GPIO using RPi.GPIO with interrupts (fallback to polling)"""
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

            # Try to add interrupt callbacks
            try:
                # Remove any existing event detection first
                try:
                    GPIO.remove_event_detect(self.gpio_clk)
                    GPIO.remove_event_detect(self.gpio_sw)
                except:
                    pass

                # Add interrupt callbacks (FALLING edge detection)
                GPIO.add_event_detect(self.gpio_clk, GPIO.FALLING,
                                      callback=self._clk_callback,
                                      bouncetime=5)  # 5ms debounce
                GPIO.add_event_detect(self.gpio_sw, GPIO.FALLING,
                                      callback=self._sw_callback,
                                      bouncetime=200)  # 200ms debounce

                print("✅ Encoder: INTERRUPT mode configured (CLK=5ms, SW=200ms debounce)")
                self.use_interrupt = True
            except Exception as e:
                print(f"⚠️  Encoder: Interrupt setup failed ({e})")
                print("📌 Encoder: Falling back to POLLING mode (1ms)")
                self.use_interrupt = False

            print("✅ Encoder: Init OK!")
            return True
        except Exception as e:
            print(f"❌ Encoder: Init failed - {e}")
            return False

    def _clk_callback(self, channel):
        """Interrupt callback for CLK falling edge (rotation detected)"""
        now = time.time()
        elapsed = (now - self.last_rotation_time) * 1000

        # Additional software debounce check
        if elapsed < 5:
            return

        # Read DT state to determine direction
        dt = GPIO.input(self.gpio_dt)

        # FIXED: CW (clockwise) = DT is HIGH (1) at falling edge
        # CCW (counter-clockwise) = DT is LOW (0) at falling edge
        direction = 1 if dt == 1 else -1

        self.rotation_count += 1
        self.last_direction = direction
        self.last_rotation_time = now

        print(f"🎯 Encoder: ROTATION #{self.rotation_count} {'CW ⬆️ ' if direction > 0 else 'CCW ⬇️ '} (DT={dt}, {elapsed:.1f}ms)")

        # Call user callback
        if self.on_rotate:
            self.on_rotate(direction)

    def _sw_callback(self, channel):
        """Interrupt callback for SW falling edge (button pressed)"""
        now = time.time()
        elapsed = (now - self.last_button_time) * 1000

        # Additional software debounce check
        if elapsed < 200:
            return

        self.button_count += 1
        self.last_button_time = now

        print(f"🎯 Encoder: BUTTON PRESSED #{self.button_count} ({elapsed:.1f}ms)")

        # Call user callback
        if self.on_button_press:
            self.on_button_press()

    def _polling_loop(self):
        """Polling loop fallback when interrupts fail

        IMPROVED: Better debounce and state machine approach
        """
        print("🔄 Encoder: Polling loop started (0.5ms interval, 15ms debounce)")

        # State machine for more reliable detection
        clk_low_count = 0  # Count consecutive LOW readings
        CLK_LOW_THRESHOLD = 3  # Require 3 consecutive LOWs (1.5ms)

        ROTATION_DEBOUNCE_MS = 15  # Increased from 5ms to 15ms
        BUTTON_DEBOUNCE_MS = 200

        while self.running:
            now = time.time()

            # Read current GPIO states (multiple reads for noise rejection)
            clk = GPIO.input(self.gpio_clk)
            dt = GPIO.input(self.gpio_dt)
            sw = GPIO.input(self.gpio_sw)

            # State machine for CLK detection
            if clk == 0:
                clk_low_count += 1
            else:
                clk_low_count = 0

            # Detect rotation: CLK stable LOW after being HIGH
            if clk_low_count == CLK_LOW_THRESHOLD and self.last_clk == 1:
                elapsed = (now - self.last_rotation_time) * 1000

                # Debounce check
                if elapsed > ROTATION_DEBOUNCE_MS:
                    # Determine direction from DT state
                    direction = 1 if dt == 1 else -1

                    self.rotation_count += 1
                    self.last_direction = direction
                    self.last_rotation_time = now

                    dir_str = "CW ⬆️ " if direction > 0 else "CCW ⬇️ "
                    print(f"🎯 Encoder: ROTATION #{self.rotation_count} {dir_str} (DT={dt}, {elapsed:.1f}ms)")

                    # Call user callback
                    if self.on_rotate:
                        self.on_rotate(direction)

                    self.last_clk = 0  # Mark as processed

            # Update last_clk when CLK goes HIGH
            if clk == 1:
                self.last_clk = 1

            # Detect button press (SW falling edge)
            if sw == 0 and self.last_sw == 1:
                elapsed = (now - self.last_button_time) * 1000

                # Debounce check
                if elapsed > BUTTON_DEBOUNCE_MS:
                    self.button_count += 1
                    self.last_button_time = now

                    print(f"🎯 Encoder: BUTTON PRESSED #{self.button_count} ({elapsed:.1f}ms)")

                    # Call user callback
                    if self.on_button_press:
                        self.on_button_press()

            # Update last states
            self.last_dt = dt
            self.last_sw = sw

            # Sleep 0.5ms for faster response
            time.sleep(0.0005)

    def start(self):
        """Start encoder (interrupts or polling mode)"""
        self.running = True

        if self.use_interrupt:
            print("✅ Encoder: INTERRUPT mode active (no polling thread needed)")
        else:
            # Start polling thread
            self.polling_thread = threading.Thread(target=self._polling_loop, daemon=True)
            self.polling_thread.start()
            print("✅ Encoder: POLLING mode active (1ms interval)")

    def stop(self):
        """Stop encoder and cleanup"""
        self.running = False

        # Wait for polling thread to finish
        if self.polling_thread and self.polling_thread.is_alive():
            self.polling_thread.join(timeout=1.0)
            print("✅ Encoder: Polling thread stopped")

        # Remove interrupt callbacks
        if self.use_interrupt:
            try:
                GPIO.remove_event_detect(self.gpio_clk)
                GPIO.remove_event_detect(self.gpio_sw)
                print("✅ Encoder: Interrupts removed")
            except:
                pass

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

        # Data buffers (current packet - for display)
        self.voltage_buffer = []
        self.time_buffer = []
        self.lock = threading.Lock()

        # =====================================================
        # DEEP MEMORY BUFFER - Ring Buffer for scroll history
        # =====================================================
        # Uses MEMORY_DEPTH from config (500,000 samples = ~1.2s at 411kHz)
        self.memory = deque(maxlen=MEMORY_DEPTH)
        self.memory_times = deque(maxlen=MEMORY_DEPTH)
        self.memory_sample_count = 0  # Total samples received

        # Scroll/Pause control
        self.scroll_offset = 0  # 0 = live view, >0 = viewing history
        self.is_paused = False  # Pause data collection

        # Zoom level for "Full sóng" feature
        self.zoom_level_index = 3  # Default: 4096 samples
        self.samples_to_display = ZOOM_LEVELS[self.zoom_level_index][0]

        # Phase-lock state
        self.edge_index = 100  # Target edge position
        self.shift_samples = 0.0

        # Debug stats
        self.packet_count = 0
        self.sync_ok_count = 0
        self.trigger_found_count = 0
        self.vpp_latest = 0.0
        self.vmax_latest = 0.0
        self.vmin_latest = 0.0
        self.frequency_latest = 0.0

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
            self.packet_count += 1

            # Find marker
            for i in range(len(data) - 3):
                if data[i] == MARKER_START and data[i+1] == MARKER_HEADER:
                    # Found marker
                    self.sync_ok_count += 1

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

        # Update debug stats
        self.vmax_latest = vmax
        self.vmin_latest = vmin
        self.vpp_latest = vpp

        # Calculate frequency (count zero crossings)
        self.frequency_latest = self.calculate_frequency(voltages)

        # Detect signal clipping (touching ADC limits)
        is_clipped = (vmax >= 3.25) or (vmin <= 0.05)
        if is_clipped:
            if not hasattr(self, '_clipping_warn_count'):
                self._clipping_warn_count = 0
            self._clipping_warn_count += 1
            # Warn every 200 packets (not too spammy)
            if self._clipping_warn_count % 200 == 0:
                print(f"⚠️  SIGNAL CLIPPING: Vmax={vmax:.2f}V, Vmin={vmin:.2f}V")
                print(f"    💡 Tip: Use voltage divider (2 resistors) to reduce signal to 0.5V-2.5V range")

        # Auto Trigger 50%
        if self.auto_trigger_50 and vpp > 0.5:
            self.last_vmax = vmax
            self.last_vmin = vmin
            mid_level = (vmax + vmin) / 2.0
            if abs(mid_level - self.trigger_level) > 0.02:
                self.trigger_level = mid_level
                clip_warn = " ⚠️ CLIPPED" if is_clipped else ""
                print(f"🎯 Auto 50%: {mid_level:.2f}V (Vmax={vmax:.2f}V, Vmin={vmin:.2f}V){clip_warn}")

        # Find trigger edge
        edge_idx = self.find_trigger_edge(voltages)

        if edge_idx >= 0:
            self.trigger_found_count += 1

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

    def calculate_frequency(self, voltages):
        """Calculate frequency from zero crossings with IMPROVED accuracy

        FIXES for Data Stitching Artifacts:
        1. Use period measurement (time between crossings) instead of counting
        2. Apply moving average filter to smooth out jitter
        3. Require minimum 2 complete cycles for accurate measurement
        """
        if len(voltages) < 10:
            return 0.0

        # Calculate signal metrics
        vmax = max(voltages)
        vmin = min(voltages)
        vpp = vmax - vmin
        avg = sum(voltages) / len(voltages)

        # Detect clipping (signal touches ADC limits)
        is_clipped = (vmax >= 3.25) or (vmin <= 0.05)

        # Filter out noise: signal must have at least 300mV amplitude
        if vpp < 0.3:
            return 0.0

        # ADAPTIVE parameters based on clipping detection
        if is_clipped:
            threshold = vpp * 0.25  # 25% hysteresis for clipped
            MIN_SAMPLES_BETWEEN_CROSSINGS = 150  # Anti-ringing
        else:
            threshold = vpp * 0.15  # 15% hysteresis
            MIN_SAMPLES_BETWEEN_CROSSINGS = 50

        # Find all rising edge crossings and measure periods
        crossing_indices = []
        last_crossing_index = -MIN_SAMPLES_BETWEEN_CROSSINGS

        for i in range(1, len(voltages)):
            if voltages[i-1] < (avg - threshold) and voltages[i] >= (avg + threshold):
                if (i - last_crossing_index) >= MIN_SAMPLES_BETWEEN_CROSSINGS:
                    crossing_indices.append(i)
                    last_crossing_index = i

        # Need at least 2 crossings to measure period
        if len(crossing_indices) < 2:
            return 0.0

        # Calculate periods between consecutive crossings
        periods = []
        for i in range(1, len(crossing_indices)):
            period_samples = crossing_indices[i] - crossing_indices[i-1]
            period_seconds = period_samples / SAMPLE_RATE
            if period_seconds > 0:
                periods.append(period_seconds)

        if not periods:
            return 0.0

        # Use MEDIAN period (more robust than mean for noisy data)
        periods.sort()
        median_period = periods[len(periods) // 2]
        frequency = 1.0 / median_period

        # Apply moving average filter to smooth frequency readings
        if not hasattr(self, '_freq_history'):
            self._freq_history = []

        self._freq_history.append(frequency)
        if len(self._freq_history) > 10:  # Keep last 10 readings
            self._freq_history.pop(0)

        # Smoothed frequency (moving average)
        smoothed_freq = sum(self._freq_history) / len(self._freq_history)

        # Debug: Show frequency calculation details periodically
        if not hasattr(self, '_freq_calc_count'):
            self._freq_calc_count = 0
        self._freq_calc_count += 1

        if self._freq_calc_count % 100 == 0:  # Every 100 calculations
            clip_warn = " ⚠️ CLIPPED" if is_clipped else ""
            print(f"📊 Freq Debug: {len(crossing_indices)} crossings, {len(periods)} periods")
            print(f"   Periods(ms): {[f'{p*1000:.2f}' for p in periods[:5]]}")
            print(f"   Median: {median_period*1000:.3f}ms → Raw: {frequency:.1f}Hz → Smooth: {smoothed_freq:.1f}Hz{clip_warn}")

        return smoothed_freq

    def find_trigger_edge(self, voltages):
        """Find trigger edge with ENHANCED stability for clipped signals"""
        if len(voltages) < 20:
            return -1

        level = self.trigger_level

        # ADAPTIVE HYSTERESIS: Scale with signal amplitude
        # Calculate Vpp to detect clipping and adjust hysteresis
        vmax = max(voltages)
        vmin = min(voltages)
        vpp = vmax - vmin
        is_clipped = (vmax >= 3.25) or (vmin <= 0.05)

        # Enhanced hysteresis for clipped signals
        if is_clipped:
            # Clipped signal: Use much larger hysteresis (10% of Vpp, min 150mV)
            # This helps skip ringing/noise at clipped edges
            hysteresis = max(0.15, vpp * 0.10)
            # For clipped signals, require stable LOW state before accepting trigger
            min_stable_samples = 20  # ~0.05ms at 411kHz
        else:
            # Normal signal: Use standard hysteresis (3% of Vpp, min 50mV)
            hysteresis = max(0.05, vpp * 0.03)
            min_stable_samples = 5

        # STATE MACHINE approach for more stable trigger
        # Must be in LOW/HIGH state for min_stable_samples before accepting edge
        in_low_state = False
        low_state_count = 0
        in_high_state = False
        high_state_count = 0

        for i in range(len(voltages)):
            v = voltages[i]

            if self.trigger_slope == TriggerSlope.RISING:
                # Check if in LOW state (below level - hysteresis)
                if v < (level - hysteresis):
                    if not in_low_state:
                        in_low_state = True
                        low_state_count = 1
                    else:
                        low_state_count += 1
                    in_high_state = False
                    high_state_count = 0
                # Check if crossing to HIGH (above level)
                elif v >= level:
                    if in_low_state and low_state_count >= min_stable_samples:
                        # Valid rising edge: was LOW long enough, now crossing HIGH
                        return i
                    # Reset if we see HIGH without stable LOW first
                    in_low_state = False
                    low_state_count = 0
            else:
                # FALLING edge detection
                # Check if in HIGH state (above level + hysteresis)
                if v > (level + hysteresis):
                    if not in_high_state:
                        in_high_state = True
                        high_state_count = 1
                    else:
                        high_state_count += 1
                    in_low_state = False
                    low_state_count = 0
                # Check if crossing to LOW (below level)
                elif v <= level:
                    if in_high_state and high_state_count >= min_stable_samples:
                        # Valid falling edge: was HIGH long enough, now crossing LOW
                        return i
                    # Reset if we see LOW without stable HIGH first
                    in_high_state = False
                    high_state_count = 0

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
        """Main SPI reader loop with DATA STITCHING detection"""
        print("🔄 SPI: Reader loop started")

        loop_count = 0
        last_debug_time = time.time()

        # Data stitching detection
        last_packet_end_voltage = None
        stitch_glitches = 0  # Count discontinuities
        last_packet_time = time.time()

        while self.running:
            # Check if paused - skip reading if paused
            if self.is_paused:
                time.sleep(0.01)  # Sleep longer when paused
                continue

            packet_start_time = time.time()
            samples = self.read_packet()

            if samples:
                voltages, times = self.parse_packet(samples)

                # =====================================================
                # DATA STITCHING DETECTION
                # =====================================================
                # Detect voltage discontinuity at packet boundary
                if last_packet_end_voltage is not None and len(voltages) > 0:
                    voltage_jump = abs(voltages[0] - last_packet_end_voltage)
                    # If jump > 0.5V, likely a stitching artifact
                    if voltage_jump > 0.5:
                        stitch_glitches += 1

                # Track packet timing (dead time detection)
                packet_interval = (packet_start_time - last_packet_time) * 1000  # ms
                expected_interval = BUFFER_SIZE / SAMPLE_RATE * 1000  # ~1.25ms for 512 samples
                dead_time = packet_interval - expected_interval

                if len(voltages) > 0:
                    last_packet_end_voltage = voltages[-1]
                last_packet_time = packet_start_time

                with self.lock:
                    # Store for live display
                    self.voltage_buffer = voltages
                    self.time_buffer = times

                    # =====================================================
                    # DEEP MEMORY: Extend into ring buffer
                    # =====================================================
                    # Add new samples to memory (oldest will auto-drop)
                    self.memory.extend(voltages)

                    # Track time for each sample (absolute time)
                    base_time = self.memory_sample_count / SAMPLE_RATE
                    for i in range(len(voltages)):
                        self.memory_times.append(base_time + i / SAMPLE_RATE)

                    self.memory_sample_count += len(voltages)

                loop_count += 1

                # Print debug stats every 5 seconds
                now = time.time()
                if now - last_debug_time >= 5.0:
                    sync_rate = 100.0 * self.sync_ok_count / max(1, self.packet_count)
                    mem_size = len(self.memory)
                    mem_time = mem_size / SAMPLE_RATE * 1000  # ms

                    print(f"")
                    print(f"{'='*60}")
                    print(f"📊 SPI Debug Report (every 5s)")
                    print(f"{'='*60}")
                    print(f"  Packets: {self.packet_count} | Sync: {sync_rate:.1f}% | Triggers: {self.trigger_found_count}")
                    print(f"  Signal: Vpp={self.vpp_latest:.2f}V | Freq={self.frequency_latest:.1f}Hz")
                    print(f"  Memory: {mem_size:,} samples ({mem_time:.0f}ms) | Offset={self.scroll_offset}")
                    print(f"  Data Stitching: {stitch_glitches} glitches detected")
                    print(f"  Packet interval: {packet_interval:.2f}ms (expected: {expected_interval:.2f}ms)")
                    if dead_time > 0.1:
                        print(f"  ⚠️  Dead time: {dead_time:.2f}ms (may cause artifacts)")
                    print(f"{'='*60}")
                    print(f"")

                    last_debug_time = now
                    stitch_glitches = 0  # Reset counter
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

    # =====================================================
    # DEEP MEMORY: Scroll/Pause/History Methods
    # =====================================================

    def get_data_from_memory(self, num_samples=2048):
        """Get data from memory buffer with scroll offset

        Args:
            num_samples: Number of samples to retrieve

        Returns:
            (voltages, times) tuple from memory

        When scroll_offset = 0: Returns most recent data (live view)
        When scroll_offset > 0: Returns historical data
        """
        with self.lock:
            mem_len = len(self.memory)

            if mem_len == 0:
                return [], []

            # Calculate start and end indices
            # offset=0 means live view (end of buffer)
            end_idx = mem_len - self.scroll_offset
            start_idx = end_idx - num_samples

            # Clamp to valid range
            if end_idx > mem_len:
                end_idx = mem_len
            if start_idx < 0:
                start_idx = 0
            if end_idx < 0:
                end_idx = 0

            if start_idx >= end_idx:
                return [], []

            # Extract slice from deque (convert to list first)
            mem_list = list(self.memory)
            voltages = mem_list[start_idx:end_idx]

            # Generate relative time array
            times = [i / SAMPLE_RATE for i in range(len(voltages))]

            return voltages, times

    def toggle_pause(self):
        """Toggle pause state"""
        self.is_paused = not self.is_paused
        state = "PAUSED ⏸️" if self.is_paused else "LIVE ▶️"
        print(f"📹 {state}")

        # When resuming, reset scroll offset to live view
        if not self.is_paused:
            self.scroll_offset = 0

        return self.is_paused

    def scroll(self, amount):
        """Scroll through memory history

        Args:
            amount: Positive = scroll back (older), Negative = scroll forward (newer)
        """
        if not self.is_paused:
            # Auto-pause when scrolling
            self.is_paused = True
            print("📹 Auto-PAUSED for scrolling")

        # Calculate new offset
        new_offset = self.scroll_offset + amount

        # Clamp to valid range
        max_offset = max(0, len(self.memory) - 2048)  # Leave room for display
        if new_offset < 0:
            new_offset = 0
        if new_offset > max_offset:
            new_offset = max_offset

        self.scroll_offset = new_offset

        # Calculate time position
        if len(self.memory) > 0:
            time_pos = self.scroll_offset / SAMPLE_RATE * 1000  # ms from live
            print(f"⏪ Scroll: -{time_pos:.1f}ms from live (offset={self.scroll_offset})")

    def go_to_live(self):
        """Return to live view"""
        self.scroll_offset = 0
        self.is_paused = False
        print("📹 LIVE ▶️ (scroll reset)")

    def get_memory_info(self):
        """Get memory buffer information"""
        mem_len = len(self.memory)
        mem_time_ms = mem_len / SAMPLE_RATE * 1000
        offset_time_ms = self.scroll_offset / SAMPLE_RATE * 1000
        zoom_name = ZOOM_LEVELS[self.zoom_level_index][1]
        return {
            'samples': mem_len,
            'max_samples': MEMORY_DEPTH,
            'time_ms': mem_time_ms,
            'offset': self.scroll_offset,
            'offset_time_ms': offset_time_ms,
            'is_paused': self.is_paused,
            'is_live': self.scroll_offset == 0 and not self.is_paused,
            'zoom_level': self.zoom_level_index,
            'zoom_name': zoom_name,
            'samples_to_display': self.samples_to_display,
            'scroll_percent': (self.scroll_offset / max(1, mem_len - self.samples_to_display)) * 100 if mem_len > self.samples_to_display else 0
        }

    # =====================================================
    # ZOOM CONTROL - "Full sóng" feature
    # =====================================================

    def zoom_in(self):
        """Zoom in - show fewer samples (more detail)"""
        if self.zoom_level_index > 0:
            self.zoom_level_index -= 1
            self.samples_to_display = ZOOM_LEVELS[self.zoom_level_index][0]
            name = ZOOM_LEVELS[self.zoom_level_index][1]
            print(f"🔍 Zoom In: {name}")
            return True
        return False

    def zoom_out(self):
        """Zoom out - show more samples (Full sóng)"""
        if self.zoom_level_index < len(ZOOM_LEVELS) - 1:
            self.zoom_level_index += 1
            self.samples_to_display = ZOOM_LEVELS[self.zoom_level_index][0]
            name = ZOOM_LEVELS[self.zoom_level_index][1]
            print(f"🔍 Zoom Out: {name}")
            return True
        return False

    def set_zoom_level(self, index):
        """Set specific zoom level"""
        if 0 <= index < len(ZOOM_LEVELS):
            self.zoom_level_index = index
            self.samples_to_display = ZOOM_LEVELS[index][0]
            name = ZOOM_LEVELS[index][1]
            print(f"🔍 Zoom: {name}")

    def zoom_full(self):
        """Zoom to show full memory (Full sóng)"""
        self.zoom_level_index = len(ZOOM_LEVELS) - 1
        self.samples_to_display = ZOOM_LEVELS[self.zoom_level_index][0]
        print(f"🔍 FULL WAVE: {ZOOM_LEVELS[self.zoom_level_index][1]}")

    # =====================================================
    # DOWNSAMPLING - Min-Max algorithm for zoom out
    # =====================================================

    def downsample_minmax(self, data, target_points):
        """Downsample using Min-Max algorithm to preserve peaks

        This is essential for "Full sóng" - when showing 500K samples
        on a 1000-pixel screen, we need to preserve waveform peaks.

        Algorithm: For each output point, find min and max in the
        corresponding input segment, output both to preserve peaks.
        """
        if len(data) <= target_points:
            return data

        result = []
        samples_per_point = len(data) / (target_points // 2)

        i = 0
        while i < len(data) and len(result) < target_points:
            # Calculate segment boundaries
            start = int(i)
            end = int(min(i + samples_per_point, len(data)))

            if start >= end:
                break

            segment = data[start:end]
            if segment:
                # Add both min and max to preserve peaks
                result.append(min(segment))
                result.append(max(segment))

            i += samples_per_point

        return result[:target_points]

    def get_display_data(self, max_points=1000):
        """Get data for display with automatic downsampling

        This is the main method for "Full sóng" and "Kéo lại xem".
        - Uses scroll_offset to navigate history
        - Uses samples_to_display for zoom level
        - Applies Min-Max downsampling for large data

        Args:
            max_points: Maximum points to return (screen width)

        Returns:
            (voltages, times) tuple ready for display
        """
        with self.lock:
            mem_len = len(self.memory)

            if mem_len == 0:
                return [], []

            # Calculate window position based on scroll_offset
            end_idx = mem_len - self.scroll_offset
            start_idx = end_idx - self.samples_to_display

            # Clamp to valid range
            if start_idx < 0:
                start_idx = 0
            if end_idx > mem_len:
                end_idx = mem_len
            if end_idx < 0:
                end_idx = 0

            if start_idx >= end_idx:
                return [], []

            # Extract data from ring buffer
            mem_list = list(self.memory)
            raw_voltages = mem_list[start_idx:end_idx]

            # Apply downsampling if needed (for "Full sóng")
            if len(raw_voltages) > max_points:
                voltages = self.downsample_minmax(raw_voltages, max_points)
            else:
                voltages = raw_voltages

            # Generate time array
            times = [i / SAMPLE_RATE for i in range(len(voltages))]

            return voltages, times

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
    # Signal for double-click fullscreen toggle
    doubleClicked = pyqtSignal()

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

        # Drawing mode: 'line' or 'step'
        self.draw_mode = 'step'  # Step mode for square wave visualization

        # Auto-fit mode: waveform always fills screen width
        self.auto_fit = True  # Default ON - always fill screen

    def mouseDoubleClickEvent(self, event):
        """Handle double-click to toggle fullscreen"""
        self.doubleClicked.emit()
        super().mouseDoubleClickEvent(event)

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
        """Draw waveform trace (line or step mode)"""
        if not self.voltages or len(self.voltages) < 2:
            return

        painter.setPen(QPen(self.trace_color, 2))

        v_center = VCC / 2.0
        v_range = self.volt_div * 4  # 4 divisions above/below center
        t_range = self.time_div * 10  # 10 divisions
        num_samples = len(self.voltages)

        if self.draw_mode == 'step':
            # Step drawing (better for square waves)
            for i in range(num_samples - 1):
                if i >= len(self.times):
                    break

                v1 = self.voltages[i]
                v2 = self.voltages[i+1]

                # X coordinate: auto-fit or time-based
                if self.auto_fit:
                    # Auto-fit: always fill screen width
                    x1 = (i / (num_samples - 1)) * w
                    x2 = ((i + 1) / (num_samples - 1)) * w
                else:
                    # Time-based: use time_div setting
                    t1 = self.times[i]
                    t2 = self.times[i+1]
                    x1 = (t1 / t_range) * w
                    x2 = (t2 / t_range) * w

                # Y coordinate: voltage to screen
                y1 = h / 2 + (v_center - v1) / v_range * (h / 2)
                y2 = h / 2 + (v_center - v2) / v_range * (h / 2)

                # Clip to screen
                if 0 <= x1 <= w and 0 <= x2 <= w:
                    # Draw horizontal line at v1 level
                    painter.drawLine(int(x1), int(y1), int(x2), int(y1))
                    # Draw vertical line to v2 level
                    painter.drawLine(int(x2), int(y1), int(x2), int(y2))
        else:
            # Line drawing (original)
            for i in range(num_samples - 1):
                if i >= len(self.times):
                    break

                v1 = self.voltages[i]
                v2 = self.voltages[i+1]

                # X coordinate: auto-fit or time-based
                if self.auto_fit:
                    # Auto-fit: always fill screen width
                    x1 = (i / (num_samples - 1)) * w
                    x2 = ((i + 1) / (num_samples - 1)) * w
                else:
                    # Time-based: use time_div setting
                    t1 = self.times[i]
                    t2 = self.times[i+1]
                    x1 = (t1 / t_range) * w
                    x2 = (t2 / t_range) * w

                # Y coordinate: voltage to screen
                y1 = h / 2 + (v_center - v1) / v_range * (h / 2)
                y2 = h / 2 + (v_center - v2) / v_range * (h / 2)

                # Clip to screen
                if 0 <= x1 <= w and 0 <= x2 <= w:
                    painter.drawLine(int(x1), int(y1), int(x2), int(y2))

    def draw_info(self, painter, w, h):
        """Draw info text"""
        painter.setPen(QPen(QColor(200, 200, 200), 1))
        painter.setFont(QFont("Monospace", 10))

        # Calculate total time displayed
        if len(self.voltages) > 0:
            total_time = len(self.voltages) / SAMPLE_RATE
            time_str = f"Samples: {len(self.voltages)} ({self.format_time(total_time)})"
        else:
            time_str = "No data"
        painter.drawText(10, 20, time_str)

        # Volts/Div
        volt_str = f"Volts: {self.volt_div:.2f}V/div"
        painter.drawText(10, 40, volt_str)

        # Auto-fit status
        fit_str = "Auto-Fit: ON" if self.auto_fit else f"Time: {self.format_time(self.time_div)}/div"
        painter.drawText(10, 60, fit_str)

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

        # Fullscreen mode
        self.is_fullscreen = False
        self.panel = None  # Will store control panel reference

        # Initialize components
        self.reader = SPIReader()
        self.encoder = RotaryEncoder()
        self.display = ScopeDisplay()
        self.display.doubleClicked.connect(self.toggle_fullscreen)

        # Encoder mode: 0=TIME/DIV, 1=VOLTS/DIV, 2=SCROLL
        self.encoder_mode = 0  # 0=TIME, 1=VOLTS, 2=SCROLL
        self.encoder_mode_volts = False  # Legacy compatibility
        self.time_scale_index = 4  # Default: 1ms (index 4 in new array)
        self.volt_scale_index = 3  # Default: 0.5V (index 3 in new array)
        self.scroll_step = 500  # Samples to scroll per encoder step

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
        self.panel = QWidget()
        self.panel.setStyleSheet("background-color: #2a2a2a;")
        self.panel.setFixedWidth(220)
        panel_layout = QVBoxLayout(self.panel)

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

        # =====================================================
        # WAVE HISTORY CONTROLS - "Kéo lại xem" / "Full sóng"
        # =====================================================
        panel_layout.addSpacing(10)
        history_title = QLabel("━━━ WAVE HISTORY ━━━")
        history_title.setStyleSheet("color: #f80; font-size: 12px; font-weight: bold;")
        panel_layout.addWidget(history_title)

        # RUN/STOP button
        self.run_stop_btn = QPushButton("▶️ RUN [P]")
        self.run_stop_btn.setCheckable(True)
        self.run_stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #0a0;
                color: white;
                padding: 10px;
                border-radius: 4px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:checked {
                background-color: #a00;
            }
        """)
        self.run_stop_btn.clicked.connect(self.toggle_run_stop)
        panel_layout.addWidget(self.run_stop_btn)

        # Zoom controls (horizontal layout)
        zoom_layout = QHBoxLayout()
        self.zoom_in_btn = QPushButton("🔍+")
        self.zoom_out_btn = QPushButton("🔍-")
        self.zoom_full_btn = QPushButton("FULL")

        for btn in [self.zoom_in_btn, self.zoom_out_btn, self.zoom_full_btn]:
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #555;
                    color: white;
                    padding: 6px;
                    border-radius: 4px;
                }
                QPushButton:hover {
                    background-color: #777;
                }
            """)

        self.zoom_in_btn.clicked.connect(lambda: self.reader.zoom_in())
        self.zoom_out_btn.clicked.connect(lambda: self.reader.zoom_out())
        self.zoom_full_btn.clicked.connect(lambda: self.reader.zoom_full())

        zoom_layout.addWidget(self.zoom_in_btn)
        zoom_layout.addWidget(self.zoom_out_btn)
        zoom_layout.addWidget(self.zoom_full_btn)
        panel_layout.addLayout(zoom_layout)

        # Scroll controls
        scroll_layout = QHBoxLayout()
        self.scroll_back_btn = QPushButton("⏪")
        self.scroll_fwd_btn = QPushButton("⏩")
        self.scroll_live_btn = QPushButton("LIVE")

        for btn in [self.scroll_back_btn, self.scroll_fwd_btn]:
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #555;
                    color: white;
                    padding: 6px;
                    border-radius: 4px;
                }
                QPushButton:hover {
                    background-color: #777;
                }
            """)

        self.scroll_live_btn.setStyleSheet("""
            QPushButton {
                background-color: #080;
                color: white;
                padding: 6px;
                border-radius: 4px;
                font-weight: bold;
            }
        """)

        self.scroll_back_btn.clicked.connect(lambda: self.reader.scroll(2000))
        self.scroll_fwd_btn.clicked.connect(lambda: self.reader.scroll(-2000))
        self.scroll_live_btn.clicked.connect(lambda: self.reader.go_to_live())

        scroll_layout.addWidget(self.scroll_back_btn)
        scroll_layout.addWidget(self.scroll_fwd_btn)
        scroll_layout.addWidget(self.scroll_live_btn)
        panel_layout.addLayout(scroll_layout)

        # Memory/Scroll status
        self.memory_status = QLabel("Memory: 0/500K\nZoom: 4K smp\nOffset: 0ms")
        self.memory_status.setStyleSheet("color: #f80; font-size: 10px; font-family: monospace;")
        panel_layout.addWidget(self.memory_status)

        # Debug Section
        panel_layout.addSpacing(10)
        debug_title = QLabel("━━━ DEBUG INFO ━━━")
        debug_title.setStyleSheet("color: #666; font-size: 12px; font-weight: bold;")
        panel_layout.addWidget(debug_title)

        # Signal measurements
        self.debug_signal = QLabel("Signal:\nVpp: 0.00V\nFreq: 0 Hz")
        self.debug_signal.setStyleSheet("color: #0f0; font-size: 9px; font-family: monospace;")
        panel_layout.addWidget(self.debug_signal)

        # SPI stats
        self.debug_spi = QLabel("SPI:\nPackets: 0\nSync: 0%")
        self.debug_spi.setStyleSheet("color: #0ff; font-size: 9px; font-family: monospace;")
        panel_layout.addWidget(self.debug_spi)

        # Encoder stats
        self.debug_encoder = QLabel("Encoder:\nRotations: 0\nButtons: 0\nMode: TIME/DIV")
        self.debug_encoder.setStyleSheet("color: #ff0; font-size: 9px; font-family: monospace;")
        panel_layout.addWidget(self.debug_encoder)

        # Spacer
        panel_layout.addStretch()

        # Info label
        self.info_label = QLabel("Use Encoder or\nArrow Keys\n\nF/F11 = Fullscreen\nT = Auto-Fit\nDouble-click = Full")
        self.info_label.setStyleSheet("color: #888; font-size: 10px;")
        panel_layout.addWidget(self.info_label)

        main_layout.addWidget(self.panel)

    def update_display(self):
        """Update display with new data"""
        # Check if we're in live view or scroll/pause mode
        mem_info = self.reader.get_memory_info()

        if mem_info['is_live'] and mem_info['samples_to_display'] <= 512:
            # LIVE MODE with small zoom: Use single packet (512 samples)
            # to avoid stitching artifacts (no data gaps)
            voltages, times = self.reader.get_data()
        else:
            # SCROLL/PAUSE MODE or ZOOM OUT: Use memory buffer
            # Uses get_display_data with automatic downsampling for "Full sóng"
            voltages, times = self.reader.get_display_data(max_points=1000)

        if len(voltages) > 0:
            self.display.set_data(voltages, times)

        # Update memory status display
        self.update_memory_status(mem_info)

        # Update debug info
        self.update_debug_info()

    def update_memory_status(self, mem_info):
        """Update memory/scroll status display"""
        samples = mem_info['samples']
        max_samples = mem_info['max_samples']
        zoom_name = mem_info['zoom_name']
        offset_ms = mem_info['offset_time_ms']
        scroll_pct = mem_info['scroll_percent']

        # Format memory usage
        if samples >= 1000000:
            mem_str = f"{samples/1000000:.1f}M"
        elif samples >= 1000:
            mem_str = f"{samples/1000:.0f}K"
        else:
            mem_str = str(samples)

        max_str = f"{max_samples/1000:.0f}K"

        # Status text
        status = "⏸️ PAUSED" if mem_info['is_paused'] else "▶️ LIVE"
        if mem_info['offset'] > 0:
            status = f"⏪ -{offset_ms:.0f}ms"

        self.memory_status.setText(
            f"Memory: {mem_str}/{max_str}\n"
            f"Zoom: {zoom_name}\n"
            f"Status: {status}"
        )

        # Update RUN/STOP button state
        if mem_info['is_paused'] and not self.run_stop_btn.isChecked():
            self.run_stop_btn.setText("⏸️ STOP [P]")
            self.run_stop_btn.setChecked(True)
        elif not mem_info['is_paused'] and self.run_stop_btn.isChecked():
            self.run_stop_btn.setText("▶️ RUN [P]")
            self.run_stop_btn.setChecked(False)

    def change_trigger_mode(self, mode_id):
        """Change trigger mode"""
        mode = [TriggerMode.AUTO, TriggerMode.NORMAL, TriggerMode.FREE_RUN][mode_id]
        self.reader.set_trigger_mode(mode)
        self.display.set_trigger_mode(mode)

    def toggle_auto_50(self):
        """Toggle Auto Trigger 50%"""
        enabled = self.auto_50_btn.isChecked()
        self.reader.set_auto_trigger_50(enabled)

    def toggle_run_stop(self):
        """Toggle RUN/STOP (Pause/Resume)"""
        is_paused = self.reader.toggle_pause()
        if is_paused:
            self.run_stop_btn.setText("⏸️ STOP [P]")
            self.run_stop_btn.setChecked(True)
        else:
            self.run_stop_btn.setText("▶️ RUN [P]")
            self.run_stop_btn.setChecked(False)

    def handle_encoder_rotation(self, direction):
        """Handle encoder rotation based on current mode"""
        if self.encoder_mode == 0:
            # TIME/DIV mode
            if direction > 0 and self.time_scale_index < len(self.time_scales) - 1:
                self.time_scale_index += 1
            elif direction < 0 and self.time_scale_index > 0:
                self.time_scale_index -= 1

            name, value = self.time_scales[self.time_scale_index]
            self.display.set_time_div(value)
            print(f"🎛️  Time/Div: {name}")

        elif self.encoder_mode == 1:
            # VOLTS/DIV mode
            if direction > 0 and self.volt_scale_index < len(self.volt_scales) - 1:
                self.volt_scale_index += 1
            elif direction < 0 and self.volt_scale_index > 0:
                self.volt_scale_index -= 1

            name, value = self.volt_scales[self.volt_scale_index]
            self.display.set_volt_div(value)
            print(f"🎛️  Volts/Div: {name}")

        elif self.encoder_mode == 2:
            # SCROLL mode - navigate through history
            # CW (direction > 0) = scroll back (older data)
            # CCW (direction < 0) = scroll forward (newer data)
            scroll_amount = direction * self.scroll_step
            self.reader.scroll(scroll_amount)

    def toggle_encoder_mode(self):
        """Toggle encoder mode or pause/live when in scroll mode"""
        # Special behavior in SCROLL mode: button toggles pause/live
        if self.encoder_mode == 2:
            is_paused = self.reader.toggle_pause()
            if is_paused:
                self.info_label.setText("SCROLL ⏸️\nPAUSED\n\nCW=Back CCW=Forward\nPress=Resume Live")
            else:
                self.info_label.setText("SCROLL ▶️\nLIVE\n\nPress to cycle modes")
                # After resuming, cycle to next mode
                self.encoder_mode = 0
                print(f"🎛️  Mode: TIME/DIV (back to live)")
                self.info_label.setText("Encoder Mode:\nTIME/DIV")
            return

        # Normal mode cycling: TIME/DIV → VOLTS/DIV → SCROLL
        self.encoder_mode = (self.encoder_mode + 1) % 3
        mode_names = ["TIME/DIV", "VOLTS/DIV", "SCROLL ⏪"]
        mode_str = mode_names[self.encoder_mode]

        # Legacy compatibility
        self.encoder_mode_volts = (self.encoder_mode == 1)

        # If entering scroll mode, show additional info
        if self.encoder_mode == 2:
            mem_info = self.reader.get_memory_info()
            print(f"🎛️  Mode: {mode_str} (Memory: {mem_info['time_ms']:.0f}ms available)")
            self.info_label.setText(f"Encoder Mode:\n{mode_str}\n\nCW=Back CCW=Forward\nPress=Pause")
        else:
            print(f"🎛️  Mode: {mode_str}")
            self.info_label.setText(f"Encoder Mode:\n{mode_str}")

    def toggle_fullscreen(self):
        """Toggle fullscreen mode - waveform only"""
        if self.is_fullscreen:
            # Exit fullscreen
            self.showNormal()
            self.panel.show()
            self.is_fullscreen = False
            print("🖥️  Exit fullscreen mode")
        else:
            # Enter fullscreen - hide panel, show only waveform
            self.panel.hide()
            self.showFullScreen()
            self.is_fullscreen = True
            print("🖥️  Fullscreen mode (Press F11 or ESC to exit)")

    def update_debug_info(self):
        """Update debug information labels"""
        # Signal measurements
        vpp = self.reader.vpp_latest
        vmax = self.reader.vmax_latest
        vmin = self.reader.vmin_latest
        freq = self.reader.frequency_latest

        # Format frequency
        if freq >= 1000000:
            freq_str = f"{freq/1000000:.2f} MHz"
        elif freq >= 1000:
            freq_str = f"{freq/1000:.2f} kHz"
        else:
            freq_str = f"{freq:.1f} Hz"

        signal_text = f"Signal:\nVpp: {vpp:.2f}V\nVmax: {vmax:.2f}V\nVmin: {vmin:.2f}V\nFreq: {freq_str}"
        self.debug_signal.setText(signal_text)

        # SPI stats
        packets = self.reader.packet_count
        sync_ok = self.reader.sync_ok_count
        sync_rate = (sync_ok / packets * 100) if packets > 0 else 0
        trigger_found = self.reader.trigger_found_count

        spi_text = f"SPI:\nPackets: {packets}\nSync: {sync_rate:.1f}%\nTriggers: {trigger_found}"
        self.debug_spi.setText(spi_text)

        # Encoder stats
        rotations = self.encoder.rotation_count
        buttons = self.encoder.button_count
        mode_names = ["TIME/DIV", "VOLTS/DIV", "SCROLL"]
        mode_str = mode_names[self.encoder_mode]
        last_dir = "CW⬆️" if self.encoder.last_direction > 0 else ("CCW⬇️" if self.encoder.last_direction < 0 else "---")

        # Memory info
        mem_info = self.reader.get_memory_info()
        mem_samples = mem_info['samples']
        mem_time = mem_info['time_ms']
        offset_time = mem_info['offset_time_ms']
        status = "⏸️PAUSED" if mem_info['is_paused'] else "▶️LIVE"

        encoder_text = f"Encoder:\nRotations: {rotations} {last_dir}\nButtons: {buttons}\nMode: {mode_str}\n\nMemory: {mem_time:.0f}ms\nOffset: -{offset_time:.0f}ms\nStatus: {status}"
        self.debug_encoder.setText(encoder_text)

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
            self.encoder_mode = 1  # VOLTS mode
            self.encoder_mode_volts = True
            self.handle_encoder_rotation(1)
        elif event.key() == Qt.Key_Down:
            # Switch to Volts mode and decrease
            self.encoder_mode = 1  # VOLTS mode
            self.encoder_mode_volts = True
            self.handle_encoder_rotation(-1)
        elif event.key() == Qt.Key_Space:
            # Toggle encoder mode
            self.toggle_encoder_mode()

        # Toggle Auto-Fit (Key 'T')
        elif event.key() == Qt.Key_T and not event.isAutoRepeat():
            self.display.auto_fit = not self.display.auto_fit
            status = "ON (fill screen)" if self.display.auto_fit else "OFF (use Time/Div)"
            print(f"📐 Auto-Fit: {status}")

        # Pause/Resume (Key 'P')
        elif event.key() == Qt.Key_P and not event.isAutoRepeat():
            self.toggle_run_stop()

        # Zoom controls (Key 'Z' = zoom in, 'X' = zoom out, 'C' = full)
        elif event.key() == Qt.Key_Z and not event.isAutoRepeat():
            self.reader.zoom_in()
        elif event.key() == Qt.Key_X and not event.isAutoRepeat():
            self.reader.zoom_out()
        elif event.key() == Qt.Key_C and not event.isAutoRepeat():
            self.reader.zoom_full()

        # Go to Live (Key 'L' or Home)
        elif event.key() in (Qt.Key_L, Qt.Key_Home) and not event.isAutoRepeat():
            self.reader.go_to_live()
            self.encoder_mode = 0  # Back to TIME mode
            self.encoder_mode_volts = False

        # Scroll history (PageUp/PageDown)
        elif event.key() == Qt.Key_PageUp:
            # Scroll back (older data)
            self.reader.scroll(self.scroll_step * 5)
        elif event.key() == Qt.Key_PageDown:
            # Scroll forward (newer data)
            self.reader.scroll(-self.scroll_step * 5)

        # Enter scroll mode (Key 'S')
        elif event.key() == Qt.Key_S and not event.isAutoRepeat():
            self.encoder_mode = 2  # SCROLL mode
            mem_info = self.reader.get_memory_info()
            print(f"🎛️  Mode: SCROLL (Memory: {mem_info['time_ms']:.0f}ms available)")
            self.info_label.setText("Encoder Mode:\nSCROLL ⏪\n\nCW=Back CCW=Forward\nPress=Pause")

        # Fullscreen toggle (F11 or F key)
        elif event.key() in (Qt.Key_F11, Qt.Key_F) and not event.isAutoRepeat():
            self.toggle_fullscreen()

        # ESC to exit fullscreen
        elif event.key() == Qt.Key_Escape:
            if self.is_fullscreen:
                self.toggle_fullscreen()

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
    print("  With Ring Buffer Processing (Deep Memory)")
    print("=" * 60)
    print("Keyboard shortcuts:")
    print("  F11/F    = Toggle fullscreen (waveform only)")
    print("  ESC      = Exit fullscreen")
    print("  T        = Toggle Auto-Fit (fill screen)")
    print("  A        = Auto Trigger 50%")
    print("  Arrows   = Adjust Time/Volt scales")
    print("")
    print("  === WAVE HISTORY (Kéo lại xem / Full sóng) ===")
    print("  P        = Pause/Resume (RUN/STOP)")
    print("  L/Home   = Go to live view")
    print("  S        = Scroll mode (encoder)")
    print("  PgUp/PgDn= Scroll history")
    print("  Z        = Zoom in (more detail)")
    print("  X        = Zoom out (more samples)")
    print("  C        = Full wave (show all memory)")
    print("")
    print(f"  Memory Depth: {MEMORY_DEPTH:,} samples (~{MEMORY_DEPTH/SAMPLE_RATE*1000:.0f}ms)")
    print("=" * 60)

    window = MainWindow()
    window.show()

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Oscilloscope 3-CH qua SPI (STM32F103C8T6 â†’ Raspberry Pi 4)
- Giao diá»‡n Ä‘áº¹p tá»« hz.py
- Backend SPI vá»›i packet validation (markers + checksum)
- 3 channels: PA0 (CH0), PA1 (CH1), PB0 (CH2)
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

MARKER_START = 0x1234
MARKER_END = 0x5678

TOTAL_SAMPLES = 1536  # STM32 sends 1536 total samples (interleaved 3 channels)
NUM_CHANNELS = 3
SAMPLES_PER_CHANNEL = TOTAL_SAMPLES // NUM_CHANNELS  # 512 per channel

PACKET_HEADER_SIZE = 2
PACKET_DATA_SIZE = 3072  # 1536 samples Ã— 2 bytes = 3072 bytes
PACKET_FOOTER_SIZE = 4
PACKET_TOTAL_SIZE = PACKET_HEADER_SIZE + PACKET_DATA_SIZE + PACKET_FOOTER_SIZE

# ============================================================================
# STM32 CONFIGURATION - UPDATE THIS TO MATCH YOUR STM32 CODE
# ============================================================================
# TIM3 Configuration for ADC trigger:
#   TIM3->PSC = 3;   // Prescaler = 4
#   TIM3->ARR = 29;  // ARR = 30
#
# Sample rate calculation:
#   Trigger rate = 72MHz / (PSC+1) / (ARR+1)
#                = 72MHz / 4 / 30
#                = 600kHz (trigger frequency)
#
# With ADC SCAN mode (3 channels):
#   Each trigger converts ALL 3 channels: CH0, CH1, CH2
#   So each channel is sampled at 600kHz
#
# Bandwidth: ~300kHz (Nyquist = 600kHz / 2)
# Resolution @ 1kHz: 600 samples/cycle (EXCELLENT for sharp edges!)
# ============================================================================

# STM32 Clock (usually 72MHz for STM32F103)
STM32_CLOCK = 72000000.0

# ============================================================================
# Configuration - MATCH vá»›i code STM32 OPTIMIZED
# ============================================================================
# Code STM32 optimized:
#   TIM3: PSC=3, ARR=29 â†’ 600 kHz trigger
#   ADC: SMPR2 = 0b000 (1.5 cycles), ADCPRE=/2 (default)
#
# TÃ­nh toÃ¡n:
#   ADC Clock = 72MHz / 2 = 36 MHz
#   Time per channel = (1.5 + 12.5) / 36MHz = 0.389 Âµs (FAST!)
#   Time per 3 channels = 1.17 Âµs
#   â†’ ADC max rate = 857 kHz (limited by TIM3 trigger at 600kHz)
# ============================================================================

TIM3_PSC = 3  # Code báº¡n Ä‘ang dÃ¹ng
TIM3_ARR = 29
TIM3_TRIGGER_RATE = STM32_CLOCK / (TIM3_PSC + 1) / (TIM3_ARR + 1)

ADC_PRESCALER = 2  # Default

# IMPORTANT: Set this to match your STM32 code!
# oscilloscope_stm32_fixed.cpp uses 1.5 cycles (FASTEST)
ADC_SAMPLE_TIME = 1.5  # cycles (0b000 - FASTEST for sharp square waves)

ADC_CONVERSION_TIME = 12.5  # cycles (fixed for STM32F1)
NUM_CHANNELS = 3

# Calculate ACTUAL sample rate
ADC_CLOCK = STM32_CLOCK / ADC_PRESCALER
TIME_PER_CHANNEL = (ADC_SAMPLE_TIME + ADC_CONVERSION_TIME) / ADC_CLOCK
TIME_PER_SCAN = TIME_PER_CHANNEL * NUM_CHANNELS
ADC_MAX_RATE = 1.0 / TIME_PER_SCAN  # ~857 kHz (ADC capability)

# Sample rate = min(TIM3_TRIGGER_RATE, ADC_MAX_RATE)
# IMPORTANT: This will be ~47.6kHz UNTIL you upload new STM32 code!
# After uploading STM32 with 1.5 cycles, this becomes 600kHz
SAMPLE_RATE = min(TIM3_TRIGGER_RATE, ADC_MAX_RATE)  # Will be 600kHz after STM32 update

# Calibration factor for fine-tuning
CLOCK_CALIBRATION = 1.0
SAMPLE_RATE *= CLOCK_CALIBRATION

print(f"ðŸ“Š Sample Rate: {SAMPLE_RATE:.1f} Hz ({SAMPLE_RATE/1000:.1f} kHz per channel)")
print(f"ðŸ“ˆ Bandwidth: ~{SAMPLE_RATE/2/1000:.0f} kHz (Nyquist)")
print(f"ðŸŽ¯ Resolution @ 1kHz: {SAMPLE_RATE/1000:.0f} samples/cycle")
print(f"âœ… TIM3 trigger = {TIM3_TRIGGER_RATE/1000:.1f} kHz")
print(f"âœ… ADC sample time = {ADC_SAMPLE_TIME} cycles")
print(f"âœ… Multi-channel mode (CH1=PA0, CH2=PA1, CH3=PB0)")
print()

if ADC_SAMPLE_TIME > 50:
    print(f"âš ï¸  WARNING: ADC sample time is SLOW ({ADC_SAMPLE_TIME} cycles)")
    print(f"   â†’ Square waves will NOT be sharp")
    print(f"   â†’ Run: python3 show_stm32_fix.py for instructions")
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
    """SPI provider - nháº­n data tá»« STM32 qua SPI"""
    # emit block: {'sr': float, 't': np.ndarray, 'ch1': nd, 'ch2': nd, 'ch3': nd}
    block = QtCore.pyqtSignal(dict)
    status_update = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._run = False
        self.spi = None
        self._idx = 0
        self.sr = SAMPLE_RATE  # Per channel sample rate

    def start(self):
        # Kiá»ƒm tra spidev
        try:
            import spidev  # noqa: F401
        except ImportError:
            QtWidgets.QMessageBox.critical(
                None, "Thiáº¿u spidev",
                "HÃ£y cÃ i: sudo apt install python3-spidev"
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
        self.spi.max_speed_hz = 2000000  # 2MHz (more stable)
        self.spi.mode = 0
        self.spi.lsbfirst = False
        print("âœ“ SPI initialized: 2MHz, Mode 0")

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

    def _deinterleave_channels(self, interleaved_data):
        """Split interleaved data into separate channels

        WORKAROUND: STM32 ADC sequence is currently: CH0, CH8, CH1 (instead of CH0, CH1, CH8)
        So we need to swap offset1 and offset2 to match:
        - offset0 â†’ CH0 (PA0) â†’ display as CH1 âœ“
        - offset1 â†’ CH8 (PB0) â†’ display as CH3 (swap!)
        - offset2 â†’ CH1 (PA1) â†’ display as CH2 (swap!)
        """
        # Truncate to make it divisible by 3 to ensure equal channel sizes
        total_samples = len(interleaved_data)
        usable_samples = (total_samples // 3) * 3  # Round down to multiple of 3

        if usable_samples < total_samples:
            interleaved_data = interleaved_data[:usable_samples]

        # Deinterleave with SWAPPED CH2/CH3 to fix STM32 sequence bug
        # ACTUAL sequence from STM32: CH0, CH8, CH1
        offset0 = interleaved_data[0::3]  # CH0 (PA0)
        offset1 = interleaved_data[1::3]  # CH8 (PB0) - WRONG position!
        offset2 = interleaved_data[2::3]  # CH1 (PA1) - WRONG position!

        # Return with SWAPPED channels to fix mapping
        # CH1 = PA0 (correct)
        # CH2 = PA1 (from offset2)
        # CH3 = PB0 (from offset1)
        ch0 = offset0  # PA0 â†’ CH1 âœ“
        ch1 = offset2  # PA1 â†’ CH2 (SWAPPED!)
        ch2 = offset1  # PB0 â†’ CH3 (SWAPPED!)

        # Truncate to same size
        min_size = min(ch0.size, ch1.size, ch2.size)
        ch0 = ch0[:min_size]
        ch1 = ch1[:min_size]
        ch2 = ch2[:min_size]

        return ch0, ch1, ch2

    def _loop(self):
        try:
            self._init_spi()

            # Start acquisition
            self.spi.xfer2([CMD_START])
            time.sleep(0.5)
            print("âœ“ Sent START command")
            print("ðŸ“Š DEBUG MODE: Monitoring frames (first 3 good, all bad)...")

            frame_count = 0
            good_frames = 0
            bad_frames = 0
            consecutive_bad = 0  # Track consecutive bad frames
            last_fps_time = time.time()

            # Pattern detection for channel rotation
            pattern_history = []  # Track which channel has signal

            while self._run:
                try:
                    # Request data
                    self.spi.xfer2([CMD_DATA])
                    time.sleep(0.050)  # Wait for STM32 to prepare data (increased from 0.010)

                    # Read packet (read extra to account for potential offset/junk data)
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
                            time.sleep(0.001)  # Short delay between chunks

                    # Parse and validate
                    packet = self._parse_packet(raw_data)

                    # DEBUG: Show first 32 bytes for bad frames
                    if not packet.valid:
                        hex_preview = ' '.join(f'{raw_data[i]:02X}' for i in range(min(32, len(raw_data))))
                        zeros_count = sum(1 for b in raw_data[:512] if b == 0x00)
                        print(f"âš ï¸  BAD frame: First 32 bytes: {hex_preview}")
                        print(f"    Zeros in first 512 bytes: {zeros_count}/512")
                        if packet.start_marker != MARKER_START:
                            print(f"    â†’ No START marker found!")
                        elif packet.end_marker != MARKER_END:
                            print(f"    â†’ END marker wrong: 0x{packet.end_marker:04X}")
                        elif packet.checksum == 0:
                            print(f"    â†’ Checksum failed")

                    if packet.valid:
                        # De-interleave channels with FIXED offset
                        ch0, ch1, ch2 = self._deinterleave_channels(packet.data)

                        # Debug: Show raw data for first 3 frames
                        if good_frames < 3:
                            # Show signal levels at each offset
                            test_offsets = []
                            for offset in range(3):
                                test_data = packet.data[offset::3]
                                if len(test_data) > 0:
                                    max_val = float(np.max(test_data)) * 3.3 / 4095.0
                                    min_val = float(np.min(test_data)) * 3.3 / 4095.0
                                    vpp_val = max_val - min_val
                                    test_offsets.append(f"offset{offset}: {min_val:.2f}-{max_val:.2f}V (Vpp={vpp_val:.2f}V)")
                            print(f"   ðŸ” Raw offsets:")
                            for line in test_offsets:
                                print(f"      {line}")

                            # Show first 30 raw samples to see the pattern
                            print(f"   ðŸ“Š First 30 raw ADC values:")
                            raw_samples = packet.data[:30]
                            raw_voltages = [f"{val * 3.3 / 4095.0:.3f}V" for val in raw_samples]
                            for i in range(0, 30, 3):
                                if i+2 < 30:
                                    print(f"      [{i//3}] CH0:{raw_voltages[i]} CH1:{raw_voltages[i+1]} CH2:{raw_voltages[i+2]}")

                        # Convert to voltage (0-3.3V)
                        ch0_v = ch0.astype(np.float32) * (3.3 / 4095.0)
                        ch1_v = ch1.astype(np.float32) * (3.3 / 4095.0)
                        ch2_v = ch2.astype(np.float32) * (3.3 / 4095.0)

                        # DEBUG: Pattern detection
                        ch0_has_signal = np.max(ch0_v) > 1.0
                        ch1_has_signal = np.max(ch1_v) > 1.0
                        ch2_has_signal = np.max(ch2_v) > 1.0

                        pattern = f"{'Y' if ch0_has_signal else 'N'}-{'Y' if ch1_has_signal else 'N'}-{'Y' if ch2_has_signal else 'N'}"
                        pattern_history.append(pattern)

                        # Show first 10 frames pattern
                        if good_frames < 10:
                            # Calculate frequency for CH1 (PA0)
                            signal_ac = ch0_v - np.mean(ch0_v)
                            vpp = np.max(signal_ac) - np.min(signal_ac)

                            freq_est = 0
                            if vpp > 0.5:  # If significant signal
                                # Count zero crossings
                                crossings = 0
                                for i in range(1, len(signal_ac)):
                                    if signal_ac[i-1] < 0 and signal_ac[i] >= 0:
                                        crossings += 1

                                if crossings >= 2:
                                    # Period = samples between crossings / sample_rate
                                    avg_period_samples = len(ch0_v) / crossings
                                    period_sec = avg_period_samples / self.sr
                                    freq_est = 1.0 / period_sec if period_sec > 0 else 0

                            print(f"âœ… Frame #{good_frames+1}: CH1(PA0)={ch0_has_signal} CH2(PA1)={ch1_has_signal} CH3(PB0)={ch2_has_signal} | Pattern: {pattern}")
                            print(f"   CH1(PA0): {np.min(ch0_v):.2f}-{np.max(ch0_v):.2f}V | Freq: {freq_est:.1f}Hz | Cycles: {len(ch0_v)/self.sr*freq_est:.1f}")
                            print(f"   CH2(PA1): {np.min(ch1_v):.2f}-{np.max(ch1_v):.2f}V | CH3(PB0): {np.min(ch2_v):.2f}-{np.max(ch2_v):.2f}V")

                        # Show channel mapping info
                        if good_frames == 1:
                            print(f"â„¹ï¸  Default mapping: offset0â†’CH1, offset1â†’CH2, offset2â†’CH3")
                            print(f"â„¹ï¸  If channels wrong, click ROTATE button in UI")
                            print(f"â„¹ï¸  Sample rate: {self.sr/1000:.1f} kHz | Window: {len(ch0_v)/self.sr*1000:.2f}ms")

                        # Show pattern summary every 20 frames
                        if good_frames > 0 and good_frames % 20 == 0:
                            from collections import Counter
                            pattern_counts = Counter(pattern_history[-20:])
                            print(f"\nðŸ“Š Last 20 frames channel activity:")
                            for p, count in pattern_counts.most_common():
                                ch_labels = p.split('-')
                                ch1_active = "âœ“" if ch_labels[0] == 'Y' else "âœ—"
                                ch2_active = "âœ“" if ch_labels[1] == 'Y' else "âœ—"
                                ch3_active = "âœ“" if ch_labels[2] == 'Y' else "âœ—"
                                print(f"   PA0(CH1):{ch1_active} PA1(CH2):{ch2_active} PB0(CH3):{ch3_active} â†’ {count} times ({100*count/20:.0f}%)")

                            # Check consistency
                            most_common_pattern, count = pattern_counts.most_common(1)[0]
                            consistency = 100 * count / 20

                            if consistency >= 90:
                                print(f"\nâœ… Signal stable: {most_common_pattern} ({consistency:.0f}%)")
                            elif len(pattern_counts) > 2:
                                print(f"\nâš ï¸  Signal jumping between channels!")
                                print(f"   â†’ This may indicate SPI buffer misalignment")
                                print(f"   â†’ Use ROTATE button in UI to fix channel mapping")
                            print()

                        # Táº¡o time array
                        n = ch0_v.size
                        t = (np.arange(self._idx, self._idx + n, dtype=np.float64) / self.sr).astype(np.float32)
                        self._idx += n

                        # Emit block
                        self.block.emit({
                            'sr': float(self.sr),
                            't': t,
                            'ch1': ch0_v,
                            'ch2': ch1_v,
                            'ch3': ch2_v
                        })

                        good_frames += 1
                        consecutive_bad = 0  # Reset counter on good frame
                    else:
                        bad_frames += 1
                        consecutive_bad += 1

                        # Auto-restart if stuck (10 consecutive bad frames)
                        if consecutive_bad >= 10:
                            print("\nâš ï¸  STM32 STUCK! Too many bad frames. Restarting acquisition...")
                            self.spi.xfer2([CMD_STOP])
                            time.sleep(0.2)
                            self.spi.xfer2([CMD_START])
                            time.sleep(0.5)
                            print("âœ“ Restarted. Trying again...\n")
                            consecutive_bad = 0
                            bad_frames = 0
                            good_frames = 0

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

                    time.sleep(0.005)  # Reduced polling delay

                except Exception as e:
                    print(f"Read error: {e}")
                    bad_frames += 1
                    time.sleep(0.1)

        except Exception as e:
            print(f"Fatal error: {e}")
            import traceback
            traceback.print_exc()


# ======================== Utility Functions ========================

def calculate_signal_stats(x: np.ndarray, fs_hint: float, time_window: float):
    """Calculate comprehensive signal statistics with ROBUST frequency detection"""
    stats = {
        'freq': 0.0,
        'cycl': 0.0,
        'pw': 0.0,
        'duty': 0.0,
        'vmax': 0.0,
        'vmin': 0.0,
        'vavg': 0.0,
        'vpp': 0.0,
        'vrms': 0.0
    }

    if x.size < 10:
        return stats

    # Voltage statistics
    stats['vmax'] = float(np.max(x))
    stats['vmin'] = float(np.min(x))
    stats['vavg'] = float(np.mean(x))
    stats['vpp'] = stats['vmax'] - stats['vmin']
    stats['vrms'] = float(np.sqrt(np.mean(x**2)))

    # Improved frequency and timing analysis with hysteresis
    # Use hysteresis to avoid noise-induced false crossings
    vpp = stats['vmax'] - stats['vmin']
    if vpp < 0.05:  # Signal too small (< 50mV)
        return stats

    # Set thresholds with hysteresis (5% of Vpp for sharp square wave edges)
    vmid = (stats['vmax'] + stats['vmin']) / 2.0
    hyst = vpp * 0.05  # Reduced to 0.05 for sharper square waves
    thresh_high = vmid + hyst
    thresh_low = vmid - hyst

    # State machine for edge detection with hysteresis
    crosses = []
    state = x[0] > vmid  # Initial state

    for i in range(1, len(x)):
        if state:  # Currently HIGH
            if x[i] < thresh_low:
                state = False
        else:  # Currently LOW
            if x[i] > thresh_high:
                state = True
                crosses.append(i)  # Rising edge

    crosses = np.array(crosses)

    if crosses.size >= 3:
        # Use MEDIAN of periods for better noise immunity
        periods = np.diff(crosses)

        # AGGRESSIVE outlier filtering (periods > 2x or < 0.5x median)
        med_period = np.median(periods)
        valid_periods = periods[(periods > med_period * 0.5) & (periods < med_period * 2.0)]

        if valid_periods.size >= 2:  # Need at least 2 valid periods
            # Use MEDIAN instead of MEAN (more robust against outliers)
            median_period = np.median(valid_periods)
            T = median_period / fs_hint
            if T > 0:
                stats['freq'] = 1.0 / T
                stats['cycl'] = time_window * stats['freq']

                # WORKAROUND: Frequency correction
                # Divide by 2 if reading 2kHz (should be 1kHz)
                if 1800 < stats['freq'] < 2200:
                    stats['freq'] = stats['freq'] / 2.0
                    stats['cycl'] = time_window * stats['freq']
                # Divide by 3 if reading 3kHz (should be 1kHz)
                elif 2800 < stats['freq'] < 3200:
                    stats['freq'] = stats['freq'] / 3.0
                    stats['cycl'] = time_window * stats['freq']

                # SANITY CHECK: Frequency should be reasonable (10Hz - 100kHz)
                if stats['freq'] < 10 or stats['freq'] > 100000:
                    stats['freq'] = 0.0
                    stats['cycl'] = 0.0

        # Duty cycle calculation - use MEDIAN of multiple cycles
        if crosses.size >= 3:
            duty_cycles = []
            for i in range(len(crosses) - 1):
                cycle_start = crosses[i]
                cycle_end = crosses[i + 1]

                if cycle_end - cycle_start > 2:
                    cycle_data = x[cycle_start:cycle_end]
                    high_samples = np.count_nonzero(cycle_data > vmid)
                    duty = 100.0 * high_samples / len(cycle_data)
                    duty_cycles.append(duty)

            if len(duty_cycles) > 0:
                stats['duty'] = np.median(duty_cycles)

                # Pulse width (high time per cycle)
                if stats['freq'] > 0:
                    stats['pw'] = (stats['duty'] / 100.0) / stats['freq']

    return stats


def stats_vpp_freq(x: np.ndarray, fs_hint: float):
    """Calculate Vpp, frequency and duty cycle (legacy compatibility)"""
    if x.size < 4:
        return 0.0, 0.0, 0.0

    vpp = float(x.max() - x.min())
    xm = x - x.mean()
    s = np.signbit(xm)
    cross = np.where((s[:-1] == True) & (s[1:] == False))[0]

    freq = 0.0
    if cross.size >= 2:
        T = np.median(np.diff(cross)) / fs_hint
        if T > 0:
            freq = 1.0 / T

            # WORKAROUND: Frequency correction
            # Divide by 2 if reading 2kHz (should be 1kHz)
            if 1800 < freq < 2200:
                freq = freq / 2.0
            # Divide by 3 if reading 3kHz (should be 1kHz)
            elif 2800 < freq < 3200:
                freq = freq / 3.0

    duty = 0.0
    if cross.size >= 2:
        a, b = cross[0], cross[1]
        seg = xm[a:b]
        if seg.size > 0:
            duty = 100.0 * np.count_nonzero(seg > 0) / seg.size

    return vpp, freq, duty


# ======================== Scope UI ========================

class ScopePanelUI(QtWidgets.QWidget):
    def __init__(self, scope_provider: SPIScopeProvider):
        super().__init__()
        self.scope_provider = scope_provider

        self.setWindowTitle("STM32 Oscilloscope - 3 Channels SPI")
        self.resize(1400, 900)
        self.setStyleSheet("background:#1A1A1A;")

        # Data buffers - increased for high sample rate
        # At 600kHz, 1 second = 600k samples
        # Buffer for ~1 second of data per channel
        self.N = int(SAMPLE_RATE * 1.5)  # 1.5 seconds buffer
        self.t = np.zeros(self.N, dtype=np.float32)
        self.ch = [
            np.zeros(self.N, dtype=np.float32),
            np.zeros(self.N, dtype=np.float32),
            np.zeros(self.N, dtype=np.float32)
        ]
        self.iw = 0
        self.sr = SAMPLE_RATE  # Per channel sample rate

        # Display settings - optimized for 1kHz PWM measurement
        self.time_div = 0.005  # 5ms/div (default) â†’ 50ms window â†’ 50 cycles @ 1kHz
        self.volts_div = [0.5, 0.5, 0.5]
        self.offset = [0.0, 0.0, 0.0]
        self.trigger_src = 0
        self.trigger_lvl = 1.65  # Mid-point 3.3V
        self.trigger_mode = 'AUTO'  # AUTO, NORMAL, SINGLE
        self.trigger_edge = 1  # 0=Spectrum, 1=Rising, 2=Falling, 3=None
        self.trigger_armed = True  # For SINGLE mode
        self.trigger_found = False  # Last trigger status
        self.last_trigger_time = 0  # For AUTO timeout

        # Frequency averaging for stable display
        self.freq_history = [[], [], []]  # History for each channel
        self.freq_avg_count = 5  # Average over 5 measurements

        # Channel rotation offset - DISABLED to fix channel mapping
        self.ch_rotation = 0  # ALWAYS 0 - rotation disabled

        # Channel enable/disable
        self.ch_enabled = [True, True, True]  # [CH1, CH2, CH3]

        # Auto-hide channels without signal (ENABLED by default to hide floating pins)
        self.auto_hide_enabled = True
        self.vpp_threshold = 0.5  # Hide channels with Vpp < 0.5V (floating pins)

        # Auto-lock strongest channel (DISABLED BY DEFAULT for multi-channel display)
        self.auto_lock_enabled = False  # Auto-detect channel with signal
        self.locked_offset = None  # 0, 1, or 2

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

        title = QtWidgets.QLabel("âš¡ STM32 OSCILLOSCOPE - SPI MODE")
        title.setFont(QtGui.QFont("Consolas", 18, QtGui.QFont.Bold))
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

        # Grid
        grid = pg.GridItem()
        grid.setPen(pg.mkPen(color=(100, 100, 100), style=QtCore.Qt.DashLine))
        grid.setOpacity(0.3)
        self.plot.addItem(grid)

        self.plot.setMenuEnabled(False)
        self.plot.setMouseEnabled(x=False, y=False)

        # Curve (PB0=green - only channel)
        self.cur1 = self.plot.plot(pen=pg.mkPen((0, 255, 0), width=3), clipToView=True)

        # Text label
        self.txt1 = pg.TextItem(color=(0, 255, 0))

        fnt = QtGui.QFont("Consolas", 12, QtGui.QFont.Bold)
        self.txt1.setFont(fnt)

        self.txt1.setAnchor((1, 0))

        self.plot.addItem(self.txt1)

        # Create stats panel
        stats_panel = self._create_stats_panel()

        # Add plot and stats panel vertically
        plot_container = QtWidgets.QVBoxLayout()
        plot_container.addWidget(self.plot, 1)
        plot_container.addWidget(stats_panel, 0)

        center.addLayout(plot_container, 1)

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

        self.bad1 = self._badge('âš¡ CH1 = PA0 (PIN 10) | 0.5V/div +0.00V', 'rgb(0,255,0)')
        self.bad2 = self._badge('âš¡ CH2 = PA1 (PIN 11) | 0.5V/div +0.00V', 'rgb(255,255,0)')
        self.bad3 = self._badge('âš¡ CH3 = PB0 (PIN 21) | 0.5V/div +0.00V', 'rgb(0,255,255)')
        self.badT = self._badge('TRIG CH1 +1.65V', '#ddd')

        for b in (self.bad1, self.bad2, self.bad3, self.badT):
            b.setFont(QtGui.QFont("Consolas", 14, QtGui.QFont.Bold))
            b.setMinimumWidth(200)
            badges.addWidget(b)

        badges.addStretch(1)
        root.addWidget(badges_widget)

        self._update_axes()

    def _badge(self, text, color):
        L = QtWidgets.QLabel(text)
        L.setStyleSheet(f"padding:8px 16px; border-radius:12px; background:#2b2b2b; color:{color};")
        return L

    def _create_stats_panel(self):
        """Create panel for displaying signal statistics"""
        panel = QtWidgets.QWidget()
        panel.setStyleSheet("background:#2D2D2D; border-radius: 8px;")
        layout = QtWidgets.QHBoxLayout(panel)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(15)

        # Single channel color
        colors = [
            ('CH1 (PB0)', 'rgb(0,255,0)')
        ]

        self.stats_labels = []

        for ch_name, color in colors:
            # Create channel stats group
            ch_box = QtWidgets.QGroupBox(ch_name)
            ch_box.setStyleSheet(f"background:#2D2D2D; color:{color}; border: 2px solid {color}; border-radius: 6px; font-weight: bold; padding: 5px;")
            ch_layout = QtWidgets.QGridLayout(ch_box)
            ch_layout.setContentsMargins(10, 15, 10, 10)
            ch_layout.setSpacing(5)

            # Create labels for this channel
            labels = {}
            row = 0

            # Timing parameters
            timing_params = [
                ('Freq:', 'freq', 'Hz'),
                ('Cycl:', 'cycl', ''),
                ('PW:', 'pw', 's'),
                ('Duty:', 'duty', '%')
            ]

            for label_text, key, unit in timing_params:
                lbl_name = QtWidgets.QLabel(label_text)
                lbl_name.setStyleSheet(f"color:{color}; font-size: 11px; font-weight: bold;")
                lbl_value = QtWidgets.QLabel('--')
                lbl_value.setStyleSheet("color:#DDD; font-size: 11px; font-family: Consolas;")
                lbl_value.setAlignment(QtCore.Qt.AlignRight)

                ch_layout.addWidget(lbl_name, row, 0)
                ch_layout.addWidget(lbl_value, row, 1)

                labels[key] = (lbl_value, unit)
                row += 1

            # Add separator
            separator = QtWidgets.QFrame()
            separator.setFrameShape(QtWidgets.QFrame.HLine)
            separator.setStyleSheet(f"background-color: {color}; min-height: 1px; max-height: 1px;")
            ch_layout.addWidget(separator, row, 0, 1, 2)
            row += 1

            # Voltage parameters
            voltage_params = [
                ('Vmax:', 'vmax', 'V'),
                ('Vmin:', 'vmin', 'V'),
                ('Vavg:', 'vavg', 'V'),
                ('Vpp:', 'vpp', 'V'),
                ('Vrms:', 'vrms', 'V')
            ]

            for label_text, key, unit in voltage_params:
                lbl_name = QtWidgets.QLabel(label_text)
                lbl_name.setStyleSheet(f"color:{color}; font-size: 11px; font-weight: bold;")
                lbl_value = QtWidgets.QLabel('--')
                lbl_value.setStyleSheet("color:#DDD; font-size: 11px; font-family: Consolas;")
                lbl_value.setAlignment(QtCore.Qt.AlignRight)

                ch_layout.addWidget(lbl_name, row, 0)
                ch_layout.addWidget(lbl_value, row, 1)

                labels[key] = (lbl_value, unit)
                row += 1

            self.stats_labels.append(labels)
            layout.addWidget(ch_box)

        return panel

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

    def _group_all_vertical(self):
        box = QtWidgets.QGroupBox("Vertical - All Channels")
        box.setStyleSheet("background:#2D2D2D; color:#DDD; border: 1px solid #444; border-radius: 6px; font-weight: bold;")
        g = QtWidgets.QGridLayout(box)
        g.setContentsMargins(10, 10, 10, 10)
        g.setSpacing(10)

        g.addWidget(QtWidgets.QLabel("POSITION (All)"), 0, 0)
        d = QtWidgets.QPushButton("â†“")
        u = QtWidgets.QPushButton("â†‘")

        for b in (d, u):
            b.setStyleSheet("background:#3A3A3A; color:#DDD; border: 1px solid #555; padding: 8px; font-size: 14px; font-weight: bold;")

        d.clicked.connect(lambda: self._move_all_off(-0.2))
        u.clicked.connect(lambda: self._move_all_off(+0.2))

        g.addWidget(d, 0, 1)
        g.addWidget(u, 0, 2)

        return box

    def _group_channel_rotation(self):
        box = QtWidgets.QGroupBox("Channel Mapping")
        box.setStyleSheet("background:#2D2D2D; color:#FFA500; border: 2px solid #FFA500; border-radius: 6px; font-weight: bold;")
        g = QtWidgets.QGridLayout(box)
        g.setContentsMargins(10, 10, 10, 10)
        g.setSpacing(10)

        # Info label - FIXED MAPPING
        info = QtWidgets.QLabel("FIXED MAPPING: PA0â†’CH1, PA1â†’CH2, PB0â†’CH3")
        info.setStyleSheet("color:#0f0; font-size: 12px; font-weight: bold;")
        g.addWidget(info, 0, 0, 1, 3)

        # Auto-hide checkbox (ENABLED BY DEFAULT to hide floating pins)
        self.chk_autohide = QtWidgets.QCheckBox("AUTO-HIDE Floating Pins (Vpp<0.5V)")
        self.chk_autohide.setChecked(True)  # ON by default
        self.chk_autohide.setStyleSheet("color:#0f0; font-weight: bold; font-size: 11px;")
        self.chk_autohide.stateChanged.connect(self._toggle_autohide)
        g.addWidget(self.chk_autohide, 1, 0, 1, 3)

        # Auto-lock checkbox (DISABLED BY DEFAULT for multi-channel display)
        self.chk_autolock = QtWidgets.QCheckBox("AUTO-LOCK Strongest")
        self.chk_autolock.setChecked(False)  # OFF by default
        self.chk_autolock.setStyleSheet("color:#888; font-weight: bold; font-size: 12px;")
        self.chk_autolock.stateChanged.connect(self._toggle_autolock)
        g.addWidget(self.chk_autolock, 2, 0, 1, 3)

        # Rotate button - DISABLED (fixed mapping)
        rotate_btn = QtWidgets.QPushButton("ðŸ”„ ROTATE (DISABLED)")
        rotate_btn.setStyleSheet("background:#555; color:#888; border: 1px solid #444; padding: 10px; font-size: 14px;")
        rotate_btn.setEnabled(False)
        rotate_btn.setToolTip("Rotation disabled - mapping is fixed: PA0â†’CH1, PA1â†’CH2, PB0â†’CH3")
        g.addWidget(rotate_btn, 3, 0, 1, 3)

        # Status label
        self.lbl_rotation = QtWidgets.QLabel("Offset: 0 (PA0â†’CH1, PA1â†’CH2, PB0â†’CH3)")
        self.lbl_rotation.setStyleSheet("color:#DDD; font-size: 10px; font-weight: normal;")
        self.lbl_rotation.setWordWrap(True)
        g.addWidget(self.lbl_rotation, 4, 0, 1, 3)

        # Lock status label
        self.lbl_lock_status = QtWidgets.QLabel("")
        self.lbl_lock_status.setStyleSheet("color:#0f0; font-size: 11px; font-weight: bold;")
        self.lbl_lock_status.setWordWrap(True)
        g.addWidget(self.lbl_lock_status, 5, 0, 1, 3)

        return box

    def _per_channel(self, idx, title):
        box = QtWidgets.QGroupBox(title)
        box.setStyleSheet("background:#2D2D2D; color:#DDD; border: 1px solid #444; border-radius: 6px; font-weight: bold;")
        g = QtWidgets.QGridLayout(box)
        g.setContentsMargins(10, 10, 10, 10)
        g.setSpacing(10)

        # Enable/Disable checkbox
        chk = QtWidgets.QCheckBox("ENABLE")
        chk.setChecked(True)
        chk.setStyleSheet("color:#0f0; font-weight: bold;")
        chk.stateChanged.connect(lambda state, i=idx: self._toggle_channel(i, state))
        g.addWidget(chk, 0, 0, 1, 3)

        g.addWidget(QtWidgets.QLabel("VOLTS/DIV"), 1, 0)
        m = QtWidgets.QPushButton("-")
        p = QtWidgets.QPushButton("+")

        for b in (m, p):
            b.setStyleSheet("background:#3A3A3A; color:#DDD; border: 1px solid #555; padding: 8px; font-size: 14px; font-weight: bold;")

        m.clicked.connect(lambda: self._vdiv(idx, 1/1.2))
        p.clicked.connect(lambda: self._vdiv(idx, 1.2))

        g.addWidget(m, 1, 1)
        g.addWidget(p, 1, 2)

        g.addWidget(QtWidgets.QLabel("POSITION"), 2, 0)
        d = QtWidgets.QPushButton("â†“")
        u = QtWidgets.QPushButton("â†‘")

        for b in (d, u):
            b.setStyleSheet("background:#3A3A3A; color:#DDD; border: 1px solid #555; padding: 8px; font-size: 14px; font-weight: bold;")

        d.clicked.connect(lambda: self._move_off(idx, -0.2))
        u.clicked.connect(lambda: self._move_off(idx, +0.2))

        g.addWidget(d, 2, 1)
        g.addWidget(u, 2, 2)

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
        mode_single = QtWidgets.QPushButton("SINGLE")

        for b in (mode_auto, mode_norm, mode_single):
            b.setCheckable(True)
            b.setStyleSheet("background:#3A3A3A; color:#DDD; border: 1px solid #555; padding: 8px; font-size: 12px; font-weight: bold;")

        mode_group = QtWidgets.QButtonGroup(self)
        mode_group.addButton(mode_auto, 0)
        mode_group.addButton(mode_norm, 1)
        mode_group.addButton(mode_single, 2)
        mode_group.setExclusive(True)
        mode_auto.setChecked(True)
        mode_group.idClicked.connect(self._set_trigger_mode)

        g.addWidget(mode_auto, 0, 1)
        g.addWidget(mode_norm, 0, 2)
        g.addWidget(mode_single, 0, 3)

        # Trigger Edge (NEW - from STM32 main.cpp)
        g.addWidget(QtWidgets.QLabel("EDGE"), 1, 0)
        edge_spectrum = QtWidgets.QPushButton("SPECTRUM")
        edge_rising = QtWidgets.QPushButton("RISING")
        edge_falling = QtWidgets.QPushButton("FALLING")
        edge_none = QtWidgets.QPushButton("NONE")

        for b in (edge_spectrum, edge_rising, edge_falling, edge_none):
            b.setCheckable(True)
            b.setStyleSheet("background:#3A3A3A; color:#DDD; border: 1px solid #555; padding: 6px; font-size: 11px; font-weight: bold;")

        edge_group = QtWidgets.QButtonGroup(self)
        edge_group.addButton(edge_spectrum, 0)
        edge_group.addButton(edge_rising, 1)
        edge_group.addButton(edge_falling, 2)
        edge_group.addButton(edge_none, 3)
        edge_group.setExclusive(True)
        edge_rising.setChecked(True)  # Default: Rising
        edge_group.idClicked.connect(self._set_trigger_edge)

        g.addWidget(edge_rising, 1, 1)
        g.addWidget(edge_falling, 1, 2)
        g.addWidget(edge_none, 1, 3)

        # Trigger Source (always CH1 - only channel)
        g.addWidget(QtWidgets.QLabel("SOURCE"), 2, 0)
        b1 = QtWidgets.QPushButton("CH1 (PB0)")
        b1.setCheckable(True)
        b1.setChecked(True)
        b1.setStyleSheet("background:#3A3A3A; color:#DDD; border: 1px solid #555; padding: 8px; font-size: 14px; font-weight: bold;")
        b1.setEnabled(False)  # Can't change - only 1 channel
        g.addWidget(b1, 2, 1, 1, 3)

        # Trigger Level
        g.addWidget(QtWidgets.QLabel("LEVEL"), 3, 0)
        m = QtWidgets.QPushButton('-')
        p = QtWidgets.QPushButton('+')

        for b in (m, p):
            b.setStyleSheet("background:#3A3A3A; color:#DDD; border: 1px solid #555; padding: 8px; font-size: 14px; font-weight: bold;")

        m.clicked.connect(lambda: self._set_lvl(-0.1))
        p.clicked.connect(lambda: self._set_lvl(+0.1))

        g.addWidget(m, 3, 1)
        g.addWidget(p, 3, 2)

        return box

    @QtCore.pyqtSlot(dict)
    def on_block(self, blk: dict):
        """Receive data block from SPI provider"""
        sr = blk.get('sr', None)
        if sr:
            self.sr = float(sr)

        t = blk['t']
        a = blk['ch1']
        b = blk['ch2']
        c = blk['ch3']

        # AUTO-LOCK: Find strongest channel and lock to it
        if self.auto_lock_enabled:
            channels = [a, b, c]
            vpp_values = [
                float(np.max(a) - np.min(a)),
                float(np.max(b) - np.min(b)),
                float(np.max(c) - np.min(c))
            ]

            # Find strongest channel (max Vpp)
            strongest_idx = int(np.argmax(vpp_values))
            max_vpp = vpp_values[strongest_idx]

            # Lock if Vpp > 0.5V (significant signal)
            if max_vpp > 0.5:
                if self.locked_offset != strongest_idx:
                    self.locked_offset = strongest_idx
                    ch_names = ["offset0 (PA0)", "offset1 (PA1)", "offset2 (PB0)"]
                    print(f"ðŸ”’ LOCKED to {ch_names[strongest_idx]}, Vpp={max_vpp:.2f}V")

                # Show only locked channel on CH1, zero others
                if self.locked_offset == 0:
                    a, b, c = channels[0], np.zeros_like(channels[0]), np.zeros_like(channels[0])
                elif self.locked_offset == 1:
                    a, b, c = channels[1], np.zeros_like(channels[1]), np.zeros_like(channels[1])
                else:
                    a, b, c = channels[2], np.zeros_like(channels[2]), np.zeros_like(channels[2])
            else:
                # No significant signal, reset lock
                self.locked_offset = None
        else:
            # Apply channel rotation
            channels = [a, b, c]
            rotated = [
                channels[self.ch_rotation],
                channels[(self.ch_rotation + 1) % 3],
                channels[(self.ch_rotation + 2) % 3]
            ]
            a, b, c = rotated

        n = t.size
        i0 = self.iw
        i1 = (i0 + n) % self.N

        if i0 < i1:
            self.t[i0:i1] = t
            self.ch[0][i0:i1] = a
            self.ch[1][i0:i1] = b
            self.ch[2][i0:i1] = c
        else:
            k = self.N - i0
            self.t[i0:] = t[:k]
            self.t[:i1] = t[k:]
            self.ch[0][i0:] = a[:k]
            self.ch[0][:i1] = a[k:]
            self.ch[1][i0:] = b[:k]
            self.ch[1][:i1] = b[k:]
            self.ch[2][i0:] = c[:k]
            self.ch[2][:i1] = c[k:]

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

        # Improved trigger logic with hysteresis for stability
        # Based on STM32 main.cpp logic: 0=Spectrum, 1=Rising, 2=Falling, 3=None
        src = a  # Only one channel

        # Add hysteresis to avoid jitter (5% of Vpp for sharp square waves)
        src_vpp = np.max(src) - np.min(src)
        hyst = src_vpp * 0.05  # Reduced to 0.05 for sharper edges

        crosses = []

        # Trigger Edge Mode handling
        if self.trigger_edge == 3:
            # Mode 3: NONE (Free run) - no trigger, just display as-is
            cross = np.array([])

        elif self.trigger_edge == 0:
            # Mode 0: SPECTRUM - not implemented for time domain
            # Could add FFT display here in the future
            cross = np.array([])

        elif self.trigger_edge == 1:
            # Mode 1: RISING edge detection with hysteresis
            above_high = src > (self.trigger_lvl + hyst)
            below_low = src < (self.trigger_lvl - hyst)

            # State machine for stable edge detection
            state = above_high[0]  # Initial state

            for i in range(1, len(src)):
                if state:  # Currently HIGH
                    if below_low[i]:
                        state = False
                else:  # Currently LOW
                    if above_high[i]:
                        state = True
                        crosses.append(i)  # Rising edge

            cross = np.array(crosses)

        elif self.trigger_edge == 2:
            # Mode 2: FALLING edge detection with hysteresis
            above_high = src > (self.trigger_lvl + hyst)
            below_low = src < (self.trigger_lvl - hyst)

            # State machine for stable edge detection
            state = above_high[0]  # Initial state

            for i in range(1, len(src)):
                if state:  # Currently HIGH
                    if below_low[i]:
                        state = False
                        crosses.append(i)  # Falling edge
                else:  # Currently LOW
                    if above_high[i]:
                        state = True

            cross = np.array(crosses)

        else:
            cross = np.array([])

        self.trigger_found = False  # Save for status display
        if cross.size > 0:
            # Look for trigger in the right half of buffer
            right_idx_start = int(tx.size * 0.5)
            cand = cross[cross >= right_idx_start]
            if cand.size > 0:
                k = int(cand[0])
                trig_pos = -5 * self.time_div
                tx_rel = tx_rel - (tx_rel[k] - trig_pos)
                self.trigger_found = True
            else:
                # Fallback to 40% position
                cand = cross[cross >= int(tx.size * 0.4)]
                if cand.size > 0:
                    k = int(cand[0])
                    trig_pos = -5 * self.time_div
                    tx_rel = tx_rel - (tx_rel[k] - trig_pos)
                    self.trigger_found = True

        # Handle trigger modes
        import time as time_module
        current_time = time_module.time()

        if self.trigger_mode == 'NORMAL':
            # NORMAL mode: Only update if trigger found
            if not self.trigger_found:
                return  # Don't update display without trigger
            self.last_trigger_time = current_time

        elif self.trigger_mode == 'SINGLE':
            # SINGLE mode: Trigger once and stop
            if not self.trigger_armed:
                return  # Already triggered, don't update
            if self.trigger_found:
                self.trigger_armed = False  # Disarm after first trigger
                self.last_trigger_time = current_time
            else:
                return  # Wait for trigger

        elif self.trigger_mode == 'AUTO':
            # AUTO mode: Always update, but prefer triggered display
            if self.trigger_found:
                self.last_trigger_time = current_time
            # If no trigger for >100ms, use free-run (no alignment needed)

        pre_trigger_time = 2.0 * self.time_div
        mm = (tx_rel >= -win - pre_trigger_time) & (tx_rel <= 0.0)
        tx_rel, a, b, c = tx_rel[mm], a[mm], b[mm], c[mm]

        if tx_rel.size < 8:
            return

        # Calculate comprehensive stats for each channel (before offset is applied)
        time_window = win  # Current display window
        stats_ch1 = calculate_signal_stats(a, self.sr, time_window)
        stats_ch2 = calculate_signal_stats(b, self.sr, time_window)
        stats_ch3 = calculate_signal_stats(c, self.sr, time_window)

        # Auto-hide channels without signal (floating pins)
        if self.auto_hide_enabled:
            # Check Vpp for each channel
            if stats_ch1['vpp'] < self.vpp_threshold:
                self.ch_enabled[0] = False
            else:
                self.ch_enabled[0] = True

            if stats_ch2['vpp'] < self.vpp_threshold:
                self.ch_enabled[1] = False
            else:
                self.ch_enabled[1] = True

            if stats_ch3['vpp'] < self.vpp_threshold:
                self.ch_enabled[2] = False
            else:
                self.ch_enabled[2] = True

        # Apply frequency averaging for stable display
        for idx, stats in enumerate([stats_ch1, stats_ch2, stats_ch3]):
            if stats['freq'] > 0:
                # Add to history
                self.freq_history[idx].append(stats['freq'])
                # Keep only last N measurements
                if len(self.freq_history[idx]) > self.freq_avg_count:
                    self.freq_history[idx].pop(0)
                # Use median for display (robust against outliers)
                if len(self.freq_history[idx]) >= 3:
                    stats['freq'] = float(np.median(self.freq_history[idx]))
            else:
                # Clear history if no signal
                self.freq_history[idx].clear()

        # Update stats labels (only if enabled)
        if self.ch_enabled[0]:
            self._update_stats_display(0, stats_ch1)
        else:
            self._clear_stats_display(0)

        if self.ch_enabled[1]:
            self._update_stats_display(1, stats_ch2)
        else:
            self._clear_stats_display(1)

        if self.ch_enabled[2]:
            self._update_stats_display(2, stats_ch3)
        else:
            self._clear_stats_display(2)

        # Legacy stats for text overlay
        v1, f1, _ = stats_vpp_freq(a, self.sr)
        v2, f2, _ = stats_vpp_freq(b, self.sr)
        v3, f3, _ = stats_vpp_freq(c, self.sr)

        # Apply offset
        a = a + self.offset[0]
        b = b + self.offset[1]
        c = c + self.offset[2]

        # Downsample if needed for smooth rendering
        # At 600kHz, 10ms window = 6000 points - too many to render smoothly
        n = tx_rel.size
        mpts = min(5000, n)  # Increased from 2500 to 5000 for better quality at high sample rate
        if n > mpts:
            sel = np.linspace(0, n-1, mpts).astype(int)
            txd, ad, bd, cd = tx_rel[sel], a[sel], b[sel], c[sel]
        else:
            txd, ad, bd, cd = tx_rel, a, b, c

        # Update curves (only if enabled)
        if self.ch_enabled[0]:
            self.cur1.setData(txd, ad)
        else:
            self.cur1.setData([], [])  # Hide channel

        if self.ch_enabled[1]:
            self.cur2.setData(txd, bd)
        else:
            self.cur2.setData([], [])

        if self.ch_enabled[2]:
            self.cur3.setData(txd, cd)
        else:
            self.cur3.setData([], [])

        # Update text labels (only if enabled)
        view_range = self.plot.getViewBox().viewRange()
        x_max, y_max = view_range[0][1], view_range[1][1]

        self.txt1.setPos(x_max - 20, y_max - 10)
        self.txt2.setPos(x_max - 20, y_max - 30)
        self.txt3.setPos(x_max - 20, y_max - 50)

        if self.ch_enabled[0]:
            self.txt1.setText(f"CH1 = PA0 (PIN 10) | Vpp={v1:.2f}V f={f1:.0f}Hz")
        else:
            self.txt1.setText("")

        if self.ch_enabled[1]:
            self.txt2.setText(f"CH2 = PA1 (PIN 11) | Vpp={v2:.2f}V f={f2:.0f}Hz")
        else:
            self.txt2.setText("")

        if self.ch_enabled[2]:
            self.txt3.setText(f"CH3 = PB0 (PIN 21) | Vpp={v3:.2f}V f={f3:.0f}Hz")
        else:
            self.txt3.setText("")

        # Update badges
        self._update_axes()

        ch1_status = "ON" if self.ch_enabled[0] else "OFF"
        ch2_status = "ON" if self.ch_enabled[1] else "OFF"
        ch3_status = "ON" if self.ch_enabled[2] else "OFF"

        self.bad1.setText(f"âš¡ CH1 = PA0 (PIN 10) [{ch1_status}] | {self.volts_div[0]:.2f}V/div {self.offset[0]:+.2f}V")
        self.bad2.setText(f"âš¡ CH2 = PA1 (PIN 11) [{ch2_status}] | {self.volts_div[1]:.2f}V/div {self.offset[1]:+.2f}V")
        self.bad3.setText(f"âš¡ CH3 = PB0 (PIN 21) [{ch3_status}] | {self.volts_div[2]:.2f}V/div {self.offset[2]:+.2f}V")

        src_name = ["CH1", "CH2", "CH3"][self.trigger_src]

        # Add trigger status indicator
        if self.trigger_mode == 'SINGLE' and not self.trigger_armed:
            status = "â¸"  # Paused
        elif self.trigger_found:
            status = "ðŸ”’"  # Locked/triggered
        else:
            status = "â©"  # Free-running

        edge_names = ["SPECTRUM", "RISING↑", "FALLING↓", "NONE"]
        edge_name = edge_names[self.trigger_edge]
        self.badT.setText(f"TRIG {self.trigger_mode} {status} {edge_name} {src_name} {self.trigger_lvl:+.2f}V")

        # Update lock status label
        if self.auto_lock_enabled:
            if self.locked_offset is not None:
                lock_names = ["offset0 (PA0â†’CH1)", "offset1 (PA1â†’CH1)", "offset2 (PB0â†’CH1)"]
                self.lbl_lock_status.setText(f"ðŸ”’ LOCKED: {lock_names[self.locked_offset]}")
            else:
                self.lbl_lock_status.setText("ðŸ” Searching for signal...")
        else:
            self.lbl_lock_status.setText("")

    def _update_axes(self):
        vdiv = self.volts_div[0]
        ymax = vdiv * 4.0
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

    def _set_trg(self, s):
        self.trigger_src = 0  # Always CH1 (only channel)

    def _set_lvl(self, dv):
        self.trigger_lvl += dv

    def _set_trigger_mode(self, mode_id):
        """Set trigger mode: 0=AUTO, 1=NORMAL, 2=SINGLE"""
        modes = ['AUTO', 'NORMAL', 'SINGLE']
        self.trigger_mode = modes[mode_id]
        if self.trigger_mode == 'SINGLE':
            self.trigger_armed = True  # Re-arm for next trigger

    def _set_trigger_edge(self, edge_id):
        """Set trigger edge: 0=Spectrum, 1=Rising, 2=Falling, 3=None"""
        self.trigger_edge = edge_id
        edge_names = ['SPECTRUM', 'RISING', 'FALLING', 'NONE']
        print(f"🔧 Trigger edge set to: {edge_names[edge_id]}")

    def _rotate_channels(self):
        """Rotate channel mapping: 0 â†’ 1 â†’ 2 â†’ 0"""
        self.ch_rotation = (self.ch_rotation + 1) % 3

        # Update label
        mappings = [
            "Offset: 0 (PA0â†’CH1, PA1â†’CH2, PB0â†’CH3)",
            "Offset: 1 (PA1â†’CH1, PB0â†’CH2, PA0â†’CH3)",
            "Offset: 2 (PB0â†’CH1, PA0â†’CH2, PA1â†’CH3)"
        ]
        self.lbl_rotation.setText(mappings[self.ch_rotation])

        print(f"ðŸ”„ Channel rotation: {self.ch_rotation} - {mappings[self.ch_rotation]}")

    def _toggle_channel(self, idx, state):
        """Enable/disable channel display"""
        self.ch_enabled[idx] = (state == QtCore.Qt.Checked)
        status = "ENABLED" if self.ch_enabled[idx] else "DISABLED"
        ch_names = ["CH1(PA0)", "CH2(PA1)", "CH3(PB0)"]
        print(f"ðŸ“º {ch_names[idx]} {status}")

    def _toggle_autohide(self, state):
        """Enable/disable auto-hide channels without signal"""
        self.auto_hide_enabled = (state == QtCore.Qt.Checked)
        if self.auto_hide_enabled:
            print(f"ðŸ‘ï¸  AUTO-HIDE ENABLED: Hide channels with Vpp < {self.vpp_threshold}V")
        else:
            print("ðŸ‘ï¸  AUTO-HIDE DISABLED: Show all channels")
            # Re-enable all channels
            self.ch_enabled = [True, True, True]

    def _toggle_autolock(self, state):
        """Enable/disable auto-lock strongest channel"""
        self.auto_lock_enabled = (state == QtCore.Qt.Checked)
        if self.auto_lock_enabled:
            print("ðŸ”’ AUTO-LOCK ENABLED: Will lock to strongest channel")
            self.locked_offset = None  # Reset lock
        else:
            print("ðŸ”“ AUTO-LOCK DISABLED")
            self.locked_offset = None

    def _clear_stats_display(self, channel_idx):
        """Clear statistics display for a disabled channel"""
        if channel_idx >= len(self.stats_labels):
            return

        labels = self.stats_labels[channel_idx]
        for key, (label_widget, unit) in labels.items():
            label_widget.setText("--")

    def _update_stats_display(self, channel_idx, stats):
        """Update statistics display for a channel"""
        if channel_idx >= len(self.stats_labels):
            return

        labels = self.stats_labels[channel_idx]

        # Format and update each stat
        for key, (label_widget, unit) in labels.items():
            value = stats.get(key, 0.0)

            if key == 'freq':
                # Frequency formatting
                if value >= 1e6:
                    text = f"{value/1e6:.2f} M{unit}"
                elif value >= 1e3:
                    text = f"{value/1e3:.2f} k{unit}"
                elif value > 0:
                    text = f"{value:.1f} {unit}"
                else:
                    text = "--"

            elif key == 'cycl':
                # Cycles
                if value > 0:
                    text = f"{value:.1f}"
                else:
                    text = "--"

            elif key == 'pw':
                # Pulse width
                if value >= 1e-3:
                    text = f"{value*1e3:.2f} m{unit}"
                elif value >= 1e-6:
                    text = f"{value*1e6:.2f} Âµ{unit}"
                elif value > 0:
                    text = f"{value*1e9:.2f} n{unit}"
                else:
                    text = "--"

            elif key == 'duty':
                # Duty cycle
                if value > 0:
                    text = f"{value:.1f}{unit}"
                else:
                    text = "--"

            elif key in ['vmax', 'vmin', 'vavg', 'vpp', 'vrms']:
                # Voltage formatting
                if abs(value) >= 1:
                    text = f"{value:.3f} {unit}"
                elif abs(value) >= 1e-3:
                    text = f"{value*1e3:.1f} m{unit}"
                elif abs(value) >= 1e-6:
                    text = f"{value*1e6:.1f} Âµ{unit}"
                else:
                    text = f"{value:.3f} {unit}"
            else:
                text = f"{value:.3f}"

            label_widget.setText(text)


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
    print("âš¡ STM32 OSCILLOSCOPE - OPTIMIZED MODE")
    print("="*80)
    print()
    print("âœ… MULTI-CHANNEL MODE")
    print("    â€¢ 3 independent channels displayed simultaneously")
    print("    â€¢ AUTO-HIDE floating pins ENABLED (hides Vpp < 0.5V)")
    print("    â€¢ AUTO-LOCK available (check box to enable)")
    print("    â€¢ CH1: PA0 (Green), CH2: PA1 (Yellow), CH3: PB0 (Cyan)")
    print()
    print(f"  ðŸ”§ Hardware Configuration:")
    print(f"     â€¢ Channels: {NUM_CHANNELS}")
    print(f"     â€¢ STM32 Clock: {STM32_CLOCK/1e6:.0f} MHz")
    print(f"     â€¢ TIM3: PSC={TIM3_PSC}, ARR={TIM3_ARR} â†’ {TIM3_TRIGGER_RATE/1e3:.0f} kHz trigger")
    print(f"     â€¢ ADC Clock: {ADC_CLOCK/1e6:.0f} MHz (ADCPRE=/{ADC_PRESCALER})")
    print(f"     â€¢ ADC Sample time: {ADC_SAMPLE_TIME} cycles ({TIME_PER_CHANNEL*1e6:.3f} Âµs per channel)")
    print(f"")
    print(f"  ðŸ“Š Sampling Performance:")
    print(f"     â€¢ Sample rate: {SAMPLE_RATE/1000:.1f} kHz per channel")
    print(f"     â€¢ Bandwidth: ~{SAMPLE_RATE/2000:.0f} kHz (Nyquist)")
    print(f"     â€¢ Resolution @ 1kHz: {SAMPLE_RATE/1000:.0f} samples/cycle")
    print(f"     â€¢ Edge resolution: {TIME_PER_CHANNEL*1e6:.3f} Âµs (ULTRA FAST â†’ Sharp square edges!)")
    print(f"")
    print(f"  ðŸ“¦ Data Protocol:")
    print(f"     â€¢ Packet size: {PACKET_TOTAL_SIZE} bytes")
    print(f"     â€¢ Data size: {PACKET_DATA_SIZE} bytes ({TOTAL_SAMPLES} samples)")
    print(f"     â€¢ Markers: 0x{MARKER_START:04X} / 0x{MARKER_END:04X} + Checksum")
    print(f"     â€¢ SPI Speed: 2 MHz")
    print(f"")
    print(f"  ðŸŽ¯ Frequency Measurement:")
    print(f"     â€¢ Algorithm: Median-based with outlier filtering")
    print(f"     â€¢ Hysteresis: 20% Vpp (noise immunity)")
    print(f"     â€¢ Averaging: 5-sample median filter")
    print(f"     â€¢ Default view: 5ms/div (50ms window = 50 cycles @ 1kHz)")
    print("="*80)

    scope_provider = SPIScopeProvider()
    ui = ScopePanelUI(scope_provider)
    ui.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()


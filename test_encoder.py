#!/usr/bin/env python3
"""
Test EC11 Rotary Encoder với RPi.GPIO
Sử dụng để kiểm tra encoder hardware trước khi chạy oscilloscope

Chạy:
    sudo python3 test_encoder.py
"""

import RPi.GPIO as GPIO
import time

# GPIO pins
CLK = 17  # Pin 11
DT = 27   # Pin 13
SW = 22   # Pin 15

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(CLK, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(DT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(SW, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("=" * 60)
print("Test EC11 Rotary Encoder")
print("=" * 60)
print("Xoay encoder hoặc nhấn nút...")
print("Ctrl+C để thoát\n")

# State
last_clk = 1
last_sw = 1
last_rotation_time = time.time()
last_button_time = time.time()

rotation_count = 0
button_count = 0

try:
    while True:
        now = time.time()

        # Read current state
        clk = GPIO.input(CLK)
        dt = GPIO.input(DT)
        sw = GPIO.input(SW)

        # Detect rotation (CLK falling edge)
        if clk == 0 and last_clk == 1:
            elapsed = (now - last_rotation_time) * 1000

            # Debounce (> 5ms)
            if elapsed > 5:
                # FIXED: CW (clockwise) = DT is HIGH (1) at falling edge
                # CCW (counter-clockwise) = DT is LOW (0) at falling edge
                direction = "CW ⬆️  (tăng)" if dt == 1 else "CCW ⬇️  (giảm)"
                rotation_count += 1
                print(f"🎯 ROTATION #{rotation_count}: {direction} (DT={dt}, {elapsed:.1f}ms)")
                last_rotation_time = now

        # Detect button press (SW falling edge)
        if sw == 0 and last_sw == 1:
            elapsed = (now - last_button_time) * 1000

            # Debounce (> 200ms)
            if elapsed > 200:
                button_count += 1
                print(f"🔘 BUTTON PRESSED #{button_count} ({elapsed:.1f}ms)")
                last_button_time = now

        # Update state
        last_clk = clk
        last_sw = sw

        # Display current state
        print(f"\rCLK:{clk} DT:{dt} SW:{sw} {'[PRESSED]' if sw == 0 else '         '}",
              end='', flush=True)

        time.sleep(0.01)  # Poll every 10ms

except KeyboardInterrupt:
    print("\n\n" + "=" * 60)
    print(f"Tổng kết:")
    print(f"  Rotations: {rotation_count}")
    print(f"  Button presses: {button_count}")
    print("=" * 60)
    GPIO.cleanup()
    print("Done!\n")

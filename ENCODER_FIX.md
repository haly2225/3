# Fix Rotary Encoder EC11 For Modern Raspberry Pi

## Problem
Raspberry Pi với kernel 5.x+ không hỗ trợ `/sys/class/gpio/` (sysfs) nữa.
Lỗi: `Invalid argument` khi export GPIO

## Solution: Use libgpiod

### 1. Install libgpiod (on real Raspberry Pi):
```bash
sudo apt-get update
sudo apt-get install -y libgpiod-dev gpiod
```

### 2. Test encoder wiring:
```bash
# Read GPIO once
sudo gpioget gpiochip0 17 27 22

# Monitor in real-time (rotate encoder to test)
watch -n 0.1 'sudo gpioget gpiochip0 17 27 22'
```

Expected behavior:
- Rotate CW/CCW → GPIO 17 and 27 change
- Press button → GPIO 22 changes 1→0

### 3. Compile with libgpiod:
```bash
cd ~/3
g++ ol.cpp -o scope -std=c++17 \
    $(pkg-config --cflags --libs Qt5Widgets) \
    -lgpiod -fPIC

# Run (libgpiod doesn't need sudo!)
./scope
```

## Wiring Check
```
EC11    → RPi Pin → GPIO  → Function
VCC     → Pin 1   → 3.3V  → Power
GND     → Pin 9   → GND   → Ground
CLK (A) → Pin 11  → GPIO 17 → Rotation A
DT (B)  → Pin 13  → GPIO 27 → Rotation B  
SW      → Pin 15  → GPIO 22 → Button
```

## Alternative: Enable Legacy sysfs (NOT RECOMMENDED)
If you really want to use old sysfs GPIO:
```bash
# Edit /boot/firmware/config.txt (or /boot/config.txt on older systems)
sudo nano /boot/firmware/config.txt

# Add this line at the end:
dtoverlay=gpio-sysfs

# Reboot
sudo reboot
```

After reboot, `/sys/class/gpio/export` will work.
But libgpiod is the recommended modern way!

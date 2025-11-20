# Professional Oscilloscope - Python Version 🎛️

Phiên bản Python của oscilloscope chuyên nghiệp, dễ cài đặt và chạy hơn C++!

## Tính năng

✅ **SPI Communication** - Đọc data từ STM32 qua SPI
✅ **Phase-Locked Alignment** - Waveform ổn định như Tektronix
✅ **Auto Trigger 50%** - Tự động set trigger level = (Vmax + Vmin) / 2
✅ **Rotary Encoder EC11** - Xoay để chỉnh Time/Div và Volts/Div
✅ **Trigger Modes** - AUTO / NORMAL / FREE_RUN
✅ **PyQt5 GUI** - Giao diện chuyên nghiệp

## Cài đặt (Raspberry Pi)

```bash
# 1. Install dependencies
sudo apt-get update
sudo apt-get install -y python3-pyqt5 python3-spidev python3-rpi.gpio

# 2. Enable SPI
sudo raspi-config
# → Interface Options → SPI → Enable

# 3. Reboot
sudo reboot
```

## Chạy Oscilloscope

```bash
cd ~/3
sudo python3 scope.py
```

**Cần sudo!** RPi.GPIO cần quyền root để access GPIO.

## Sử dụng

### Rotary Encoder EC11
- **Xoay CW/CCW**: Tăng/giảm Time/Div hoặc Volts/Div
- **Nhấn nút**: Chuyển đổi Time/Div ↔ Volts/Div mode

### Keyboard Shortcuts
- **A**: Toggle Auto Trigger 50%
- **← →**: Điều chỉnh Time/Div
- **↑ ↓**: Điều chỉnh Volts/Div
- **Space**: Chuyển đổi Time/Div ↔ Volts/Div mode

### Wiring EC11
```
EC11    → RPi Pin → GPIO  → Function
VCC     → Pin 1   → 3.3V  → Power
GND     → Pin 9   → GND   → Ground
CLK (A) → Pin 11  → GPIO 17 → Rotation A
DT (B)  → Pin 13  → GPIO 27 → Rotation B
SW      → Pin 15  → GPIO 22 → Button
```

## Scales

### Time/Div
- 100µs, 200µs, 500µs, 1ms, 2ms, 5ms

### Volts/Div
- 0.2V, 0.5V, 1V, 2V

## Troubleshooting

### Lỗi: "ImportError: No module named 'PyQt5'"
```bash
sudo apt-get install -y python3-pyqt5
```

### Lỗi: "ImportError: No module named 'spidev'"
```bash
sudo apt-get install -y python3-spidev
```

### Lỗi: "ImportError: No module named 'RPi.GPIO'" hoặc "ImportError: No module named 'RPi'"
```bash
sudo apt-get install -y python3-rpi.gpio
```

### Test Encoder Hardware
```bash
# Monitor GPIO in real-time
watch -n 0.1 'gpioget gpiochip0 17 27 22'

# Rotate encoder → GPIO 17, 27 thay đổi
# Press button → GPIO 22 thay đổi (1→0)
```

## So sánh C++ vs Python

| Feature | C++ (ol.cpp) | Python (scope.py) |
|---------|-------------|------------------|
| Compile | ✗ Cần g++, Qt5 headers | ✅ Không cần compile |
| Dependencies | libgpiod-dev, Qt5 | RPi.GPIO, PyQt5 |
| Permissions | Cần sudo | ✅ Cần sudo cho GPIO |
| GPIO API | libgpiod v1.x (lỗi) | ✅ RPi.GPIO (stable) |
| Performance | Nhanh hơn | ✅ Đủ nhanh (20 FPS) |
| Dễ maintain | Phức tạp | ✅ Dễ đọc, dễ sửa |

## Kết luận

**Python version** đơn giản hơn, dễ chạy hơn và không có vấn đề compile!

Phù hợp cho:
- ✅ Development và testing
- ✅ Học tập và demo
- ✅ Tất cả Raspberry Pi OS (RPi.GPIO stable và được hỗ trợ tốt)

Nếu cần performance tối đa → dùng C++ version (nhưng cần fix libgpiod)
Nếu cần dễ dùng và chạy ngay → **dùng Python version** 🎉

## Encoder hoạt động như thế nào?

### Xoay CW (Clockwise - Xoay phải):
1. CLK chuyển từ 1→0 (falling edge)
2. Tại thời điểm CLK=0, kiểm tra DT
3. Nếu DT=0 → Direction = +1 (CW)
4. Callback `on_rotate(+1)` được gọi
5. Time/Div hoặc Volts/Div tăng lên

### Xoay CCW (Counter-Clockwise - Xoay trái):
1. CLK chuyển từ 1→0 (falling edge)
2. Tại thời điểm CLK=0, kiểm tra DT
3. Nếu DT=1 → Direction = -1 (CCW)
4. Callback `on_rotate(-1)` được gọi
5. Time/Div hoặc Volts/Div giảm xuống

### Nhấn nút:
1. SW chuyển từ 1→0 (falling edge)
2. Debounce 200ms để tránh nhiễu
3. Callback `on_button_press()` được gọi
4. Chuyển đổi giữa Time/Div mode ↔ Volts/Div mode

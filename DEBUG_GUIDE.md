# Quick Reference - Oscilloscope Debug

## Chạy Oscilloscope

```bash
cd ~/3
sudo python3 scope.py
```

## Debug Panel (Góc phải màn hình)

### 📊 Signal (màu xanh lá)
```
Signal:
Vpp: 3.25V       ← Voltage peak-to-peak (biên độ)
Vmax: 3.30V      ← Điện áp cao nhất
Vmin: 0.05V      ← Điện áp thấp nhất
Freq: 1.2 kHz    ← Tần số tín hiệu
```

**Ý nghĩa:**
- **Vpp = 0V** → Không có tín hiệu hoặc tín hiệu DC flat
- **Freq = 0 Hz** → Không detect được frequency (tín hiệu quá chậm hoặc DC)
- **Freq hiển thị** → Tín hiệu AC đang hoạt động

### 📡 SPI (màu xanh dương)
```
SPI:
Packets: 1234    ← Tổng số packets đã đọc
Sync: 98.5%      ← Tỷ lệ sync thành công
Triggers: 456    ← Số lần tìm thấy trigger edge
```

**Ý nghĩa:**
- **Sync: 100%** → SPI hoạt động hoàn hảo ✅
- **Sync: <90%** → Có vấn đề với SPI connection ⚠️
- **Sync: 0%** → STM32 không gửi data hoặc SPI không hoạt động ❌
- **Triggers tăng** → AUTO/NORMAL trigger đang detect edges

### 🎛️ Encoder (màu vàng)
```
Encoder:
Rotations: 15 CW⬆️   ← Số lần xoay, chiều xoay gần nhất
Buttons: 3          ← Số lần nhấn nút
Mode: TIME/DIV      ← Mode hiện tại
```

**Ý nghĩa:**
- **Rotations tăng** → Encoder đang hoạt động ✅
- **CW⬆️** → Xoay phải gần nhất (tăng)
- **CCW⬇️** → Xoay trái gần nhất (giảm)
- **Rotations không tăng** → Encoder không hoạt động hoặc chưa kết nối ❌
- **Buttons tăng** → Nút nhấn hoạt động ✅

## Troubleshooting

### 1. Encoder không hoạt động
**Triệu chứng:** Rotations = 0 khi xoay encoder

**Kiểm tra:**
```bash
# Test encoder hardware
sudo python3 test_encoder.py
# Xoay encoder → xem có detect không
```

**Kiểm tra wiring:**
```
EC11  →  Pi Pin  →  GPIO
CLK   →  Pin 11  →  GPIO 17
DT    →  Pin 13  →  GPIO 27
SW    →  Pin 15  →  GPIO 22
VCC   →  Pin 1   →  3.3V
GND   →  Pin 9   →  GND
```

### 2. SPI Sync thấp (<90%)
**Triệu chứng:** Sync: 50% hoặc thấp hơn

**Nguyên nhân:**
- STM32 chưa chạy hoặc chưa flash firmware
- SPI cable bị lỏng
- Tốc độ SPI không khớp (16MHz)

**Giải pháp:**
- Flash lại firmware STM32
- Kiểm tra kết nối SPI (MISO, MOSI, CLK, CS)
- Enable SPI trên Raspberry Pi: `sudo raspi-config` → Interface → SPI

### 3. Không có tín hiệu (Vpp = 0V)
**Triệu chứng:** Vpp: 0.00V, màn hình flat line

**Nguyên nhân:**
- Không có tín hiệu input vào ADC của STM32
- STM32 không sampling
- SPI không truyền data

**Kiểm tra:**
- Sync% có >0 không? (nếu 0% → SPI lỗi)
- Packets có tăng không? (nếu không tăng → SPI không đọc được)
- Dùng multimeter đo tín hiệu input STM32

### 4. Frequency = 0 Hz
**Triệu chứng:** Freq: 0 Hz nhưng có waveform

**Nguyên nhân:**
- Tín hiệu DC (không có zero crossing)
- Tín hiệu quá chậm (<1Hz)
- Waveform không crossing qua mid-level

**Bình thường:** Nếu tín hiệu là DC hoặc rất chậm, frequency = 0 Hz là đúng.

## Keyboard Controls

| Key | Action |
|-----|--------|
| **A** | Toggle Auto Trigger 50% |
| **← →** | Adjust Time/Div |
| **↑ ↓** | Adjust Volts/Div |
| **Space** | Toggle Time/Div ↔ Volts/Div mode |

## Encoder Controls

| Action | Result |
|--------|--------|
| **Xoay CW (phải)** | Tăng Time/Div hoặc Volts/Div |
| **Xoay CCW (trái)** | Giảm Time/Div hoặc Volts/Div |
| **Nhấn nút** | Chuyển mode: TIME/DIV ↔ VOLTS/DIV |

## Scales Available

**Time/Div:** 50µs, 100µs, 200µs, 500µs, 1ms, 2ms, 5ms, 10ms, 20ms, 50ms, 100ms

**Volts/Div:** 0.05V, 0.1V, 0.2V, 0.5V, 1V, 2V, 5V

## Normal Operation Expected Values

```
Signal:
Vpp: 0.5V - 3.3V     ← Phụ thuộc tín hiệu input
Vmax: 0.1V - 3.3V
Vmin: 0.0V - 3.0V
Freq: 10 Hz - 100 kHz ← Phụ thuộc SAMPLE_RATE (411kHz)

SPI:
Packets: tăng liên tục   ← ~20 packets/second
Sync: >95%              ← Gần 100% là tốt nhất
Triggers: tăng          ← Nếu ở AUTO/NORMAL mode

Encoder:
Rotations: tăng khi xoay
Buttons: tăng khi nhấn
Mode: TIME/DIV hoặc VOLTS/DIV
```

## Liên hệ

Nếu gặp lỗi không giải quyết được, check terminal output (console) để xem debug messages chi tiết:
- `🔧 Encoder: Initializing...`
- `✅ Encoder: Init OK!`
- `🎯 Encoder: ROTATION #X CW⬆️`
- `✅ SPI: Opened /dev/spidev0.0`
- `🎯 Auto 50%: 1.65V`

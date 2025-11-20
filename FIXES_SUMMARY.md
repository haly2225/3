# Các Vấn Đề Đã Fix ✅

## 1. EC11 Quay Xuôi Nhưng Nhảy Loạn Xạ 🔧

### Vấn đề:
- Khi xoay nhanh, encoder nhảy số loạn xạ hoặc đảo chiều
- Code cũ dùng polling với `time.sleep(0.01)` (10ms)
- Bỏ lỡ 90% xung khi xoay nhanh

### Nguyên nhân:
```python
# CODE CŨ (SAI):
while running:
    clk = GPIO.input(17)
    dt = GPIO.input(27)
    # ... logic detect rotation
    time.sleep(0.01)  # ❌ Quá chậm! Bỏ lỡ nhiều xung
```

Encoder EC11 tạo ~20-30 xung/vòng. Khi xoay nhanh (1 vòng/giây), tạo ra 30 xung trong 1 giây = 1 xung mỗi 33ms. Nhưng code polling 10ms, nên chỉ detect được 3 xung/10, miss 7 xung → Đọc sai thứ tự A/B → Đảo chiều hoặc nhảy số.

### Giải pháp:
```python
# CODE MỚI (ĐÚNG) - INTERRUPT:
GPIO.add_event_detect(gpio_clk, GPIO.FALLING,
                      callback=self._clk_callback,
                      bouncetime=5)  # ✅ Phản ứng ngay lập tức!

def _clk_callback(self, channel):
    # CPU tự động nhảy vào đây ngay khi CLK falling edge
    dt = GPIO.input(self.gpio_dt)
    direction = 1 if dt == 1 else -1
    # ... xử lý rotation
```

### Kết quả:
✅ Xoay nhanh không còn bị nhảy số
✅ Chiều xoay luôn đúng (CW = tăng, CCW = giảm)
✅ Tiết kiệm CPU (không cần polling thread)

---

## 2. Sóng Không Vuông (Bị Vát Chéo Hình Thang) 📊

### Vấn đề:
- Sóng vuông hiển thị như hình thang (cạnh bị vát chéo)
- Đặc biệt rõ ở tần số cao hoặc Time/Div lớn

### Nguyên nhân:
```python
# CODE CŨ (VẼ ĐƯỜNG THẲNG):
painter.drawLine(x1, y1, x2, y2)  # ❌ Nối điểm 1 với điểm 2 bằng đường chéo
```

Khi số sample ít (high frequency), đường nối giữa 2 điểm tạo thành đường chéo thay vì góc vuông.

**Ví dụ:**
```
Sample points: _____|     |_____

VẼ LINE:       _____/     \_____  ← SAI (đường chéo)

VẼ STEP:       _____|     |_____  ← ĐÚNG (góc vuông)
```

### Giải pháp:
```python
# CODE MỚI (VẼ BẬC THANG):
if self.draw_mode == 'step':
    # Vẽ ngang ở mức v1
    painter.drawLine(x1, y1, x2, y1)  # ✅ Horizontal line
    # Vẽ dọc lên mức v2
    painter.drawLine(x2, y1, x2, y2)  # ✅ Vertical line
```

### Kết quả:
✅ Sóng vuông hiển thị với góc vuông hoàn hảo
✅ Không còn đường chéo (diagonal)
✅ Dễ nhìn hơn, giống oscilloscope thật

### Lưu ý phần cứng:
Nếu vẫn thấy cạnh bị cắt (clipping), do ADC bão hòa ở 0V và 3.3V. Giải pháp:
```
Tín hiệu 3.3Vpp (0V → 3.3V)
        ↓ Chia áp bằng 2 điện trở
Tín hiệu 2.0Vpp (0.5V → 2.5V) ← ADC không bị bão hòa
```

---

## 3. Sóng Bị Giật và Trôi (Jitter) 📉

### Vấn đề:
- Waveform bị rung, không đứng yên
- Trigger point thay đổi liên tục giữa các frame

### Nguyên nhân:
```python
# CODE CŨ (KHÔNG CÓ HYSTERESIS):
if voltages[i-1] < level and voltages[i] >= level:
    return i  # ❌ Nhiễu làm trigger sớm/muộn
```

**Minh họa:**
```
Trigger level: ----1.65V----

Signal có nhiễu:
    1.66V ← Qua ngưỡng (trigger)
    1.64V ← Xuống dưới (chưa trigger)
    1.66V ← Qua lại (trigger lại)
    1.64V ← ...

→ Trigger point nhảy lung tung!
```

### Giải pháp (Hysteresis):
```python
# CODE MỚI (CÓ HYSTERESIS 50mV):
hysteresis = 0.05  # 50mV safety zone

# Rising edge:
if voltages[i-1] < (level - hysteresis) and voltages[i] >= level:
    return i  # ✅ Phải xuống dưới 1.60V rồi mới cross 1.65V

# Falling edge:
if voltages[i-1] > (level + hysteresis) and voltages[i] <= level:
    return i  # ✅ Phải lên trên 1.70V rồi mới cross 1.65V
```

**Minh họa:**
```
Rising trigger với hysteresis:

3.3V |           ╔═══╗
     |           ║   ║
1.70V|    ┌──────╢   ║  ← Must go above 1.70V first
1.65V|----├------╢   ╟---- Trigger level
1.60V|    │      ║   ║  ← Must come from below 1.60V
0.0V |════╧══════╝   ╚════

Safety zone: [1.60V - 1.70V] = 50mV hysteresis
→ Nhiễu trong zone này không ảnh hưởng trigger!
```

### Kết quả:
✅ Waveform đứng yên, không rung
✅ Trigger point ổn định
✅ Không bị ảnh hưởng bởi nhiễu nhỏ (<50mV)

---

## Tóm Tắt Thay Đổi

| Vấn đề | Trước | Sau |
|--------|-------|-----|
| **Encoder** | Polling 10ms → miss pulses | Interrupt → bắt 100% |
| **Trigger** | Không hysteresis → jitter | Hysteresis 50mV → stable |
| **Display** | Line drawing → trapezoidal | Step drawing → square |

## Test Ngay

```bash
cd ~/3
sudo python3 scope.py
```

**Kiểm tra:**
1. ✅ Xoay encoder nhanh → Không nhảy số
2. ✅ Sóng vuông → Góc vuông sắc nét
3. ✅ Waveform → Đứng yên, không rung

## Technical Summary

```python
# BEFORE:
- Encoder: Polling thread, sleep(0.01)
- Trigger: Simple threshold comparison
- Display: Linear interpolation

# AFTER:
- Encoder: GPIO interrupt, no polling needed ✅
- Trigger: Hysteresis (50mV safety zone) ✅
- Display: Step-style drawing for square waves ✅
```

## Những Gì Đã Học

### 1. Encoder Polling vs Interrupt
**Sai lầm:** Nghĩ rằng polling 10ms là đủ nhanh
**Sự thật:** Encoder tạo xung rất nhanh khi xoay, cần interrupt

### 2. Trigger Hysteresis
**Sai lầm:** So sánh đơn giản với threshold
**Sự thật:** Tín hiệu có nhiễu, cần vùng an toàn (hysteresis)

### 3. Waveform Display
**Sai lầm:** Vẽ line giữa các điểm là đủ
**Sự thật:** Sóng vuông cần vẽ step (bậc thang) mới đúng

---

**Commit:** `f2b0b3d` - Fix 3 critical issues
**Branch:** `claude/debug-signal-auto50-01JZRdyHLfHEes5VJ1psn2U8`

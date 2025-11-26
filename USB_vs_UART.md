# Giải Thích: USB Port vs UART Pins

## ⚠️ QUAN TRỌNG: USB Port ≠ UART Hardware

Bạn nói "đang cắm chân USB từ STM32 sang Pi4" - đây là một sự nhầm lẫn phổ biến!

### USB Port trên STM32 Blue Pill:

**Cổng USB nhỏ trên STM32** được dùng để:
- ✅ Flash firmware (upload code)
- ✅ Debug (qua ST-Link/USB programmer)
- ❌ **KHÔNG phải UART!** (trừ khi config USB CDC - rất phức tạp)

### UART Hardware Pins:

**Code UART trong firmware** sử dụng:
- **PA9** = USART1_TX (chân hardware riêng biệt)
- **PA10** = USART1_RX (chân hardware riêng biệt)

---

## 🔌 Bạn Có 3 Lựa Chọn:

### **Option 1: Dùng USB-to-Serial Adapter (ĐỀ XUẤT!)**

**Ưu điểm:**
- Đơn giản, rẻ (20-30k VND)
- Không cần sửa code gì
- Build firmware hiện tại là OK

**Cần mua:**
- USB-to-Serial adapter: CH340, CP2102, hoặc FT232

**Kết nối:**
```
STM32 Blue Pill          USB-to-Serial Adapter
────────────────────────────────────────────────
PA9  (USART1_TX)    →    RX
GND                 →    GND
```

**Cắm:**
- USB của STM32 → PC (để flash firmware)
- USB-to-Serial adapter → Pi4 (để nhận data)

**Test:**
```bash
# Trên Pi4:
python3 uart_test.py /dev/ttyUSB0 2000000
```

---

### **Option 2: Kết Nối Trực Tiếp với Pi4 UART**

**Không cần adapter, dùng UART của Pi4:**

**Kết nối dây:**
```
STM32 Blue Pill          Raspberry Pi 4
──────────────────────────────────────────
PA9  (USART1_TX)    →    GPIO15 (Pin 10, RXD)
GND                 →    GND (Pin 6, 9, 14, 20, 25, 30, 34, 39)
```

**Enable UART trên Pi4:**
```bash
sudo raspi-config
# 3. Interface Options
# I6. Serial Port
#   - Login shell over serial: NO
#   - Serial hardware enabled: YES
sudo reboot
```

**Test:**
```bash
python3 uart_test.py /dev/ttyAMA0 2000000
# Hoặc:
python3 uart_test.py /dev/serial0 2000000
```

**Lưu ý:**
- Cần disable Bluetooth để dùng hardware UART
- Thêm vào `/boot/config.txt`:
  ```
  dtoverlay=disable-bt
  ```

---

### **Option 3: USB CDC (Phức Tạp - KHÔNG ĐỀ XUẤT)**

**Để dùng USB port của STM32 như Serial:**

Cần:
1. Mở STM32CubeMX project (.ioc file)
2. Enable USB Device Middleware
3. Set USB class = CDC (Communication Device Class)
4. Regenerate code
5. Rebuild firmware

**Nếu làm option này:**
- Device sẽ xuất hiện như `/dev/ttyACM0` trên Pi4
- Test với: `python3 uart_test.py /dev/ttyACM0 2000000`

**Nhưng rất phức tạp và không ổn định!**

---

## 🛠️ Bây Giờ Làm Gì?

### **Build Lại Firmware (Đã Fix Lỗi):**

Tôi đã enable UART HAL module, bây giờ build sẽ thành công:

1. **Trong STM32CubeIDE:**
   - Project → Clean → Clean all projects
   - Project → Build (Ctrl+B)

2. **Kết quả:** Build sẽ thành công, tạo file `.elf` và `.bin`

3. **Flash vào STM32:**
   - Run → Debug (F11)
   - Hoặc Run → Run (Ctrl+F11)

### **Sau Khi Flash:**

**Kiểm tra LED PC13:**
- Lúc khởi động: Nhấp nháy 3 lần nhanh
- Khi chạy: Nhấp nháy liên tục (mỗi lần ADC xong)
- Nếu KHÔNG nhấp nháy → firmware chưa chạy

### **Kết Nối Hardware:**

**Nếu có USB-to-Serial adapter:**
```
PA9 → Adapter RX
GND → Adapter GND
Adapter USB → Pi4
```

**Nếu không có adapter:**
```
PA9 → Pi4 GPIO15 (Pin 10)
GND → Pi4 GND (Pin 6)
```

### **Test Trên Pi4:**

```bash
# Check port:
ls -l /dev/ttyUSB* /dev/ttyAMA* /dev/ttyACM*

# Test với adapter:
python3 uart_test.py /dev/ttyUSB0 2000000

# Test với Pi4 UART:
python3 uart_test.py /dev/ttyAMA0 2000000

# Test với USB CDC (nếu đã config):
python3 uart_test.py /dev/ttyACM0 2000000
```

**Kết quả mong đợi:**
```
✓ Packet #   1 | Frame: 0001 | Samples: 512 | Vpp: 3.300V
  First 10 ADC values:    0,    8,   16,   24...
```

Sẽ thấy **sóng răng cưa** (test pattern).

---

## 🔍 Troubleshooting:

### Build Vẫn Lỗi?
- Đảm bảo đã pull code mới nhất: `git pull origin claude/refactor-oscilloscope-code-01T7qhXcaqP7Y8PUXUEhzrk3`
- Clean project: Project → Clean → Clean all
- Close và reopen STM32CubeIDE

### Không Thấy Port /dev/ttyUSB0?
- Cắm USB-to-Serial adapter vào Pi4
- Chạy: `dmesg | grep tty` để xem log
- Chạy: `lsusb` để xem device

### Permission Denied?
```bash
sudo usermod -a -G dialout $USER
# Logout và login lại
```

### Không Nhận Được Data?
1. Check LED PC13 có nhấp nháy không?
2. Check kết nối: PA9 → RX (không phải TX!)
3. Thử baudrate thấp hơn: 1000000 hoặc 921600
4. Đổi dây nối (có thể bị đứt)

---

## 📊 Tóm Tắt:

| Phương Pháp | Phần Cứng | Độ Khó | Giá | Khuyến Nghị |
|-------------|-----------|--------|-----|-------------|
| USB-to-Serial | CH340 adapter | ⭐ Dễ | 20-30k | ✅ **ĐỀ XUẤT** |
| Pi4 UART | Dây nối | ⭐⭐ Trung bình | 0đ | ✅ OK |
| USB CDC | Không cần thêm | ⭐⭐⭐⭐⭐ Khó | 0đ | ❌ Phức tạp |

**Lời khuyên:** Dùng USB-to-Serial adapter (CH340) - dễ nhất, rẻ nhất, ổn định nhất!

---

## ✅ Checklist:

- [ ] Build firmware thành công (không lỗi)
- [ ] Flash vào STM32
- [ ] LED PC13 nhấp nháy
- [ ] Chuẩn bị USB-to-Serial adapter hoặc dây nối Pi4
- [ ] Kết nối PA9 → RX, GND → GND
- [ ] Enable UART trên Pi4 (nếu dùng Pi4 UART)
- [ ] Test: `python3 uart_test.py`
- [ ] Thấy test pattern (sóng răng cưa)
- [ ] Chuyển sang ADC thực và test lại

Bạn build lại firmware ngay nhé! Lần này sẽ thành công! 🚀

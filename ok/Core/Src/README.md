# Core/Src - STM32 Oscilloscope Source Files

Thư mục này chứa các file mã nguồn chính cho firmware STM32F103 Oscilloscope.

## 📁 Cấu trúc thành phần

### 1️⃣ `main.c` - Chương trình chính
**Chức năng**: File chương trình chính điều khiển toàn bộ hệ thống oscilloscope

**Tính năng chính**:
- **ADC Capture**: Thu thập dữ liệu analog qua ADC1 với DMA
  - Buffer size: 256 mẫu (12-bit resolution)
  - Trigger tự động bởi Timer TIM1
- **SPI Communication**: Truyền dữ liệu tới Raspberry Pi
  - Mode: SPI Slave với DMA
  - Protocol: Frame counter + ADC data (header 0xAA55)
- **Data Processing**: Đóng gói dữ liệu ADC 16-bit thành bytes stream
- **Callbacks**: Xử lý ngắt DMA cho ADC và SPI

**Các biến toàn cục quan trọng**:
```c
uint16_t adc_buffer[256]    // Buffer lưu dữ liệu ADC
uint8_t  tx_buffer[]        // Buffer truyền SPI
volatile uint32_t adc_count // Đếm số lần ADC hoàn thành
volatile uint32_t spi_count // Đếm số lần SPI transfer
```

**Workflow**:
```
TIM1 Trigger → ADC + DMA → Buffer đầy → Pack data → SPI TX → Raspberry Pi
```

---

### 2️⃣ `stm32f1xx_hal_msp.c` - MSP Initialization
**Chức năng**: MCU Support Package - Khởi tạo phần cứng tầng thấp

**Nhiệm vụ**:
- Cấu hình Clock cho các peripheral (ADC, SPI, TIM, DMA)
- Cấu hình GPIO pins (alternate functions)
- Cấu hình DMA channels và priorities
- Setup interrupt priorities (NVIC)

**Các hàm quan trọng**:
- `HAL_ADC_MspInit()` - Khởi tạo ADC và DMA channel
- `HAL_SPI_MspInit()` - Khởi tạo SPI và DMA TX/RX
- `HAL_TIM_MspInit()` - Khởi tạo Timer trigger
- `HAL_XXX_MspDeInit()` - Deinitialize khi không dùng

**Lưu ý**: File này được generate tự động bởi STM32CubeMX, nhưng có thể chỉnh sửa trong `USER CODE` sections.

---

### 3️⃣ `stm32f1xx_it.c` - Interrupt Service Routines
**Chức năng**: Xử lý tất cả các ngắt của hệ thống

**Các loại interrupt**:

**System Exceptions**:
- `NMI_Handler()` - Non-Maskable Interrupt
- `HardFault_Handler()` - Hard Fault (lỗi nghiêm trọng)
- `MemManage_Handler()` - Memory management fault
- `BusFault_Handler()` - Bus fault
- `UsageFault_Handler()` - Usage fault

**DMA Interrupts**:
- `DMA1_Channel1_IRQHandler()` - DMA cho ADC1
- `DMA1_Channel2_IRQHandler()` - DMA cho SPI1 RX
- `DMA1_Channel3_IRQHandler()` - DMA cho SPI1 TX

**Peripheral Interrupts**:
- `EXTI4_IRQHandler()` - External interrupt cho NSS pin (PA4)
- `TIM1_UP_TIM16_IRQHandler()` - Timer 1 update

**Flow ngắt**:
```
Hardware Event → IRQHandler → HAL_XXX_IRQHandler() → Callback trong main.c
```

---

### 4️⃣ `system_stm32f1xx.c` - System Initialization
**Chức năng**: Khởi tạo hệ thống STM32F1xx cơ bản

**Nhiệm vụ chính**:
- Cấu hình System Clock (HSE/HSI, PLL)
- Setup vector table location
- Cấu hình FPU (nếu có)
- Initialize system before `main()`

**Các biến/hàm quan trọng**:
- `SystemInit()` - Gọi đầu tiên trước `main()`
- `SystemCoreClock` - Biến lưu tần số CPU (72 MHz)
- `SystemCoreClockUpdate()` - Update lại giá trị clock

**Clock configuration**:
```
HSE (8 MHz) → PLL (x9) → SYSCLK (72 MHz) → AHB → APB1/APB2
```

---

### 5️⃣ `syscalls.c` - System Calls
**Chức năng**: Cài đặt các system calls cho newlib (C standard library)

**Các hàm thường thấy**:
- `_write()` - Hỗ trợ `printf()` qua UART/SWO
- `_read()` - Đọc input (thường không dùng cho embedded)
- `_sbrk()` - Memory allocation cho heap
- `_close()`, `_lseek()`, `_fstat()` - File operations (stub)

**Ứng dụng**:
- Enable `printf()` debug qua SWD/SWO
- Hỗ trợ `malloc()`, `free()`

---

### 6️⃣ `sysmem.c` - System Memory Management
**Chức năng**: Quản lý memory allocation (heap) cho embedded system

**Nhiệm vụ**:
- Cài đặt `_sbrk()` cho heap allocation
- Quản lý boundary giữa heap và stack
- Phát hiện heap overflow

**Memory layout**:
```
[Flash: Code + Constants] → [SRAM: .data + .bss + heap ↑ | ↓ stack]
```

---

## 🔧 Dependencies

Các file này phụ thuộc vào:
- **HAL Library**: `stm32f1xx_hal_*.h/c` (trong `Drivers/STM32F1xx_HAL_Driver/`)
- **CMSIS**: Core ARM Cortex-M3 files (trong `Drivers/CMSIS/`)
- **Header files**: `main.h`, `stm32f1xx_it.h` (trong `Core/Inc/`)

---

## 🚀 Build Process

1. **Compile**: Tất cả `.c` files được compile thành `.o` objects
2. **Link**: Linker ghép tất cả objects theo `STM32F103C8TX_FLASH.ld`
3. **Output**: Tạo file `.elf` → Convert sang `.bin`/`.hex` để flash

---

## 📊 Cấu hình hiện tại

- **MCU**: STM32F103C8T6 (64KB Flash, 20KB RAM)
- **Clock**: 72 MHz (max speed)
- **ADC**: 1 channel, 12-bit, DMA mode
- **SPI**: Slave mode, DMA TX/RX
- **Timer**: TIM1 trigger ADC

---

## 🐛 Debug Tips

**Khi gặp HardFault**:
- Check `stm32f1xx_it.c` → `HardFault_Handler()`
- Dùng debugger xem register `R0-R3, LR, PC`

**Khi DMA không hoạt động**:
- Check `stm32f1xx_hal_msp.c` → DMA channel config
- Verify interrupt priorities trong NVIC

**Khi SPI bị lỗi**:
- Check NSS pin timing (EXTI interrupt)
- Verify buffer alignment (4-byte aligned)

---

## 📝 Notes

- Các file `*_it.c` và `*_msp.c` được generate bởi CubeMX
- Chỉ sửa code trong `/* USER CODE BEGIN */` và `/* USER CODE END */`
- File `main.c` chứa logic chính của ứng dụng

---

**Tác giả**: STM32 Oscilloscope Project
**Version**: 1.0
**Date**: 2024

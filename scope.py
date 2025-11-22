#!/usr/bin/env python3
"""
Oscilloscope với Ring Buffer Processing
- Full sóng (Memory Depth)
- Kéo lại xem (Scroll History/Roll Mode)
- Pause/Stop để soi dữ liệu cũ
"""

import threading
import time
import numpy as np
from collections import deque
from typing import Optional, Tuple
import struct

# ============================================
# CẤU HÌNH OSCILLOSCOPE
# ============================================
class ScopeConfig:
    # Memory Depth - Kích thước bộ nhớ sâu (số mẫu)
    MEMORY_DEPTH = 100000  # 100.000 mẫu

    # SPI Config
    SPI_PACKET_SIZE = 2048  # Số mẫu mỗi gói SPI
    SPI_SPEED_HZ = 2000000  # 2MHz SPI clock

    # Display Config
    SCREEN_WIDTH = 800
    SCREEN_HEIGHT = 480
    DISPLAY_POINTS = 1000  # Số điểm hiển thị trên màn hình
    FPS = 30  # Frame per second

    # Time/Div settings (samples per division)
    TIME_DIV_OPTIONS = [100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000]

    # Voltage/Div settings
    VOLT_DIV_OPTIONS = [0.1, 0.2, 0.5, 1.0, 2.0, 5.0]


# ============================================
# RING BUFFER - BỘ NHỚ SÂU
# ============================================
class RingBuffer:
    """
    Bộ đệm vòng sử dụng deque
    - Tự động xóa dữ liệu cũ khi đầy
    - Thread-safe với lock
    """

    def __init__(self, maxlen: int = ScopeConfig.MEMORY_DEPTH):
        self.buffer = deque(maxlen=maxlen)
        self.lock = threading.Lock()
        self.maxlen = maxlen

    def extend(self, data):
        """Thêm nhiều mẫu vào buffer (thread-safe)"""
        with self.lock:
            self.buffer.extend(data)

    def append(self, sample):
        """Thêm một mẫu vào buffer (thread-safe)"""
        with self.lock:
            self.buffer.append(sample)

    def get_slice(self, start: int, end: int) -> list:
        """
        Lấy một đoạn dữ liệu từ buffer
        start, end: index từ cuối buffer (0 = mới nhất)
        """
        with self.lock:
            length = len(self.buffer)
            if length == 0:
                return []

            # Chuyển đổi index từ cuối sang đầu
            actual_start = max(0, length - end)
            actual_end = max(0, length - start)

            return list(self.buffer)[actual_start:actual_end]

    def get_latest(self, count: int) -> list:
        """Lấy count mẫu mới nhất"""
        with self.lock:
            length = len(self.buffer)
            if length == 0:
                return []
            start = max(0, length - count)
            return list(self.buffer)[start:]

    def get_with_offset(self, count: int, offset: int) -> list:
        """
        Lấy dữ liệu với offset (để scroll lịch sử)
        count: số mẫu cần lấy
        offset: vị trí lùi về quá khứ (0 = live, >0 = quá khứ)
        """
        with self.lock:
            length = len(self.buffer)
            if length == 0:
                return []

            # Tính toán index
            end_idx = length - offset
            start_idx = end_idx - count

            # Giới hạn
            start_idx = max(0, start_idx)
            end_idx = max(0, min(length, end_idx))

            if start_idx >= end_idx:
                return []

            return list(self.buffer)[start_idx:end_idx]

    def __len__(self):
        with self.lock:
            return len(self.buffer)

    def clear(self):
        """Xóa toàn bộ buffer"""
        with self.lock:
            self.buffer.clear()


# ============================================
# SPI READER THREAD - LUỒNG ĐỌC DỮ LIỆU
# ============================================
class SPIReader(threading.Thread):
    """
    Luồng đọc dữ liệu từ SPI
    - Chạy độc lập, liên tục bơm dữ liệu vào RingBuffer
    - Có thể Pause/Resume
    """

    def __init__(self, buffer: RingBuffer, spi_device=None):
        super().__init__(daemon=True)
        self.buffer = buffer
        self.spi = spi_device

        # Control flags
        self.is_running = False
        self.is_paused = False
        self._stop_event = threading.Event()

        # Statistics
        self.samples_read = 0
        self.read_errors = 0

    def setup_spi(self):
        """Khởi tạo SPI interface"""
        try:
            import spidev
            self.spi = spidev.SpiDev()
            self.spi.open(0, 0)  # Bus 0, Device 0
            self.spi.max_speed_hz = ScopeConfig.SPI_SPEED_HZ
            self.spi.mode = 0
            print("[SPI] Khởi tạo SPI thành công")
            return True
        except ImportError:
            print("[SPI] Không tìm thấy spidev - Chạy chế độ giả lập")
            return False
        except Exception as e:
            print(f"[SPI] Lỗi khởi tạo: {e}")
            return False

    def read_packet(self) -> Optional[list]:
        """Đọc một gói dữ liệu từ SPI"""
        if self.spi is None:
            # Chế độ giả lập - tạo dữ liệu sine wave
            return self._generate_fake_data()

        try:
            # Đọc raw bytes từ SPI
            raw_data = self.spi.readbytes(ScopeConfig.SPI_PACKET_SIZE * 2)

            # Chuyển đổi sang 16-bit samples
            samples = []
            for i in range(0, len(raw_data), 2):
                sample = (raw_data[i] << 8) | raw_data[i + 1]
                samples.append(sample)

            return samples

        except Exception as e:
            self.read_errors += 1
            print(f"[SPI] Lỗi đọc: {e}")
            return None

    def _generate_fake_data(self) -> list:
        """Tạo dữ liệu giả lập để test"""
        t = time.time()
        samples = []
        for i in range(ScopeConfig.SPI_PACKET_SIZE):
            # Sine wave + noise
            phase = (t * 1000 + i) * 0.01
            value = int(2048 + 1800 * np.sin(phase) + np.random.randint(-50, 50))
            value = max(0, min(4095, value))  # Clamp to 12-bit
            samples.append(value)
        return samples

    def run(self):
        """Main loop của thread"""
        self.is_running = True
        print("[SPI Reader] Bắt đầu đọc dữ liệu...")

        while not self._stop_event.is_set():
            # Kiểm tra Pause
            if self.is_paused:
                time.sleep(0.01)  # Sleep ngắn khi pause
                continue

            # Đọc packet
            packet = self.read_packet()

            if packet:
                # Bơm dữ liệu vào Ring Buffer
                self.buffer.extend(packet)
                self.samples_read += len(packet)

            # Nhỏ delay để không chiếm hết CPU
            time.sleep(0.001)

        self.is_running = False
        print("[SPI Reader] Đã dừng")

    def pause(self):
        """Tạm dừng đọc dữ liệu"""
        self.is_paused = True
        print("[SPI Reader] PAUSED - Buffer đóng băng")

    def resume(self):
        """Tiếp tục đọc dữ liệu"""
        self.is_paused = False
        print("[SPI Reader] RESUMED - Tiếp tục đọc")

    def stop(self):
        """Dừng hoàn toàn thread"""
        self._stop_event.set()
        if self.spi:
            self.spi.close()


# ============================================
# OSCILLOSCOPE DISPLAY - LUỒNG HIỂN THỊ
# ============================================
class OscilloscopeDisplay:
    """
    Quản lý hiển thị Oscilloscope
    - Lấy dữ liệu từ RingBuffer
    - Hỗ trợ Zoom (Time/Div)
    - Hỗ trợ Scroll (xem lịch sử)
    """

    def __init__(self, buffer: RingBuffer):
        self.buffer = buffer

        # Display settings
        self.time_div_index = 3  # Index trong TIME_DIV_OPTIONS
        self.volt_div_index = 3  # Index trong VOLT_DIV_OPTIONS

        # Scroll offset (0 = live, >0 = quá khứ)
        self.scroll_offset = 0
        self.max_scroll_offset = 0

        # Mode
        self.is_live = True  # True = xem live, False = xem lịch sử

        # Display buffer (dữ liệu đã được xử lý để vẽ)
        self.display_data = []

        # Trigger settings
        self.trigger_level = 2048  # Mức trigger (giữa 12-bit)
        self.trigger_enabled = False

    @property
    def time_div(self) -> int:
        """Số mẫu trên mỗi division"""
        return ScopeConfig.TIME_DIV_OPTIONS[self.time_div_index]

    @property
    def volt_div(self) -> float:
        """Volt per division"""
        return ScopeConfig.VOLT_DIV_OPTIONS[self.volt_div_index]

    @property
    def samples_to_display(self) -> int:
        """Tổng số mẫu cần lấy để vẽ (10 divisions)"""
        return self.time_div * 10

    def increase_time_div(self):
        """Zoom out - Hiển thị nhiều dữ liệu hơn"""
        if self.time_div_index < len(ScopeConfig.TIME_DIV_OPTIONS) - 1:
            self.time_div_index += 1
            print(f"[Display] Time/Div: {self.time_div} samples")

    def decrease_time_div(self):
        """Zoom in - Hiển thị ít dữ liệu hơn"""
        if self.time_div_index > 0:
            self.time_div_index -= 1
            print(f"[Display] Time/Div: {self.time_div} samples")

    def increase_volt_div(self):
        """Giảm độ nhạy"""
        if self.volt_div_index < len(ScopeConfig.VOLT_DIV_OPTIONS) - 1:
            self.volt_div_index += 1
            print(f"[Display] Volt/Div: {self.volt_div}V")

    def decrease_volt_div(self):
        """Tăng độ nhạy"""
        if self.volt_div_index > 0:
            self.volt_div_index -= 1
            print(f"[Display] Volt/Div: {self.volt_div}V")

    def scroll_backward(self, steps: int = 100):
        """Cuộn về quá khứ"""
        buffer_len = len(self.buffer)
        self.max_scroll_offset = max(0, buffer_len - self.samples_to_display)

        self.scroll_offset = min(self.scroll_offset + steps, self.max_scroll_offset)
        self.is_live = (self.scroll_offset == 0)

        if not self.is_live:
            print(f"[Display] Scroll: -{self.scroll_offset} samples")

    def scroll_forward(self, steps: int = 100):
        """Cuộn về hiện tại"""
        self.scroll_offset = max(0, self.scroll_offset - steps)
        self.is_live = (self.scroll_offset == 0)

        if self.is_live:
            print("[Display] LIVE mode")
        else:
            print(f"[Display] Scroll: -{self.scroll_offset} samples")

    def go_to_live(self):
        """Quay về chế độ Live"""
        self.scroll_offset = 0
        self.is_live = True
        print("[Display] LIVE mode")

    def get_display_data(self) -> list:
        """
        Lấy dữ liệu để hiển thị
        - Tự động nén nếu cần (khi zoom out)
        """
        # Số mẫu cần lấy
        samples_needed = self.samples_to_display

        # Lấy dữ liệu từ buffer với offset
        raw_data = self.buffer.get_with_offset(samples_needed, self.scroll_offset)

        if not raw_data:
            return []

        # Nén dữ liệu nếu nhiều hơn số điểm hiển thị
        if len(raw_data) > ScopeConfig.DISPLAY_POINTS:
            self.display_data = self._downsample(raw_data, ScopeConfig.DISPLAY_POINTS)
        else:
            self.display_data = raw_data

        return self.display_data

    def _downsample(self, data: list, target_points: int) -> list:
        """
        Nén dữ liệu xuống số điểm target
        Sử dụng Min-Max để giữ được đỉnh sóng
        """
        if len(data) <= target_points:
            return data

        result = []
        chunk_size = len(data) // (target_points // 2)

        for i in range(0, len(data) - chunk_size, chunk_size):
            chunk = data[i:i + chunk_size]
            if chunk:
                result.append(min(chunk))
                result.append(max(chunk))

        return result[:target_points]

    def convert_to_screen_coords(self, data: list) -> list:
        """Chuyển đổi dữ liệu sang tọa độ màn hình"""
        if not data:
            return []

        coords = []
        width = ScopeConfig.SCREEN_WIDTH
        height = ScopeConfig.SCREEN_HEIGHT

        # Scale factors
        x_scale = width / len(data)
        y_center = height // 2
        y_scale = height / (4096 * self.volt_div)  # 12-bit ADC

        for i, sample in enumerate(data):
            x = int(i * x_scale)
            y = int(y_center - (sample - 2048) * y_scale)
            y = max(0, min(height - 1, y))
            coords.append((x, y))

        return coords


# ============================================
# MAIN OSCILLOSCOPE CLASS
# ============================================
class Oscilloscope:
    """
    Class chính điều khiển Oscilloscope
    - Quản lý RingBuffer, SPIReader, Display
    - Xử lý input từ người dùng (encoder, button)
    """

    def __init__(self):
        # Khởi tạo Ring Buffer (bộ nhớ sâu)
        self.memory = RingBuffer(maxlen=ScopeConfig.MEMORY_DEPTH)

        # Khởi tạo SPI Reader
        self.reader = SPIReader(self.memory)

        # Khởi tạo Display
        self.display = OscilloscopeDisplay(self.memory)

        # Control flags
        self.is_paused = False
        self.is_running = False

        # Current mode
        self.mode = "NORMAL"  # NORMAL, SCROLL, MEASURE

        print(f"[Oscilloscope] Khởi tạo với Memory Depth: {ScopeConfig.MEMORY_DEPTH}")

    def start(self):
        """Bắt đầu oscilloscope"""
        self.is_running = True

        # Setup và start SPI reader
        self.reader.setup_spi()
        self.reader.start()

        print("[Oscilloscope] Đã khởi động")

    def stop(self):
        """Dừng oscilloscope"""
        self.is_running = False
        self.reader.stop()
        print("[Oscilloscope] Đã dừng")

    def pause(self):
        """Tạm dừng - đóng băng buffer để xem lịch sử"""
        self.is_paused = True
        self.reader.pause()
        print("[Oscilloscope] PAUSED - Có thể scroll xem lịch sử")

    def resume(self):
        """Tiếp tục chạy"""
        self.is_paused = False
        self.reader.resume()
        self.display.go_to_live()
        print("[Oscilloscope] RESUMED")

    def toggle_pause(self):
        """Toggle pause/resume"""
        if self.is_paused:
            self.resume()
        else:
            self.pause()

    # ============================================
    # ENCODER HANDLERS
    # ============================================
    def encoder_rotate(self, direction: int):
        """
        Xử lý xoay encoder
        direction: 1 = CW (clockwise), -1 = CCW
        """
        if self.mode == "NORMAL":
            # Điều chỉnh Time/Div
            if direction > 0:
                self.display.increase_time_div()
            else:
                self.display.decrease_time_div()

        elif self.mode == "SCROLL":
            # Scroll lịch sử
            if direction > 0:
                self.display.scroll_backward(100)
            else:
                self.display.scroll_forward(100)

        elif self.mode == "MEASURE":
            # Điều chỉnh cursor
            pass

    def encoder_button(self):
        """Xử lý nhấn encoder"""
        # Cycle qua các mode
        modes = ["NORMAL", "SCROLL", "MEASURE"]
        current_idx = modes.index(self.mode)
        self.mode = modes[(current_idx + 1) % len(modes)]
        print(f"[Oscilloscope] Mode: {self.mode}")

    # ============================================
    # BUTTON HANDLERS
    # ============================================
    def button_run_stop(self):
        """Nút Run/Stop"""
        self.toggle_pause()

    def button_single(self):
        """Nút Single trigger"""
        pass

    def button_auto(self):
        """Nút Auto setup"""
        self.display.go_to_live()
        self.display.time_div_index = 3
        self.display.volt_div_index = 3
        print("[Oscilloscope] Auto setup")

    # ============================================
    # MAIN DISPLAY LOOP
    # ============================================
    def update_display(self) -> list:
        """
        Cập nhật và lấy dữ liệu hiển thị
        Gọi hàm này trong main loop để vẽ
        """
        return self.display.get_display_data()

    def get_screen_coords(self) -> list:
        """Lấy tọa độ màn hình để vẽ"""
        data = self.update_display()
        return self.display.convert_to_screen_coords(data)

    def get_status(self) -> dict:
        """Lấy trạng thái để hiển thị trên OSD"""
        return {
            "mode": self.mode,
            "is_paused": self.is_paused,
            "is_live": self.display.is_live,
            "time_div": self.display.time_div,
            "volt_div": self.display.volt_div,
            "scroll_offset": self.display.scroll_offset,
            "buffer_size": len(self.memory),
            "samples_read": self.reader.samples_read
        }


# ============================================
# PYGAME DISPLAY (Optional)
# ============================================
def run_pygame_display(scope: Oscilloscope):
    """Chạy display với Pygame"""
    try:
        import pygame
    except ImportError:
        print("[Display] Pygame không được cài đặt")
        return

    pygame.init()
    screen = pygame.display.set_mode((ScopeConfig.SCREEN_WIDTH, ScopeConfig.SCREEN_HEIGHT))
    pygame.display.set_caption("Oscilloscope - Ring Buffer")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 24)

    # Colors
    BLACK = (0, 0, 0)
    GREEN = (0, 255, 0)
    DARK_GREEN = (0, 100, 0)
    WHITE = (255, 255, 255)
    YELLOW = (255, 255, 0)
    RED = (255, 0, 0)

    running = True
    while running:
        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    scope.toggle_pause()
                elif event.key == pygame.K_LEFT:
                    scope.display.scroll_backward(500)
                elif event.key == pygame.K_RIGHT:
                    scope.display.scroll_forward(500)
                elif event.key == pygame.K_UP:
                    scope.display.decrease_time_div()
                elif event.key == pygame.K_DOWN:
                    scope.display.increase_time_div()
                elif event.key == pygame.K_HOME:
                    scope.display.go_to_live()
                elif event.key == pygame.K_m:
                    scope.encoder_button()
                elif event.key == pygame.K_ESCAPE:
                    running = False

        # Clear screen
        screen.fill(BLACK)

        # Draw grid
        for i in range(0, ScopeConfig.SCREEN_WIDTH, ScopeConfig.SCREEN_WIDTH // 10):
            pygame.draw.line(screen, DARK_GREEN, (i, 0), (i, ScopeConfig.SCREEN_HEIGHT), 1)
        for i in range(0, ScopeConfig.SCREEN_HEIGHT, ScopeConfig.SCREEN_HEIGHT // 8):
            pygame.draw.line(screen, DARK_GREEN, (0, i), (ScopeConfig.SCREEN_WIDTH, i), 1)

        # Draw waveform
        coords = scope.get_screen_coords()
        if len(coords) > 1:
            pygame.draw.lines(screen, GREEN, False, coords, 2)

        # Draw OSD
        status = scope.get_status()

        # Mode indicator
        mode_color = RED if status["is_paused"] else GREEN
        mode_text = "STOP" if status["is_paused"] else "RUN"
        text = font.render(f"{mode_text} | {status['mode']}", True, mode_color)
        screen.blit(text, (10, 10))

        # Time/Div
        text = font.render(f"Time: {status['time_div']} smp/div", True, WHITE)
        screen.blit(text, (10, 35))

        # Buffer info
        text = font.render(f"Buffer: {status['buffer_size']}/{ScopeConfig.MEMORY_DEPTH}", True, WHITE)
        screen.blit(text, (10, 60))

        # Scroll indicator
        if not status["is_live"]:
            text = font.render(f"HISTORY: -{status['scroll_offset']}", True, YELLOW)
            screen.blit(text, (ScopeConfig.SCREEN_WIDTH // 2 - 50, 10))
        else:
            text = font.render("LIVE", True, GREEN)
            screen.blit(text, (ScopeConfig.SCREEN_WIDTH // 2 - 20, 10))

        # Help
        help_text = "SPACE:Run/Stop | Arrows:Scroll/Zoom | HOME:Live | M:Mode | ESC:Quit"
        text = font.render(help_text, True, (100, 100, 100))
        screen.blit(text, (10, ScopeConfig.SCREEN_HEIGHT - 25))

        pygame.display.flip()
        clock.tick(ScopeConfig.FPS)

    pygame.quit()


# ============================================
# CONSOLE DISPLAY (Fallback)
# ============================================
def run_console_display(scope: Oscilloscope):
    """Chạy display trên console (fallback)"""
    print("\n" + "="*60)
    print("OSCILLOSCOPE - Console Mode")
    print("="*60)
    print("Commands: p=Pause, r=Resume, l=Live, +/-=Zoom, [/]=Scroll, q=Quit")
    print("="*60 + "\n")

    while scope.is_running:
        try:
            status = scope.get_status()
            data = scope.update_display()

            # Simple ASCII display
            if data:
                min_val = min(data)
                max_val = max(data)
                avg_val = sum(data) / len(data)

                # Status line
                mode_str = "STOP" if status["is_paused"] else "RUN "
                live_str = "LIVE" if status["is_live"] else f"HIST-{status['scroll_offset']}"

                print(f"\r[{mode_str}] {live_str} | "
                      f"Buffer: {status['buffer_size']:>6} | "
                      f"Min: {min_val:>4} Max: {max_val:>4} Avg: {avg_val:>6.1f} | "
                      f"Time: {status['time_div']:>5} smp/div", end="")

            time.sleep(0.1)

        except KeyboardInterrupt:
            print("\n\nĐã nhận Ctrl+C, đang dừng...")
            break

    print("\n[Console] Đã dừng display")


# ============================================
# MAIN ENTRY POINT
# ============================================
def main():
    """Main function"""
    print("\n" + "="*60)
    print("  OSCILLOSCOPE with Ring Buffer Processing")
    print("  Memory Depth: {:,} samples".format(ScopeConfig.MEMORY_DEPTH))
    print("="*60 + "\n")

    # Khởi tạo oscilloscope
    scope = Oscilloscope()

    try:
        # Start oscilloscope
        scope.start()

        # Chọn display mode
        try:
            import pygame
            print("[Main] Sử dụng Pygame display")
            run_pygame_display(scope)
        except ImportError:
            print("[Main] Pygame không có, sử dụng Console display")
            run_console_display(scope)

    except KeyboardInterrupt:
        print("\n[Main] Đã nhận Ctrl+C")

    finally:
        scope.stop()
        print("[Main] Oscilloscope đã tắt")


# ============================================
# EXAMPLE USAGE
# ============================================
if __name__ == "__main__":
    main()


# ============================================
# API REFERENCE
# ============================================
"""
CÁCH SỬ DỤNG:

1. Khởi tạo:
   scope = Oscilloscope()
   scope.start()

2. Điều khiển:
   scope.pause()           # Dừng để xem lịch sử
   scope.resume()          # Tiếp tục
   scope.toggle_pause()    # Toggle

3. Scroll lịch sử (khi đã pause):
   scope.display.scroll_backward(100)  # Lùi 100 mẫu
   scope.display.scroll_forward(100)   # Tiến 100 mẫu
   scope.display.go_to_live()          # Quay về live

4. Zoom:
   scope.display.increase_time_div()   # Zoom out (xem nhiều)
   scope.display.decrease_time_div()   # Zoom in (xem ít)

5. Lấy dữ liệu để vẽ:
   data = scope.update_display()       # Dữ liệu thô
   coords = scope.get_screen_coords()  # Tọa độ màn hình

6. Encoder:
   scope.encoder_rotate(1)   # Xoay CW
   scope.encoder_rotate(-1)  # Xoay CCW
   scope.encoder_button()    # Nhấn

7. Kết thúc:
   scope.stop()
"""

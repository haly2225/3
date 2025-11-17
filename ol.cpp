/*
 * Professional STM32 Oscilloscope - 600kHz
 * Features:
 * - AUTO TRIGGER: Smart switching between triggered/free-run
 * - NORMAL TRIGGER: Strict trigger mode
 * - FREE RUN: Continuous update
 * - Signal detection with timeout
 * - Adaptive edge learning
 * 
 * Compile: g++ scope_pro.cpp -o scope -std=c++17 $(pkg-config --cflags --libs Qt5Widgets) -fPIC
 */

#include <QApplication>
#include <QWidget>
#include <QPainter>
#include <QTimer>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QButtonGroup>
#include <QTextEdit>
#include <QKeyEvent>
#include <cstdint>
#include <vector>
#include <array>
#include <atomic>
#include <thread>
#include <mutex>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <cmath>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

constexpr uint8_t  MARKER_START = 0xAA;
constexpr uint8_t  MARKER_HEADER = 0x55;
constexpr uint16_t BUFFER_SIZE = 512;
constexpr uint16_t PACKET_SIZE = 4 + BUFFER_SIZE * 2;
constexpr float    SAMPLE_RATE = 600000.0f;
constexpr float    VCC = 3.3f;
constexpr uint16_t ADC_MAX = 4095;
constexpr size_t   CAPTURE_SIZE = 1200;  // ~2ms @ 600kHz - always collect same number of samples after edge

// Trigger modes
enum class TriggerMode {
    AUTO,      // Auto trigger if signal present, else free run
    NORMAL,    // Only trigger when condition met
    FREE_RUN   // Always free run
};

// ============================================================================
// Logger
// ============================================================================
class DebugLogger {
private:
    std::mutex log_mutex;
    std::vector<std::string> log_buffer;
    const size_t max_lines = 150;
    
public:
    void log(const std::string& msg) {
        std::lock_guard<std::mutex> lock(log_mutex);
        
        auto now = std::chrono::system_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()
        ) % 1000;
        
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&time_t);
        
        std::ostringstream oss;
        oss << std::setfill('0') 
            << std::setw(2) << tm.tm_hour << ":"
            << std::setw(2) << tm.tm_min << ":"
            << std::setw(2) << tm.tm_sec << "."
            << std::setw(3) << ms.count() << " | " << msg;
        
        log_buffer.push_back(oss.str());
        if (log_buffer.size() > max_lines) {
            log_buffer.erase(log_buffer.begin());
        }
        
        std::cout << oss.str() << std::endl;
    }
    
    std::vector<std::string> get_logs() {
        std::lock_guard<std::mutex> lock(log_mutex);
        return log_buffer;
    }
};

// ============================================================================
// Statistics
// ============================================================================
struct TimingStats {
    std::atomic<uint64_t> total_packets{0};
    std::atomic<uint64_t> good_packets{0};
    std::atomic<uint64_t> trigger_count{0};
    std::atomic<uint64_t> auto_switches{0};
    
    void reset() {
        total_packets = 0;
        good_packets = 0;
        trigger_count = 0;
        auto_switches = 0;
    }
};

struct TimingSnapshot {
    uint64_t total_packets;
    uint64_t good_packets;
    uint64_t trigger_count;
    uint64_t auto_switches;
};

// ============================================================================
// SPI Reader - Professional Oscilloscope
// ============================================================================
class SPIReader {
private:
    int spi_fd = -1;
    std::atomic<bool> running{false};
    std::thread reader_thread;
    std::mutex data_mutex;
    
    std::array<float, 8192> display_voltage;
    std::array<float, 8192> display_time;
    std::atomic<size_t> display_size{0};
    
    std::atomic<bool> new_data{false};
    std::atomic<float> fps{0.0f};
    std::atomic<bool> signal_present{false};
    std::atomic<TriggerMode> trigger_mode{TriggerMode::AUTO};
    
    enum class State { IDLE, LEARNING, TRIGGERED, COLLECTING, HOLDOFF };
    State state = State::IDLE;
    
    // Timing
    std::chrono::steady_clock::time_point last_trigger_time;
    std::chrono::steady_clock::time_point last_signal_time;
    std::chrono::steady_clock::time_point last_auto_switch_time;
    const int HOLDOFF_MS = 50;
    const int SIGNAL_TIMEOUT_MS = 300;
    const int AUTO_FALLBACK_MS = 200;  // Switch to free run if no trigger
    
    // Capture
    std::array<float, 8192> temp_buffer;
    size_t temp_size = 0;
    size_t collect_count = 0;

    // Circular buffer for pre-trigger data (keeps last 4 buffers = 2048 samples)
    static constexpr int HISTORY_BUFFERS = 4;
    std::array<std::array<float, BUFFER_SIZE>, HISTORY_BUFFERS> buffer_history;
    int history_write_pos = 0;
    int buffers_in_history = 0;

    // Trigger consistency - store first edge position to ensure all buffers match
    int first_edge_position = -1;

    // Learning
    std::vector<int> edge_history;
    int stable_edge_offset = -1;
    int learn_count = 0;
    const int LEARN_FRAMES = 15;
    uint16_t first_trigger_frame = 0;
    bool phase_locked = false;
    bool frame_initialized = false;
    
    // Signal detection
    float last_vpp = 0.0f;
    const float TRIGGER_THRESHOLD = 2.0f;
    bool was_triggered = false;

    // Trigger level adjustment (like real oscilloscope)
    float trigger_level = 1.65f;  // Default: mid-point of 0-3.3V
    const float TRIGGER_LEVEL_STEP = 0.05f;  // 50mV steps
    int trigger_level_changes = 0;  // Count changes for periodic logging
    
    std::chrono::steady_clock::time_point last_packet_time;
    bool first_packet_received = false;
    
    DebugLogger logger;
    TimingStats stats;

public:
    SPIReader() {
        display_voltage.fill(0);
        display_time.fill(0);
        temp_buffer.fill(0);
        for (auto& buf : buffer_history) {
            buf.fill(0);
        }
    }
    
    ~SPIReader() { stop(); }

    // Trigger level adjustment (like real oscilloscope)
    void increaseTriggerLevel() {
        trigger_level = std::min(trigger_level + TRIGGER_LEVEL_STEP, 3.3f);
        trigger_level_changes++;

        // Only log every 4 changes (0.2V) or at max boundary
        if (trigger_level >= 3.3f || trigger_level_changes % 4 == 0) {
            std::ostringstream oss;
            oss << "🎯 Trigger Level: " << std::fixed << std::setprecision(2) << trigger_level << "V";
            logger.log(oss.str());
        }
    }

    void decreaseTriggerLevel() {
        trigger_level = std::max(trigger_level - TRIGGER_LEVEL_STEP, 0.0f);
        trigger_level_changes++;

        // Only log every 4 changes (0.2V) or at min boundary
        if (trigger_level <= 0.0f || trigger_level_changes % 4 == 0) {
            std::ostringstream oss;
            oss << "🎯 Trigger Level: " << std::fixed << std::setprecision(2) << trigger_level << "V";
            logger.log(oss.str());
        }
    }

    float getTriggerLevel() const { return trigger_level; }

    bool init() {
        logger.log("🔧 Init SPI...");
        
        spi_fd = open("/dev/spidev0.0", O_RDWR);
        if (spi_fd < 0) {
            logger.log("❌ Cannot open SPI");
            return false;
        }
        
        uint8_t mode = SPI_MODE_0;
        uint8_t bits = 8;
        uint32_t speed = 16000000;
        
        if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0 ||
            ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
            ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
            logger.log("❌ Config failed");
            close(spi_fd);
            return false;
        }
        
        logger.log("✅ SPI OK: 16MHz");
        return true;
    }
    
    void start() {
        logger.log("▶️  Start");
        running = true;
        stats.reset();
        signal_present = false;
        reader_thread = std::thread(&SPIReader::reader_loop, this);
    }
    
    void stop() {
        logger.log("⏹️  Stop");
        running = false;
        if (reader_thread.joinable()) reader_thread.join();
        if (spi_fd >= 0) { close(spi_fd); spi_fd = -1; }
    }
    
    bool get_data(std::vector<float>& volt, std::vector<float>& time) {
        if (!new_data.load()) return false;
        
        std::lock_guard<std::mutex> lock(data_mutex);
        size_t n = display_size.load();
        if (n == 0) return false;
        
        volt.clear();
        time.clear();
        volt.reserve(n);
        time.reserve(n);
        
        for (size_t i = 0; i < n; i++) {
            volt.push_back(display_voltage[i]);
            time.push_back(display_time[i]);
        }
        
        new_data = false;
        return true;
    }
    
    TimingSnapshot get_stats() const {
        TimingSnapshot snap;
        snap.total_packets = stats.total_packets.load();
        snap.good_packets = stats.good_packets.load();
        snap.trigger_count = stats.trigger_count.load();
        snap.auto_switches = stats.auto_switches.load();
        return snap;
    }
    
    std::vector<std::string> get_logs() {
        return logger.get_logs();
    }
    
    float get_fps() const { return fps.load(); }
    bool has_signal() const { return signal_present.load(); }
    float get_vpp() const { return last_vpp; }
    TriggerMode get_mode() const { return trigger_mode.load(); }
    
    void set_trigger_mode(TriggerMode mode) {
        trigger_mode = mode;
        // Don't reset state - allow seamless mode switching!

        std::string mode_str;
        switch(mode) {
            case TriggerMode::AUTO: mode_str = "🤖 AUTO"; break;
            case TriggerMode::NORMAL: mode_str = "🎯 NORMAL"; break;
            case TriggerMode::FREE_RUN: mode_str = "🔄 FREE RUN"; break;
        }
        logger.log(mode_str + " MODE");
    }

private:
    void reader_loop() {
        uint32_t frame_count = 0;
        auto last_fps_time = std::chrono::steady_clock::now();
        
        std::vector<uint8_t> rx_buf(PACKET_SIZE);
        std::vector<uint8_t> tx_buf(PACKET_SIZE, 0x00);
        
        logger.log("🔁 Loop start");
        last_signal_time = std::chrono::steady_clock::now();
        last_auto_switch_time = last_signal_time;
        
        while (running) {
            struct spi_ioc_transfer tr{};
            tr.tx_buf = reinterpret_cast<uint64_t>(tx_buf.data());
            tr.rx_buf = reinterpret_cast<uint64_t>(rx_buf.data());
            tr.len = PACKET_SIZE;
            tr.speed_hz = 16000000;
            tr.bits_per_word = 8;
            
            if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
                usleep(1000);
                continue;
            }
            
            usleep(600);
            stats.total_packets++;
            
            auto now = std::chrono::steady_clock::now();
            
            if (parse_packet(rx_buf)) {
                stats.good_packets++;
                frame_count++;
            }
            
            // Signal timeout check
            auto signal_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_signal_time
            ).count();
            
            if (signal_elapsed > SIGNAL_TIMEOUT_MS) {
                if (signal_present.load()) {
                    signal_present = false;
                    logger.log("⚠️  SIGNAL LOST");
                    
                    // Clear display
                    std::lock_guard<std::mutex> lock(data_mutex);
                    display_size = 0;
                    new_data = true;
                }
            }
            
            // FPS calculation
            auto fps_elapsed = std::chrono::duration<float>(now - last_fps_time).count();
            if (fps_elapsed >= 1.0f) {
                fps = frame_count / fps_elapsed;
                frame_count = 0;
                last_fps_time = now;
            }
        }
        
        logger.log("🔁 Exit");
    }
    
    bool parse_packet(const std::vector<uint8_t>& buf) {
        // Find marker
        int marker_pos = -1;
        for (size_t i = 0; i <= buf.size() - PACKET_SIZE; i++) {
            if (buf[i] == MARKER_START && buf[i+1] == MARKER_HEADER) {
                marker_pos = i;
                break;
            }
        }

        if (marker_pos < 0) return false;

        uint16_t frame_num = (static_cast<uint16_t>(buf[marker_pos + 2]) << 8) |
                             buf[marker_pos + 3];

        if (!frame_initialized) {
            frame_initialized = true;
            logger.log("📍 Sync OK");
        }

        // Parse ADC
        std::array<uint16_t, BUFFER_SIZE> samples;
        for (size_t i = 0; i < BUFFER_SIZE; i++) {
            size_t offset = marker_pos + 4 + i * 2;
            uint16_t val = (static_cast<uint16_t>(buf[offset]) << 8) | buf[offset + 1];
            if (val > ADC_MAX) return false;
            samples[i] = val;
        }

        // Convert to voltage
        std::array<float, BUFFER_SIZE> voltage;
        for (size_t i = 0; i < BUFFER_SIZE; i++) {
            voltage[i] = samples[i] * (VCC / ADC_MAX);
        }

        // Store in circular buffer history for continuous data
        buffer_history[history_write_pos] = voltage;
        history_write_pos = (history_write_pos + 1) % HISTORY_BUFFERS;
        if (buffers_in_history < HISTORY_BUFFERS) {
            buffers_in_history++;
        }
        
        // Measure Vpp
        float vmax = 0, vmin = 3.3f;
        for (size_t i = 0; i < BUFFER_SIZE; i++) {
            if (voltage[i] > vmax) vmax = voltage[i];
            if (voltage[i] < vmin) vmin = voltage[i];
        }
        float vpp = vmax - vmin;
        last_vpp = vpp;

        // DEBUG: Log signal measurements every 100 frames
        static int debug_counter = 0;
        if (debug_counter++ % 100 == 0) {
            std::ostringstream oss;
            oss << "📊 Vpp=" << std::fixed << std::setprecision(2) << vpp
                << "V, Max=" << vmax << "V, Min=" << vmin << "V, Frame=" << frame_num;
            logger.log(oss.str());
        }
        
        // Signal detection
        if (vpp > 0.5f) {
            signal_present = true;
            last_signal_time = std::chrono::steady_clock::now();
        }
        
        // Route to appropriate handler
        TriggerMode mode = trigger_mode.load();
        
        if (mode == TriggerMode::FREE_RUN) {
            return handle_free_run(voltage);
        } else if (mode == TriggerMode::NORMAL) {
            return handle_normal_trigger(voltage, frame_num, vpp);
        } else { // AUTO
            return handle_auto_trigger(voltage, frame_num, vpp);
        }
    }
    
    bool handle_free_run(const std::array<float, BUFFER_SIZE>& voltage) {
        std::lock_guard<std::mutex> lock(data_mutex);
        for (size_t i = 0; i < BUFFER_SIZE; i++) {
            display_voltage[i] = voltage[i];
            display_time[i] = static_cast<float>(i) / SAMPLE_RATE;
        }
        display_size = BUFFER_SIZE;
        new_data = true;
        return true;
    }
    
    bool handle_auto_trigger(const std::array<float, BUFFER_SIZE>& voltage, 
                            uint16_t frame_num, float vpp) {
        auto now = std::chrono::steady_clock::now();
        
        // If signal strong enough, try to trigger
        if (vpp > TRIGGER_THRESHOLD) {
            bool triggered = handle_normal_trigger(voltage, frame_num, vpp);
            
            if (triggered) {
                was_triggered = true;
                last_auto_switch_time = now;
                return true;
            }
        }
        
        // AUTO FALLBACK: If no successful trigger for AUTO_FALLBACK_MS, switch to free run
        auto since_last_trigger = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_auto_switch_time
        ).count();
        
        if (since_last_trigger > AUTO_FALLBACK_MS && state != State::COLLECTING) {
            // Switch to free run temporarily
            if (was_triggered) {
                logger.log("🔄 AUTO → Free run (no trigger)");
                stats.auto_switches++;
                was_triggered = false;
            }
            
            return handle_free_run(voltage);
        }
        
        return false;
    }
    
    // Extract samples from circular buffer history
    // offset: how many samples back from the most recent sample (can be negative for future)
    // count: how many samples to extract
    bool extract_from_history(int offset_from_latest, size_t count) {
        if (buffers_in_history == 0) return false;

        temp_size = 0;

        // Current buffer position (most recently written)
        int current_buf = (history_write_pos - 1 + HISTORY_BUFFERS) % HISTORY_BUFFERS;

        // Start from (BUFFER_SIZE - 1 - offset_from_latest) in the history
        // offset_from_latest=0 means most recent sample
        // offset_from_latest=BUFFER_SIZE means one buffer back

        int total_available = buffers_in_history * BUFFER_SIZE;
        if (offset_from_latest + (int)count > total_available) {
            return false;  // Not enough history
        }

        int samples_extracted = 0;
        int current_offset = offset_from_latest;

        while (samples_extracted < (int)count && temp_size < temp_buffer.size()) {
            // Which buffer and which sample within that buffer?
            int buffer_idx = current_offset / BUFFER_SIZE;
            int sample_idx = current_offset % BUFFER_SIZE;

            // Get the actual buffer index in circular buffer (going backwards from current)
            int actual_buf_idx = (current_buf - buffer_idx + HISTORY_BUFFERS * 10) % HISTORY_BUFFERS;

            // Extract sample
            temp_buffer[temp_size++] = buffer_history[actual_buf_idx][BUFFER_SIZE - 1 - sample_idx];
            samples_extracted++;
            current_offset++;
        }

        return temp_size == count;
    }

    bool handle_normal_trigger(const std::array<float, BUFFER_SIZE>& voltage,
                               uint16_t frame_num, float vpp) {
        auto now = std::chrono::steady_clock::now();

        switch (state) {
            case State::HOLDOFF: {
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - last_trigger_time
                ).count();

                if (elapsed >= HOLDOFF_MS) {
                    state = State::IDLE;
                }
                return false;
            }

            case State::IDLE: {
                // Find rising edge and trigger
                if (vpp > TRIGGER_THRESHOLD && buffers_in_history >= 2) {
                    int edge_idx = find_simple_edge(voltage);

                    if (edge_idx >= 0) {
                        stats.trigger_count++;

                        // Log only every 20 triggers to reduce spam
                        static int trigger_log_count = 0;
                        if (++trigger_log_count % 20 == 1) {
                            std::ostringstream oss;
                            oss << "🎯 TRIGGER! edge=" << edge_idx
                                << ", vpp=" << std::fixed << std::setprecision(2) << vpp << "V";
                            logger.log(oss.str());
                        }

                        // Start collecting from edge position
                        temp_size = 0;

                        // Copy from edge position to end of current buffer
                        for (int i = edge_idx; i < BUFFER_SIZE; i++) {
                            temp_buffer[temp_size++] = voltage[i];
                        }

                        // If we have enough data already, display it
                        if (temp_size >= CAPTURE_SIZE) {
                            std::lock_guard<std::mutex> lock(data_mutex);
                            for (size_t i = 0; i < temp_size; i++) {
                                display_voltage[i] = temp_buffer[i];
                                display_time[i] = static_cast<float>(i) / SAMPLE_RATE;
                            }
                            display_size = temp_size;
                            new_data = true;
                            state = State::HOLDOFF;
                            last_trigger_time = now;
                            return true;
                        }

                        // Need more data - transition to COLLECTING
                        state = State::COLLECTING;
                        collect_count = 1;
                        return false;
                    }
                }
                return false;
            }

            case State::LEARNING:  // No longer used, but keep for compatibility
            case State::TRIGGERED:
            case State::COLLECTING: {
                // Continue collecting from subsequent buffers to reach CAPTURE_SIZE
                if (vpp > TRIGGER_THRESHOLD) {
                    // Collect ENTIRE buffer for continuous timeline
                    for (size_t i = 0; i < BUFFER_SIZE && temp_size < CAPTURE_SIZE; i++) {
                        temp_buffer[temp_size++] = voltage[i];
                    }
                    collect_count++;
                } else {
                    // Signal lost - abort and go back to IDLE
                    state = State::IDLE;
                    return false;
                }

                // Check if we have enough data
                if (temp_size >= CAPTURE_SIZE) {
                    std::lock_guard<std::mutex> lock(data_mutex);

                    size_t copy_size = std::min(temp_size, CAPTURE_SIZE);
                    for (size_t i = 0; i < copy_size; i++) {
                        display_voltage[i] = temp_buffer[i];
                        display_time[i] = static_cast<float>(i) / SAMPLE_RATE;
                    }
                    display_size = copy_size;
                    new_data = true;

                    state = State::HOLDOFF;
                    last_trigger_time = now;
                    return true;
                }

                return false;
            }
        }

        return false;
    }
    
    // Simple edge detection - like a real oscilloscope!
    // Find edge - like real oscilloscope!
    int find_simple_edge(const std::array<float, BUFFER_SIZE>& voltage) {
        // Search window for good trigger rate, return ACTUAL edge position for zero jitter
        // Position: 20% of buffer (102) ±2 samples
        const int TARGET = BUFFER_SIZE / 5;  // 102
        const int WINDOW = 2;  // ±2 samples

        // Check for rising edge in window 100-104
        for (int i = TARGET - WINDOW; i <= TARGET + WINDOW; i++) {
            if (voltage[i - 1] < trigger_level && voltage[i] >= trigger_level) {
                // Return ACTUAL edge position (not TARGET)
                // This ensures edge always appears at same display position (start of waveform)
                return i;
            }
        }

        return -1;  // No edge in window
    }

    // Old edge detection (for reference, not used)
    int find_edge(const std::array<float, BUFFER_SIZE>& voltage) {
        int candidates = 0;
        int rejected_pre = 0;
        int rejected_post = 0;

        for (size_t i = 20; i < BUFFER_SIZE - 20; i++) {
            if (voltage[i-1] < 1.0f && voltage[i] > 2.3f) {
                candidates++;
                // Verify clean
                bool clean = true;
                for (int j = -5; j < 0; j++) {
                    if (voltage[i+j] > 1.2f) {
                        clean = false;
                        rejected_pre++;
                        break;
                    }
                }
                if (clean) {
                    for (int j = 1; j <= 5; j++) {
                        if (voltage[i+j] < 2.0f) {
                            clean = false;
                            rejected_post++;
                            break;
                        }
                    }
                }
                if (clean) {
                    std::ostringstream oss;
                    oss << "✅ Edge found at i=" << i << " (candidates=" << candidates << ")";
                    logger.log(oss.str());
                    return i;
                }
            }
        }

        if (candidates > 0) {
            std::ostringstream oss;
            oss << "❌ No clean edge. Candidates=" << candidates
                << ", rejected_pre=" << rejected_pre
                << ", rejected_post=" << rejected_post;
            logger.log(oss.str());
        }
        return -1;
    }
    
    int find_edge_near(const std::array<float, BUFFER_SIZE>& voltage, int expected) {
        // Widen search window due to high jitter (seen 20-79 range in learning)
        int start = std::max(20, expected - 60);
        int end = std::min((int)BUFFER_SIZE - 20, expected + 60);
        int candidates = 0;
        int best_edge = -1;
        int best_distance = 999;

        // First try: look for rising edge 0V → 3.3V anywhere in window
        for (int i = start; i < end; i++) {
            if (voltage[i-1] < 1.65f && voltage[i] >= 1.65f) {  // Simple threshold crossing
                candidates++;
                int distance = std::abs(i - expected);
                if (distance < best_distance) {
                    best_edge = i;
                    best_distance = distance;
                }
            }
        }

        if (best_edge >= 0) {
            std::ostringstream oss;
            oss << "🔍 find_edge_near: Found at " << best_edge
                << " (expected=" << expected << ", delta=" << (best_edge - expected)
                << ", candidates=" << candidates << ")";
            logger.log(oss.str());
            return best_edge;
        }

        static int fallback_count = 0;
        if (++fallback_count % 20 == 0) {
            std::ostringstream oss;
            oss << "⚠️  find_edge_near FAIL: expected=" << expected
                << ", range=[" << start << "," << end << "], candidates=" << candidates;
            logger.log(oss.str());
        }
        return -1;
    }
};

// ============================================================================
// Display
// ============================================================================
class ScopeDisplay : public QWidget {
    Q_OBJECT
    
private:
    std::vector<float> voltage;
    std::vector<float> time;
    float time_div = 0.001f;
    float volt_div = 0.5f;
    TriggerMode trigger_mode = TriggerMode::AUTO;
    float trigger_level = 1.65f;  // Display trigger level line

public:
    ScopeDisplay(QWidget *parent = nullptr) : QWidget(parent) {
        setMinimumSize(800, 600);
    }

    void update_waveform(const std::vector<float>& v, const std::vector<float>& t) {
        voltage = v;
        time = t;
        update();
    }

    void set_time_div(float div) { time_div = div; update(); }
    void set_volt_div(float div) { volt_div = div; update(); }
    void set_trigger_mode(TriggerMode mode) { trigger_mode = mode; update(); }
    void set_trigger_level(float level) { trigger_level = level; update(); }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        
        int w = width();
        int h = height();
        int margin = 50;
        int grid_w = w - 2 * margin;
        int grid_h = h - 2 * margin;
        
        p.fillRect(0, 0, w, h, QColor(15, 15, 15));
        
        // Grid
        p.setPen(QPen(QColor(0, 100, 0), 1));
        for (int i = 0; i <= 10; i++) {
            int x = margin + i * grid_w / 10;
            p.drawLine(x, margin, x, margin + grid_h);
        }
        for (int i = 0; i <= 8; i++) {
            int y = margin + i * grid_h / 8;
            p.drawLine(margin, y, margin + grid_w, y);
        }
        
        // Center lines
        p.setPen(QPen(QColor(0, 180, 0), 2));
        int cx = margin + grid_w / 2;
        int cy = margin + grid_h / 2;
        p.drawLine(cx, margin, cx, margin + grid_h);
        p.drawLine(margin, cy, margin + grid_w, cy);
        
        // Trigger marker (vertical line at left edge)
        if (trigger_mode != TriggerMode::FREE_RUN) {
            p.setPen(QPen(QColor(255, 140, 0), 3));
            p.drawLine(margin, margin, margin, margin + grid_h);
        }

        // Trigger level line (horizontal line)
        if (trigger_mode != TriggerMode::FREE_RUN) {
            float v_center = VCC / 2.0f;
            float v_range = volt_div * 8.0f;
            int trig_y = cy - static_cast<int>((trigger_level - v_center) / v_range * grid_h);

            // Draw trigger level line
            p.setPen(QPen(QColor(255, 140, 0), 2, Qt::DashLine));
            p.drawLine(margin, trig_y, margin + grid_w, trig_y);

            // Draw trigger arrow pointing to level
            p.setBrush(QColor(255, 140, 0));
            QPolygon arrow;
            arrow << QPoint(margin - 15, trig_y)
                  << QPoint(margin - 5, trig_y - 8)
                  << QPoint(margin - 5, trig_y + 8);
            p.drawPolygon(arrow);

            // Draw trigger level text
            p.setPen(QColor(255, 140, 0));
            p.setFont(QFont("Monospace", 11, QFont::Bold));
            p.drawText(margin + grid_w + 5, trig_y + 5,
                       QString("%1V").arg(trigger_level, 0, 'f', 2));
        }
        
        // Waveform
        if (!voltage.empty() && voltage.size() > 10) {
            // Set clipping region to prevent drawing outside grid
            p.setClipRect(margin, margin, grid_w, grid_h);

            p.setPen(QPen(QColor(255, 220, 0), 2));

            float t_window = time_div * 10.0f;
            float v_center = VCC / 2.0f;
            float v_range = volt_div * 8.0f;

            for (size_t i = 0; i < voltage.size() - 1; i++) {
                float t1 = time[i];
                float t2 = time[i + 1];

                // Skip if both points are completely outside the time window
                if (t2 < 0 || t1 > t_window) continue;

                // Clamp time values to window bounds to prevent drawing outside
                float t1_clamped = std::max(0.0f, std::min(t1, t_window));
                float t2_clamped = std::max(0.0f, std::min(t2, t_window));

                int x1 = margin + static_cast<int>((t1_clamped / t_window) * grid_w);
                int x2 = margin + static_cast<int>((t2_clamped / t_window) * grid_w);
                int y1 = cy - static_cast<int>((voltage[i] - v_center) / v_range * grid_h);
                int y2 = cy - static_cast<int>((voltage[i+1] - v_center) / v_range * grid_h);

                // Additional safety: clamp x coordinates to grid bounds
                x1 = std::max(margin, std::min(x1, margin + grid_w));
                x2 = std::max(margin, std::min(x2, margin + grid_w));

                p.drawLine(x1, y1, x2, y2);
            }

            // Reset clipping for subsequent drawing
            p.setClipping(false);
            
            // Measurements
            float vmin = *std::min_element(voltage.begin(), voltage.end());
            float vmax = *std::max_element(voltage.begin(), voltage.end());
            
            p.setPen(QColor(255, 255, 100));
            p.setFont(QFont("Monospace", 11, QFont::Bold));
            p.drawText(10, 30, QString("N: %1").arg(voltage.size()));
            p.drawText(10, 50, QString("Vpp: %1V").arg(vmax - vmin, 0, 'f', 2));
            p.drawText(10, 70, QString("Max: %1V").arg(vmax, 0, 'f', 2));
            p.drawText(10, 90, QString("Min: %1V").arg(vmin, 0, 'f', 2));
        } else {
            // No signal
            p.setPen(QColor(150, 150, 150));
            p.setFont(QFont("Arial", 16, QFont::Bold));
            p.drawText(rect(), Qt::AlignCenter, "NO SIGNAL");
        }
        
        // Scale info
        p.setPen(QColor(200, 200, 200));
        p.setFont(QFont("Monospace", 12, QFont::Bold));
        QString time_str = (time_div >= 0.001f) 
            ? QString("%1ms/div").arg(time_div * 1000, 0, 'f', 1)
            : QString("%1µs/div").arg(time_div * 1e6, 0, 'f', 0);
        p.drawText(margin, h - 15, time_str);
        p.drawText(margin + 200, h - 15, QString("%1V/div").arg(volt_div, 0, 'f', 2));
    }
};

// ============================================================================
// Main Window
// ============================================================================
class MainWindow : public QWidget {
    Q_OBJECT
    
private:
    SPIReader reader;
    ScopeDisplay *display;
    QLabel *stats_label;
    QTextEdit *debug_log;
    QTimer *timer;
    QButtonGroup *trigger_group;

    // Debounce for trigger level adjustment
    std::chrono::steady_clock::time_point last_trigger_adjust;
    const int TRIGGER_ADJUST_DEBOUNCE_MS = 100;  // Minimum 100ms between adjustments

public:
    MainWindow() : last_trigger_adjust(std::chrono::steady_clock::now() - std::chrono::seconds(1)) {
        setWindowTitle("Professional Oscilloscope - AUTO TRIGGER");
        resize(1400, 900);

        QVBoxLayout *main_layout = new QVBoxLayout(this);
        QHBoxLayout *top = new QHBoxLayout();
        
        display = new ScopeDisplay();
        top->addWidget(display, 3);
        
        // Control Panel
        QWidget *panel = new QWidget();
        panel->setStyleSheet("background-color: #2a2a2a;");
        panel->setFixedWidth(220);
        
        QVBoxLayout *panel_layout = new QVBoxLayout(panel);
        panel_layout->setSpacing(10);
        panel_layout->setContentsMargins(10, 10, 10, 10);
        
        // Trigger Mode
        QLabel *trig_label = new QLabel("TRIGGER MODE");
        trig_label->setStyleSheet("color: #ff8800; font-weight: bold; font-size: 14px;");
        panel_layout->addWidget(trig_label);
        
        trigger_group = new QButtonGroup(this);
        
        QPushButton *auto_btn = new QPushButton("🤖 AUTO");
        auto_btn->setCheckable(true);
        auto_btn->setChecked(true);
        auto_btn->setStyleSheet(
            "QPushButton { background-color: #444; color: white; padding: 8px; font-weight: bold; }"
            "QPushButton:checked { background-color: #ff8800; color: black; }"
        );
        trigger_group->addButton(auto_btn, 0);
        panel_layout->addWidget(auto_btn);
        
        QPushButton *normal_btn = new QPushButton("🎯 NORMAL");
        normal_btn->setCheckable(true);
        normal_btn->setStyleSheet(
            "QPushButton { background-color: #444; color: white; padding: 8px; font-weight: bold; }"
            "QPushButton:checked { background-color: #ff8800; color: black; }"
        );
        trigger_group->addButton(normal_btn, 1);
        panel_layout->addWidget(normal_btn);
        
        QPushButton *free_btn = new QPushButton("🔄 FREE RUN");
        free_btn->setCheckable(true);
        free_btn->setStyleSheet(
            "QPushButton { background-color: #444; color: white; padding: 8px; font-weight: bold; }"
            "QPushButton:checked { background-color: #ff8800; color: black; }"
        );
        trigger_group->addButton(free_btn, 2);
        panel_layout->addWidget(free_btn);
        
        connect(trigger_group, QOverload<int>::of(&QButtonGroup::idClicked),
                this, &MainWindow::change_trigger_mode);
        
        panel_layout->addSpacing(20);
        
        // Time/Div
        QLabel *time_label = new QLabel("TIME/DIV");
        time_label->setStyleSheet("color: #00ff00; font-weight: bold;");
        panel_layout->addWidget(time_label);
        
        std::vector<std::pair<QString, float>> time_scales = {
            {"100µs", 0.0001f}, {"200µs", 0.0002f}, {"500µs", 0.0005f},
            {"1ms", 0.001f}, {"2ms", 0.002f}, {"5ms", 0.005f}
        };
        
        for (const auto& [name, val] : time_scales) {
            QPushButton *btn = new QPushButton(name);
            btn->setStyleSheet("background-color: #555; color: white; padding: 6px;");
            connect(btn, &QPushButton::clicked, [this, val]() { 
                display->set_time_div(val); 
            });
            panel_layout->addWidget(btn);
        }
        
        panel_layout->addSpacing(15);
        
        // Volts/Div
        QLabel *volt_label = new QLabel("VOLTS/DIV");
        volt_label->setStyleSheet("color: #ffff00; font-weight: bold;");
        panel_layout->addWidget(volt_label);
        
        std::vector<std::pair<QString, float>> volt_scales = {
            {"0.5V", 0.5f}, {"1.0V", 1.0f}, {"2.0V", 2.0f}
        };
        
        for (const auto& [name, val] : volt_scales) {
            QPushButton *btn = new QPushButton(name);
            btn->setStyleSheet("background-color: #555; color: white; padding: 6px;");
            connect(btn, &QPushButton::clicked, [this, val]() { 
                display->set_volt_div(val); 
            });
            panel_layout->addWidget(btn);
        }
        
        panel_layout->addStretch();
        
        // Stats
        stats_label = new QLabel();
        stats_label->setStyleSheet("color: #00ffff; font-size: 9px; font-family: monospace;");
        stats_label->setWordWrap(true);
        panel_layout->addWidget(stats_label);
        
        top->addWidget(panel);
        main_layout->addLayout(top, 3);
        
        // Debug log
        QLabel *debug_title = new QLabel("📊 DEBUG LOG:");
        debug_title->setStyleSheet("color: #ffff00; font-weight: bold; font-size: 12px;");
        main_layout->addWidget(debug_title);
        
        debug_log = new QTextEdit();
        debug_log->setReadOnly(true);
        debug_log->setStyleSheet(
            "background-color: #1a1a1a; color: #00ff00; "
            "font-family: monospace; font-size: 10px;"
        );
        debug_log->setMaximumHeight(200);
        main_layout->addWidget(debug_log, 1);
        
        if (!reader.init()) {
            stats_label->setText("❌ INIT FAILED");
            return;
        }
        
        reader.start();
        
        timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, &MainWindow::update_display);
        timer->start(50);
    }
    
    ~MainWindow() {
        reader.stop();
    }

private slots:
    void update_display() {
        std::vector<float> volt, time_vec;
        if (reader.get_data(volt, time_vec)) {
            display->update_waveform(volt, time_vec);
        }

        // Update trigger level display
        display->set_trigger_level(reader.getTriggerLevel());
        
        TimingSnapshot snap = reader.get_stats();
        float fps_val = reader.get_fps();
        float vpp = reader.get_vpp();
        bool has_sig = reader.has_signal();
        
        float success = (snap.total_packets > 0) 
            ? (100.0f * snap.good_packets / snap.total_packets) : 0;
        
        QString mode_str;
        switch(reader.get_mode()) {
            case TriggerMode::AUTO: mode_str = "AUTO"; break;
            case TriggerMode::NORMAL: mode_str = "NORMAL"; break;
            case TriggerMode::FREE_RUN: mode_str = "FREE RUN"; break;
        }
        
        float trig_level = reader.getTriggerLevel();

        QString stats_text = QString(
            "MODE: %1\n"
            "FPS: %2\n"
            "Vpp: %3V\n"
            "Signal: %4\n\n"
            "🎯 Trigger: %5V\n"
            "(↑↓ to adjust)\n\n"
            "Total: %6\n"
            "Good: %7\n"
            "OK: %8%%\n\n"
            "Triggers: %9\n"
            "Auto↔: %10"
        ).arg(mode_str)
         .arg(fps_val, 0, 'f', 1)
         .arg(vpp, 0, 'f', 2)
         .arg(has_sig ? "✅" : "❌")
         .arg(trig_level, 0, 'f', 2)
         .arg(snap.total_packets)
         .arg(snap.good_packets)
         .arg(success, 0, 'f', 1)
         .arg(snap.trigger_count)
         .arg(snap.auto_switches);
        
        stats_label->setText(stats_text);
        
        auto logs = reader.get_logs();
        QString log_text;
        for (const auto& line : logs) {
            log_text += QString::fromStdString(line) + "\n";
        }
        debug_log->setPlainText(log_text);
        debug_log->moveCursor(QTextCursor::End);
    }
    
    void change_trigger_mode(int id) {
        TriggerMode mode;
        switch(id) {
            case 0: mode = TriggerMode::AUTO; break;
            case 1: mode = TriggerMode::NORMAL; break;
            case 2: mode = TriggerMode::FREE_RUN; break;
            default: mode = TriggerMode::AUTO;
        }
        reader.set_trigger_mode(mode);
        display->set_trigger_mode(mode);
    }

protected:
    void keyPressEvent(QKeyEvent *event) override {
        // Only process trigger level keys with debouncing
        if (event->key() != Qt::Key_Up && event->key() != Qt::Key_Down) {
            QWidget::keyPressEvent(event);
            return;
        }

        // Ignore auto-repeat for trigger adjustment
        if (event->isAutoRepeat()) {
            return;
        }

        // Debounce trigger level adjustment
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_trigger_adjust).count();

        if (elapsed < TRIGGER_ADJUST_DEBOUNCE_MS) {
            return;  // Too soon, ignore
        }

        if (event->key() == Qt::Key_Up) {
            reader.increaseTriggerLevel();
            last_trigger_adjust = now;
        } else if (event->key() == Qt::Key_Down) {
            reader.decreaseTriggerLevel();
            last_trigger_adjust = now;
        }
    }
};

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return app.exec();
}

#include "ol.moc"

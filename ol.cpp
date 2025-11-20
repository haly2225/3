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
#include <deque>
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
constexpr float    SAMPLE_RATE = 411000.0f;  // Actual rate from timing analysis
constexpr float    VCC = 3.3f;
constexpr uint16_t ADC_MAX = 4095;
constexpr size_t   CAPTURE_SIZE = 512;   // Snapshot mode: exactly 1 packet for stable display

// Trigger modes
enum class TriggerMode {
    AUTO,      // Auto trigger if signal present, else free run
    NORMAL,    // Only trigger when condition met
    FREE_RUN   // Always free run
};

// Trigger slope
enum class TriggerSlope {
    RISING,    // Trigger on rising edge
    FALLING    // Trigger on falling edge
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
// Measurements
// ============================================================================
struct SignalMeasurements {
    float frequency = 0.0f;
    float period = 0.0f;
    float vrms = 0.0f;
    float vmean = 0.0f;
    float vpp = 0.0f;
    float vmax = 0.0f;
    float vmin = 0.0f;
    float duty_cycle = 0.0f;
    float rise_time = 0.0f;   // 10% to 90% rise time
    float fall_time = 0.0f;   // 90% to 10% fall time
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
    std::mutex buffer_history_mutex;  // Protect buffer_history from race conditions
    
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
    const int HOLDOFF_MS = 20;  // 20ms = 50Hz max trigger rate - prevents race conditions
    const int SIGNAL_TIMEOUT_MS = 300;
    const int AUTO_FALLBACK_MS = 200;  // Switch to free run if no trigger
    
    // Capture
    std::array<float, 8192> temp_buffer;
    size_t temp_size = 0;
    size_t collect_count = 0;

    // Circular buffer for pre-trigger data (keeps last 6 buffers = 3072 samples)
    static constexpr int HISTORY_BUFFERS = 6;
    std::array<std::array<float, BUFFER_SIZE>, HISTORY_BUFFERS> buffer_history;
    int history_write_pos = 0;
    int buffers_in_history = 0;

    // Trigger consistency - store first edge position to ensure all buffers match
    int first_edge_position = -1;

    // Delayed trigger for continuous data
    int trigger_buffer_index = -1;  // Which buffer in history contains the edge
    int trigger_edge_position = -1;  // Edge position within that buffer
    int buffers_since_trigger = 0;   // Count buffers after trigger detected

    // Phase-locked trigger state (professional oscilloscope technique)
    struct PhaseLockedState {
        bool enabled = true;  // Enable phase-locked mode
        float measured_period = 0.0f;
        int edges_found = 0;
        std::vector<int> edge_positions;  // Store all edge positions in capture

        void reset() {
            measured_period = 0.0f;
            edges_found = 0;
            edge_positions.clear();
        }
    };
    PhaseLockedState phase_lock;

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
    std::atomic<float> trigger_level{1.65f};  // Default: mid-point of 0-3.3V
    const float TRIGGER_LEVEL_STEP = 0.05f;  // 50mV steps
    std::atomic<int> trigger_level_changes{0};  // Count changes for periodic logging

    // Trigger slope (rising/falling edge)
    std::atomic<TriggerSlope> trigger_slope{TriggerSlope::RISING};

    // Auto Trigger 50% (automatically set trigger to mid-point)
    std::atomic<bool> auto_trigger_50_percent{false};
    float last_vmax = 3.3f;
    float last_vmin = 0.0f;

    // Pre-trigger samples (show waveform BEFORE trigger point)
    static constexpr int PRE_TRIGGER_SAMPLES = 100;  // Show 100 samples before edge

    // Horizontal/Vertical position offset
    std::atomic<float> h_offset{0.0f};  // Horizontal offset in seconds
    std::atomic<float> v_offset{0.0f};  // Vertical offset in volts

    // Acquisition control
    std::atomic<bool> acquisition_running{true};  // RUN/STOP state
    std::atomic<bool> single_shot_mode{false};    // Single shot trigger
    std::atomic<bool> single_shot_armed{false};   // Armed for single shot

    std::chrono::steady_clock::time_point last_packet_time;
    bool first_packet_received = false;

    DebugLogger logger;
    TimingStats stats;

    // Signal measurements
    SignalMeasurements last_measurements;
    std::mutex measurements_mutex;

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
        float new_level = std::min(trigger_level.load() + TRIGGER_LEVEL_STEP, 3.3f);
        trigger_level.store(new_level);
        int changes = ++trigger_level_changes;

        // Only log every 4 changes (0.2V) or at max boundary
        if (new_level >= 3.3f || changes % 4 == 0) {
            std::ostringstream oss;
            oss << "🎯 Trigger Level: " << std::fixed << std::setprecision(2) << new_level << "V";
            logger.log(oss.str());
        }
    }

    void decreaseTriggerLevel() {
        float new_level = std::max(trigger_level.load() - TRIGGER_LEVEL_STEP, 0.0f);
        trigger_level.store(new_level);
        int changes = ++trigger_level_changes;

        // Only log every 4 changes (0.2V) or at min boundary
        if (new_level <= 0.0f || changes % 4 == 0) {
            std::ostringstream oss;
            oss << "🎯 Trigger Level: " << std::fixed << std::setprecision(2) << new_level << "V";
            logger.log(oss.str());
        }
    }

    float getTriggerLevel() const { return trigger_level.load(); }

    // Auto Trigger 50% control
    void set_auto_trigger_50(bool enabled) {
        auto_trigger_50_percent = enabled;
        std::string status = enabled ? "ENABLED" : "DISABLED";
        logger.log("🎯 Auto Trigger 50%: " + status);
    }

    bool get_auto_trigger_50() const { return auto_trigger_50_percent.load(); }

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

    SignalMeasurements get_measurements() {
        std::lock_guard<std::mutex> lock(measurements_mutex);
        return last_measurements;
    }

    // Acquisition control
    void set_running(bool run) {
        acquisition_running = run;
        if (run) {
            // Reset single shot mode when resuming RUN
            single_shot_mode = false;
            single_shot_armed = false;
            logger.log("▶️  RUN (continuous)");
        } else {
            logger.log("⏸️  STOP");
        }
    }

    bool is_running() const { return acquisition_running.load(); }

    void arm_single_shot() {
        single_shot_mode = true;
        single_shot_armed = true;
        acquisition_running = true;
        logger.log("🎯 SINGLE SHOT ARMED");
    }

    bool is_single_shot() const { return single_shot_mode.load(); }
    bool is_single_armed() const { return single_shot_armed.load(); }

    // Trigger slope control
    void set_trigger_slope(TriggerSlope slope) {
        trigger_slope = slope;
        std::string slope_str = (slope == TriggerSlope::RISING) ? "↗️ RISING" : "↘️ FALLING";
        logger.log(slope_str + " EDGE");
    }

    TriggerSlope get_trigger_slope() const { return trigger_slope.load(); }

    // Horizontal/Vertical offset
    void set_h_offset(float offset) { h_offset = offset; }
    void set_v_offset(float offset) { v_offset = offset; }
    float get_h_offset() const { return h_offset.load(); }
    float get_v_offset() const { return v_offset.load(); }

    // Phase-lock control
    void set_phase_lock(bool enabled) {
        phase_lock.enabled = enabled;
        std::string status = enabled ? "ENABLED" : "DISABLED";
        logger.log("🔒 Phase-lock: " + status);
    }
    bool get_phase_lock() const { return phase_lock.enabled; }

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

        // logger.log("🔁 Loop start");  // Disabled for performance
        logger.log("🔁 Loop start - waiting for STM32 data...");
        last_signal_time = std::chrono::steady_clock::now();
        last_auto_switch_time = last_signal_time;

        int loop_counter = 0;
        int no_sync_warning_shown = 0;

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

            // Removed usleep(600) - SPI 16MHz needs full speed to keep up with STM32
            stats.total_packets++;
            loop_counter++;

            auto now = std::chrono::steady_clock::now();

            if (parse_packet(rx_buf)) {
                stats.good_packets++;
                frame_count++;
            } else {
                // Debug: log first packet failures
                if (no_sync_warning_shown < 3) {
                    no_sync_warning_shown++;
                    std::ostringstream oss;
                    oss << "⚠️  No sync after " << loop_counter << " loops. First 10 bytes: ";
                    for (int i = 0; i < 10 && i < (int)rx_buf.size(); i++) {
                        oss << "0x" << std::hex << std::setw(2) << std::setfill('0')
                            << (int)rx_buf[i] << " ";
                    }
                    logger.log(oss.str());
                }
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

        // logger.log("🔁 Exit");  // Disabled for performance
    }
    
    bool parse_packet(const std::vector<uint8_t>& buf) {
        // Find marker - scan entire buffer for 0xAA 0x55
        int marker_pos = -1;
        static int scan_count = 0;

        for (size_t i = 0; i < buf.size() - 1; i++) {
            if (buf[i] == MARKER_START && buf[i+1] == MARKER_HEADER) {
                marker_pos = i;

                // Log first marker found
                static bool first_marker_logged = false;
                if (!first_marker_logged) {
                    first_marker_logged = true;
                    std::ostringstream oss;
                    oss << "✅ Found marker 0xAA 0x55 at position " << i
                        << " (need " << PACKET_SIZE << " bytes total)";
                    logger.log(oss.str());
                }
                break;
            }
        }

        if (marker_pos < 0) {
            // Debug: scan for any 0xAA in buffer
            if (scan_count++ < 5) {
                bool has_aa = false;
                for (size_t i = 0; i < buf.size(); i++) {
                    if (buf[i] == 0xAA) {
                        has_aa = true;
                        std::ostringstream oss;
                        oss << "🔍 Found 0xAA at position " << i
                            << ", next byte: 0x" << std::hex << (int)buf[i+1];
                        logger.log(oss.str());
                        break;
                    }
                }
                if (!has_aa) {
                    logger.log("⚠️  No 0xAA marker found in packet - STM32 may not be transmitting");
                }
            }
            return false;
        }

        // Check if we have enough bytes after marker
        if (marker_pos + PACKET_SIZE > (int)buf.size()) {
            static int incomplete_count = 0;
            if (incomplete_count++ < 3) {
                std::ostringstream oss;
                oss << "⚠️  Marker found at " << marker_pos
                    << " but only " << (buf.size() - marker_pos)
                    << " bytes remaining (need " << PACKET_SIZE << ")";
                logger.log(oss.str());
            }
            return false;
        }

        uint16_t frame_num = (static_cast<uint16_t>(buf[marker_pos + 2]) << 8) |
                             buf[marker_pos + 3];

        // --- ANTI-DRIFT DISABLED for Snapshot mode ---
        // Allow buffer to fill even with packet gaps
        /*
        static uint16_t last_frame_num = 0;
        bool gap_detected = false;

        if (frame_initialized) {
            uint16_t expected = static_cast<uint16_t>(last_frame_num + 1);
            if (frame_num != expected) {
                gap_detected = true;
            }
        }
        last_frame_num = frame_num;
        */

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
        {
            std::lock_guard<std::mutex> lock(buffer_history_mutex);

            // Gap detection disabled for Snapshot mode
            // Allow buffer to fill for trigger to work

            buffer_history[history_write_pos] = voltage;
            history_write_pos = (history_write_pos + 1) % HISTORY_BUFFERS;
            if (buffers_in_history < HISTORY_BUFFERS) {
                buffers_in_history++;
            }
        }
        
        // Measure Vpp
        float vmax = 0, vmin = 3.3f;
        for (size_t i = 0; i < BUFFER_SIZE; i++) {
            if (voltage[i] > vmax) vmax = voltage[i];
            if (voltage[i] < vmin) vmin = voltage[i];
        }
        float vpp = vmax - vmin;
        last_vpp = vpp;

        // Auto Trigger 50%: automatically set trigger to mid-point of signal
        if (auto_trigger_50_percent.load() && vpp > 0.5f) {
            last_vmax = vmax;
            last_vmin = vmin;
            float mid_level = (vmax + vmin) / 2.0f;

            // Only update if change is significant (> 20mV) to avoid jitter
            if (std::abs(mid_level - trigger_level.load()) > 0.02f) {
                trigger_level.store(mid_level);

                // Log occasionally (every 100 updates)
                static int auto_trig_log_count = 0;
                if (++auto_trig_log_count % 100 == 1) {
                    std::ostringstream oss;
                    oss << "🎯 Auto 50%: " << std::fixed << std::setprecision(2)
                        << mid_level << "V (Vmax=" << vmax << "V, Vmin=" << vmin << "V)";
                    logger.log(oss.str());
                }
            }
        }

        // DEBUG logging disabled for performance
        // static int debug_counter = 0;
        // if (debug_counter++ % 500 == 0) {
        //     std::ostringstream oss;
        //     oss << "📊 Vpp=" << std::fixed << std::setprecision(2) << vpp
        //         << "V, Max=" << vmax << "V, Min=" << vmin << "V, Frame=" << frame_num;
        //     logger.log(oss.str());
        // }
        
        // Signal detection
        if (vpp > 0.5f) {
            signal_present = true;
            last_signal_time = std::chrono::steady_clock::now();
        }

        // Check if acquisition is running (RUN/STOP control)
        if (!acquisition_running.load()) {
            return false;  // STOP - don't update display
        }

        // Route to appropriate handler
        TriggerMode mode = trigger_mode.load();

        bool triggered = false;
        if (mode == TriggerMode::FREE_RUN) {
            triggered = handle_free_run(voltage);
        } else if (mode == TriggerMode::NORMAL) {
            triggered = handle_normal_trigger(voltage, frame_num, vpp);
        } else { // AUTO
            triggered = handle_auto_trigger(voltage, frame_num, vpp);
        }

        // Handle single shot mode
        if (triggered && single_shot_mode.load() && single_shot_armed.load()) {
            single_shot_armed = false;
            acquisition_running = false;
            logger.log("🎯 SINGLE SHOT TRIGGERED - STOPPED");
        }

        return triggered;
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
                    phase_lock.reset();  // Reset phase lock state
                }
                return false;
            }

            case State::IDLE: {
                // Find rising edge and trigger
                // Only need 1 buffer for Snapshot mode (512 samples)
                if (vpp > TRIGGER_THRESHOLD && buffers_in_history >= 1) {
                    int edge_idx = find_simple_edge(voltage);

                    if (edge_idx >= 0) {
                        stats.trigger_count++;

                        // TRIGGER logging disabled for performance
                        // static int trigger_log_count = 0;
                        // if (++trigger_log_count % 20 == 1) {
                        //     std::ostringstream oss;
                        //     oss << "🎯 TRIGGER! edge=" << edge_idx
                        //         << ", vpp=" << std::fixed << std::setprecision(2) << vpp << "V";
                        //     logger.log(oss.str());
                        // }

                        // DELAYED TRIGGER: Save trigger info and wait for more buffers
                        // This allows us to collect full 1200 samples from history
                        trigger_buffer_index = (history_write_pos - 1 + HISTORY_BUFFERS) % HISTORY_BUFFERS;
                        trigger_edge_position = edge_idx;
                        buffers_since_trigger = 0;

                        state = State::COLLECTING;
                        return false;
                    }
                }
                return false;
            }

            case State::LEARNING:  // No longer used, but keep for compatibility
            case State::TRIGGERED:
            case State::COLLECTING: {
                // Wait for 2 more buffers after trigger, then extract from history
                if (vpp < TRIGGER_THRESHOLD) {
                    // Signal lost - abort
                    state = State::IDLE;
                    return false;
                }

                buffers_since_trigger++;

                // CRITICAL: Must wait 4 buffers for full 2400 samples
                // Math: 2400 samples / 512 per buffer = 4.6875 buffers needed
                //
                // Trigger buffer (index 0): ~410 samples (from pos 102 to end)
                // Buffer +1 (index 1): 512 samples (total: 922)
                // Buffer +2 (index 2): 512 samples (total: 1434)
                // Buffer +3 (index 3): 512 samples (total: 1946)
                // Buffer +4 (index 4): 454 samples (total: 2400) ✓
                //
                // After 4 arrivals: have buffers 0,1,2,3,4 (5 total) - SAFE with HISTORY=6
                // After 5 arrivals: wraparound occurs! So 4 is MAX safe wait
                if (buffers_since_trigger >= 4) {
                    // Extract data from circular buffer history with proper locking
                    temp_size = 0;
                    std::vector<int> buffers_used;
                    std::vector<int> samples_per_buffer;
                    bool log_this_extraction = false;

                    {
                        // Lock buffer_history to prevent race conditions during extraction
                        std::lock_guard<std::mutex> lock(buffer_history_mutex);

                        // CORRECTED EXTRACTION - Start exactly PRE_TRIGGER_SAMPLES before edge
                        // Edge at position trigger_edge_position → start 100 samples before it
                        // This ensures edge ALWAYS appears at index 100 in display buffer

                        int start_buf_idx;
                        int start_pos;

                        if (trigger_edge_position >= PRE_TRIGGER_SAMPLES) {
                            // Edge is far enough in buffer, start in same buffer
                            start_buf_idx = trigger_buffer_index;
                            start_pos = trigger_edge_position - PRE_TRIGGER_SAMPLES;
                        } else {
                            // Edge is near start of buffer, need to start in previous buffer
                            start_buf_idx = (trigger_buffer_index - 1 + HISTORY_BUFFERS) % HISTORY_BUFFERS;
                            start_pos = BUFFER_SIZE - (PRE_TRIGGER_SAMPLES - trigger_edge_position);
                        }

                        // DEBUG logging disabled for performance
                        // static int extract_log_count = 0;
                        // log_this_extraction = (++extract_log_count % 10 == 1);
                        // if (log_this_extraction) {
                        //     std::ostringstream oss;
                        //     oss << "🔧 EXTRACT: edge=" << trigger_edge_position
                        //         << " start_buf=" << start_buf_idx
                        //         << " start_pos=" << start_pos;
                        //     logger.log(oss.str());
                        // }
                        bool log_this_extraction = false;

                        // Extract samples starting from calculated position
                        int buf_idx = start_buf_idx;
                        int pos = start_pos;
                        int current_buf = buf_idx;
                        int samples_from_current = 0;

                        while (temp_size < CAPTURE_SIZE) {
                            temp_buffer[temp_size++] = buffer_history[buf_idx][pos++];
                            samples_from_current++;

                            // Move to next buffer when current buffer ends
                            if (pos >= BUFFER_SIZE) {
                                if (log_this_extraction) {
                                    buffers_used.push_back(current_buf);
                                    samples_per_buffer.push_back(samples_from_current);
                                }

                                pos = 0;
                                buf_idx = (buf_idx + 1) % HISTORY_BUFFERS;
                                current_buf = buf_idx;
                                samples_from_current = 0;
                            }
                        }

                        // Log last buffer
                        if (log_this_extraction && samples_from_current > 0) {
                            buffers_used.push_back(current_buf);
                            samples_per_buffer.push_back(samples_from_current);

                            std::ostringstream oss;
                            oss << "📦 BUFFERS: ";
                            for (size_t i = 0; i < buffers_used.size(); i++) {
                                oss << "buf[" << buffers_used[i] << "]=" << samples_per_buffer[i] << " ";
                            }
                            oss << "total=" << temp_size;
                            logger.log(oss.str());
                        }
                    }  // Release buffer_history_mutex lock

                    // **PHASE-LOCKED ALIGNMENT** - professional oscilloscope technique
                    if (phase_lock.enabled) {
                        apply_phase_locked_alignment();
                    } else {
                        // Standard copy without alignment
                        std::lock_guard<std::mutex> lock(data_mutex);
                        for (size_t i = 0; i < temp_size; i++) {
                            display_voltage[i] = temp_buffer[i];
                            display_time[i] = static_cast<float>((int)i - PRE_TRIGGER_SAMPLES) / SAMPLE_RATE;
                        }
                        display_size = temp_size;
                        new_data = true;
                    }

                    // DEBUG: Show sample values at key positions
                    if (log_this_extraction) {
                        std::ostringstream oss;
                        oss << "📊 SAMPLES: edge[100]=" << std::fixed << std::setprecision(2) << temp_buffer[100]
                            << "V, last_4cycles[" << (temp_size - 600) << "-" << (temp_size - 1) << "]: "
                            << "start=" << temp_buffer[temp_size - 600] << "V "
                            << "mid=" << temp_buffer[temp_size - 300] << "V "
                            << "end=" << temp_buffer[temp_size - 1] << "V";
                        logger.log(oss.str());
                    }

                    state = State::HOLDOFF;
                    last_trigger_time = now;

                    // Calculate measurements for new waveform (outside data_mutex lock)
                    calculate_measurements();

                    return true;
                }

                return false;
            }
        }

        return false;
    }

    // Hysteresis edge detection - stable trigger like Tektronix/Keysight
    // Returns integer sample index where edge crosses trigger level with hysteresis
    int find_simple_edge(const std::array<float, BUFFER_SIZE>& voltage) {
        // Search nearly entire buffer to reliably find rising edge
        int start = 5;
        int end = BUFFER_SIZE - 20;  // Leave small margin at end

        float trig_lvl = trigger_level.load();
        TriggerSlope slope = trigger_slope.load();

        // Hysteresis parameters (prevent jitter from noise)
        const float HYSTERESIS = 0.05f;  // 50mV hysteresis band
        float high_level = trig_lvl + HYSTERESIS;
        float low_level = trig_lvl - HYSTERESIS;

        // State machine: track if signal was below low level
        bool below_low = (voltage[start] < low_level);
        bool above_high = (voltage[start] > high_level);

        // Search for edge in wider range with hysteresis
        for (int i = start; i < end; i++) {
            if (slope == TriggerSlope::RISING) {
                // Track state
                if (voltage[i] < low_level) {
                    below_low = true;
                    above_high = false;
                }
                // Rising edge: was below LOW, now above HIGH
                if (below_low && voltage[i] > high_level) {
                    return i;  // Found trigger!
                }
            } else {
                // Falling edge with hysteresis
                // Track state
                if (voltage[i] > high_level) {
                    above_high = true;
                    below_low = false;
                }
                // Falling edge: was above HIGH, now below LOW
                if (above_high && voltage[i] < low_level) {
                    return i;  // Found trigger!
                }
            }
        }

        return -1;  // No edge found
    }

    // Phase-Locked Multi-Cycle Alignment - professional oscilloscope technique
    // Uses sub-sample interpolation to perfectly align waveform to trigger point
    void apply_phase_locked_alignment() {
        float trig_lvl = trigger_level.load();
        TriggerSlope slope = trigger_slope.load();

        // Step 1: Find edge NEAR PRE_TRIGGER_SAMPLES with sub-sample precision
        // Search in a window around the expected trigger position (not from start!)
        int first_edge_idx = -1;
        float fractional_offset = 0.0f;

        // Search window: PRE_TRIGGER_SAMPLES ± 10 samples (tight window)
        int search_start = std::max(1, PRE_TRIGGER_SAMPLES - 10);
        int search_end = std::min((int)temp_size - 1, PRE_TRIGGER_SAMPLES + 10);

        for (int i = search_start; i < search_end; i++) {
            bool edge_found = false;

            if (slope == TriggerSlope::RISING) {
                edge_found = (temp_buffer[i - 1] < trig_lvl && temp_buffer[i] >= trig_lvl);
            } else {
                edge_found = (temp_buffer[i - 1] > trig_lvl && temp_buffer[i] <= trig_lvl);
            }

            if (edge_found) {
                first_edge_idx = i;

                // Calculate fractional position for sub-sample accuracy
                float v0 = temp_buffer[i - 1];
                float v1 = temp_buffer[i];
                float delta = v1 - v0;

                if (std::abs(delta) > 0.001f) {
                    fractional_offset = (trig_lvl - v0) / delta;
                } else {
                    fractional_offset = 0.0f;
                }

                fractional_offset = std::max(0.0f, std::min(1.0f, fractional_offset));
                break;
            }
        }

        if (first_edge_idx < 0) {
            // No edge found - use standard copy
            std::lock_guard<std::mutex> lock(data_mutex);
            for (size_t i = 0; i < temp_size; i++) {
                display_voltage[i] = temp_buffer[i];
                display_time[i] = static_cast<float>((int)i - PRE_TRIGGER_SAMPLES) / SAMPLE_RATE;
            }
            display_size = temp_size;
            new_data = true;
            return;
        }

        // Step 2: Calculate total shift needed to align edge to PRE_TRIGGER_SAMPLES
        // Edge is at (first_edge_idx - 1 + fractional_offset)
        // We want it at PRE_TRIGGER_SAMPLES exactly
        float actual_edge_pos = (first_edge_idx - 1) + fractional_offset;
        float shift = actual_edge_pos - PRE_TRIGGER_SAMPLES;

        static int align_log_count = 0;
        if (++align_log_count % 20 == 1) {
            std::ostringstream oss;
            oss << "🔒 PHASE-LOCK: edge_idx=" << first_edge_idx
                << " frac=" << std::fixed << std::setprecision(3) << fractional_offset
                << " shift=" << std::setprecision(3) << shift << " samples";
            logger.log(oss.str());
        }

        // Step 3: Apply interpolation shift to entire waveform
        std::lock_guard<std::mutex> lock(data_mutex);

        for (size_t i = 0; i < temp_size; i++) {
            // Source position with shift applied
            float src_pos = i + shift;

            if (src_pos < 0 || src_pos >= temp_size - 1) {
                // Out of bounds - use edge value
                display_voltage[i] = (src_pos < 0) ? temp_buffer[0] : temp_buffer[temp_size - 1];
            } else {
                // Linear interpolation
                int src_idx = static_cast<int>(src_pos);
                float frac = src_pos - src_idx;

                display_voltage[i] = temp_buffer[src_idx] * (1.0f - frac) + temp_buffer[src_idx + 1] * frac;
            }

            display_time[i] = static_cast<float>((int)i - PRE_TRIGGER_SAMPLES) / SAMPLE_RATE;
        }

        display_size = temp_size;
        new_data = true;
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

    // Calculate signal measurements from current display buffer
    void calculate_measurements() {
        std::lock_guard<std::mutex> lock(data_mutex);
        size_t n = display_size.load();
        if (n < 10) return;

        // Basic measurements
        float vmin = 3.3f, vmax = 0.0f;
        float sum = 0.0f, sum_sq = 0.0f;

        for (size_t i = 0; i < n; i++) {
            float v = display_voltage[i];
            if (v < vmin) vmin = v;
            if (v > vmax) vmax = v;
            sum += v;
            sum_sq += v * v;
        }

        float vmean = sum / n;
        float vrms = std::sqrt(sum_sq / n);
        float vpp = vmax - vmin;

        // Frequency measurement - count zero crossings at mid-point with debounce
        float mid_level = (vmax + vmin) / 2.0f;
        int rising_edges = 0;
        float first_edge_time = -1.0f;
        float last_edge_time = -1.0f;
        float prev_edge_time = -999.0f;  // Initialize to very old time

        // DEBUG: Track edge spacings for analysis
        std::vector<float> edge_times;
        std::vector<float> edge_spacings;

        // Debounce threshold: minimum time between edges
        // Use 800us to filter all noise and harmonics, only count 1kHz main signal
        const float MIN_EDGE_SPACING = 800e-6f;  // 800 microseconds (max 1.25kHz)

        for (size_t i = 1; i < n; i++) {
            if (display_voltage[i-1] < mid_level && display_voltage[i] >= mid_level) {
                float edge_time = display_time[i];

                // Only count edge if it's far enough from previous edge (debounce)
                if (edge_time - prev_edge_time > MIN_EDGE_SPACING) {
                    rising_edges++;
                    if (first_edge_time < 0) {
                        first_edge_time = edge_time;
                    }
                    last_edge_time = edge_time;

                    // Track for debugging
                    edge_times.push_back(edge_time);
                    if (prev_edge_time > -999.0f) {
                        edge_spacings.push_back(edge_time - prev_edge_time);
                    }

                    prev_edge_time = edge_time;
                }
            }
        }

        float frequency = 0.0f;
        float period = 0.0f;
        if (rising_edges >= 2 && last_edge_time > first_edge_time) {
            period = (last_edge_time - first_edge_time) / (rising_edges - 1);
            frequency = 1.0f / period;

            // DEBUG logging disabled for performance
            // static int freq_log_count = 0;
            // if (++freq_log_count % 100 == 1) {
            //     std::ostringstream oss;
            //     oss << "📊 FREQ: edges=" << rising_edges
            //         << " period=" << std::fixed << std::setprecision(6) << period
            //         << "s freq=" << std::setprecision(1) << frequency << "Hz"
            //         << " time_span=" << std::setprecision(6) << (last_edge_time - first_edge_time) << "s";
            //     logger.log(oss.str());
            //     if (!edge_spacings.empty()) {
            //         std::ostringstream oss2;
            //         oss2 << "⏱️  PERIODS: ";
            //         for (size_t i = 0; i < std::min(edge_spacings.size(), size_t(5)); i++) {
            //             oss2 << std::fixed << std::setprecision(3) << (edge_spacings[i] * 1000.0f) << "ms ";
            //         }
            //         if (edge_spacings.size() > 5) {
            //             oss2 << "... (showing first 5 of " << edge_spacings.size() << ")";
            //         }
            //         logger.log(oss2.str());
            //     }
            // }
        }

        // Duty cycle - time above mid-level
        int samples_high = 0;
        for (size_t i = 0; i < n; i++) {
            if (display_voltage[i] > mid_level) {
                samples_high++;
            }
        }
        float duty_cycle = (float)samples_high / n * 100.0f;

        // Rise/Fall time measurement (10% to 90%)
        float rise_time = 0.0f;
        float fall_time = 0.0f;
        float v10 = vmin + 0.1f * vpp;
        float v90 = vmin + 0.9f * vpp;

        // Find first rising edge (10% to 90%)
        int rise_start = -1, rise_end = -1;
        for (size_t i = 1; i < n; i++) {
            if (rise_start < 0 && display_voltage[i-1] < v10 && display_voltage[i] >= v10) {
                rise_start = i;
            }
            if (rise_start >= 0 && rise_end < 0 && display_voltage[i-1] < v90 && display_voltage[i] >= v90) {
                rise_end = i;
                break;
            }
        }
        if (rise_start >= 0 && rise_end > rise_start) {
            rise_time = (rise_end - rise_start) / SAMPLE_RATE;
        }

        // Find first falling edge (90% to 10%)
        int fall_start = -1, fall_end = -1;
        for (size_t i = 1; i < n; i++) {
            if (fall_start < 0 && display_voltage[i-1] > v90 && display_voltage[i] <= v90) {
                fall_start = i;
            }
            if (fall_start >= 0 && fall_end < 0 && display_voltage[i-1] > v10 && display_voltage[i] <= v10) {
                fall_end = i;
                break;
            }
        }
        if (fall_start >= 0 && fall_end > fall_start) {
            fall_time = (fall_end - fall_start) / SAMPLE_RATE;
        }

        // Store measurements
        {
            std::lock_guard<std::mutex> mlock(measurements_mutex);
            last_measurements.frequency = frequency;
            last_measurements.period = period;
            last_measurements.vrms = vrms;
            last_measurements.vmean = vmean;
            last_measurements.vpp = vpp;
            last_measurements.vmax = vmax;
            last_measurements.vmin = vmin;
            last_measurements.duty_cycle = duty_cycle;
            last_measurements.rise_time = rise_time;
            last_measurements.fall_time = fall_time;
        }
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
    SignalMeasurements measurements;
    float h_offset = 0.0f;  // Horizontal offset
    float v_offset = 0.0f;  // Vertical offset
    TriggerSlope trigger_slope = TriggerSlope::RISING;
    bool dots_mode = false;  // false=lines, true=dots
    bool smoothing_enabled = false;  // Disabled by default for sharp square waves
    bool glow_enabled = true;  // Glow effect for CRT-like appearance

    // Segmented Memory / Waveform History (like real oscilloscope)
    static const int MAX_HISTORY = 64;  // Store up to 64 frames
    std::deque<std::pair<std::vector<float>, std::vector<float>>> waveform_history;
    bool persistence_mode = false;
    int playback_frame = -1;  // -1 = live view, 0..N-1 = viewing historical frame
    bool playback_mode = false;  // True when stopped and viewing history

    // Smoothing helper function (weighted moving average)
    float smooth_value(const std::vector<float>& data, size_t idx) {
        if (idx <= 0 || idx >= data.size() - 1) return data[idx];
        // Weighted average: (prev + 2*current + next) / 4
        return (data[idx - 1] + 2.0f * data[idx] + data[idx + 1]) / 4.0f;
    }

    // Apply smoothing to entire waveform
    std::vector<float> apply_smoothing(const std::vector<float>& data) {
        if (!smoothing_enabled || data.size() < 3) return data;

        std::vector<float> smoothed(data.size());
        smoothed[0] = data[0];
        smoothed[data.size() - 1] = data[data.size() - 1];

        for (size_t i = 1; i < data.size() - 1; i++) {
            smoothed[i] = smooth_value(data, i);
        }
        return smoothed;
    }

public:
    ScopeDisplay(QWidget *parent = nullptr) : QWidget(parent) {
        setMinimumSize(800, 600);
    }

    void update_waveform(const std::vector<float>& v, const std::vector<float>& t) {
        voltage = v;
        time = t;

        // Always store frames in segmented memory (like real oscilloscope)
        if (!v.empty()) {
            waveform_history.push_back({v, t});
            if (waveform_history.size() > MAX_HISTORY) {
                waveform_history.pop_front();
            }
            // Reset playback to latest when new data arrives
            if (playback_mode) {
                playback_frame = waveform_history.size() - 1;
            }
        }

        update();
    }

    void set_time_div(float div) { time_div = div; update(); }
    void set_volt_div(float div) { volt_div = div; update(); }
    void set_trigger_mode(TriggerMode mode) { trigger_mode = mode; update(); }
    void set_trigger_level(float level) { trigger_level = level; update(); }
    void set_measurements(const SignalMeasurements& m) { measurements = m; update(); }
    void set_h_offset(float offset) { h_offset = offset; update(); }
    void set_v_offset(float offset) { v_offset = offset; update(); }
    void set_trigger_slope(TriggerSlope slope) { trigger_slope = slope; update(); }
    void toggle_dots_mode() { dots_mode = !dots_mode; update(); }
    bool get_dots_mode() const { return dots_mode; }
    float get_time_div() const { return time_div; }
    float get_h_offset() const { return h_offset; }

    void toggle_persistence() {
        persistence_mode = !persistence_mode;
        update();
    }
    bool get_persistence() const { return persistence_mode; }
    void clear_history() {
        waveform_history.clear();
        playback_frame = -1;
        update();
    }

    // Smoothing toggle
    void toggle_smoothing() {
        smoothing_enabled = !smoothing_enabled;
        update();
    }
    bool get_smoothing() const { return smoothing_enabled; }

    // Glow effect toggle
    void toggle_glow() {
        glow_enabled = !glow_enabled;
        update();
    }
    bool get_glow() const { return glow_enabled; }

    // Playback mode controls (for viewing history when stopped)
    void enter_playback() {
        playback_mode = true;
        if (!waveform_history.empty()) {
            playback_frame = waveform_history.size() - 1;  // Start at latest
        }
        update();
    }

    void exit_playback() {
        playback_mode = false;
        playback_frame = -1;
        update();
    }

    void prev_frame() {
        if (!waveform_history.empty() && playback_frame > 0) {
            playback_frame--;
            update();
        }
    }

    void next_frame() {
        if (!waveform_history.empty() && playback_frame < (int)waveform_history.size() - 1) {
            playback_frame++;
            update();
        }
    }

    void first_frame() {
        if (!waveform_history.empty()) {
            playback_frame = 0;
            update();
        }
    }

    void last_frame() {
        if (!waveform_history.empty()) {
            playback_frame = waveform_history.size() - 1;
            update();
        }
    }

    int get_frame_count() const { return waveform_history.size(); }
    int get_current_frame() const { return playback_frame; }
    bool is_playback_mode() const { return playback_mode; }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        // Enable antialiasing for smooth professional waveforms
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
        
        // Trigger marker (vertical line at trigger point t=0)
        if (trigger_mode != TriggerMode::FREE_RUN) {
            // Calculate where t=0 (trigger point) is on screen with h_offset
            float t_window = time_div * 10.0f;
            float trigger_x_pos = (0.0f - h_offset) / t_window;  // t=0 with offset
            int trigger_x = margin + static_cast<int>(trigger_x_pos * grid_w);

            // Draw trigger marker at t=0 position
            if (trigger_x >= margin && trigger_x <= margin + grid_w) {
                p.setPen(QPen(QColor(255, 140, 0), 3));
                p.drawLine(trigger_x, margin, trigger_x, margin + grid_h);

                // Draw slope indicator arrow
                p.setBrush(QColor(255, 140, 0));
                QPolygon slope_arrow;
                int arrow_x = trigger_x - 20;
                int arrow_y = margin + 30;
                if (trigger_slope == TriggerSlope::RISING) {
                    // Up arrow for rising
                    slope_arrow << QPoint(arrow_x, arrow_y + 10)
                                << QPoint(arrow_x + 10, arrow_y)
                                << QPoint(arrow_x + 20, arrow_y + 10);
                } else {
                    // Down arrow for falling
                    slope_arrow << QPoint(arrow_x, arrow_y)
                                << QPoint(arrow_x + 10, arrow_y + 10)
                                << QPoint(arrow_x + 20, arrow_y);
                }
                p.drawPolygon(slope_arrow);
            }
        }

        // Trigger level line (horizontal line)
        if (trigger_mode != TriggerMode::FREE_RUN) {
            float v_center = VCC / 2.0f + v_offset;
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
        // Select data source: playback frame or live data
        const std::vector<float>* display_v = &voltage;
        const std::vector<float>* display_t = &time;

        if (playback_mode && playback_frame >= 0 && playback_frame < (int)waveform_history.size()) {
            display_v = &waveform_history[playback_frame].first;
            display_t = &waveform_history[playback_frame].second;
        }

        if (!display_v->empty() && display_v->size() > 10) {
            // Set clipping region to prevent drawing outside grid
            p.setClipRect(margin, margin, grid_w, grid_h);

            float t_window = time_div * 10.0f;
            float v_center = VCC / 2.0f + v_offset;  // Apply vertical offset
            float v_range = volt_div * 8.0f;

            // Draw persistence/history waveforms (older = more faded)
            if (persistence_mode && !waveform_history.empty()) {
                int history_size = waveform_history.size();
                for (int h = 0; h < history_size; h++) {
                    const auto& hist = waveform_history[h];
                    const auto& hist_v = hist.first;
                    const auto& hist_t = hist.second;

                    if (hist_v.empty()) continue;

                    // Alpha fades from 30 (oldest) to 150 (newest in history)
                    int alpha = 30 + (h * 120) / std::max(1, history_size - 1);
                    QColor hist_color(255, 220, 0, alpha);

                    if (dots_mode) {
                        p.setPen(QPen(hist_color, 2));
                        for (size_t i = 0; i < hist_v.size(); i++) {
                            float t = hist_t[i] - h_offset;
                            if (t < 0 || t > t_window) continue;

                            int x = margin + static_cast<int>((t / t_window) * grid_w);
                            int y = cy - static_cast<int>((hist_v[i] - v_center) / v_range * grid_h);
                            x = std::max(margin, std::min(x, margin + grid_w));
                            p.drawPoint(x, y);
                        }
                    } else {
                        p.setPen(QPen(hist_color, 1));
                        for (size_t i = 0; i < hist_v.size() - 1; i++) {
                            float t1 = hist_t[i] - h_offset;
                            float t2 = hist_t[i + 1] - h_offset;
                            if (t2 < 0 || t1 > t_window) continue;

                            float t1_c = std::max(0.0f, std::min(t1, t_window));
                            float t2_c = std::max(0.0f, std::min(t2, t_window));

                            int x1 = margin + static_cast<int>((t1_c / t_window) * grid_w);
                            int x2 = margin + static_cast<int>((t2_c / t_window) * grid_w);
                            int y1 = cy - static_cast<int>((hist_v[i] - v_center) / v_range * grid_h);
                            int y2 = cy - static_cast<int>((hist_v[i+1] - v_center) / v_range * grid_h);

                            x1 = std::max(margin, std::min(x1, margin + grid_w));
                            x2 = std::max(margin, std::min(x2, margin + grid_w));
                            p.drawLine(x1, y1, x2, y2);
                        }
                    }
                }
            }

            // Apply smoothing filter for professional look
            std::vector<float> smoothed_v = apply_smoothing(*display_v);
            const std::vector<float>* final_v = &smoothed_v;

            // Draw current/playback waveform (brightest)
            if (dots_mode) {
                // Dots mode - draw individual sample points
                p.setPen(QPen(QColor(255, 220, 0), 3));

                for (size_t i = 0; i < final_v->size(); i++) {
                    float t = (*display_t)[i] - h_offset;

                    // Skip if outside the time window
                    if (t < 0 || t > t_window) continue;

                    int x = margin + static_cast<int>((t / t_window) * grid_w);
                    int y = cy - static_cast<int>(((*final_v)[i] - v_center) / v_range * grid_h);

                    // Safety bounds
                    x = std::max(margin, std::min(x, margin + grid_w));

                    p.drawPoint(x, y);
                }
            } else {
                // Lines mode - connect samples with lines

                // GLOW EFFECT: Draw thick semi-transparent line first (CRT phosphor glow)
                if (glow_enabled) {
                    p.setPen(QPen(QColor(255, 220, 0, 60), 6));  // Wide, semi-transparent

                    for (size_t i = 0; i < final_v->size() - 1; i++) {
                        float t1 = (*display_t)[i] - h_offset;
                        float t2 = (*display_t)[i + 1] - h_offset;

                        if (t2 < 0 || t1 > t_window) continue;

                        float t1_clamped = std::max(0.0f, std::min(t1, t_window));
                        float t2_clamped = std::max(0.0f, std::min(t2, t_window));

                        int x1 = margin + static_cast<int>((t1_clamped / t_window) * grid_w);
                        int x2 = margin + static_cast<int>((t2_clamped / t_window) * grid_w);
                        int y1 = cy - static_cast<int>(((*final_v)[i] - v_center) / v_range * grid_h);
                        int y2 = cy - static_cast<int>(((*final_v)[i+1] - v_center) / v_range * grid_h);

                        x1 = std::max(margin, std::min(x1, margin + grid_w));
                        x2 = std::max(margin, std::min(x2, margin + grid_w));

                        p.drawLine(x1, y1, x2, y2);
                    }
                }

                // MAIN WAVEFORM: Draw bright thin line on top
                p.setPen(QPen(QColor(255, 220, 0), 2));

                for (size_t i = 0; i < final_v->size() - 1; i++) {
                    // Apply horizontal offset to time values
                    float t1 = (*display_t)[i] - h_offset;
                    float t2 = (*display_t)[i + 1] - h_offset;

                    // Skip if both points are completely outside the time window
                    if (t2 < 0 || t1 > t_window) continue;

                    // Clamp time values to window bounds to prevent drawing outside
                    float t1_clamped = std::max(0.0f, std::min(t1, t_window));
                    float t2_clamped = std::max(0.0f, std::min(t2, t_window));

                    int x1 = margin + static_cast<int>((t1_clamped / t_window) * grid_w);
                    int x2 = margin + static_cast<int>((t2_clamped / t_window) * grid_w);
                    int y1 = cy - static_cast<int>(((*final_v)[i] - v_center) / v_range * grid_h);
                    int y2 = cy - static_cast<int>(((*final_v)[i+1] - v_center) / v_range * grid_h);

                    // Additional safety: clamp x coordinates to grid bounds
                    x1 = std::max(margin, std::min(x1, margin + grid_w));
                    x2 = std::max(margin, std::min(x2, margin + grid_w));

                    p.drawLine(x1, y1, x2, y2);
                }
            }

            // Reset clipping for subsequent drawing
            p.setClipping(false);

            // Measurements - show comprehensive signal info
            p.setPen(QColor(255, 255, 100));
            p.setFont(QFont("Monospace", 11, QFont::Bold));

            int y_pos = 30;

            // Frame counter for playback mode
            if (playback_mode && !waveform_history.empty()) {
                p.setPen(QColor(100, 255, 100));  // Green for playback indicator
                p.drawText(10, y_pos, QString("PLAYBACK Frame %1/%2")
                           .arg(playback_frame + 1)
                           .arg(waveform_history.size()));
                y_pos += 20;
                p.setPen(QColor(255, 255, 100));  // Back to yellow
            } else if (!waveform_history.empty()) {
                // Show history count in live mode
                p.drawText(10, y_pos, QString("History: %1 frames").arg(waveform_history.size()));
                y_pos += 20;
            }

            p.drawText(10, y_pos, QString("N: %1").arg(display_v->size()));
            y_pos += 20;

            // Frequency display
            if (measurements.frequency > 0) {
                QString freq_str;
                if (measurements.frequency >= 1000.0f) {
                    freq_str = QString("Freq: %1kHz").arg(measurements.frequency / 1000.0f, 0, 'f', 2);
                } else {
                    freq_str = QString("Freq: %1Hz").arg(measurements.frequency, 0, 'f', 1);
                }
                p.drawText(10, y_pos, freq_str);
                y_pos += 20;

                // Period display
                QString period_str;
                if (measurements.period < 0.001f) {
                    period_str = QString("T: %1µs").arg(measurements.period * 1e6f, 0, 'f', 1);
                } else {
                    period_str = QString("T: %1ms").arg(measurements.period * 1000.0f, 0, 'f', 3);
                }
                p.drawText(10, y_pos, period_str);
                y_pos += 20;
            }

            p.drawText(10, y_pos, QString("Vpp: %1V").arg(measurements.vpp, 0, 'f', 2));
            y_pos += 20;
            p.drawText(10, y_pos, QString("Max: %1V").arg(measurements.vmax, 0, 'f', 2));
            y_pos += 20;
            p.drawText(10, y_pos, QString("Min: %1V").arg(measurements.vmin, 0, 'f', 2));
            y_pos += 20;
            p.drawText(10, y_pos, QString("Mean: %1V").arg(measurements.vmean, 0, 'f', 2));
            y_pos += 20;
            p.drawText(10, y_pos, QString("RMS: %1V").arg(measurements.vrms, 0, 'f', 2));
            y_pos += 20;

            if (measurements.frequency > 0) {
                p.drawText(10, y_pos, QString("Duty: %1%").arg(measurements.duty_cycle, 0, 'f', 1));
                y_pos += 20;
            }

            // Rise/Fall time
            if (measurements.rise_time > 0) {
                QString rise_str;
                if (measurements.rise_time < 1e-6f) {
                    rise_str = QString("Rise: %1ns").arg(measurements.rise_time * 1e9f, 0, 'f', 1);
                } else if (measurements.rise_time < 1e-3f) {
                    rise_str = QString("Rise: %1µs").arg(measurements.rise_time * 1e6f, 0, 'f', 2);
                } else {
                    rise_str = QString("Rise: %1ms").arg(measurements.rise_time * 1e3f, 0, 'f', 3);
                }
                p.drawText(10, y_pos, rise_str);
                y_pos += 20;
            }

            if (measurements.fall_time > 0) {
                QString fall_str;
                if (measurements.fall_time < 1e-6f) {
                    fall_str = QString("Fall: %1ns").arg(measurements.fall_time * 1e9f, 0, 'f', 1);
                } else if (measurements.fall_time < 1e-3f) {
                    fall_str = QString("Fall: %1µs").arg(measurements.fall_time * 1e6f, 0, 'f', 2);
                } else {
                    fall_str = QString("Fall: %1ms").arg(measurements.fall_time * 1e3f, 0, 'f', 3);
                }
                p.drawText(10, y_pos, fall_str);
            }
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
    QPushButton *run_stop_btn;
    QPushButton *single_btn;
    QPushButton *slope_btn;
    QLabel *offset_label;

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

        // RUN/STOP and SINGLE buttons
        QLabel *acq_label = new QLabel("ACQUISITION");
        acq_label->setStyleSheet("color: #00ff00; font-weight: bold; font-size: 14px;");
        panel_layout->addWidget(acq_label);

        run_stop_btn = new QPushButton("▶️ RUN");
        run_stop_btn->setStyleSheet(
            "QPushButton { background-color: #00aa00; color: white; padding: 10px; font-weight: bold; font-size: 14px; }"
        );
        connect(run_stop_btn, &QPushButton::clicked, this, &MainWindow::toggle_run_stop);
        panel_layout->addWidget(run_stop_btn);

        single_btn = new QPushButton("🎯 SINGLE");
        single_btn->setStyleSheet(
            "QPushButton { background-color: #aa5500; color: white; padding: 8px; font-weight: bold; }"
        );
        connect(single_btn, &QPushButton::clicked, this, &MainWindow::arm_single_shot);
        panel_layout->addWidget(single_btn);

        panel_layout->addSpacing(10);

        // Trigger Slope
        slope_btn = new QPushButton("↗️ RISING");
        slope_btn->setStyleSheet(
            "QPushButton { background-color: #555; color: white; padding: 8px; font-weight: bold; }"
        );
        connect(slope_btn, &QPushButton::clicked, this, &MainWindow::toggle_trigger_slope);
        panel_layout->addWidget(slope_btn);

        // Phase-Lock control
        QPushButton *phase_lock_btn = new QPushButton("🔒 PHASE-LOCK: ON");
        phase_lock_btn->setCheckable(true);
        phase_lock_btn->setChecked(true);
        phase_lock_btn->setStyleSheet(
            "QPushButton { background-color: #555; color: white; padding: 6px; font-weight: bold; }"
            "QPushButton:checked { background-color: #00aa00; color: white; }"
        );
        connect(phase_lock_btn, &QPushButton::toggled, [this, phase_lock_btn](bool checked) {
            reader.set_phase_lock(checked);
            phase_lock_btn->setText(checked ? "🔒 PHASE-LOCK: ON" : "🔓 PHASE-LOCK: OFF");
        });
        panel_layout->addWidget(phase_lock_btn);

        // Auto Trigger 50% control
        QPushButton *auto_trig_50_btn = new QPushButton("🎯 AUTO 50%: OFF");
        auto_trig_50_btn->setCheckable(true);
        auto_trig_50_btn->setChecked(false);
        auto_trig_50_btn->setStyleSheet(
            "QPushButton { background-color: #555; color: white; padding: 6px; font-weight: bold; }"
            "QPushButton:checked { background-color: #ff8800; color: white; }"
        );
        connect(auto_trig_50_btn, &QPushButton::toggled, [this, auto_trig_50_btn](bool checked) {
            reader.set_auto_trigger_50(checked);
            auto_trig_50_btn->setText(checked ? "🎯 AUTO 50%: ON" : "🎯 AUTO 50%: OFF");
        });
        panel_layout->addWidget(auto_trig_50_btn);

        panel_layout->addSpacing(10);

        // H/V Position Offset
        QLabel *pos_label = new QLabel("POSITION");
        pos_label->setStyleSheet("color: #00ffff; font-weight: bold;");
        panel_layout->addWidget(pos_label);

        offset_label = new QLabel("H: 0µs  V: 0.0V");
        offset_label->setStyleSheet("color: #00ffff; font-size: 10px;");
        panel_layout->addWidget(offset_label);

        QPushButton *reset_pos_btn = new QPushButton("Reset Position");
        reset_pos_btn->setStyleSheet("background-color: #555; color: white; padding: 4px;");
        connect(reset_pos_btn, &QPushButton::clicked, [this]() {
            reader.set_h_offset(0.0f);
            reader.set_v_offset(0.0f);
            display->set_h_offset(0.0f);
            display->set_v_offset(0.0f);
            update_offset_label();
        });
        panel_layout->addWidget(reset_pos_btn);

        panel_layout->addSpacing(15);

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
        timer->start(30);  // 30ms = ~33 FPS for smoother display
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

        // Update measurements
        display->set_measurements(reader.get_measurements());

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

        // Update debug log less frequently to reduce lag (every 5 frames = ~375ms)
        static int log_update_counter = 0;
        if (++log_update_counter >= 5) {
            log_update_counter = 0;
            auto logs = reader.get_logs();
            QString log_text;
            // Limit to last 100 lines for performance
            size_t start_idx = logs.size() > 100 ? logs.size() - 100 : 0;
            for (size_t i = start_idx; i < logs.size(); i++) {
                log_text += QString::fromStdString(logs[i]) + "\n";
            }
            debug_log->setPlainText(log_text);
            debug_log->moveCursor(QTextCursor::End);
        }

        // Update Run/Stop button state
        update_run_stop_button();
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

    void toggle_run_stop() {
        bool currently_running = reader.is_running();
        reader.set_running(!currently_running);
        update_run_stop_button();
    }

    void arm_single_shot() {
        reader.arm_single_shot();
        update_run_stop_button();
    }

    void update_run_stop_button() {
        if (reader.is_running()) {
            if (reader.is_single_armed()) {
                run_stop_btn->setText("🎯 ARMED");
                run_stop_btn->setStyleSheet(
                    "QPushButton { background-color: #aa5500; color: white; padding: 10px; font-weight: bold; font-size: 14px; }"
                );
            } else {
                run_stop_btn->setText("▶️ RUN");
                run_stop_btn->setStyleSheet(
                    "QPushButton { background-color: #00aa00; color: white; padding: 10px; font-weight: bold; font-size: 14px; }"
                );
            }
        } else {
            run_stop_btn->setText("⏹️ STOP");
            run_stop_btn->setStyleSheet(
                "QPushButton { background-color: #aa0000; color: white; padding: 10px; font-weight: bold; font-size: 14px; }"
            );
        }
    }

    void toggle_trigger_slope() {
        TriggerSlope current = reader.get_trigger_slope();
        TriggerSlope new_slope = (current == TriggerSlope::RISING) ? TriggerSlope::FALLING : TriggerSlope::RISING;
        reader.set_trigger_slope(new_slope);
        display->set_trigger_slope(new_slope);

        if (new_slope == TriggerSlope::RISING) {
            slope_btn->setText("↗️ RISING");
        } else {
            slope_btn->setText("↘️ FALLING");
        }
    }

    void update_offset_label() {
        float h = reader.get_h_offset();
        float v = reader.get_v_offset();
        QString h_str = (std::abs(h) < 1e-3f)
            ? QString("%1µs").arg(h * 1e6f, 0, 'f', 1)
            : QString("%1ms").arg(h * 1e3f, 0, 'f', 2);
        offset_label->setText(QString("H: %1  V: %2V").arg(h_str).arg(v, 0, 'f', 2));
    }

protected:
    void keyPressEvent(QKeyEvent *event) override {
        // Ignore auto-repeat
        if (event->isAutoRepeat()) {
            return;
        }

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_trigger_adjust).count();

        bool ctrl_pressed = (event->modifiers() & Qt::ControlModifier);
        bool shift_pressed = (event->modifiers() & Qt::ShiftModifier);

        // H/V Position Offset control with Shift modifier
        if (shift_pressed) {
            float h = display->get_h_offset();
            float v = reader.get_v_offset();
            float h_step = display->get_time_div() * 0.1f;
            if (h_step == 0) h_step = 0.0001f;  // Default 100µs step
            float v_step = 0.1f;  // 100mV step

            switch (event->key()) {
                case Qt::Key_Left:
                    reader.set_h_offset(h - h_step);
                    display->set_h_offset(h - h_step);
                    update_offset_label();
                    break;
                case Qt::Key_Right:
                    reader.set_h_offset(h + h_step);
                    display->set_h_offset(h + h_step);
                    update_offset_label();
                    break;
                case Qt::Key_Up:
                    reader.set_v_offset(v + v_step);
                    display->set_v_offset(v + v_step);
                    update_offset_label();
                    break;
                case Qt::Key_Down:
                    reader.set_v_offset(v - v_step);
                    display->set_v_offset(v - v_step);
                    update_offset_label();
                    break;
            }
            return;
        }

        // Trigger level adjustment (Up/Down without modifier)
        if (event->key() == Qt::Key_Up || event->key() == Qt::Key_Down) {
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
            return;
        }

        // Acquisition control: R=RUN, S=STOP, Space=SINGLE, D=dots, P=persistence
        switch (event->key()) {
            case Qt::Key_R:
                reader.set_running(true);
                display->exit_playback();  // Exit playback when running
                // Reset h_offset when resuming run
                reader.set_h_offset(0.0f);
                display->set_h_offset(0.0f);
                update_offset_label();
                return;
            case Qt::Key_S:
                reader.set_running(false);
                display->enter_playback();  // Enter playback mode when stopped
                return;
            case Qt::Key_Space:
                reader.arm_single_shot();
                return;
            case Qt::Key_D:
                display->toggle_dots_mode();
                return;
            case Qt::Key_P:
                display->toggle_persistence();
                return;
            case Qt::Key_C:
                // Clear history
                display->clear_history();
                return;
            case Qt::Key_L:
                // Toggle phase-lock
                reader.set_phase_lock(!reader.get_phase_lock());
                return;
            case Qt::Key_M:
                // Toggle smoothing filter
                display->toggle_smoothing();
                return;
            case Qt::Key_G:
                // Toggle glow effect
                display->toggle_glow();
                return;
            case Qt::Key_A:
                // Toggle Auto Trigger 50%
                reader.set_auto_trigger_50(!reader.get_auto_trigger_50());
                return;

            // Playback frame navigation
            case Qt::Key_PageUp:
                display->prev_frame();
                return;
            case Qt::Key_PageDown:
                display->next_frame();
                return;
            case Qt::Key_Home:
                display->first_frame();
                return;
            case Qt::Key_End:
                display->last_frame();
                return;
        }

        // Horizontal scroll with Left/Right when STOPPED (no modifier)
        if (!reader.is_running() && (event->key() == Qt::Key_Left || event->key() == Qt::Key_Right)) {
            float h = display->get_h_offset();
            // Scroll 1 time division per key press (10% of display)
            float h_step = display->get_time_div();
            if (h_step == 0) h_step = 0.001f;  // Default 1ms step

            if (event->key() == Qt::Key_Left) {
                reader.set_h_offset(h - h_step);
                display->set_h_offset(h - h_step);
            } else {
                reader.set_h_offset(h + h_step);
                display->set_h_offset(h + h_step);
            }
            update_offset_label();
            return;
        }

        QWidget::keyPressEvent(event);
    }
};

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return app.exec();
}

#include "ol.moc"

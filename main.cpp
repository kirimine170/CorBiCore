#include <random>
#include <iostream>
#include <thread>
#include <iomanip>
#include <deque>
#include <vector>
#include <cmath>
#include <numeric>
#include <atomic>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <sstream>

#include <simpleble/SimpleBLE.h>

#define SLEEP_TIME 50 // Java側のバッファ用。初期値：1(ms)

#define SERVICE_PULSEOXIMETER_UUID "c360fb9d-497f-4a0d-bfd3-6cbecd1786e1" // パルスオキシメータ
#define CHARA_IR_UUID "0c1f518c-ffdf-4b0f-8f2f-ca1edc6dabae"              // 赤外線
#define CHARA_RED_UUID "1d5b21fa-1a88-4ccb-8be8-9d8f07b0180c"             // 赤色
#define CHARA_ORDER_UUID "9331cae2-0aec-4a90-a44b-7cde8cbc3257"           // オーダー

void print_byte_array_hex(SimpleBLE::ByteArray array);
void print_byte_array_float(SimpleBLE::ByteArray array);
void print_byte_array_uint16(SimpleBLE::ByteArray array);
void print_byte_array_int(SimpleBLE::ByteArray array);
SimpleBLE::Peripheral findCorBi(SimpleBLE::Adapter adaper);
std::vector<uint16_t> decode_uint16_samples(const SimpleBLE::ByteArray &array);
bool setOutputMode(const std::string &input);
void print_current_data(SimpleBLE::ByteArray ir_data, SimpleBLE::ByteArray red_data, double heart_rate);
void write_log_data(const std::vector<uint16_t> &ir_samples, const std::vector<uint16_t> &red_samples, double heart_rate);
std::string samples_to_csv(const std::vector<uint16_t> &samples);
uint64_t now_ms();
// TODO 接続周りはコールバックに変更。
// TODO エラーハンドリングしっかり。

enum class outputMode
{
    ALL,
    IR,
    RED,
    IR_RED
};

SimpleBLE::Peripheral CorBiReader;
std::atomic<outputMode> mode{outputMode::ALL};
std::ofstream logFile;

class HeartRateEstimator
{
public:
    void addSamples(const std::vector<uint16_t> &samples)
    {
        for (uint16_t sample : samples)
        {
            processSample(sample);
        }
    }

    double estimateBpm() const
    {
        if (sampleIndex < MIN_SAMPLES || quality < MIN_QUALITY)
            return 0.0;

        const size_t samplesSinceBeat = sampleIndex - lastBeatSample;
        if (lastBeatSample == 0 || samplesSinceBeat > MAX_STALE_SAMPLES)
            return 0.0;

        return smoothedBpm;
    }

private:
    static constexpr double SAMPLE_RATE_HZ = 100.0;
    static constexpr size_t MIN_SAMPLES = 180;
    static constexpr size_t WINDOW_SIZE = 400;
    static constexpr size_t MAX_INTERVALS = 8;
    static constexpr size_t MIN_BEAT_INTERVAL = static_cast<size_t>(SAMPLE_RATE_HZ * 60.0 / 220.0);
    static constexpr size_t MAX_BEAT_INTERVAL = static_cast<size_t>(SAMPLE_RATE_HZ * 60.0 / 38.0);
    static constexpr size_t MAX_STALE_SAMPLES = static_cast<size_t>(SAMPLE_RATE_HZ * 3.0);
    static constexpr int MIN_BPM = 40;
    static constexpr int MAX_BPM = 220;
    static constexpr double MIN_QUALITY = 0.16;
    static constexpr double DC_ALPHA = 0.01;
    static constexpr double FILTER_ALPHA = 0.18;
    static constexpr double THRESHOLD_SCALE = 0.30;
    static constexpr double BPM_SMOOTHING = 0.28;

    void processSample(uint16_t raw)
    {
        sampleIndex++;
        const double sample = static_cast<double>(raw);
        if (!initialized)
        {
            dc = sample;
            initialized = true;
        }

        dc += DC_ALPHA * (sample - dc);
        filtered += FILTER_ALPHA * ((sample - dc) - filtered);

        filteredWindow.push_back(filtered);
        if (filteredWindow.size() > WINDOW_SIZE)
            filteredWindow.pop_front();

        updateQuality();
        detectBeat(filtered);
    }

    void updateQuality()
    {
        if (filteredWindow.size() < MIN_SAMPLES)
        {
            quality = 0.0;
            return;
        }

        double mean = std::accumulate(filteredWindow.begin(), filteredWindow.end(), 0.0) / filteredWindow.size();
        double energy = 0.0;
        double peak = 0.0;
        for (double value : filteredWindow)
        {
            const double centered = value - mean;
            energy += centered * centered;
            peak = std::max(peak, std::abs(centered));
        }

        const double rms = std::sqrt(energy / filteredWindow.size());
        const double amplitudeQuality = clamp(peak / 260.0, 0.0, 1.0);
        const double energyQuality = clamp(rms / 120.0, 0.0, 1.0);
        const double intervalQuality = intervalConsistency();
        quality = clamp((amplitudeQuality * 0.55) + (energyQuality * 0.30) + (intervalQuality * 0.15), 0.0, 1.0);
        adaptiveThreshold = std::max(24.0, rms * THRESHOLD_SCALE);
    }

    void detectBeat(double current)
    {
        if (filteredWindow.size() < MIN_SAMPLES)
        {
            previousPrevious = previous;
            previous = current;
            return;
        }

        const bool localMaximum = previous > previousPrevious && previous >= current;
        const bool strongEnough = previous > adaptiveThreshold;
        const size_t candidateSample = sampleIndex - 1;
        const size_t interval = lastBeatSample == 0 ? 0 : candidateSample - lastBeatSample;
        const bool outsideRefractory = lastBeatSample == 0 || interval >= MIN_BEAT_INTERVAL;

        if (localMaximum && strongEnough && outsideRefractory)
        {
            if (lastBeatSample == 0 || interval <= MAX_BEAT_INTERVAL)
            {
                acceptBeat(candidateSample, interval);
            }
        }

        previousPrevious = previous;
        previous = current;
    }

    void acceptBeat(size_t beatSample, size_t interval)
    {
        if (interval == 0)
        {
            lastBeatSample = beatSample;
            return;
        }

        const double intervalBpm = 60.0 * SAMPLE_RATE_HZ / static_cast<double>(interval);
        if (intervalBpm >= MIN_BPM && intervalBpm <= MAX_BPM && isConsistent(interval))
        {
            beatIntervals.push_back(interval);
            if (beatIntervals.size() > MAX_INTERVALS)
                beatIntervals.pop_front();

            const double bpm = 60.0 * SAMPLE_RATE_HZ / medianInterval();
            smoothedBpm = smoothedBpm == 0.0 ? bpm : smoothedBpm + BPM_SMOOTHING * (bpm - smoothedBpm);
            lastBeatSample = beatSample;
        }
    }

    bool isConsistent(size_t interval) const
    {
        if (beatIntervals.size() < 3)
            return true;

        const double median = medianInterval();
        const double deviation = std::abs(static_cast<double>(interval) - median) / median;
        return deviation < 0.35;
    }

    double intervalConsistency() const
    {
        if (beatIntervals.size() < 3)
            return 0.0;

        const double median = medianInterval();
        double error = 0.0;
        for (size_t interval : beatIntervals)
        {
            error += std::abs(static_cast<double>(interval) - median) / median;
        }
        error /= beatIntervals.size();
        return clamp(1.0 - (error / 0.22), 0.0, 1.0);
    }

    double medianInterval() const
    {
        std::vector<size_t> intervals(beatIntervals.begin(), beatIntervals.end());
        std::sort(intervals.begin(), intervals.end());
        const size_t middle = intervals.size() / 2;
        if (intervals.size() % 2 == 0)
            return (static_cast<double>(intervals[middle - 1]) + static_cast<double>(intervals[middle])) / 2.0;
        return static_cast<double>(intervals[middle]);
    }

    static double clamp(double value, double minValue, double maxValue)
    {
        return std::min(maxValue, std::max(minValue, value));
    }

    bool initialized = false;
    size_t sampleIndex = 0;
    size_t lastBeatSample = 0;
    double dc = 0.0;
    double filtered = 0.0;
    double previous = 0.0;
    double previousPrevious = 0.0;
    double adaptiveThreshold = 120.0;
    double smoothedBpm = 0.0;
    double quality = 0.0;
    std::deque<double> filteredWindow;
    std::deque<size_t> beatIntervals;
};

void CorBiCore_exit()
{
    CorBiReader.disconnect();
    std::cerr << "CorBiCore is exit." << std::endl;
}

void userInputListener()
{
    std::string input;
    while (std::cin >> input)
    {
        if (input == "e")
        {
            exit(0);
        }
        else if (!setOutputMode(input))
        {
            std::cerr << "Invalid input." << std::endl;
        }
    }
}

int main(int argc, char **argv)
{
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if ((arg == "--mode" || arg == "-m") && i + 1 < argc)
        {
            if (!setOutputMode(argv[++i]))
            {
                std::cerr << "Invalid mode: " << argv[i] << std::endl;
                return 1;
            }
        }
        else if (arg == "--log" && i + 1 < argc)
        {
            logFile.open(argv[++i], std::ios::out | std::ios::trunc);
            if (!logFile.is_open())
            {
                std::cerr << "Failed to open log file: " << argv[i] << std::endl;
                return 1;
            }
            logFile << "timestamp_ms\tbpm\tir_samples\tred_samples" << std::endl;
            std::cerr << "Logging samples to: " << argv[i] << std::endl;
        }
        else if (arg == "--log")
        {
            std::cerr << "Usage: CorBiCore [--mode all|ir|red|ir-red] [--log path]" << std::endl;
            return 1;
        }
        else if (!setOutputMode(arg))
        {
            std::cerr << "Usage: CorBiCore [--mode all|ir|red|ir-red] [--log path]" << std::endl;
            return 1;
        }
    }

    std::atexit(CorBiCore_exit);
    std::thread inputThread(userInputListener);
    inputThread.detach();

    for (;;) // FIXME 流石に関数として括り出した方がいいかも。connect関数とか、read関数とか。
    {

        if (!SimpleBLE::Adapter::bluetooth_enabled())
        {
            std::cerr << "Bluetooth is not enabled." << std::endl;
            return 1;
        }

        std::vector<SimpleBLE::Adapter> adapters = SimpleBLE::Adapter::get_adapters();
        if (adapters.empty())
        {
            std::cerr << "No BLE adapters found." << std::endl;
            return 1;
        }

        SimpleBLE::Adapter adapter = adapters[0];

        // std::cout << "Adapter: " << adapter.identifier() << std::endl;
        // std::cout << "Address: " << adapter.address() << std::endl;

        CorBiReader = findCorBi(adapter);
        std::cerr << "CorBi found: " << CorBiReader.identifier() << " " << CorBiReader.address() << std::endl;
        if (CorBiReader.is_connectable())
        {
            std::cerr << "Connecting to CorBi..." << std::endl;
            CorBiReader.connect();
        }
        else
        {
            std::cerr << "CorBi is not connectable." << std::endl;
            return 1;
        }
        if (!CorBiReader.is_connected())
        {
            std::cerr << "Failed to connect to CorBi." << std::endl;
            return 1;
        }
        std::cerr << "Connected to CorBi." << std::endl;
        SimpleBLE::ByteArray old_data = "";
        HeartRateEstimator heartRateEstimator;
        for (;;)
        {
            try
            {
                SimpleBLE::ByteArray rx_data = CorBiReader.read(SERVICE_PULSEOXIMETER_UUID, CHARA_ORDER_UUID);
                SimpleBLE::ByteArray rx_data_RED = CorBiReader.read(SERVICE_PULSEOXIMETER_UUID, CHARA_RED_UUID);
                SimpleBLE::ByteArray rx_data_IR = CorBiReader.read(SERVICE_PULSEOXIMETER_UUID, CHARA_IR_UUID);
                if (rx_data_RED != old_data)
                {
                    std::vector<uint16_t> ir_samples = decode_uint16_samples(rx_data_IR);
                    std::vector<uint16_t> red_samples = decode_uint16_samples(rx_data_RED);
                    heartRateEstimator.addSamples(ir_samples);
                    const double heartRate = heartRateEstimator.estimateBpm();
                    print_current_data(rx_data_IR, rx_data_RED, heartRate);
                    write_log_data(ir_samples, red_samples, heartRate);
                }
                old_data = rx_data_RED;
                // print_byte_array_hex(rx_data);
            }
            catch (std::runtime_error &e)
            {
                // std::cout << "Error: " << e.what() << std::endl;
                break;
            }
        }

        // std::cout << "Connected to CorBi." << std::endl;
    }
    return 0;
}

std::vector<uint16_t> decode_uint16_samples(const SimpleBLE::ByteArray &array)
{
    std::vector<uint16_t> samples;
    samples.reserve(array.size() / 2);
    for (size_t i = 0; i + 1 < array.size(); i += 2)
    {
        uint16_t number = (static_cast<unsigned char>(array[i]) << 8) | static_cast<unsigned char>(array[i + 1]);
        samples.push_back(number);
    }
    return samples;
}

bool setOutputMode(const std::string &input)
{
    if (input == "0" || input == "all")
    {
        mode = outputMode::ALL;
        return true;
    }
    if (input == "1" || input == "ir")
    {
        mode = outputMode::IR;
        return true;
    }
    if (input == "2" || input == "red")
    {
        mode = outputMode::RED;
        return true;
    }
    if (input == "3" || input == "ir-red" || input == "ir_red")
    {
        mode = outputMode::IR_RED;
        return true;
    }
    return false;
}

void print_current_data(SimpleBLE::ByteArray ir_data, SimpleBLE::ByteArray red_data, double heart_rate)
{
    switch (mode.load())
    {
    case outputMode::IR:
        print_byte_array_uint16(ir_data);
        break;
    case outputMode::RED:
        print_byte_array_uint16(red_data);
        break;
    case outputMode::IR_RED:
        print_byte_array_uint16(ir_data);
        std::cout << " ";
        print_byte_array_uint16(red_data);
        break;
    case outputMode::ALL:
        print_byte_array_uint16(ir_data);
        std::cout << " ";
        print_byte_array_uint16(red_data);
        std::cout << " ";
        std::cout << std::fixed << std::setprecision(1) << heart_rate;
        break;
    }
    std::cout << std::endl;
}

void write_log_data(const std::vector<uint16_t> &ir_samples, const std::vector<uint16_t> &red_samples, double heart_rate)
{
    if (!logFile.is_open())
        return;

    logFile << now_ms() << "\t"
            << std::fixed << std::setprecision(1) << heart_rate << "\t"
            << samples_to_csv(ir_samples) << "\t"
            << samples_to_csv(red_samples) << std::endl;
}

std::string samples_to_csv(const std::vector<uint16_t> &samples)
{
    std::ostringstream out;
    for (size_t i = 0; i < samples.size(); i++)
    {
        if (i > 0)
            out << ",";
        out << samples[i];
    }
    return out.str();
}

uint64_t now_ms()
{
    const auto now = std::chrono::system_clock::now();
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    return static_cast<uint64_t>(millis.count());
}

void print_byte_array_hex(SimpleBLE::ByteArray array)
{
    std::reverse(array.begin(), array.end());
    for (auto byte : array)
        std::cout << std::hex << std::setfill('0') << std::setw(2) << (uint32_t)((uint8_t)byte) << " ";
    std::cout << std::endl;
}

void print_byte_array_float(SimpleBLE::ByteArray array)
{
    float f = *reinterpret_cast<float *>(array.data());
    std::cout << f << std::endl;
}

void print_byte_array_uint16(SimpleBLE::ByteArray array)
{
    for (size_t i = 0; i < array.size(); i += 2)
    {
        uint16_t number = (static_cast<unsigned char>(array[i]) << 8) | static_cast<unsigned char>(array[i + 1]);
        std::cout << number << ",";
    }
}

void print_byte_array_int(SimpleBLE::ByteArray array)
{
    int i = *reinterpret_cast<int *>(array.data());
    std::cout << i;
}

// FIXME タイムアウトとか実装した方が良さそうだぞ
// bool Peripheral.ininitialized()があるっぽ
SimpleBLE::Peripheral findCorBi(SimpleBLE::Adapter adapter)
{
    for (;;)
    {
        SimpleBLE::Peripheral CorBiReader = SimpleBLE::Peripheral();
        adapter.scan_for(500);
        std::vector<SimpleBLE::Peripheral> peripherals = adapter.scan_get_results();
        for (auto peripheral : peripherals)
        {
            if (peripheral.identifier() == "CorBi")
            {
                // std::cout << "CorBi is found." << std::endl;
                CorBiReader = peripheral;
                if (CorBiReader.is_connectable())
                    return CorBiReader;
            }
            // std::cout << "Peripheral: " << peripheral.identifier() << std::endl;
            // std::cout << "Address: " << peripheral.address() << std::endl;
        }
    }
}

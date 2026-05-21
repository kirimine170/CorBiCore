#include <random>
#include <iostream>
#include <thread>
#include <iomanip>
#include <deque>
#include <vector>
#include <cmath>
#include <numeric>

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
// TODO 接続周りはコールバックに変更。
// TODO エラーハンドリングしっかり。

SimpleBLE::Peripheral CorBiReader;

class HeartRateEstimator
{
public:
    void addSamples(const std::vector<uint16_t> &samples)
    {
        for (uint16_t sample : samples)
        {
            window.push_back(sample);
            if (window.size() > WINDOW_SIZE)
                window.pop_front();
        }
    }

    double estimateBpm() const
    {
        if (window.size() < MIN_WINDOW_SIZE)
            return 0.0;

        const double mean = std::accumulate(window.begin(), window.end(), 0.0) / window.size();
        double best_bpm = 0.0;
        double best_power = 0.0;
        for (int bpm = MIN_BPM; bpm <= MAX_BPM; bpm++)
        {
            const double frequency = bpm / 60.0;
            double real = 0.0;
            double imag = 0.0;
            for (size_t i = 0; i < window.size(); i++)
            {
                const double sample = static_cast<double>(window[i]) - mean;
                const double angle = 2.0 * M_PI * frequency * static_cast<double>(i) / SAMPLE_RATE_HZ;
                real += sample * std::cos(angle);
                imag -= sample * std::sin(angle);
            }

            const double power = real * real + imag * imag;
            if (power > best_power)
            {
                best_power = power;
                best_bpm = bpm;
            }
        }
        return best_bpm;
    }

private:
    static constexpr double SAMPLE_RATE_HZ = 100.0;
    static constexpr size_t MIN_WINDOW_SIZE = 128;
    static constexpr size_t WINDOW_SIZE = 256;
    static constexpr int MIN_BPM = 40;
    static constexpr int MAX_BPM = 300;

    std::deque<uint16_t> window;
};

void CorBiCore_exit()
{
    CorBiReader.disconnect();
    std::cerr << "CorBiCore is exit." << std::endl;
}

int main(int argc, char **argv)
{
    std::atexit(CorBiCore_exit);
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
        if (CorBiReader.is_connectable())
            CorBiReader.connect();
        else
            return 1;
        if (!CorBiReader.is_connected())
            return 1;
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
                    heartRateEstimator.addSamples(decode_uint16_samples(rx_data_IR));

                    print_byte_array_uint16(rx_data_IR);
                    std::cout << " ";
                    print_byte_array_uint16(rx_data_RED);
                    std::cout << " ";
                    std::cout << std::fixed << std::setprecision(1) << heartRateEstimator.estimateBpm();
                    std::cout << std::endl;
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

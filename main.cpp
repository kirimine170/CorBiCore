#include <random>
#include <iostream>
#include <thread>
#include <iomanip>

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
// TODO 接続周りはコールバックに変更。
// TODO エラーハンドリングしっかり。

SimpleBLE::Peripheral CorBiReader;
outputMode mode = outputMode::ALL;

enum class outputMode
{
    ALL,
    IR,
    RED,
    IR_RED
};

void CorBiCore_exit()
{
    CorBiReader.disconnect();
    std::cout << "CorBiCore is exit." << std::endl;
}

void userInputListener()
{
    std::string input;
    while (true)
    {
        std::cin >> input;
        if (input == "e")
        {
            exit(0);
        }
        else if (input == "0")
            mode = outputMode::ALL;
        else if (input == "1")
            mode = outputMode::IR;
        else if (input == "2")
            mode = outputMode::RED;
        else if (input == "3")
            mode = outputMode::IR_RED;
        else
        {
            std::cout << "Invalid input." << std::endl;
        }
    }
}

int main(int argc, char **argv)
{
    std::atexit(CorBiCore_exit);
    std::thread inputThread(userInputListener);

    for (;;) // FIXME 流石に関数として括り出した方がいいかも。connect関数とか、read関数とか。
    {

        if (!SimpleBLE::Adapter::bluetooth_enabled())
        {
            std::cout << "Bluetooth is not enabled." << std::endl;
            return 1;
        }

        std::vector<SimpleBLE::Adapter> adapters = SimpleBLE::Adapter::get_adapters();
        if (adapters.empty())
        {
            std::cout << "No BLE adapters found." << std::endl;
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
        for (;;)
        {
            try
            {
                SimpleBLE::ByteArray rx_data = CorBiReader.read(SERVICE_PULSEOXIMETER_UUID, CHARA_ORDER_UUID);
                SimpleBLE::ByteArray rx_data_RED = CorBiReader.read(SERVICE_PULSEOXIMETER_UUID, CHARA_RED_UUID);
                SimpleBLE::ByteArray rx_data_IR = CorBiReader.read(SERVICE_PULSEOXIMETER_UUID, CHARA_IR_UUID);
                if (rx_data_RED != old_data)
                {

                    print_byte_array_uint16(rx_data_IR);
                    std::cout << " ";
                    print_byte_array_uint16(rx_data_RED);
                    std::cout << " ";
                    print_byte_array_int(rx_data);
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
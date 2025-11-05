#ifndef SERIAL_READ_HPP
#define SERIAL_READ_HPP

#include "mbed.h"
#include <string>
#include <sstream>
#include <vector>
#include <queue>

class serial_unit
{
public:
    serial_unit(BufferedSerial &serial);
    void start_event_mode();
    bool get_message(std::string &msg);
    
private:
    void rx_event_callback();
    void process_received_data();
    
    BufferedSerial &men_serial;
    std::string rx_buffer;
    std::queue<std::string> message_queue;
    Mutex queue_mutex;
    
    // 割り込みハンドラ用の軽量バッファ
    static const size_t RX_CHUNK_SIZE = 128;
    char rx_temp_buffer[RX_CHUNK_SIZE];
    volatile bool data_available;
    
    static const size_t MAX_BUFFER_SIZE = 512;  // 最大バッファサイズ（400文字以上に設定）
    static const size_t MAX_QUEUE_SIZE = 10;     // キューの最大サイズ
};

// language: cpp
extern serial_unit serial;
void key_binding();
void serial_read();
void move_aa(std::string msg);

std::vector<double> to_numbers(const std::string &input);
inline serial_unit::serial_unit(BufferedSerial &serial) : men_serial(serial), data_available(false) {}

#endif
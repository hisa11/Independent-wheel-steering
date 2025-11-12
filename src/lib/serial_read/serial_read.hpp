#ifndef SERIAL_READ_HPP
#define SERIAL_READ_HPP

#include "mbed.h"
#include <string>
#include <sstream>
#include <vector>

class serial_unit
{
public:
    serial_unit(BufferedSerial &serial);
    void start_event_mode();
    bool get_message(std::string &msg);
    
private:
    BufferedSerial &men_serial;
    
    static const size_t MAX_BUFFER_SIZE = 512;  // 最大バッファサイズ
};

// language: cpp
extern serial_unit serial;
void key_binding();
void serial_read();
void move_aa(std::string msg);

std::vector<double> to_numbers(const std::string &input);
inline serial_unit::serial_unit(BufferedSerial &serial) : men_serial(serial) {}

#endif